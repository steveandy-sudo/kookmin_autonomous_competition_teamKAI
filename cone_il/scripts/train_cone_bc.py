#!/usr/bin/env python3
import argparse
import csv
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, random_split
from tqdm import tqdm

from cone_il.model import ConeBCNet
from cone_il.preprocess import load_saved_image_as_tensor


class ConeDataset(Dataset):
    def __init__(self, dataset_dir: Path, max_steer_deg: float = 50.0, augment: bool = False):
        self.dataset_dir = Path(dataset_dir)
        self.max_steer_deg = float(max_steer_deg)
        self.augment = augment
        csv_path = self.dataset_dir / 'labels.csv'
        if not csv_path.exists():
            raise FileNotFoundError(csv_path)
        self.rows = []
        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                image_name = row.get('image_path', '')
                angle_text = row.get('angle', '')
                if not image_name or not angle_text:
                    continue
                image_path = self.dataset_dir / str(image_name)
                if image_path.exists():
                    self.rows.append((image_path, float(angle_text)))
        if len(self.rows) == 0:
            raise RuntimeError('No valid rows found in labels.csv')

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        path, angle_deg = self.rows[idx]
        x = load_saved_image_as_tensor(str(path))

        if self.augment:
            x = self._augment(x)

        y = np.clip(angle_deg / self.max_steer_deg, -1.0, 1.0).astype(np.float32)
        return torch.from_numpy(x), torch.tensor(y, dtype=torch.float32)

    def _augment(self, x: np.ndarray) -> np.ndarray:
        # x is RGB CHW [0,1]
        if random.random() < 0.6:
            gain = random.uniform(0.75, 1.25)
            bias = random.uniform(-0.06, 0.06)
            x = np.clip(x * gain + bias, 0.0, 1.0)
        if random.random() < 0.25:
            # small blur to make the model less brittle
            hwc = np.transpose(x, (1, 2, 0))
            hwc = cv2.GaussianBlur(hwc, (3, 3), 0)
            x = np.transpose(hwc, (2, 0, 1))
        return x.astype(np.float32)


def train(args):
    dataset_dir = Path(args.dataset_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_dir = output_dir / 'checkpoints'
    save_epoch_checkpoints = (
        bool(args.save_every_epoch)
        or int(args.save_every_n_epochs) > 0
    )
    if save_epoch_checkpoints:
        checkpoint_dir.mkdir(parents=True, exist_ok=True)

    full_ds = ConeDataset(dataset_dir, max_steer_deg=args.max_steer_deg, augment=True)
    val_size = max(1, int(len(full_ds) * args.val_ratio))
    train_size = len(full_ds) - val_size
    if train_size < 1:
        raise RuntimeError('Dataset too small. Collect more samples.')

    train_ds, val_ds = random_split(
        full_ds,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(args.seed),
    )
    # Disable augmentation for validation subset by wrapping another dataset is overkill;
    # this is good enough for quick project iteration.

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)

    device = torch.device('cuda' if torch.cuda.is_available() and not args.cpu else 'cpu')
    model = ConeBCNet().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    criterion = nn.SmoothL1Loss()

    best_val = float('inf')
    best_epoch = 0
    best_path = output_dir / 'cone_bc_best.pth'
    scripted_path = output_dir / 'cone_bc_scripted.pt'
    history = []

    print(f'dataset={dataset_dir}, samples={len(full_ds)}, train={train_size}, val={val_size}, device={device}')

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for x, y in tqdm(train_loader, desc=f'epoch {epoch}/{args.epochs} train'):
            x = x.to(device)
            y = y.to(device)
            pred = model(x)
            loss = criterion(pred, y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * x.size(0)
        train_loss /= train_size

        model.eval()
        val_loss = 0.0
        val_mae_deg = 0.0
        with torch.no_grad():
            for x, y in tqdm(val_loader, desc=f'epoch {epoch}/{args.epochs} val'):
                x = x.to(device)
                y = y.to(device)
                pred = model(x)
                loss = criterion(pred, y)
                val_loss += loss.item() * x.size(0)
                val_mae_deg += torch.mean(torch.abs(pred - y)).item() * args.max_steer_deg * x.size(0)
        val_loss /= val_size
        val_mae_deg /= val_size

        print(f'epoch={epoch:03d} train_loss={train_loss:.5f} val_loss={val_loss:.5f} val_mae={val_mae_deg:.2f} deg')
        metrics = {
            'epoch': epoch,
            'train_loss': train_loss,
            'val_loss': val_loss,
            'val_mae_deg': val_mae_deg,
        }
        history.append(metrics)

        save_this_epoch = bool(args.save_every_epoch)
        if int(args.save_every_n_epochs) > 0 and epoch % int(args.save_every_n_epochs) == 0:
            save_this_epoch = True
        if save_this_epoch:
            checkpoint_path = checkpoint_dir / f'cone_bc_epoch_{epoch:03d}.pth'
            checkpoint_scripted_path = checkpoint_dir / f'cone_bc_epoch_{epoch:03d}.pt'
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'max_steer_deg': args.max_steer_deg,
                'dataset_dir': str(dataset_dir),
                'train_loss': train_loss,
                'val_loss': val_loss,
                'val_mae_deg': val_mae_deg,
            }, checkpoint_path)
            example = torch.zeros(1, 3, args.resize_height, args.resize_width, device=device)
            scripted = torch.jit.trace(model, example)
            scripted.save(str(checkpoint_scripted_path))
            print(f'  saved epoch checkpoint: {checkpoint_path}')
            print(f'  saved epoch scripted: {checkpoint_scripted_path}')

        if val_loss < best_val:
            best_val = val_loss
            best_epoch = epoch
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'max_steer_deg': args.max_steer_deg,
                'dataset_dir': str(dataset_dir),
                'train_loss': train_loss,
                'val_loss': val_loss,
                'val_mae_deg': val_mae_deg,
            }, best_path)
            example = torch.zeros(1, 3, args.resize_height, args.resize_width, device=device)
            scripted = torch.jit.trace(model, example)
            scripted.save(str(scripted_path))
            print(f'  saved best: {best_path}')
            print(f'  saved scripted: {scripted_path}')

    config = {
        'max_steer_deg': args.max_steer_deg,
        'resize_width': args.resize_width,
        'resize_height': args.resize_height,
        'roi_top_ratio': args.roi_top_ratio,
        'best_val_loss': best_val,
        'best_epoch': best_epoch,
        'scripted_model': str(scripted_path),
        'save_every_epoch': bool(args.save_every_epoch),
        'save_every_n_epochs': int(args.save_every_n_epochs),
    }
    with open(output_dir / 'model_config.json', 'w') as f:
        json.dump(config, f, indent=2)
    with open(output_dir / 'training_history.json', 'w') as f:
        json.dump(history, f, indent=2)
    print('done')


def parse_args():
    p = argparse.ArgumentParser(description='Train cone behavior cloning model')
    p.add_argument('--dataset-dir', required=True, help='Path to dataset directory containing labels.csv')
    p.add_argument('--output-dir', default='~/cone_il_model', help='Where to save model files')
    p.add_argument('--epochs', type=int, default=30)
    p.add_argument('--batch-size', type=int, default=64)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--weight-decay', type=float, default=1e-4)
    p.add_argument('--val-ratio', type=float, default=0.15)
    p.add_argument('--num-workers', type=int, default=2)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--max-steer-deg', type=float, default=50.0)
    p.add_argument('--resize-width', type=int, default=160)
    p.add_argument('--resize-height', type=int, default=90)
    p.add_argument('--roi-top-ratio', type=float, default=0.45)
    p.add_argument(
        '--save-every-epoch',
        action='store_true',
        help='Save checkpoint and TorchScript model for every epoch under output-dir/checkpoints',
    )
    p.add_argument(
        '--save-every-n-epochs',
        type=int,
        default=0,
        help='Save checkpoint and TorchScript model every N epochs; 0 disables this option',
    )
    p.add_argument('--cpu', action='store_true')
    return p.parse_args()


if __name__ == '__main__':
    train(parse_args())
