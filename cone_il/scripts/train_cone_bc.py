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
from torch.utils.data import DataLoader, Dataset
try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable, *args, **kwargs):
        return iterable

from cone_il.model import build_model
from cone_il.preprocess import load_saved_image_as_tensor


# 설명: 복사로 늘린 같은 주행 샘플이 학습/검증 양쪽에 동시에 들어가지 않도록 묶음 키를 만든다.
def make_split_key(row):
    return (
        row.get('stamp', ''),
        row.get('angle', ''),
        row.get('speed', ''),
        row.get('roi_top_ratio', ''),
        row.get('resize_width', ''),
        row.get('resize_height', ''),
    )


# 설명: 같은 묶음 키를 가진 샘플을 한쪽에 몰아넣어 더 정직한 학습/검증 분리를 만든다.
def split_rows_by_group(rows, val_ratio: float, seed: int):
    groups = {}
    for item in rows:
        groups.setdefault(item[2], []).append(item)

    grouped_rows = list(groups.values())
    random.Random(seed).shuffle(grouped_rows)

    total = sum(len(group) for group in grouped_rows)
    target_val_size = max(1, int(total * val_ratio))
    train_rows = []
    val_rows = []

    for group in grouped_rows:
        if len(val_rows) < target_val_size:
            val_rows.extend(group)
        else:
            train_rows.extend(group)

    if not train_rows:
        raise RuntimeError('Dataset too small. Collect more samples.')
    return train_rows, val_rows


class ConeDataset(Dataset):
    # 설명: labels.csv를 읽어 학습에 사용할 이미지 경로와 조향 라벨 목록을 초기화한다.
    def __init__(self, dataset_dir: Path, max_steer_deg: float = 50.0, augment: bool = False, rows=None):
        self.dataset_dir = Path(dataset_dir)
        self.max_steer_deg = float(max_steer_deg)
        self.augment = augment
        csv_path = self.dataset_dir / 'labels.csv'
        if not csv_path.exists():
            raise FileNotFoundError(csv_path)
        if rows is not None:
            self.rows = list(rows)
            if len(self.rows) == 0:
                raise RuntimeError('No valid rows found in labels.csv')
            return
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
                    self.rows.append((image_path, float(angle_text), make_split_key(row)))
        if len(self.rows) == 0:
            raise RuntimeError('No valid rows found in labels.csv')

    # 설명: 데이터셋에 포함된 유효 샘플 개수를 반환한다.
    def __len__(self):
        return len(self.rows)

    # 설명: 지정한 인덱스의 이미지와 조향 라벨을 학습용 텐서로 반환한다.
    def __getitem__(self, idx):
        path, angle_deg, _split_key = self.rows[idx]
        x = load_saved_image_as_tensor(str(path))

        if self.augment:
            x = self._augment(x)

        y = np.clip(angle_deg / self.max_steer_deg, -1.0, 1.0).astype(np.float32)
        return torch.from_numpy(x), torch.tensor(y, dtype=torch.float32)

    # 설명: 학습 이미지에 밝기 변화와 노이즈를 넣어 데이터 다양성을 높인다.
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


# 설명: 수집된 학습 데이터셋을 읽어 콘 주행 모방학습 모델을 학습하고 저장한다.
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

    device = torch.device('cuda' if torch.cuda.is_available() and not args.cpu else 'cpu')

    full_ds = ConeDataset(dataset_dir, max_steer_deg=args.max_steer_deg, augment=False)
    train_rows, val_rows = split_rows_by_group(full_ds.rows, args.val_ratio, args.seed)
    train_size = len(train_rows)
    val_size = len(val_rows)
    train_ds = ConeDataset(dataset_dir, max_steer_deg=args.max_steer_deg, augment=True, rows=train_rows)
    val_ds = ConeDataset(dataset_dir, max_steer_deg=args.max_steer_deg, augment=False, rows=val_rows)

    pin_memory = device.type == 'cuda'
    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
    )

    model = build_model(args.model_arch, pretrained=args.pretrained).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    criterion = nn.SmoothL1Loss()

    best_val = float('inf')
    best_epoch = 0
    best_mae = float('inf')
    best_mae_epoch = 0
    best_path = output_dir / 'cone_bc_best.pth'
    scripted_path = output_dir / 'cone_bc_scripted.pt'
    best_mae_path = output_dir / 'cone_bc_best_mae.pth'
    scripted_best_mae_path = output_dir / 'cone_bc_scripted_best_mae.pt'
    history = []

    print(
        f'dataset={dataset_dir}, samples={len(full_ds)}, train={train_size}, val={val_size}, '
        f'model_arch={args.model_arch}, pretrained={args.pretrained}, '
        f'max_steer_deg={args.max_steer_deg}, device={device}'
    )

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
                'model_arch': args.model_arch,
                'pretrained': bool(args.pretrained),
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
                'model_arch': args.model_arch,
                'pretrained': bool(args.pretrained),
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

        if val_mae_deg < best_mae:
            best_mae = val_mae_deg
            best_mae_epoch = epoch
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'max_steer_deg': args.max_steer_deg,
                'model_arch': args.model_arch,
                'pretrained': bool(args.pretrained),
                'dataset_dir': str(dataset_dir),
                'train_loss': train_loss,
                'val_loss': val_loss,
                'val_mae_deg': val_mae_deg,
            }, best_mae_path)
            example = torch.zeros(1, 3, args.resize_height, args.resize_width, device=device)
            scripted = torch.jit.trace(model, example)
            scripted.save(str(scripted_best_mae_path))
            print(f'  saved best mae: {best_mae_path}')
            print(f'  saved scripted best mae: {scripted_best_mae_path}')

    config = {
        'max_steer_deg': args.max_steer_deg,
        'model_arch': args.model_arch,
        'pretrained': bool(args.pretrained),
        'resize_width': args.resize_width,
        'resize_height': args.resize_height,
        'roi_top_ratio': args.roi_top_ratio,
        'split_mode': 'group_by_stamp_angle_speed_roi_size',
        'train_size': train_size,
        'val_size': val_size,
        'best_val_loss': best_val,
        'best_epoch': best_epoch,
        'best_val_mae_deg': best_mae,
        'best_mae_epoch': best_mae_epoch,
        'scripted_model': str(scripted_path),
        'scripted_best_mae_model': str(scripted_best_mae_path),
        'save_every_epoch': bool(args.save_every_epoch),
        'save_every_n_epochs': int(args.save_every_n_epochs),
    }
    with open(output_dir / 'model_config.json', 'w') as f:
        json.dump(config, f, indent=2)
    with open(output_dir / 'training_history.json', 'w') as f:
        json.dump(history, f, indent=2)
    print('done')


# 설명: 명령행 인자를 읽어 학습이나 도구 실행 옵션으로 변환한다.
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
    p.add_argument(
        '--model-arch',
        default='cnn',
        choices=['cnn', 'mobilenet_v3_small', 'mobilenet_v3_large', 'efficientnet_b0'],
        help='Model architecture to train',
    )
    p.add_argument(
        '--pretrained',
        action='store_true',
        help='Use torchvision ImageNet pretrained weights for torchvision model arches',
    )
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
