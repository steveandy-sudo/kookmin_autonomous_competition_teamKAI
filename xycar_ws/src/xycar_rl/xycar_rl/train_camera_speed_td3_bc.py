from __future__ import annotations

import argparse
from copy import deepcopy
import csv
from dataclasses import asdict
import json
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset

from xycar_rl.camera_speed_models import (
    DEFAULT_MAX_SPEED_COMMAND,
    DEFAULT_MIN_SPEED_COMMAND,
    load_camera_speed_actor,
)
from xycar_rl.td3_bc import CameraSpeedTD3BCAgent, TD3BCConfig
from xycar_rl.train_camera_speed_bc import grouped_split
from xycar_rl.train_td3_bc import resolve_device
from xycar_rl.transition_dataset import CameraSpeedTransitionDataset


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Train a camera-only steering and speed TD3+BC policy."
    )
    parser.add_argument("--transitions", action="append", required=True)
    parser.add_argument("--initial-checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-speed-command", type=float, default=DEFAULT_MIN_SPEED_COMMAND)
    parser.add_argument("--max-speed-command", type=float, default=DEFAULT_MAX_SPEED_COMMAND)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--validation-ratio", type=float, default=0.1)
    parser.add_argument(
        "--recompute-rewards",
        action="store_true",
        help="Rebuild every transition reward with the current reward contract.",
    )
    parser.add_argument(
        "--world-sdf",
        type=Path,
        default=Path("worlds/kookmin_xycar_track_final.sdf"),
    )
    parser.add_argument("--actor-lr", type=float, default=1.0e-5)
    parser.add_argument("--critic-lr", type=float, default=3.0e-4)
    parser.add_argument("--bc-alpha", type=float, default=2.5)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--tau", type=float, default=0.005)
    parser.add_argument("--policy-noise", type=float, default=0.15)
    parser.add_argument("--noise-clip", type=float, default=0.35)
    parser.add_argument("--policy-delay", type=int, default=2)
    parser.add_argument("--checkpoint-every-epochs", type=int, default=5)
    parser.add_argument(
        "--milestone-dir",
        type=Path,
        help="Optional tracked directory for actor-only deployment checkpoints.",
    )
    return parser.parse_args(argv)


@torch.no_grad()
def validation_metrics(agent, loader) -> dict[str, float]:
    if len(loader) == 0:
        return {
            "validation_steering_mse": float("nan"),
            "validation_speed_mse": float("nan"),
        }
    squared_error = np.zeros(2, dtype=np.float64)
    count = 0
    agent.actor.eval()
    for batch in loader:
        image = batch["image"].to(agent.device)
        action = batch["action"].to(agent.device)
        prediction = agent.actor(image)
        squared_error += (
            torch.sum((prediction - action) ** 2, dim=0).detach().cpu().numpy()
        )
        count += int(action.shape[0])
    agent.actor.train()
    return {
        "validation_steering_mse": float(squared_error[0] / max(1, count)),
        "validation_speed_mse": float(squared_error[1] / max(1, count)),
    }


def save_checkpoint(
    path, agent, *, epoch, train_config, metrics, model_type
) -> None:
    torch.save(
        {
            "model_type": str(model_type),
            "temporal_frames": int(agent.temporal_frames),
            "algorithm": "camera_speed_td3_bc",
            "epoch": int(epoch),
            "min_speed_command": train_config["min_speed_command"],
            "max_speed_command": train_config["max_speed_command"],
            "train_config": train_config,
            "metrics": metrics,
            "actor_state_dict": agent.actor.state_dict(),
            "actor_target_state_dict": agent.actor_target.state_dict(),
            "critic_state_dict": agent.critic.state_dict(),
            "critic_target_state_dict": agent.critic_target.state_dict(),
            "actor_optimizer_state_dict": agent.actor_optimizer.state_dict(),
            "critic_optimizer_state_dict": agent.critic_optimizer.state_dict(),
        },
        path,
    )


def suggested_shadow_cap(epoch: int, epochs: int) -> float:
    caps = (4.0, 5.0, 6.0, 8.0)
    progress = max(0.0, min(1.0, float(epoch) / max(1, int(epochs))))
    index = min(
        len(caps) - 1,
        int(max(0.0, progress * len(caps) - 1.0e-9)),
    )
    return caps[index]


def save_deployment_checkpoint(
    path: Path,
    agent: CameraSpeedTD3BCAgent,
    *,
    epoch: int,
    train_config: dict,
    metrics: dict,
    model_type: str,
    suggested_cap: float,
) -> None:
    torch.save(
        {
            "model_type": str(model_type),
            "temporal_frames": int(agent.temporal_frames),
            "algorithm": "camera_speed_td3_bc",
            "epoch": int(epoch),
            "min_speed_command": train_config["min_speed_command"],
            "max_speed_command": train_config["max_speed_command"],
            "suggested_shadow_speed_cap": float(suggested_cap),
            "metrics": metrics,
            "actor_state_dict": agent.actor.state_dict(),
        },
        path,
    )


def export_scripted_actor(
    path: Path,
    actor: torch.nn.Module,
    *,
    temporal_frames: int,
) -> None:
    export_actor = deepcopy(actor).to("cpu").eval()
    scripted = torch.jit.trace(
        export_actor,
        torch.zeros(1, 3 * temporal_frames, 90, 160),
    )
    scripted.save(str(path))


def write_milestone_manifest(
    milestone_dir: Path,
    milestones: list[dict],
) -> None:
    with (milestone_dir / "milestones.json").open("w", encoding="utf-8") as handle:
        json.dump(milestones, handle, indent=2, sort_keys=True)
    lines = [
        "# High-speed TD3+BC milestones",
        "",
        "Every checkpoint is actor-only and intended for shadow evaluation first.",
        "The suggested cap is a test order, not a real-car safety approval.",
        "",
        "```bash",
        'MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/'
        f'{milestone_dir.name}"',
        "```",
        "",
    ]
    for item in milestones:
        lines.extend(
            [
                f"## Epoch {item['epoch']:03d}",
                "",
                f"Suggested shadow cap: `{item['suggested_shadow_speed_cap']:.1f}`",
                "",
                "```bash",
                "ros2 launch xycar_rl real_shadow.launch.py \\",
                "  policy_kind:=camera_speed_td3_bc \\",
                f"  checkpoint_path:=$MODEL_DIR/{item['checkpoint']} \\",
                "  min_speed_command:=4.0 max_speed_command:=12.0 \\",
                f"  deployment_speed_cap:={item['suggested_shadow_speed_cap']:.1f} \\",
                "  drive_enabled:=false lidar_safety_enabled:=false device:=cpu",
                "```",
                "",
            ]
        )
    (milestone_dir / "README.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def main(argv=None) -> None:
    args = parse_args(argv)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = resolve_device(args.device)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    milestone_dir = (
        None
        if args.milestone_dir is None
        else args.milestone_dir.expanduser().resolve()
    )
    if milestone_dir is not None:
        milestone_dir.mkdir(parents=True, exist_ok=True)
    actor, initial_payload = load_camera_speed_actor(
        args.initial_checkpoint.expanduser().resolve(), device=device
    )
    temporal_frames = int(getattr(actor, "temporal_frames", 1))
    model_type = str(initial_payload.get("model_type", "camera_speed_resnet18"))
    dataset = CameraSpeedTransitionDataset(
        args.transitions,
        min_speed_command=args.min_speed_command,
        max_speed_command=args.max_speed_command,
        temporal_frames=temporal_frames,
        recompute_rewards=args.recompute_rewards,
        world_sdf=args.world_sdf,
    )
    train_indices, validation_indices = grouped_split(
        dataset, args.validation_ratio, args.seed
    )
    loader_args = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": device.type == "cuda",
    }
    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(
        Subset(dataset, train_indices),
        shuffle=True,
        generator=generator,
        **loader_args,
    )
    validation_loader = DataLoader(
        Subset(dataset, validation_indices), shuffle=False, **loader_args
    )
    expected_range = (
        float(initial_payload.get("min_speed_command", args.min_speed_command)),
        float(initial_payload.get("max_speed_command", args.max_speed_command)),
    )
    if expected_range != (args.min_speed_command, args.max_speed_command):
        raise ValueError(
            "initial policy and transition action ranges differ: "
            f"{expected_range} != {(args.min_speed_command, args.max_speed_command)}"
        )
    config = TD3BCConfig(
        gamma=args.gamma,
        tau=args.tau,
        policy_noise=args.policy_noise,
        noise_clip=args.noise_clip,
        policy_delay=args.policy_delay,
        bc_alpha=args.bc_alpha,
        actor_lr=args.actor_lr,
        critic_lr=args.critic_lr,
    )
    agent = CameraSpeedTD3BCAgent(actor, device=device, config=config)
    train_config = {
        **vars(args),
        "initial_checkpoint": str(args.initial_checkpoint.expanduser().resolve()),
        "output_dir": str(output_dir),
        "device": str(device),
        "dataset_rows": len(dataset),
        "train_rows": len(train_indices),
        "validation_rows": len(validation_indices),
        "td3_bc": asdict(config),
        "model_type": model_type,
        "temporal_frames": temporal_frames,
    }
    for key, value in list(train_config.items()):
        if isinstance(value, Path):
            train_config[key] = str(value)
    with (output_dir / "train_config.json").open("w", encoding="utf-8") as handle:
        json.dump(train_config, handle, indent=2, sort_keys=True)

    fields = [
        "epoch",
        "critic_loss",
        "actor_loss",
        "bc_loss",
        "q_scale",
        "validation_steering_mse",
        "validation_speed_mse",
        "elapsed_sec",
    ]
    best_metric = float("inf")
    milestones: list[dict] = []
    started = time.monotonic()
    with (output_dir / "metrics.csv").open(
        "w", encoding="utf-8", newline=""
    ) as history_file:
        writer = csv.DictWriter(history_file, fieldnames=fields)
        writer.writeheader()
        for epoch in range(1, args.epochs + 1):
            sums = {key: 0.0 for key in ("critic_loss", "actor_loss", "bc_loss", "q_scale")}
            counts = {key: 0 for key in sums}
            agent.actor.train()
            for batch in train_loader:
                metrics = agent.update(batch)
                for key, value in metrics.items():
                    if np.isfinite(value):
                        sums[key] += value
                        counts[key] += 1
            row = {
                "epoch": epoch,
                **{key: sums[key] / max(1, counts[key]) for key in sums},
                **validation_metrics(agent, validation_loader),
                "elapsed_sec": time.monotonic() - started,
            }
            writer.writerow(row)
            history_file.flush()
            save_checkpoint(
                output_dir / "camera_speed_td3_bc_latest.pth",
                agent,
                epoch=epoch,
                train_config=train_config,
                metrics=row,
                model_type=model_type,
            )
            selection_metric = (
                row["validation_steering_mse"] + row["validation_speed_mse"]
            )
            if selection_metric < best_metric:
                best_metric = selection_metric
                save_checkpoint(
                    output_dir / "camera_speed_td3_bc_best.pth",
                    agent,
                    epoch=epoch,
                    train_config=train_config,
                    metrics=row,
                    model_type=model_type,
                )
            checkpoint_period = max(1, int(args.checkpoint_every_epochs))
            if epoch % checkpoint_period == 0 or epoch == args.epochs:
                full_checkpoint = (
                    output_dir / f"camera_speed_td3_bc_epoch_{epoch:03d}.pth"
                )
                save_checkpoint(
                    full_checkpoint,
                    agent,
                    epoch=epoch,
                    train_config=train_config,
                    metrics=row,
                    model_type=model_type,
                )
                if milestone_dir is not None:
                    suggested_cap = suggested_shadow_cap(epoch, args.epochs)
                    checkpoint_name = f"camera_speed_td3_bc_epoch_{epoch:03d}.pth"
                    save_deployment_checkpoint(
                        milestone_dir / checkpoint_name,
                        agent,
                        epoch=epoch,
                        train_config=train_config,
                        metrics=row,
                        model_type=model_type,
                        suggested_cap=suggested_cap,
                    )
                    milestones.append(
                        {
                            "epoch": epoch,
                            "checkpoint": checkpoint_name,
                            "suggested_shadow_speed_cap": suggested_cap,
                            "validation_steering_mse": row[
                                "validation_steering_mse"
                            ],
                            "validation_speed_mse": row[
                                "validation_speed_mse"
                            ],
                        }
                    )
                    write_milestone_manifest(milestone_dir, milestones)
            print(
                f"epoch {epoch:03d}/{args.epochs}: critic={row['critic_loss']:.5f} "
                f"actor={row['actor_loss']:.5f} bc={row['bc_loss']:.5f} "
                f"steer={row['validation_steering_mse']:.5f} "
                f"speed={row['validation_speed_mse']:.5f}",
                flush=True,
            )

    best_actor, _ = load_camera_speed_actor(
        output_dir / "camera_speed_td3_bc_best.pth", device="cpu"
    )
    export_scripted_actor(
        output_dir / "camera_speed_td3_bc_actor_scripted.pt",
        best_actor,
        temporal_frames=temporal_frames,
    )
    print(f"wrote camera-speed TD3+BC policy: {output_dir}")


if __name__ == "__main__":
    main()
