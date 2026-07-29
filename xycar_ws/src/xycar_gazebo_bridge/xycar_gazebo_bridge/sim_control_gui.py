"""Tk coordinate panel for the SLAM-based Gazebo world."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import threading
import tkinter as tk
from tkinter import messagebox, ttk

from PIL import Image, ImageTk

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from .sim_control import GazeboController, OBSTACLE_PRESETS
from .slam_world_generator import (
    CONE_RULE_LAYOUTS,
    GLASS_LAYOUTS,
    MapTransform,
    load_map_yaml,
    map_pixel_to_xy,
    map_xy_to_pixel,
)


class CoordinateControlGui:
    def __init__(
        self,
        root: tk.Tk,
        *,
        world_name: str,
        map_yaml: Path,
        map_texture: Path,
        dry_run: bool,
    ):
        self.root = root
        self.controller = GazeboController(world_name, dry_run=dry_run)
        self.map_config = load_map_yaml(map_yaml)
        self.map_image_source = Image.open(map_texture).convert("RGB")
        self.map_scale = min(
            720 / self.map_image_source.width,
            520 / self.map_image_source.height,
            1.8,
        )
        display_size = (
            round(self.map_image_source.width * self.map_scale),
            round(self.map_image_source.height * self.map_scale),
        )
        self.map_photo = ImageTk.PhotoImage(
            self.map_image_source.resize(
                display_size,
                getattr(Image, "Resampling", Image).NEAREST,
            )
        )
        self.vars = {
            "entity": tk.StringVar(value="xycar_ackermann"),
            "x": tk.StringVar(value="8.649"),
            "y": tk.StringVar(value="7.420"),
            "z": tk.StringVar(value="0.05"),
            "yaw": tk.StringVar(value="-163.43"),
            "obstacle_name": tk.StringVar(value="obstacle_01"),
            "obstacle_type": tk.StringVar(value="person"),
            "obstacle_x": tk.StringVar(value="8.0"),
            "obstacle_y": tk.StringVar(value="7.0"),
            "obstacle_z": tk.StringVar(value=""),
            "obstacle_yaw": tk.StringVar(value="0"),
            "visual_only": tk.BooleanVar(value=False),
            "map_x": tk.StringVar(value="0"),
            "map_y": tk.StringVar(value="0"),
            "map_yaw": tk.StringVar(value="0"),
            "map_scale": tk.StringVar(value="1"),
            "target": tk.StringVar(value="vehicle"),
            "status": tk.StringVar(
                value=(
                    "DRY RUN: Gazebo 명령만 표시"
                    if dry_run
                    else "Gazebo 연결 명령 사용"
                )
            ),
        }
        self._build()

    def _build(self):
        self.root.title("Team KAI SLAM Gazebo 좌표 제어")
        self.root.geometry("1160x720+100+100")
        self.root.minsize(1020, 680)
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        map_frame = ttk.LabelFrame(
            outer,
            text="지도 클릭 → 좌표 선택",
            padding=6,
        )
        map_frame.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(
            map_frame,
            width=self.map_photo.width(),
            height=self.map_photo.height(),
            highlightthickness=1,
            highlightbackground="#555",
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.create_image(0, 0, image=self.map_photo, anchor="nw")
        for section in GLASS_LAYOUTS["upper_right"]:
            half_x = math.cos(section.yaw) * section.length * 0.5
            half_y = math.sin(section.yaw) * section.length * 0.5
            endpoints = (
                (section.x - half_x, section.y - half_y),
                (section.x + half_x, section.y + half_y),
            )
            pixels = [
                map_xy_to_pixel(
                    self.map_config,
                    self.map_image_source.width,
                    self.map_image_source.height,
                    x,
                    y,
                )
                for x, y in endpoints
            ]
            self.canvas.create_line(
                *(
                    coordinate * self.map_scale
                    for point in pixels
                    for coordinate in point
                ),
                fill="#19b5d8",
                width=5,
                tags="glass_section",
            )
        cone_layout = CONE_RULE_LAYOUTS["bottom_right_rule"]
        cone_zone_pixels = [
            map_xy_to_pixel(
                self.map_config,
                self.map_image_source.width,
                self.map_image_source.height,
                x,
                y,
            )
            for x, y in cone_layout.control_polygon
        ]
        self.canvas.create_polygon(
            *(
                coordinate * self.map_scale
                for point in cone_zone_pixels
                for coordinate in point
            ),
            fill="",
            outline="#ef6c00",
            width=3,
            dash=(8, 5),
            tags="cone_rule_zone",
        )
        reference_pixels = [
            map_xy_to_pixel(
                self.map_config,
                self.map_image_source.width,
                self.map_image_source.height,
                x,
                y,
            )
            for x, y in cone_layout.slam_reference
        ]
        self.canvas.create_line(
            *(
                coordinate * self.map_scale
                for point in reference_pixels
                for coordinate in point
            ),
            fill="#ff8f00",
            width=4,
            arrow="last",
            tags="cone_rule_reference",
        )
        self.canvas.bind("<Button-1>", self._on_map_click)
        target_frame = ttk.Frame(map_frame)
        target_frame.pack(fill="x", pady=(6, 0))
        ttk.Label(target_frame, text="클릭 좌표 적용 대상:").pack(side="left")
        ttk.Radiobutton(
            target_frame,
            text="차량/엔티티",
            value="vehicle",
            variable=self.vars["target"],
        ).pack(side="left", padx=6)
        ttk.Radiobutton(
            target_frame,
            text="장애물",
            value="obstacle",
            variable=self.vars["target"],
        ).pack(side="left")
        ttk.Label(
            target_frame,
            text="청록선: 유리 | 주황 점선: 라바콘 rule 구간",
            foreground="#087e98",
        ).pack(side="right", padx=6)

        controls = ttk.Frame(outer, padding=(10, 0, 0, 0))
        controls.pack(side="right", fill="y")
        notebook = ttk.Notebook(controls)
        notebook.pack(fill="both", expand=True)
        pose_tab = ttk.Frame(notebook, padding=10)
        obstacle_tab = ttk.Frame(notebook, padding=10)
        world_tab = ttk.Frame(notebook, padding=10)
        notebook.add(pose_tab, text="차량/맵")
        notebook.add(obstacle_tab, text="장애물")
        notebook.add(world_tab, text="월드")
        self._build_pose_tab(pose_tab)
        self._build_obstacle_tab(obstacle_tab)
        self._build_world_tab(world_tab)

        ttk.Label(
            controls,
            textvariable=self.vars["status"],
            foreground="#174f8a",
            wraplength=330,
        ).pack(fill="x", pady=(8, 4))
        self.log = tk.Text(controls, width=48, height=12, state="disabled")
        self.log.pack(fill="both", expand=True)

    @staticmethod
    def _row(parent, row, label, variable, width=14):
        ttk.Label(parent, text=label).grid(
            row=row,
            column=0,
            sticky="w",
            padx=(0, 8),
            pady=3,
        )
        entry = ttk.Entry(parent, textvariable=variable, width=width)
        entry.grid(row=row, column=1, sticky="ew", pady=3)
        return entry

    def _build_pose_tab(self, parent):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="엔티티").grid(
            row=0, column=0, sticky="w", padx=(0, 8), pady=3
        )
        ttk.Combobox(
            parent,
            textvariable=self.vars["entity"],
            values=("xycar_ackermann", "slam_map"),
            state="normal",
        ).grid(row=0, column=1, sticky="ew", pady=3)
        self._row(parent, 1, "X [m]", self.vars["x"])
        self._row(parent, 2, "Y [m]", self.vars["y"])
        self._row(parent, 3, "Z [m]", self.vars["z"])
        self._row(parent, 4, "Yaw [deg]", self.vars["yaw"])
        ttk.Button(
            parent,
            text="선택 엔티티 이동",
            command=self._set_entity_pose,
        ).grid(row=5, column=0, columnspan=2, sticky="ew", pady=(10, 4))
        ttk.Separator(parent).grid(
            row=6, column=0, columnspan=2, sticky="ew", pady=8
        )
        ttk.Label(
            parent,
            text="지도 좌표 변환",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=7, column=0, columnspan=2, sticky="w")
        self._row(parent, 8, "Map X [m]", self.vars["map_x"])
        self._row(parent, 9, "Map Y [m]", self.vars["map_y"])
        self._row(parent, 10, "Map yaw [deg]", self.vars["map_yaw"])
        self._row(parent, 11, "Map scale", self.vars["map_scale"])
        ttk.Button(
            parent,
            text="맵 위치/회전 적용",
            command=self._apply_map_pose,
        ).grid(row=12, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        ttk.Label(
            parent,
            text=(
                "맵을 옮길 때 entity=slam_map을 선택하세요.\n"
                "scale은 월드 재생성 시 적용됩니다."
            ),
            wraplength=310,
        ).grid(row=13, column=0, columnspan=2, sticky="w")

    def _build_obstacle_tab(self, parent):
        parent.columnconfigure(1, weight=1)
        self._row(parent, 0, "이름", self.vars["obstacle_name"])
        ttk.Label(parent, text="종류").grid(
            row=1, column=0, sticky="w", padx=(0, 8), pady=3
        )
        ttk.Combobox(
            parent,
            textvariable=self.vars["obstacle_type"],
            values=tuple(sorted(OBSTACLE_PRESETS)),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", pady=3)
        self._row(parent, 2, "X [m]", self.vars["obstacle_x"])
        self._row(parent, 3, "Y [m]", self.vars["obstacle_y"])
        self._row(parent, 4, "Z [m]", self.vars["obstacle_z"])
        self._row(parent, 5, "Yaw [deg]", self.vars["obstacle_yaw"])
        ttk.Checkbutton(
            parent,
            text="시각 전용(충돌/LiDAR 없음)",
            variable=self.vars["visual_only"],
        ).grid(row=6, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Button(
            parent,
            text="장애물 생성",
            command=self._spawn_obstacle,
        ).grid(row=7, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(
            parent,
            text="현재 좌표로 이동",
            command=self._move_obstacle,
        ).grid(row=8, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(
            parent,
            text="장애물 삭제",
            command=self._remove_obstacle,
        ).grid(row=9, column=0, columnspan=2, sticky="ew", pady=3)

    def _build_world_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        ttk.Button(
            parent, text="일시정지", command=lambda: self._world_pause(True)
        ).grid(row=0, column=0, sticky="ew", pady=4)
        ttk.Button(
            parent, text="재개", command=lambda: self._world_pause(False)
        ).grid(row=1, column=0, sticky="ew", pady=4)
        ttk.Button(
            parent, text="월드 초기화", command=self._reset_world
        ).grid(row=2, column=0, sticky="ew", pady=4)
        ttk.Label(
            parent,
            text=(
                "유리 시험:\n"
                "glass_panel + 시각 전용 = LiDAR 비검출\n"
                "체크 해제 = 충돌/LiDAR 검출"
            ),
            wraplength=310,
        ).grid(row=3, column=0, sticky="w", pady=(12, 0))

    def _numbers(self, *names):
        values = []
        for name in names:
            text = self.vars[name].get().strip()
            values.append(None if text == "" else float(text))
        return values

    def _run_async(self, description, callback):
        self.vars["status"].set(f"실행 중: {description}")

        def worker():
            try:
                result = callback()
                output = json.dumps(result, indent=2, ensure_ascii=False)
                self.root.after(
                    0,
                    lambda: self._finish(description, output, None),
                )
            except Exception as error:
                self.root.after(
                    0,
                    lambda: self._finish(description, "", error),
                )

        threading.Thread(target=worker, daemon=True).start()

    def _finish(self, description, output, error):
        if error is not None:
            self.vars["status"].set(f"실패: {error}")
            messagebox.showerror("Gazebo 명령 실패", str(error))
            self._append_log(f"ERROR {description}: {error}")
        else:
            self.vars["status"].set(f"완료: {description}")
            self._append_log(f"{description}\n{output}")

    def _append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_entity_pose(self):
        x, y, z, yaw = self._numbers("x", "y", "z", "yaw")
        name = self.vars["entity"].get().strip()
        self._run_async(
            f"{name} 이동",
            lambda: self.controller.set_pose(
                name,
                x=x,
                y=y,
                z=z,
                yaw_deg=yaw,
            ),
        )

    def _spawn_obstacle(self):
        x, y, z, yaw = self._numbers(
            "obstacle_x",
            "obstacle_y",
            "obstacle_z",
            "obstacle_yaw",
        )
        name = self.vars["obstacle_name"].get().strip()
        preset = self.vars["obstacle_type"].get()
        self._run_async(
            f"{name} 생성",
            lambda: self.controller.spawn_obstacle(
                name,
                preset,
                x=x,
                y=y,
                z=z,
                yaw_deg=yaw,
                collision_enabled=not self.vars["visual_only"].get(),
            ),
        )

    def _apply_map_pose(self):
        x, y, yaw = self._numbers("map_x", "map_y", "map_yaw")
        self._run_async(
            "slam_map 위치/회전 적용",
            lambda: self.controller.set_pose(
                "slam_map",
                x=x,
                y=y,
                z=0.0,
                yaw_deg=yaw,
            ),
        )

    def _move_obstacle(self):
        x, y, z, yaw = self._numbers(
            "obstacle_x",
            "obstacle_y",
            "obstacle_z",
            "obstacle_yaw",
        )
        if z is None:
            z = OBSTACLE_PRESETS[
                self.vars["obstacle_type"].get()
            ].default_z
        name = self.vars["obstacle_name"].get().strip()
        self._run_async(
            f"{name} 이동",
            lambda: self.controller.set_pose(
                name,
                x=x,
                y=y,
                z=z,
                yaw_deg=yaw,
            ),
        )

    def _remove_obstacle(self):
        name = self.vars["obstacle_name"].get().strip()
        self._run_async(
            f"{name} 삭제",
            lambda: self.controller.remove(name),
        )

    def _world_pause(self, paused):
        self._run_async(
            "월드 일시정지" if paused else "월드 재개",
            lambda: self.controller.pause(paused),
        )

    def _reset_world(self):
        self._run_async("월드 초기화", self.controller.reset_world)

    def _on_map_click(self, event):
        column = event.x / self.map_scale
        row = event.y / self.map_scale
        map_x, map_y = map_pixel_to_xy(
            self.map_config,
            self.map_image_source.width,
            self.map_image_source.height,
            column,
            row,
        )
        transform = MapTransform(
            offset_x=float(self.vars["map_x"].get()),
            offset_y=float(self.vars["map_y"].get()),
            yaw=math.radians(float(self.vars["map_yaw"].get())),
            scale=float(self.vars["map_scale"].get()),
        )
        x, y = transform.map_to_world(map_x, map_y)
        prefix = (
            "obstacle_"
            if self.vars["target"].get() == "obstacle"
            else ""
        )
        self.vars[prefix + "x"].set(f"{x:.3f}")
        self.vars[prefix + "y"].set(f"{y:.3f}")
        self.canvas.delete("selection")
        radius = 6
        self.canvas.create_oval(
            event.x - radius,
            event.y - radius,
            event.x + radius,
            event.y + radius,
            outline="#e53935",
            width=3,
            tags="selection",
        )
        self.vars["status"].set(f"선택 좌표: x={x:.3f}, y={y:.3f}")


def parse_args(args=None):
    try:
        if get_package_share_directory is None:
            raise LookupError
        share = Path(get_package_share_directory("xycar_gazebo_bridge"))
    except Exception:
        share = Path(__file__).resolve().parents[1]
    default_map = share / "maps" / "slam_glass_balanced"
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="kookmin_xycar_track")
    parser.add_argument(
        "--map-yaml",
        default=str(default_map / "slam_glass_balanced.yaml"),
    )
    parser.add_argument(
        "--map-texture",
        default=str(default_map / "slam_glass_balanced_texture.png"),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Construct one frame, print geometry, and exit",
    )
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    root = tk.Tk()
    CoordinateControlGui(
        root,
        world_name=options.world,
        map_yaml=Path(options.map_yaml),
        map_texture=Path(options.map_texture),
        dry_run=options.dry_run or shutil.which("gz") is None,
    )
    if options.smoke_test:
        root.update_idletasks()
        print(
            json.dumps(
                {
                    "title": root.title(),
                    "geometry": root.geometry(),
                    "gz_available": shutil.which("gz") is not None,
                },
                ensure_ascii=False,
            )
        )
        root.destroy()
        return
    root.mainloop()


if __name__ == "__main__":
    main()
