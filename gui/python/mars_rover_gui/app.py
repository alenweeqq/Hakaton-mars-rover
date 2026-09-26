from __future__ import annotations

import argparse
import random
import time

from .controls import HELD_CONTROLS, PULSE_CONTROLS, PULSE_KEYS, keyboard_action
from .hud import telemetry_lines
from .student_profile import create_environment, visible_catalog


class StudentPlayer:
    def __init__(self, args: argparse.Namespace):
        global tk, Image, ImageTk
        import tkinter as tk
        from PIL import Image, ImageTk

        self.root = tk.Tk()
        self.root.title("Mars Rover Manual Control")
        self.fullscreen = bool(args.fullscreen)
        self.sidebar_width = 500
        if self.fullscreen:
            self.root.attributes("-fullscreen", True)
            args.render_width = max(640, self.root.winfo_screenwidth() - self.sidebar_width - 16)
            args.render_height = max(360, self.root.winfo_screenheight() - 16)
        else:
            args.render_width = args.width
            args.render_height = args.height
        self.env = create_environment(args)
        self.seed = args.seed
        self.env.reset(seed=self.seed)
        self.frame_ms = max(1, round(1000 / max(1, args.fps)))
        self.last_frame_time = time.perf_counter()
        self.fps = 0.0
        self.last_reward = 0.0
        self.notice = ""
        self.keys: set[str] = set()
        self.mouse_action = 0
        self.pulse_action = 0
        self.current_action = 0
        self.photo = None
        self.canvas = tk.Label(self.root, bg="#000000", borderwidth=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.sidebar = tk.Frame(
            self.root, width=self.sidebar_width, bg="#171717", padx=10, pady=10
        )
        self.sidebar.grid(row=0, column=1, sticky="ns")
        self.sidebar.grid_propagate(False)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        tk.Label(
            self.sidebar,
            text="ROVER TELEMETRY",
            bg="#171717",
            fg="#ffffff",
            font=("Consolas", 14, "bold"),
        ).pack(anchor="w", pady=(0, 6))
        self.telemetry = tk.Label(
            self.sidebar,
            justify="left",
            anchor="nw",
            bg="#171717",
            fg="#f4f4f4",
            font=("Consolas", 9, "bold"),
        )
        self.telemetry.pack(fill="x")
        tk.Label(
            self.sidebar,
            text="CONTROLS",
            bg="#171717",
            fg="#ffffff",
            font=("Consolas", 12, "bold"),
        ).pack(anchor="w", pady=(8, 4))
        controls = tk.Frame(self.sidebar, bg="#171717")
        controls.pack(fill="x")
        self.control_buttons: dict[str, tuple[tk.Button, int]] = {}
        for index, (name, label, bit) in enumerate(HELD_CONTROLS + PULSE_CONTROLS):
            button = tk.Button(
                controls,
                text=label,
                bg="#252525",
                fg="#eeeeee",
                activebackground="#287a3d",
                activeforeground="#ffffff",
                font=("Consolas", 8, "bold"),
                relief="flat",
                borderwidth=1,
                height=2,
                takefocus=False,
            )
            button.grid(row=index // 3, column=index % 3, sticky="nsew", padx=1, pady=1)
            if (name, label, bit) in HELD_CONTROLS:
                button.bind("<ButtonPress-1>", lambda event, value=bit: self._hold(value, True))
                button.bind("<ButtonRelease-1>", lambda event, value=bit: self._hold(value, False))
                button.bind("<Leave>", lambda event, value=bit: self._hold(value, False))
            else:
                button.configure(command=lambda value=bit: self._pulse(value))
            self.control_buttons[name] = (button, bit)
        for column in range(3):
            controls.grid_columnconfigure(column, weight=1)
        session = tk.Frame(self.sidebar, bg="#171717")
        session.pack(fill="x", pady=(5, 0))
        tk.Button(
            session,
            text="R  RESET SAME WORLD",
            command=self._reset,
            bg="#3b3030",
            fg="#ffffff",
            font=("Consolas", 9, "bold"),
            takefocus=False,
        ).pack(side="left", expand=True, fill="x", padx=(0, 2))
        tk.Button(
            session,
            text="T  RANDOM NEW WORLD",
            command=self._new_track,
            bg="#303b46",
            fg="#ffffff",
            font=("Consolas", 9, "bold"),
            takefocus=False,
        ).pack(side="left", expand=True, fill="x", padx=(2, 0))
        self.status = tk.Label(
            self.sidebar,
            justify="left",
            anchor="sw",
            wraplength=self.sidebar_width - 20,
            bg="#171717",
            fg="#aaaaaa",
            font=("Consolas", 8),
        )
        self.status.pack(side="bottom", fill="x")
        self.root.bind_all("<KeyPress>", self._press)
        self.root.bind_all("<KeyRelease>", self._release)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after_idle(self.root.focus_force)
        self.root.after(0, self._tick)

    def _hold(self, bit: int, active: bool) -> None:
        if active:
            self.mouse_action |= bit
        else:
            self.mouse_action &= ~bit

    def _pulse(self, bit: int) -> None:
        self.pulse_action |= bit

    def _press(self, event) -> None:
        key = event.keysym.lower()
        if key in self.keys:
            return
        self.keys.add(key)
        if key in PULSE_KEYS:
            self._pulse(PULSE_KEYS[key])
        elif key == "r":
            self._reset()
        elif key == "t":
            self._new_track()
        elif key == "f11":
            self.fullscreen = not self.fullscreen
            self.root.attributes("-fullscreen", self.fullscreen)
        elif key in {"escape", "q"}:
            self._close()

    def _release(self, event) -> None:
        self.keys.discard(event.keysym.lower())

    def _reset(self) -> None:
        self.keys.clear()
        self.mouse_action = 0
        self.pulse_action = 0
        self.env.reset(seed=self.seed, options={"trial_start": False})
        self.notice = f"RESET seed={self.seed}"

    def _new_track(self) -> None:
        self.seed = random.randrange(2**31)
        self.keys.clear()
        self.mouse_action = 0
        self.pulse_action = 0
        self.env.reset(seed=self.seed, options={"trial_start": True})
        self.notice = f"NEW WORLD seed={self.seed}"

    def _tick(self) -> None:
        now = time.perf_counter()
        elapsed = now - self.last_frame_time
        self.last_frame_time = now
        if elapsed > 0.0:
            instant = 1.0 / elapsed
            self.fps = instant if self.fps == 0.0 else self.fps * 0.9 + instant * 0.1
        self.current_action = keyboard_action(self.keys) | self.mouse_action | self.pulse_action
        _, reward, terminated, truncated, _ = self.env.step(self.current_action)
        self.last_reward = reward
        self.pulse_action = 0
        debug = self.env.debug_info()
        if terminated or truncated:
            reason = debug.get("termination_reason", 0)
            self.notice = f"RUN ENDED reason={reason}  R OR T TO CONTINUE"
        frame = self.env.render()
        self.photo = ImageTk.PhotoImage(Image.fromarray(frame))
        self.canvas.configure(image=self.photo)
        self.telemetry.configure(text="\n".join(telemetry_lines(debug, self.fps, self.seed)))
        self.status.configure(text=f"reward={self.last_reward:.3f}   {self.notice}")
        self._update_buttons(debug)
        self.root.after(self.frame_ms, self._tick)

    def _update_buttons(self, debug: dict) -> None:
        for name, (button, bit) in self.control_buttons.items():
            active = (self.current_action & bit) == bit
            if name == "climb":
                active = bool(debug.get("climb_mode"))
            elif name == "propeller":
                active = bool(debug.get("propeller_mode"))
            elif name == "charge":
                active = bool(debug.get("solar_panel_requested"))
            elif name == "drive":
                layout = debug.get("drive_layout", "RWD")
                button.configure(text=f"V\n{layout}")
                active = layout != "RWD"
            elif name == "ignition":
                active = bool(debug.get("engine_running"))
            elif name == "heater":
                active = bool(debug.get("heater_active"))
            elif name == "lidar":
                active = bool(debug.get("lidar_active"))
            button.configure(bg="#287a3d" if active else "#252525")

    def _close(self) -> None:
        self.env.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    parser.add_argument("--rig")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--biome")
    parser.add_argument("--list-biomes", action="store_true")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    catalog = visible_catalog()
    if args.list_biomes:
        for item in catalog:
            print(item["id"])
        return
    if args.seed is None:
        args.seed = random.randrange(2**31)
    args.biome_index = None
    if args.biome:
        match = next((item for item in catalog if item["id"] == args.biome), None)
        if match is None:
            parser.error(f"unknown or unavailable biome: {args.biome}")
        args.biome_index = int(match["index"])
    StudentPlayer(args).run()


if __name__ == "__main__":
    main()
