#!/usr/bin/env python3
import argparse
import socket
import time
import tkinter as tk

ARM_CMD = "ARM"
DISARM_CMD = "DISARM"
THROTTLE_ON_CMD = "THROTTLE_ON"
THROTTLE_OFF_CMD = "THROTTLE_OFF"
THROTTLE_UP_CMD = "THROTTLE_UP"
THROTTLE_DOWN_CMD = "THROTTLE_DOWN"
RESET_CMD = "RESET"
GET_PID_CMD = "GET_PID"
GET_PIDI_CMD = "GET_PIDI"
PIDI_SCALE = 10000


def send_command(sock: socket.socket, addr: tuple[str, int], cmd: str) -> None:
    try:
        sock.sendto((cmd + "\n").encode("utf-8"), addr)
    except OSError:
        pass


def format_pidi_command(values: list[float]) -> str:
    ints = [int(round(value * PIDI_SCALE)) for value in values]
    formatted = " ".join(str(value) for value in ints)
    return f"PIDI {formatted}"


class UdpKeyboardGui:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.addr = (args.host, args.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.armed = False
        self.space_active = False
        self.space_pressed = False
        self.release_pending = False
        self.release_pending_at = 0.0
        self.last_keepalive = 0.0

        self.root = tk.Tk()
        self.root.title("UDP Keyboard Control")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        info = (
            f"Host {args.host}:{args.port} | "
            "A: arm/disarm, R: reset, W: emergency stop, Space: throttle, U/D: throttle base, "
            "S: set PID, G: get PID, Q: quit"
        )
        tk.Label(self.root, text=info).pack(padx=8, pady=(8, 4))

        self.status_var = tk.StringVar(value="Armed: no | Throttle: off")
        tk.Label(self.root, textvariable=self.status_var).pack(padx=8, pady=(0, 6))

        button_frame = tk.Frame(self.root)
        button_frame.pack(padx=8, pady=(0, 6), fill="x")

        self.arm_button = tk.Button(button_frame, text="ARM", width=10, command=self.toggle_arm)
        self.arm_button.pack(side="left", padx=4)

        tk.Button(button_frame, text="RESET", width=10, command=self.reset).pack(side="left", padx=4)
        tk.Button(button_frame, text="W STOP", width=10, command=self.emergency_stop).pack(side="left", padx=4)
        tk.Button(button_frame, text="UP", width=10, command=self.increase_throttle_base).pack(side="left", padx=4)
        tk.Button(button_frame, text="DOWN", width=10, command=self.decrease_throttle_base).pack(side="left", padx=4)
        tk.Button(button_frame, text="SET PID", width=10, command=self.set_pid).pack(side="left", padx=4)
        tk.Button(button_frame, text="GET PID", width=10, command=self.request_pid).pack(side="left", padx=4)

        pid_frame = tk.Frame(self.root)
        pid_frame.pack(padx=8, pady=6)

        self.pid_vars: dict[str, dict[str, tk.DoubleVar]] = {}
        self.angle_vars: dict[str, tk.StringVar] = {}
        defaults = {
            "roll": (0.0, 0.0, 0.0),
            "pitch": (0.0, 0.0, 0.0),
            "yaw": (0.0, 0.0, 0.0),
        }

        for axis in ("roll", "pitch", "yaw"):
            frame = tk.LabelFrame(pid_frame, text=f"{axis} angle")
            frame.pack(side="left", padx=6, pady=4, fill="both", expand=True)
            kp_var = tk.DoubleVar(value=defaults[axis][0])
            ki_var = tk.DoubleVar(value=defaults[axis][1])
            kd_var = tk.DoubleVar(value=defaults[axis][2])
            self.pid_vars[axis] = {"kp": kp_var, "ki": ki_var, "kd": kd_var}

            tk.Scale(
                frame,
                from_=args.pid_min,
                to=args.pid_max,
                resolution=args.pid_step,
                orient="horizontal",
                label="Kp",
                variable=kp_var,
                length=200,
            ).pack(padx=6, pady=4)
            tk.Scale(
                frame,
                from_=args.pid_min,
                to=args.pid_max,
                resolution=args.pid_step,
                orient="horizontal",
                label="Ki",
                variable=ki_var,
                length=200,
            ).pack(padx=6, pady=4)
            tk.Scale(
                frame,
                from_=args.pid_min,
                to=args.pid_max,
                resolution=args.pid_step,
                orient="horizontal",
                label="Kd",
                variable=kd_var,
                length=200,
            ).pack(padx=6, pady=4)

            angle_var = tk.StringVar(value="current: 0.00 deg")
            self.angle_vars[axis] = angle_var
            tk.Label(frame, textvariable=angle_var).pack(padx=6, pady=(2, 6))

        self.log_text = tk.Text(self.root, height=6, width=70, state="disabled")
        self.log_text.pack(padx=8, pady=(4, 8))

        self.root.bind_all("<KeyPress>", self.on_key_press)
        self.root.bind_all("<KeyRelease-space>", self.on_space_release)
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.root.after(200, self.request_pid)
        self.root.after(20, self.tick)

    def log(self, message: str) -> None:
        if not message:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message.rstrip() + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def update_status(self) -> None:
        armed_text = "yes" if self.armed else "no"
        throttle_text = "on" if self.space_active else "off"
        self.status_var.set(f"Armed: {armed_text} | Throttle: {throttle_text}")
        self.arm_button.configure(text="DISARM" if self.armed else "ARM")

    def toggle_arm(self) -> None:
        self.armed = not self.armed
        send_command(self.sock, self.addr, ARM_CMD if self.armed else DISARM_CMD)
        if not self.armed:
            self.space_active = False
            self.space_pressed = False
            self.release_pending = False
            self.release_pending_at = 0.0
            send_command(self.sock, self.addr, THROTTLE_OFF_CMD)
        self.update_status()

    def reset(self) -> None:
        self.armed = False
        self.space_active = False
        self.space_pressed = False
        self.release_pending = False
        self.release_pending_at = 0.0
        self.last_keepalive = 0.0
        send_command(self.sock, self.addr, RESET_CMD)
        self.update_status()

    def emergency_stop(self) -> None:
        self.armed = False
        self.space_active = False
        self.space_pressed = False
        self.release_pending = False
        self.release_pending_at = 0.0
        send_command(self.sock, self.addr, THROTTLE_OFF_CMD)
        send_command(self.sock, self.addr, DISARM_CMD)
        self.log("TX: THROTTLE_OFF")
        self.log("TX: DISARM")
        self.update_status()

    def set_pid(self) -> None:
        values = [
            self.pid_vars["roll"]["kp"].get(),
            self.pid_vars["roll"]["ki"].get(),
            self.pid_vars["roll"]["kd"].get(),
            self.pid_vars["pitch"]["kp"].get(),
            self.pid_vars["pitch"]["ki"].get(),
            self.pid_vars["pitch"]["kd"].get(),
            self.pid_vars["yaw"]["kp"].get(),
            self.pid_vars["yaw"]["ki"].get(),
            self.pid_vars["yaw"]["kd"].get(),
        ]
        command = format_pidi_command(values)
        send_command(self.sock, self.addr, command)
        self.log(f"TX: {command}")

    def request_pid(self) -> None:
        send_command(self.sock, self.addr, GET_PIDI_CMD)
        self.log("TX: GET_PIDI")

    def increase_throttle_base(self) -> None:
        send_command(self.sock, self.addr, THROTTLE_UP_CMD)
        self.log("TX: THROTTLE_UP")

    def decrease_throttle_base(self) -> None:
        send_command(self.sock, self.addr, THROTTLE_DOWN_CMD)
        self.log("TX: THROTTLE_DOWN")

    def apply_pid_values(self, values: list[float]) -> None:
        if len(values) != 9:
            return
        self.pid_vars["roll"]["kp"].set(values[0])
        self.pid_vars["roll"]["ki"].set(values[1])
        self.pid_vars["roll"]["kd"].set(values[2])
        self.pid_vars["pitch"]["kp"].set(values[3])
        self.pid_vars["pitch"]["ki"].set(values[4])
        self.pid_vars["pitch"]["kd"].set(values[5])
        self.pid_vars["yaw"]["kp"].set(values[6])
        self.pid_vars["yaw"]["ki"].set(values[7])
        self.pid_vars["yaw"]["kd"].set(values[8])

    def apply_attitude_values(self, roll: float, pitch: float, yaw: float) -> None:
        self.angle_vars["roll"].set(f"current: {roll:.2f} deg")
        self.angle_vars["pitch"].set(f"current: {pitch:.2f} deg")
        self.angle_vars["yaw"].set(f"current: {yaw:.2f} deg")

    def handle_incoming_line(self, line: str) -> None:
        clean = line.strip()
        if not clean:
            return
        upper = clean.upper()
        if upper.startswith("PIDI"):
            payload = clean[4:].replace(",", " ").strip()
            parts = [p for p in payload.split() if p]
            if len(parts) >= 9:
                try:
                    values = [int(p) / PIDI_SCALE for p in parts[:9]]
                except ValueError:
                    self.log(f"RX: {clean}")
                else:
                    self.apply_pid_values(values)
                    self.log(f"RX: {clean}")
                    return
        elif upper.startswith("PID"):
            payload = clean[3:].replace(",", " ").strip()
            parts = [p for p in payload.split() if p]
            if len(parts) >= 9:
                try:
                    values = [float(p) for p in parts[:9]]
                except ValueError:
                    self.log(f"RX: {clean}")
                else:
                    self.apply_pid_values(values)
                    self.log(f"RX: {clean}")
                    return
        elif upper.startswith("ATT"):
            payload = clean[3:].replace(",", " ").strip()
            parts = [p for p in payload.split() if p]
            if len(parts) >= 3:
                try:
                    values = [float(p) for p in parts[:3]]
                except ValueError:
                    self.log(f"RX: {clean}")
                else:
                    self.apply_attitude_values(values[0], values[1], values[2])
                    return
        self.log(f"RX: {clean}")

    def on_key_press(self, event: tk.Event) -> str | None:
        key = event.keysym.lower()
        if key == "a":
            self.toggle_arm()
            return "break"
        elif key == "r":
            self.reset()
            return "break"
        elif key == "s":
            self.set_pid()
            return "break"
        elif key == "g":
            self.request_pid()
            return "break"
        elif key == "u":
            self.increase_throttle_base()
            return "break"
        elif key == "d":
            self.decrease_throttle_base()
            return "break"
        elif key == "w":
            self.emergency_stop()
            return "break"
        elif key == "space":
            self.space_pressed = True
            self.release_pending = False
            self.release_pending_at = 0.0
            now = time.time()
            if self.armed:
                self.space_active = True
                if now - self.last_keepalive >= self.args.keepalive:
                    send_command(self.sock, self.addr, THROTTLE_ON_CMD)
                    self.last_keepalive = now
            self.update_status()
            return "break"
        elif key in ("q", "escape"):
            self.on_close()
            return "break"
        return None

    def on_space_release(self, _: tk.Event) -> str:
        self.space_pressed = False
        if self.space_active:
            self.release_pending = True
            self.release_pending_at = time.time()
        return "break"

    def on_focus_out(self, _: tk.Event) -> None:
        self.space_pressed = False
        self.release_pending = False
        self.release_pending_at = 0.0
        if self.space_active:
            self.space_active = False
            send_command(self.sock, self.addr, THROTTLE_OFF_CMD)
            self.update_status()

    def tick(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            else:
                text = data.decode("utf-8", errors="replace")
                for line in text.splitlines():
                    self.handle_incoming_line(line)

        now = time.time()
        if self.release_pending and not self.space_pressed:
            if (now - self.release_pending_at) >= self.args.release_grace:
                self.release_pending = False
                self.release_pending_at = 0.0
                if self.space_active:
                    self.space_active = False
                    send_command(self.sock, self.addr, THROTTLE_OFF_CMD)
                    self.log("TX: THROTTLE_OFF")
                    self.update_status()

        if self.space_active:
            if now - self.last_keepalive >= self.args.keepalive:
                send_command(self.sock, self.addr, THROTTLE_ON_CMD)
                self.last_keepalive = now

        self.root.after(20, self.tick)

    def on_close(self) -> None:
        try:
            send_command(self.sock, self.addr, THROTTLE_OFF_CMD)
            send_command(self.sock, self.addr, DISARM_CMD)
        except OSError:
            pass
        self.sock.close()
        self.root.destroy()

    def run(self) -> int:
        self.root.mainloop()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="UDP keyboard control (AP mode).")
    parser.add_argument("--host", default="192.168.4.1", help="ESP32 AP IP")
    parser.add_argument("--port", type=int, default=14550, help="UDP port")
    parser.add_argument(
        "--keepalive",
        type=float,
        default=0.01,
        help="Throttle keepalive interval (seconds)",
    )
    parser.add_argument(
        "--release-grace",
        type=float,
        default=0.2,
        help="Delay before sending THROTTLE_OFF after Space release (seconds)",
    )
    parser.add_argument("--pid-min", type=float, default=0.0, help="PID slider minimum")
    parser.add_argument("--pid-max", type=float, default=10.0, help="PID slider maximum")
    parser.add_argument("--pid-step", type=float, default=0.01, help="PID slider step")
    args = parser.parse_args()

    return UdpKeyboardGui(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
