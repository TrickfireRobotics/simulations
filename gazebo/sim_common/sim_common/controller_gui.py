#!/usr/bin/env python3
"""
Keyboard -> virtual gamepad bridge, for driving a sim robot without a physical controller.

While this window has keyboard focus:
    WASD        -> left stick
    arrow keys  -> right stick
    W / S       -> left stick vertical (shoulder up/down in arm mode)
    up / down   -> right stick vertical (elbow up/down in arm mode)
    Z / C       -> left / right trigger (gripper down/up in arm mode)
    Q / E       -> left / right bumper (claw close/open in arm mode)
    Space       -> Y button (double-tap in mission-control to switch drive/arm mode)

Spoofs a wired Xbox 360 pad (vendor 045e:028e) over uinput, reproducing its exact
raw axis/button layout. That's what lets consumers that read real gamepads - like
mission-control's browser Gamepad API - recognize it and apply the "standard"
mapping, instead of seeing an unrecognized device with buttons/axes in arbitrary
raw indices.
"""

import sys
import tkinter as tk

try:
    from evdev import AbsInfo, UInput, ecodes
except ImportError:
    print(
        "controller_gui needs python3-evdev and /dev/uinput - Linux only, "
        "unsupported on this platform",
        file=sys.stderr,
    )
    sys.exit(1)

STICK_MIN = -32768
STICK_MAX = 32767
STICK_FUZZ = 16
STICK_FLAT = 128
# a real pad's triggers are analog; a keyboard key is all or nothing, so a held
# trigger key sends full deflection
TRIGGER_MAX = 255

# Must match a real wired Xbox 360 pad's identity and raw joydev layout exactly -
# consumers keying off this vendor/product ID remap axes/buttons assuming that layout.
_VENDOR = 0x045E
_PRODUCT = 0x028E
_VERSION = 0x0110

_AXES = [
    (ecodes.ABS_X, AbsInfo(0, STICK_MIN, STICK_MAX, STICK_FUZZ, STICK_FLAT, 0)),
    (ecodes.ABS_Y, AbsInfo(0, STICK_MIN, STICK_MAX, STICK_FUZZ, STICK_FLAT, 0)),
    (ecodes.ABS_Z, AbsInfo(0, 0, TRIGGER_MAX, 0, 0, 0)),  # left trigger
    (ecodes.ABS_RX, AbsInfo(0, STICK_MIN, STICK_MAX, STICK_FUZZ, STICK_FLAT, 0)),
    (ecodes.ABS_RY, AbsInfo(0, STICK_MIN, STICK_MAX, STICK_FUZZ, STICK_FLAT, 0)),
    (ecodes.ABS_RZ, AbsInfo(0, 0, TRIGGER_MAX, 0, 0, 0)),  # right trigger
    (ecodes.ABS_HAT0X, AbsInfo(0, -1, 1, 0, 0, 0)),  # dpad, unused
    (ecodes.ABS_HAT0Y, AbsInfo(0, -1, 1, 0, 0, 0)),
]

_BUTTONS = [
    ecodes.BTN_A,
    ecodes.BTN_B,
    ecodes.BTN_X,
    ecodes.BTN_Y,  # Space - mode toggle
    ecodes.BTN_TL,  # claw close
    ecodes.BTN_TR,  # claw open
    ecodes.BTN_SELECT,
    ecodes.BTN_START,
    ecodes.BTN_THUMBL,
    ecodes.BTN_THUMBR,
]

# how long to wait after a KeyRelease before actually releasing the input, since X11
# key-repeat sends release+press pairs while a key is held down - a same-key press
# within this window cancels the release instead of causing a momentary blip
_RELEASE_DEBOUNCE_MS = 50


def make_device() -> UInput:
    return UInput(
        {ecodes.EV_ABS: _AXES, ecodes.EV_KEY: _BUTTONS},
        name="Microsoft X-Box 360 pad",
        vendor=_VENDOR,
        product=_PRODUCT,
        version=_VERSION,
        bustype=ecodes.BUS_USB,
    )


def _axis(negative_held: bool, positive_held: bool) -> int:
    if negative_held == positive_held:
        return 0
    return STICK_MIN if negative_held else STICK_MAX


def _normalize_key(keysym: str) -> str:
    # letter keys report "W"/"w" depending on shift/caps-lock; arrow keys are
    # always "Up"/"Down"/"Left"/"Right" and should pass through unchanged
    return keysym.lower() if len(keysym) == 1 else keysym


class StickView:
    """A single stick's dot-on-a-pad indicator inside a Tkinter canvas."""

    _RADIUS = 48
    _DOT_RADIUS = 7

    def __init__(self, canvas: tk.Canvas, cx: int, cy: int, label: str):
        self.canvas = canvas
        self.cx = cx
        self.cy = cy
        canvas.create_oval(
            cx - self._RADIUS,
            cy - self._RADIUS,
            cx + self._RADIUS,
            cy + self._RADIUS,
            outline="#666",
            width=2,
        )
        canvas.create_line(cx - self._RADIUS, cy, cx + self._RADIUS, cy, fill="#333")
        canvas.create_line(cx, cy - self._RADIUS, cx, cy + self._RADIUS, fill="#333")
        canvas.create_text(cx, cy + self._RADIUS + 14, text=label, fill="#aaa")
        self.dot = canvas.create_oval(0, 0, 0, 0, fill="#2ecc71", outline="")
        self.set(0, 0)

    def set(self, x: int, y: int) -> None:
        px = self.cx + (x / STICK_MAX) * self._RADIUS
        py = self.cy + (y / STICK_MAX) * self._RADIUS
        r = self._DOT_RADIUS
        self.canvas.coords(self.dot, px - r, py - r, px + r, py + r)


class TriggerView:
    """A trigger's fill bar, with an arrow for the direction the bumper modifier picks."""

    _WIDTH = 28
    _HEIGHT = 96

    def __init__(self, canvas: tk.Canvas, cx: int, cy: int, label: str):
        self.canvas = canvas
        self.cx = cx
        self.cy = cy
        top = cy - self._HEIGHT // 2
        canvas.create_rectangle(
            cx - self._WIDTH // 2,
            top,
            cx + self._WIDTH // 2,
            top + self._HEIGHT,
            outline="#666",
            width=2,
        )
        self.fill = canvas.create_rectangle(0, 0, 0, 0, fill="#2ecc71", outline="")
        self.arrow = canvas.create_text(cx, cy, text="", fill="#1e1e1e", font=("TkDefaultFont", 14))
        canvas.create_text(cx, top + self._HEIGHT + 14, text=label, fill="#aaa")
        self.set(0, False)

    def set(self, value: int, reversed_direction: bool) -> None:
        top = self.cy - self._HEIGHT // 2
        height = int((self._HEIGHT - 4) * value / TRIGGER_MAX)
        if height > 0:
            self.canvas.coords(
                self.fill,
                self.cx - self._WIDTH // 2 + 2,
                top + self._HEIGHT - height - 2,
                self.cx + self._WIDTH // 2 - 2,
                top + self._HEIGHT - 2,
            )
        else:
            # an empty rectangle still draws a line, so park it off the bar
            self.canvas.coords(self.fill, 0, 0, 0, 0)
        self.canvas.itemconfig(
            self.arrow, text=("v" if reversed_direction else "^") if value else ""
        )


class ControllerGui:
    def __init__(self, root: tk.Tk, device: UInput):
        self.root = root
        self.device = device
        self.held: set[str] = set()
        self._pending_release: dict[str, str] = {}

        root.title("Controller GUI - WASD / Arrows / Z,C / Q,E / Space")
        root.resizable(False, False)

        tk.Label(
            root,
            text="Click this window, then drive with WASD (left stick), "
            "arrows (right stick). In arm mode, left vertical is shoulder and right "
            "vertical is elbow, Z/C move the wrist down/up, and Q/E operate the claw. "
            "Space toggles modes.",
            wraplength=460,
            justify="center",
            fg="#ccc",
            bg="#1e1e1e",
        ).pack(padx=12, pady=(12, 6))

        canvas = tk.Canvas(root, width=500, height=150, bg="#1e1e1e", highlightthickness=0)
        canvas.pack(padx=12, pady=6)
        self.left_stick = StickView(canvas, 80, 70, "left stick (WASD)")
        self.right_stick = StickView(canvas, 420, 70, "right stick (arrows)")
        self.left_trigger = TriggerView(canvas, 195, 70, "left trigger (Z)")
        self.right_trigger = TriggerView(canvas, 305, 70, "right trigger (C)")

        buttons_frame = tk.Frame(root, bg="#1e1e1e")
        buttons_frame.pack(pady=(0, 6))
        self.q_light = tk.Label(
            buttons_frame, text="Q - claw close (LB)", width=18, bg="#444", fg="white"
        )
        self.q_light.pack(side=tk.LEFT, padx=6)
        self.e_light = tk.Label(
            buttons_frame, text="E - claw open (RB)", width=18, bg="#444", fg="white"
        )
        self.e_light.pack(side=tk.LEFT, padx=6)
        self.space_light = tk.Label(
            buttons_frame, text="Space - Y (mode)", width=18, bg="#444", fg="white"
        )
        self.space_light.pack(side=tk.LEFT, padx=6)

        self.status = tk.Label(root, text="virtual gamepad ready", fg="#888", bg="#1e1e1e")
        self.status.pack(pady=(0, 10))

        root.configure(bg="#1e1e1e")
        root.bind("<KeyPress>", self._on_key_press)
        root.bind("<KeyRelease>", self._on_key_release)
        root.bind("<FocusOut>", self._on_focus_out)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._sync()
        root.after(50, root.focus_force)

    def _on_key_press(self, event: tk.Event) -> None:
        key = _normalize_key(event.keysym)
        pending = self._pending_release.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if key not in self.held:
            self.held.add(key)
            self._sync()

    def _on_key_release(self, event: tk.Event) -> None:
        key = _normalize_key(event.keysym)
        self._pending_release[key] = self.root.after(
            _RELEASE_DEBOUNCE_MS, lambda: self._finish_release(key)
        )

    def _finish_release(self, key: str) -> None:
        self._pending_release.pop(key, None)
        self.held.discard(key)
        self._sync()

    def _on_focus_out(self, event: tk.Event) -> None:
        if event.widget is not self.root:
            return
        for pending in self._pending_release.values():
            self.root.after_cancel(pending)
        self._pending_release.clear()
        if self.held:
            self.held.clear()
            self._sync()
        self.status.config(text="window not focused - input paused")

    def _sync(self) -> None:
        held = self.held
        left_x = _axis("a" in held, "d" in held)
        left_y = _axis("w" in held, "s" in held)
        right_x = _axis("Left" in held, "Right" in held)
        right_y = _axis("Up" in held, "Down" in held)

        left_trigger = TRIGGER_MAX if "z" in held else 0
        right_trigger = TRIGGER_MAX if "c" in held else 0
        claw_close = "q" in held
        claw_open = "e" in held
        mode_toggle = "space" in held

        self.device.write(ecodes.EV_ABS, ecodes.ABS_X, left_x)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_Y, left_y)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_RX, right_x)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_RY, right_y)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_Z, left_trigger)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_RZ, right_trigger)
        self.device.write(ecodes.EV_KEY, ecodes.BTN_TL, int(claw_close))
        self.device.write(ecodes.EV_KEY, ecodes.BTN_TR, int(claw_open))
        self.device.write(ecodes.EV_KEY, ecodes.BTN_A, 0)
        self.device.write(ecodes.EV_KEY, ecodes.BTN_B, 0)
        self.device.write(ecodes.EV_KEY, ecodes.BTN_Y, int(mode_toggle))
        self.device.syn()

        self.left_stick.set(left_x, left_y)
        self.right_stick.set(right_x, right_y)
        self.left_trigger.set(left_trigger, False)
        self.right_trigger.set(right_trigger, False)
        self.q_light.config(bg="#2ecc71" if claw_close else "#444")
        self.e_light.config(bg="#2ecc71" if claw_open else "#444")
        self.space_light.config(bg="#2ecc71" if mode_toggle else "#444")
        self.status.config(text="virtual gamepad ready")

    def _on_close(self) -> None:
        self.device.close()
        self.root.destroy()


def main() -> None:
    try:
        device = make_device()
    except PermissionError:
        print(
            "Can't open /dev/uinput (permission denied).\n"
            "Add yourself to the 'input' group and re-login, or fix its udev rule:\n"
            '  echo \'KERNEL=="uinput", GROUP="input", MODE="0660"\' | sudo tee /etc/udev/rules.d/60-uinput.rules\n'
            "  sudo udevadm control --reload-rules && sudo udevadm trigger",
            file=sys.stderr,
        )
        sys.exit(1)
    except FileNotFoundError:
        print(
            "/dev/uinput doesn't exist on this machine. Load the kernel module:\n"
            "  sudo modprobe uinput",
            file=sys.stderr,
        )
        sys.exit(1)

    root = tk.Tk()
    gui = ControllerGui(root, device)
    try:
        root.mainloop()
    finally:
        gui.device.close()


if __name__ == "__main__":
    main()
