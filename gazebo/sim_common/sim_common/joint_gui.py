#!/usr/bin/env python3
"""
GUI for posting joint trajectory updates to ROS2 arm model.
Dynamically discovers joints from /joint_states and trajectory topics
via get_topic_names_and_types().
"""

import sys
import threading
import tkinter as tk
import xml.etree.ElementTree as et
from tkinter import ttk

import rclpy
import xacro
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# CONFIG
JOINT_MIN = -3.14
JOINT_MAX = 3.14
DEFAULT_DURATION = 2.0
DISCOVERY_TIMEOUT_S = 30.0  # seconds to wait for /joint_states


class JointPublisher(Node):
    """Main class"""

    def __init__(self, urdf_file_name: str) -> None:
        super().__init__("joint_gui")
        self.possible_joints = self._parse_urdf_joints(urdf_file_name)

        self.joint_names: list[str] = []
        self.current_positions: dict[str, float] = {}
        self._joints_ready = threading.Event()
        self.joint_publishers: dict[str, rclpy.publisher.Publisher] = {}

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

    def _parse_urdf_joints(self, urdf_file_name: str) -> dict[str, dict[str, float | int]]:
        """Reads through the urdf, gets all the urdf revolute joints, parses the
        max and min angles and returns them in the form of a dictionary"""

        doc = xacro.process_file(urdf_file_name)
        root = et.fromstring(doc.toxml())
        joints = {}
        for joint in root.iter("joint"):
            if joint.get("type") == "revolute":
                name = joint.get("name")
                limit = joint.find("limit")

                if name is not None and limit is not None:
                    # initially origin angle will be 0, when a joint is discovered update value
                    joints[name] = {
                        "min": float(limit.get("lower", JOINT_MAX)),
                        "max": float(limit.get("upper", JOINT_MIN)),
                        "origin": 0.0,
                    }
        return joints

    def _on_joint_states(self, msg: JointState) -> None:
        """
        Callback for /joint_states.
        Captures joint names on the first message and signals readiness.
        Continuously updates current_positions on every message.
        """
        if not self.joint_names and msg.name:
            # /joint_states may carry joints this GUI doesn't control (e.g. a
            # rover's continuous wheel joints alongside the arm's revolute
            # ones) - only track the ones we actually parsed sliders for.
            discovered = [name for name in msg.name if name in self.possible_joints]
            if discovered:
                self.joint_names = discovered
                self.get_logger().info(f"Discovered joints: {self.joint_names}")
                self._joints_ready.set()
                # find origin angle and then update accordingly
                for name, position in zip(msg.name, msg.position):
                    if name in self.possible_joints:
                        self.possible_joints[name]["origin"] = position

        # Always track latest positions
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def wait_for_joints(self, timeout: float = DISCOVERY_TIMEOUT_S) -> bool:
        """
        Block until joint names have been discovered or the timeout expires.
        Returns True if joints were found.
        """
        return self._joints_ready.wait(timeout=timeout)

    def discover_trajectory_topics(self) -> list[str]:
        """Return all currently advertised topics that carry JointTrajectory messages"""
        return [
            topic
            for topic, types in self.get_topic_names_and_types()
            if "trajectory_msgs/msg/JointTrajectory" in types
        ]

    def publish(
        self,
        topic: str,
        joint_names: list[str],
        positions: list[float],
        duration: float = DEFAULT_DURATION,
    ) -> None:
        """
        Build and publish a JointTrajectory message to the given topic.
        Creates the publisher lazily on first use.
        """
        if topic not in self.joint_publishers:
            self.joint_publishers[topic] = self.create_publisher(JointTrajectory, topic, 10)

        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)

        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points = [point]

        self.joint_publishers[topic].publish(msg)
        self.get_logger().info(
            f"Published to {topic}: {dict(zip(joint_names, [f'{p:.3f}' for p in positions]))}"
        )


class JointGui:
    """Joint GUI class"""

    _POLL_MS = 100
    _MAX_POLLS = int(DISCOVERY_TIMEOUT_S * 1000 / _POLL_MS)

    def __init__(self, root: tk.Tk, node: JointPublisher) -> None:
        self.root = root
        self.node = node
        self.sliders: dict[str, tk.DoubleVar] = {}
        self.value_labels: dict[str, tk.Label] = {}
        self.duration_var = tk.DoubleVar(value=DEFAULT_DURATION)
        self.status_var = tk.StringVar(value="Discovering joints from /joint_states…")
        self._topic_var = tk.StringVar()
        self._topics_combo: ttk.Combobox | None = None

        root.title("Joint GUI")
        root.minsize(420, 120)

        # Discovery / status label shown while waiting
        self._status_label = tk.Label(root, textvariable=self.status_var, fg="gray")
        self._status_label.pack(pady=16)

        # Frame that holds the real controls (hidden until joints are known)
        self._controls = tk.Frame(root)

        self.root.after(self._POLL_MS, self._poll_discovery)

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def _poll_discovery(self, attempt: int = 0) -> None:
        """
        Periodically check whether joint names have arrived.
        Builds the UI when ready or shows an error after timeout.
        """
        if self.node.joint_names:
            self._build_controls()
        elif attempt >= self._MAX_POLLS:
            self.status_var.set(
                f"No joints found after {DISCOVERY_TIMEOUT_S:.0f}s.\n"
                "Is /joint_states being published?"
            )
            self._status_label.config(fg="red")
        else:
            self.root.after(self._POLL_MS, lambda: self._poll_discovery(attempt + 1))

    def _refresh_topics(self) -> None:
        """Re-query available trajectory topics and update the combobox values"""
        topics = self.node.discover_trajectory_topics()
        if self._topics_combo is not None:
            self._topics_combo["values"] = topics
            if topics and self._topic_var.get() not in topics:
                self._topic_var.set(topics[0])
        self.status_var.set(f"Found {len(topics)} trajectory topic(s)")

    # ------------------------------------------------------------------
    # UI construction (called once joints are known)
    # ------------------------------------------------------------------

    def _build_controls(self) -> None:
        """
        Construct all UI controls (topic selector, joint sliders, duration spinbox, action buttons)
        once joints are known.
        """
        joints = self.node.joint_names
        topics = self.node.discover_trajectory_topics()
        self._topic_var.set(
            topics[0] if topics else "/joint_trajectory_controller/joint_trajectory"
        )

        # --- Topic row ---
        topic_frame = tk.Frame(self._controls, pady=4)
        topic_frame.pack(fill=tk.X, padx=16)
        tk.Label(topic_frame, text="Topic", width=14, anchor="w").pack(side=tk.LEFT)
        self._topics_combo = ttk.Combobox(
            topic_frame,
            textvariable=self._topic_var,
            values=topics,
            width=34,
        )
        self._topics_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(topic_frame, text="Refresh", command=self._refresh_topics).pack(
            side=tk.LEFT, padx=2
        )

        ttk.Separator(self._controls, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=16, pady=6)

        # --- Joint sliders ---
        for joint in joints:
            frame = tk.Frame(self._controls, pady=3)
            frame.pack(fill=tk.X, padx=16)

            tk.Label(frame, text=joint, width=14, anchor="w").pack(side=tk.LEFT)

            var = tk.DoubleVar(value=0.0)
            self.sliders[joint] = var
            max_angle = self.node.possible_joints[joint]["max"]
            min_angle = self.node.possible_joints[joint]["min"]
            ttk.Scale(
                frame,
                from_=min_angle,
                to=max_angle,
                orient=tk.HORIZONTAL,
                variable=var,
                length=220,
                command=lambda val, j=joint: self._on_slider(j, val),  # type: ignore[misc]
            ).pack(side=tk.LEFT, padx=6)

            lbl = tk.Label(frame, text=" 0.000", width=7, anchor="w")
            lbl.pack(side=tk.LEFT)
            self.value_labels[joint] = lbl

        ttk.Separator(self._controls, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=16, pady=6)

        # --- Duration row ---
        dur_frame = tk.Frame(self._controls, pady=4)
        dur_frame.pack(fill=tk.X, padx=16)
        tk.Label(dur_frame, text="Duration (s)", width=14, anchor="w").pack(side=tk.LEFT)
        ttk.Spinbox(
            dur_frame,
            from_=0.1,
            to=10.0,
            increment=0.1,
            textvariable=self.duration_var,
            width=6,
        ).pack(side=tk.LEFT, padx=6)

        # --- Buttons ---
        btn_frame = tk.Frame(self._controls, pady=8)
        btn_frame.pack()
        ttk.Button(btn_frame, text="Send", command=self._send).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_frame, text="Sync", command=self._sync).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_frame, text="Reset", command=self._reset).pack(side=tk.LEFT, padx=4)

        # Swap discovery label for controls + move status label to bottom
        self._status_label.pack_forget()
        self._controls.pack(fill=tk.BOTH, expand=True)
        self.status_var.set(f"Ready - {len(joints)} joints on {len(topics)} topic(s)")
        tk.Label(self.root, textvariable=self.status_var, fg="gray").pack(pady=(0, 8))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_slider(self, joint: str, val: str) -> None:
        """Update the numeric label next to a slider whenever its value changes"""
        self.value_labels[joint].config(text=f"{float(val):+.3f}")

    def _send(self) -> None:
        """Read current slider positions and publish a trajectory command to the selected topic"""
        joints = self.node.joint_names
        positions = [self.sliders[j].get() + self.node.possible_joints[j]["origin"] for j in joints]
        topic = self._topic_var.get().strip()
        if not topic:
            self.status_var.set("No topic selected")
            return
        self.node.publish(topic, joints, positions, self.duration_var.get())
        self.status_var.set(f"Sent to {topic}: {[f'{p:.2f}' for p in positions]}")

    def _sync(self) -> None:
        """Sync sliders to the current joint positions reported by /joint_states"""
        current = self.node.current_positions
        if not current:
            self.status_var.set("No joint state data available")
            return
        for joint in self.node.joint_names:
            if joint in current:
                offset = current[joint] - self.node.possible_joints[joint]["origin"]
                self.sliders[joint].set(offset)
                self.value_labels[joint].config(text=f"{offset:+.3f}")
        self.status_var.set("Synced sliders to current joint positions")

    def _reset(self) -> None:
        """Zero all joint sliders and reset their display labels"""
        for joint in self.node.joint_names:
            self.sliders[joint].set(0.0)
            self.value_labels[joint].config(text="0.0")
        self.status_var.set("Reset")


def main() -> None:
    """Take in the filename,
    Initialise ROS2, spin the node on a background thread, then run the Tkinter event loop"""
    if len(sys.argv) < 2:
        print("Failed no path was provided")
        sys.exit(1)

    filepath = sys.argv[1]
    rclpy.init()
    node = JointPublisher(filepath)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    JointGui(root, node)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_with_filepath(urdf_filepath: str) -> None:
    """Same as main but this is just a method to build with the urdf path in launch scripts"""
    rclpy.init()
    node = JointPublisher(urdf_filepath)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    JointGui(root, node)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
