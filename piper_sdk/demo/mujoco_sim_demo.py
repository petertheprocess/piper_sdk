#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Minimal MuJoCo demo wiring PiperSimInterface to a MuJoCo model.

Assumptions:
- An MJCF model path is provided and already defines 6 position/torque-capable
  actuators whose names match the given joint_names (default: j1..j6).
- A vcan/virtual CAN endpoint exists (e.g., via `ip link add dev vcan0 type vcan; ip link set up vcan0`).
- External process (e.g., controller) publishes Piper CAN commands to that bus.

Usage:
    python -m piper_sdk.demo.mujoco_sim_demo --mjcf path/to/arm.xml --can vcan0
"""

import argparse
import sys
import time

try:
    import mujoco
    import mujoco.viewer
except Exception as exc:  # pragma: no cover - runtime env dependent
    raise ImportError("mujoco is required for this demo") from exc

from piper_sim_interface import PiperMujocoAdapter, PiperSimInterface


def parse_args():
    parser = argparse.ArgumentParser(description="Run Piper MuJoCo bridge demo.")
    parser.add_argument("--mjcf", required=True, help="Path to MJCF model with Piper joints/actuators.")
    parser.add_argument("--can", default="vcan0", help="CAN interface name (default: vcan0).")
    parser.add_argument(
        "--joint-names",
        nargs=6,
        default=["j1", "j2", "j3", "j4", "j5", "j6"],
        help="Joint/actuator names in the MJCF matching Piper ordering.",
    )
    parser.add_argument("--dt", type=float, default=0.002, help="Simulation step (seconds).")
    return parser.parse_args()


def main():
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)

    # Bridge between CAN simulator and MuJoCo
    sim_bridge = PiperSimInterface(can_name=args.can)
    adapter = PiperMujocoAdapter(model, data, args.joint_names)

    # Wire callbacks
    sim_bridge._on_position_cmd = adapter.apply_position_targets
    sim_bridge._on_mit_cmd = adapter.apply_mit

    sim_bridge.start()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_push = time.time()
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

            now = time.time()
            if now - last_push >= args.dt:
                adapter.push_feedback(sim_bridge)
                last_push = now
            time.sleep(args.dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
