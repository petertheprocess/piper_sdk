#!/usr/bin/env python3
# -*-coding:utf8-*-

import math
import threading
import time
from typing import Callable, Iterable, List, Optional

import can

from ..piper_msgs.msg_v2 import (
    ArmMessageMapping,
    ArmMsgFeedBackJointStates,
    ArmMsgFeedbackStatus,
    ArmMsgFeedbackStatusEnum,
    ArmMsgJointCtrl,
    ArmMsgJointMitCtrl,
    ArmMsgType,
    PiperMessage,
)
from ..protocol.protocol_v2 import C_PiperParserV2
from ..utils import LogLevel, LogManager, global_area

try:
    # Optional helper for socketcan vcan setup
    from ..demo.manage_vcan import create_vcan, is_vcan
except Exception:  # pragma: no cover - helper is optional
    create_vcan = None
    is_vcan = None


class PiperSimInterface:
    """
    A lightweight simulation endpoint that exchanges Piper CAN frames over vcan.

    The simulator listens for joint position (0x155-0x157) and MIT control
    (0x15A-0x15F) messages, updates cached targets, and can publish feedback
    frames (joint state + status) so the SDK sees a responsive arm. It is kept
    intentionally small so it can be embedded in MuJoCo, gz-sim, Isaac Lab, etc.
    """

    def __init__(
        self,
        can_name: str = "vcan0",
        bustype: str = "socketcan",
        bitrate: int = 1_000_000,
        auto_setup_vcan: bool = True,
        logger_level: LogLevel = LogLevel.INFO,
        on_position_cmd: Optional[Callable[[List[int]], None]] = None,
        on_mit_cmd: Optional[Callable[[int, ArmMsgJointMitCtrl], None]] = None,
    ) -> None:
        LogManager.update_logger(
            global_area=global_area,
            local_area="PiperSimInterface",
            level=logger_level,
            log_to_file=False,
            log_file_path=None,
            file_mode="a",
            force_update=True,
        )
        self.logger = LogManager.get_logger(global_area, "PiperSimInterface")

        self._can_name = can_name
        self._bitrate = bitrate
        self._desired_bustype = bustype
        self._parser = C_PiperParserV2()
        self._joint_targets: List[int] = [0] * 6  # in 0.001 degree
        self._mit_targets: List[ArmMsgJointMitCtrl] = [ArmMsgJointMitCtrl() for _ in range(6)]
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._bus: Optional[can.BusABC] = self._create_bus(auto_setup_vcan)
        self._on_position_cmd = on_position_cmd
        self._on_mit_cmd = on_mit_cmd

    # ---------------------------- public API ---------------------------------
    def start(self) -> None:
        if not self._bus or (self._rx_thread and self._rx_thread.is_alive()):
            return
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._rx_thread.start()
        # Publish an initial healthy status
        self.publish_status()
        self.publish_joint_feedback()

    def stop(self, timeout: float = 0.2) -> None:
        self._stop_event.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=timeout)

    def publish_joint_feedback(self, joint_angles_rad: Optional[Iterable[float]] = None) -> None:
        """
        Push simulated joint states onto the bus so the SDK can consume them.

        Args:
            joint_angles_rad: iterable of six joint positions in radians. When
                omitted, the last commanded targets are re-used.
        """
        with self._lock:
            if joint_angles_rad is not None:
                self._joint_targets = [self._rad_to_milli_degree(pos) for pos in joint_angles_rad]
        self._send_joint_feedback_frames()

    def publish_status(
        self,
        ctrl_mode: ArmMsgFeedbackStatusEnum.CtrlMode = ArmMsgFeedbackStatusEnum.CtrlMode.CAN_CTRL,
        arm_status: ArmMsgFeedbackStatusEnum.ArmStatus = ArmMsgFeedbackStatusEnum.ArmStatus.NORMAL,
        move_mode: ArmMsgFeedbackStatusEnum.ModeFeed = ArmMsgFeedbackStatusEnum.ModeFeed.MOVE_J,
    ) -> None:
        status = ArmMsgFeedbackStatus(
            ctrl_mode.value,
            arm_status.value,
            move_mode.value,
            ArmMsgFeedbackStatusEnum.TeachingState.DISABLED.value,
            ArmMsgFeedbackStatusEnum.MotionStatus.REACH_TARGET_POS_SUCCESSFULLY.value,
            0,
            0,
        )
        msg = PiperMessage(type_=ArmMsgType.PiperMsgStatusFeedback, arm_status_msgs=status)
        self._send_can_message(msg)

    # ---------------------------- internals ----------------------------------
    def _create_bus(self, auto_setup_vcan: bool) -> Optional[can.BusABC]:
        if self._desired_bustype == "socketcan":
            try:
                return can.interface.Bus(channel=self._can_name, bustype="socketcan", bitrate=self._bitrate)
            except Exception as exc:  # pragma: no cover - runtime environment dependent
                self.logger.warning("socketcan %s unavailable (%s)", self._can_name, exc)
                if auto_setup_vcan and create_vcan and (not is_vcan or not is_vcan(self._can_name)):
                    try:
                        create_vcan(self._can_name, self._bitrate)
                        return can.interface.Bus(channel=self._can_name, bustype="socketcan", bitrate=self._bitrate)
                    except Exception as setup_exc:
                        self.logger.warning("auto vcan setup failed, fallback to virtual: %s", setup_exc)
        try:
            return can.interface.Bus(channel=self._can_name, bustype="virtual")
        except Exception as exc:  # pragma: no cover - only hit if python-can missing backends
            self.logger.error("Failed to create simulation bus: %s", exc)
            return None

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._bus.recv(0.01) if self._bus else None
            except Exception as exc:  # pragma: no cover - backend specific
                self.logger.error("Simulation bus recv failed: %s", exc)
                break
            if frame is None:
                continue
            self._handle_frame(frame)

    def _handle_frame(self, frame: can.Message) -> None:
        try:
            msg_type = ArmMessageMapping.get_mapping(can_id=frame.arbitration_id)
        except ValueError:
            return
        if msg_type in (
            ArmMsgType.PiperMsgJointCtrl_12,
            ArmMsgType.PiperMsgJointCtrl_34,
            ArmMsgType.PiperMsgJointCtrl_56,
        ):
            self._handle_position_ctrl(msg_type, frame.data)
        elif msg_type in (
            ArmMsgType.PiperMsgJointMitCtrl_1,
            ArmMsgType.PiperMsgJointMitCtrl_2,
            ArmMsgType.PiperMsgJointMitCtrl_3,
            ArmMsgType.PiperMsgJointMitCtrl_4,
            ArmMsgType.PiperMsgJointMitCtrl_5,
            ArmMsgType.PiperMsgJointMitCtrl_6,
        ):
            self._handle_mit_ctrl(msg_type, frame.data)

    def _handle_position_ctrl(self, msg_type: ArmMsgType, data) -> None:
        with self._lock:
            if msg_type == ArmMsgType.PiperMsgJointCtrl_12:
                self._joint_targets[0] = self._decode_int32(data, 0)
                self._joint_targets[1] = self._decode_int32(data, 4)
            elif msg_type == ArmMsgType.PiperMsgJointCtrl_34:
                self._joint_targets[2] = self._decode_int32(data, 0)
                self._joint_targets[3] = self._decode_int32(data, 4)
            elif msg_type == ArmMsgType.PiperMsgJointCtrl_56:
                self._joint_targets[4] = self._decode_int32(data, 0)
                self._joint_targets[5] = self._decode_int32(data, 4)
            joint_snapshot = list(self._joint_targets)
        self.publish_joint_feedback()
        if self._on_position_cmd:
            self._on_position_cmd(joint_snapshot)

    def _handle_mit_ctrl(self, msg_type: ArmMsgType, data) -> None:
        motor_idx = msg_type.value - ArmMsgType.PiperMsgJointMitCtrl_1.value
        mit_cmd = self._decode_mit(data)
        with self._lock:
            self._mit_targets[motor_idx] = mit_cmd
            if mit_cmd.pos_ref is not None:
                self._joint_targets[motor_idx] = self._rad_to_milli_degree(mit_cmd.pos_ref)
            joint_snapshot = list(self._joint_targets)
        self.publish_joint_feedback()
        if self._on_mit_cmd:
            self._on_mit_cmd(motor_idx + 1, mit_cmd)
        if self._on_position_cmd:
            self._on_position_cmd(joint_snapshot)

    def _send_joint_feedback_frames(self) -> None:
        with self._lock:
            joint_state = ArmMsgFeedBackJointStates(*self._joint_targets)
        pairs = [
            (ArmMsgType.PiperMsgJointFeedBack_12, (joint_state.joint_1, joint_state.joint_2)),
            (ArmMsgType.PiperMsgJointFeedBack_34, (joint_state.joint_3, joint_state.joint_4)),
            (ArmMsgType.PiperMsgJointFeedBack_56, (joint_state.joint_5, joint_state.joint_6)),
        ]
        for msg_type, (j_a, j_b) in pairs:
            fb = ArmMsgFeedBackJointStates(j_a, j_b, 0, 0, 0, 0)
            msg = PiperMessage(type_=msg_type, arm_joint_feedback=fb)
            self._send_can_message(msg)

    def _send_can_message(self, msg: PiperMessage) -> None:
        if not self._bus:
            return
        tx_frame = can.Message(is_extended_id=False)
        try:
            self._parser.EncodeMessage(msg, tx_frame)
            tx_frame.dlc = len(tx_frame.data)
            self._bus.send(tx_frame)
        except Exception as exc:  # pragma: no cover - backend specific
            self.logger.error("Simulation send failed: %s", exc)

    # ---------------------------- helpers ------------------------------------
    def _decode_int32(self, data, start: int) -> int:
        raw = self._parser.ConvertBytesToInt(data, start, start + 4)
        return self._parser.ConvertToNegative_32bit(raw)

    def _decode_mit(self, data) -> ArmMsgJointMitCtrl:
        pos_raw = self._parser.ConvertBytesToInt(data, 0, 2)
        vel_raw = ((data[2] << 4) | (data[3] >> 4)) & 0xFFF
        kp_raw = ((data[3] & 0x0F) << 8) | data[4]
        kd_raw = ((data[5] << 4) | (data[6] >> 4)) & 0xFFF
        torque_raw = ((data[6] & 0x0F) << 4) | ((data[7] >> 4) & 0x0F)
        return ArmMsgJointMitCtrl(
            pos_ref=self._uint_to_float(pos_raw, -12.5, 12.5, 16),
            vel_ref=self._uint_to_float(vel_raw, -45.0, 45.0, 12),
            kp=self._uint_to_float(kp_raw, 0.0, 500.0, 12),
            kd=self._uint_to_float(kd_raw, -5.0, 5.0, 12),
            t_ref=self._uint_to_float(torque_raw, -8.0, 8.0, 8),
            crc=data[7] & 0x0F,
        )

    def _uint_to_float(self, x_int: int, x_min: float, x_max: float, bits: int) -> float:
        span = (1 << bits) - 1
        return x_min + (x_int * (x_max - x_min)) / span

    def _rad_to_milli_degree(self, rad: float) -> int:
        return int(round(math.degrees(rad) * 1000.0))


__all__ = ["PiperSimInterface"]
