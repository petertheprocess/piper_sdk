//! Message types for Piper robot arm communication
//!
//! This module is split into three parts:
//! - Type definitions (this file)
//! - Receiving/parsing messages from CAN
//! - Sending/encoding messages to CAN

mod feedback_impl;
mod command_impl;
mod enums;

pub use enums::{CtrlMode, ArmStatusCode, MoveMode, TeachingState, MotionStatus, MotorEnableStatus};

/// Joint state information (6 joints + gripper)
#[derive(Debug, Clone, Default)]
pub struct JointState {
    /// Joint angles in degrees [J1, J2, J3, J4, J5, J6]
    pub angles: [f32; 6],
    /// Timestamp in seconds
    pub timestamp: f64,
}

/// Gripper state information
#[derive(Debug, Clone, Default)]
pub struct GripperState {
    /// Gripper position (0-1000)
    pub position: u16,
    /// Gripper status
    pub status: u8,
    /// Timestamp in seconds
    pub timestamp: f64,
}

/// End effector pose information
#[derive(Debug, Clone, Default)]
pub struct EndPose {
    /// Position [X, Y, Z] in meters
    pub position: [f32; 3],
    /// Orientation [RX, RY, RZ] in radians
    pub orientation: [f32; 3],
    /// Timestamp in seconds
    pub timestamp: f64,
}

/// Arm status information
///
/// CAN Message ID: 0x2A1
/// Message layout (8 bytes):
/// - Byte 0: Control mode (CtrlMode)
/// - Byte 1: Arm mode (ArmStatusCode)
/// - Byte 2: Mode feedback (MoveMode)
/// - Byte 3: Teaching state (TeachingState)
/// - Byte 4: Motion status (0x00: reached, 0x01: not reached)
/// - Byte 5: Current trajectory point number (0-255)
/// - Byte 6-7: Error code (u16, bit flags for joint/communication errors)
#[derive(Debug, Clone, Default)]
pub struct ArmStatus {
    /// Control mode
    pub ctrl_mode: CtrlMode,
    /// Arm mode/status
    pub arm_mode: ArmStatusCode,
    /// Mode feedback (movement mode)
    pub mode_feed: MoveMode,
    /// Teaching state
    pub teach_status: TeachingState,
    /// Motion status (0x00: target reached, 0x01: target not reached)
    pub motion_status: MotionStatus,
    /// Current trajectory point number (0-255, used in offline trajectory mode)
    pub trajectory_num: u8,
    /// Error code (bit flags: bits 0-5 for joint angle limits, bits 0-5 for communication errors)
    pub err_code: u16,
    /// Timestamp in seconds
    pub timestamp: f64,
}

/// Joint control command
#[derive(Debug, Clone)]
pub struct JointControl {
    /// Target joint angles in degrees [J1, J2, J3, J4, J5, J6]
    pub angles: [f32; 6],
}

/// MIT control parameters for a single joint
///
/// Message byte/bit layout (CAN data payload - 8 bytes):
/// - Byte 0: pos_ref [bits 15..8] (high 8 bits)
/// - Byte 1: pos_ref [bits 7..0] (low 8 bits)
/// - Byte 2: vel_ref [bits 11..4] (upper bits of 12-bit value)
/// - Byte 3: vel_ref [bits 3..0] (lower 4 bits), kp [bits 11..8] (upper 4 bits)
/// - Byte 4: kp [bits 7..0] (lower 8 bits) — kp typical reference: 10
/// - Byte 5: kd [bits 11..4] (upper bits of 12-bit value) — kd typical reference: 0.8
/// - Byte 6: kd [bits 3..0] (lower 4 bits), t_ref [bits 7..4] (upper 4 bits)
/// - Byte 7: t_ref [bits 3..0] (lower 4 bits), CRC [bits 3..0] (lower 4 bits)
///
/// CRC is computed as XOR of bytes 0..6, then masked to 4 bits.
#[derive(Debug, Clone)]
pub struct JointMitControl {
    /// Motor number (1-6)
    pub motor_num: u8,
    /// Target position in radians (range: -12.5 to 12.5)
    pub pos_ref: f32,
    /// Target velocity in rad/s (range: -45.0 to 45.0)
    pub vel_ref: f32,
    /// Proportional gain (range: 0.0 to 500.0, reference: 10.0)
    pub kp: f32,
    /// Derivative gain (range: -5.0 to 5.0, reference: 0.8)
    pub kd: f32,
    /// Target torque in Nm (range: -18.0 to 18.0)
    pub t_ref: f32,
}

/// MIT mode field for MotionCtrl2
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MitMode {
    PosVel = 0x00,
    MIT = 0xAD,
    Invalid = 0xFF,
}

impl Default for MitMode {
    fn default() -> Self { MitMode::PosVel }
}

impl From<MitMode> for u8 {
    fn from(m: MitMode) -> u8 { m as u8 }
}

impl std::convert::TryFrom<u8> for MitMode {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(MitMode::PosVel),
            0xAD => Ok(MitMode::MIT),
            0xFF => Ok(MitMode::Invalid),
            _ => Err(v),
        }
    }
}

/// Motion control mode settings
#[derive(Debug, Clone)]
pub struct MotionCtrl2 {
    /// Control mode
    pub ctrl_mode: CtrlMode,
    /// Move mode
    pub move_mode: MoveMode,
    /// Movement speed percentage (0-100)
    pub move_spd_rate: u8,
    /// MIT mode enable
    pub is_mit_mode: MitMode,
    /// Residence time for offline trajectory
    pub residence_time: u8,
    /// Installation position
    pub installation_pos: u8,
}

/// End effector pose control (Cartesian coordinates)
#[derive(Debug, Clone)]
pub struct EndPoseControl {
    /// Position [X, Y, Z] in millimeters
    pub position: [i32; 3],
    /// Orientation [RX, RY, RZ] in milliradians
    pub orientation: [i32; 3],
}

/// Motor high-speed feedback information
#[derive(Debug, Clone, Default)]
pub struct MotorHighSpeedFeedback {
    /// CAN ID (0x251-0x256)
    pub can_id: u32,
    /// Motor speed in rad/s (unit: 0.001 rad/s)
    pub motor_speed: i16,
    /// Motor current in A (unit: 0.001 A)
    pub current: u16,
    /// Motor position in radians
    pub position: i32,
    /// Motor effort/torque in N/m (unit: 0.001 N/m)
    pub effort: f32,
    /// Timestamp
    pub timestamp: f64,
}

/// Motor low-speed feedback information
#[derive(Debug, Clone, Default)]
pub struct MotorLowSpeedFeedback {
    /// CAN ID (0x261-0x266)
    pub can_id: u32,
    /// Bus voltage in V (unit: 0.1 V)
    pub voltage: u16,
    /// Driver temperature in °C
    pub driver_temp: i16,
    /// Motor temperature in °C
    pub motor_temp: i8,
    /// Driver status byte
    pub driver_status: u8,
    /// Bus current in A (unit: 0.001 A)
    pub bus_current: u16,
    /// Timestamp
    pub timestamp: f64,
}



/// Gripper control command
#[derive(Debug, Clone)]
pub struct GripperControl {
    /// Target gripper position (0-1000)
    pub position: u16,
    /// Gripper speed (0-1000)
    pub speed: u16,
}
