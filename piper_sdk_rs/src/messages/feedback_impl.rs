//! Message receiving and parsing implementations
//!
//! This module contains implementations for parsing CAN messages into structured data types

use crate::{error::{Error, Result}};
use super::{JointState, GripperState, EndPose, ArmStatus, MotorHighSpeedFeedback, MotorLowSpeedFeedback, CtrlMode, ArmStatusCode, MoveMode, TeachingState, MotionStatus, JointHighSpeedStates};

impl JointState {
    /// Parse joint state from CAN messages
    /// Joint data is split across 3 messages (0x2A5, 0x2A6, 0x2A7)
    pub fn from_bytes(data_12: &[u8], data_34: &[u8], data_56: &[u8]) -> Result<Self> {
        if data_12.len() < 8 || data_34.len() < 8 || data_56.len() < 8 {
            return Err(Error::InvalidMessage("Invalid joint data length".to_string()));
        }

        // in degrees
        let j1 = i32::from_be_bytes([data_12[0], data_12[1], data_12[2], data_12[3]]) as f32 * 0.001;
        let j2 = i32::from_be_bytes([data_12[4], data_12[5], data_12[6], data_12[7]]) as f32 * 0.001;
        let j3 = i32::from_be_bytes([data_34[0], data_34[1], data_34[2], data_34[3]]) as f32 * 0.001;
        let j4 = i32::from_be_bytes([data_34[4], data_34[5], data_34[6], data_34[7]]) as f32 * 0.001;
        let j5 = i32::from_be_bytes([data_56[0], data_56[1], data_56[2], data_56[3]]) as f32 * 0.001;
        let j6 = i32::from_be_bytes([data_56[4], data_56[5], data_56[6], data_56[7]]) as f32 * 0.001;
        
        Ok(Self {
            angles: [j1, j2, j3, j4, j5, j6],
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }
}

impl GripperState {
    /// Parse gripper state from CAN message (0x2A8)
    pub fn from_bytes(data: &[u8]) -> Result<Self> {
        if data.len() < 3 {
            return Err(Error::InvalidMessage("Invalid gripper data length".to_string()));
        }
        
        let position = u16::from_be_bytes([data[0], data[1]]);
        let status = data[2];
        
        Ok(Self {
            position,
            status,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }
}

impl EndPose {
    /// Parse end pose from CAN messages
    /// Pose data is split across 3 messages (0x2A2, 0x2A3, 0x2A4)
    pub fn from_bytes(data_1: &[u8], data_2: &[u8], data_3: &[u8]) -> Result<Self> {
        if data_1.len() < 8 || data_2.len() < 8 || data_3.len() < 8 {
            return Err(Error::InvalidMessage("Invalid end pose data length".to_string()));
        }
        
        let x = i32::from_be_bytes([data_1[0], data_1[1], data_1[2], data_1[3]]) as f32 * 0.001;
        let y = i32::from_be_bytes([data_1[4], data_1[5], data_1[6], data_1[7]]) as f32 * 0.001;
        let z = i32::from_be_bytes([data_2[0], data_2[1], data_2[2], data_2[3]]) as f32 * 0.001;
        let rx = i32::from_be_bytes([data_2[4], data_2[5], data_2[6], data_2[7]]) as f32 * 0.001;
        let ry = i32::from_be_bytes([data_3[0], data_3[1], data_3[2], data_3[3]]) as f32 * 0.001;
        let rz = i32::from_be_bytes([data_3[4], data_3[5], data_3[6], data_3[7]]) as f32 * 0.001;
        
        Ok(Self {
            position: [x, y, z],
            orientation: [rx, ry, rz],
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }
}

impl ArmStatus {
    /// Parse arm status from CAN message (0x2A1)
    /// 
    /// Message layout (8 bytes):
    /// - Byte 0: Control mode
    /// - Byte 1: Arm mode/status
    /// - Byte 2: Mode feedback
    /// - Byte 3: Teaching state
    /// - Byte 4: Motion status
    /// - Byte 5: Trajectory point number
    /// - Byte 6-7: Error code (u16)
    pub fn from_bytes(data: &[u8]) -> Result<Self> {
        if data.len() < 8 {
            return Err(Error::InvalidMessage("Invalid status data length".to_string()));
        }
        
        let ctrl_mode = CtrlMode::try_from(data[0])
            .map_err(|_| Error::InvalidMessage(format!("Invalid control mode: {}", data[0])))?;
        let arm_mode = ArmStatusCode::try_from(data[1])
            .map_err(|_| Error::InvalidMessage(format!("Invalid arm mode: {}", data[1])))?;
        let mode_feed = MoveMode::try_from(data[2])
            .map_err(|_| Error::InvalidMessage(format!("Invalid move mode: {}", data[2])))?;
        let teach_status = TeachingState::try_from(data[3])
            .map_err(|_| Error::InvalidMessage(format!("Invalid teaching state: {}", data[3])))?;
        let motion_status: MotionStatus = MotionStatus::try_from(data[4])
            .map_err(|_| Error::InvalidMessage(format!("Invalid motion status: {}", data[4])))?;
        let trajectory_num = data[5];
        let err_code = u16::from_be_bytes([data[6], data[7]]);
        
        Ok(Self {
            ctrl_mode,
            arm_mode,
            mode_feed,
            teach_status,
            motion_status,
            trajectory_num,
            err_code,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }
}

impl JointHighSpeedStates {
    /// Create from borrowed array reference
    pub fn from_motor_high_speed_feedbacks(feedbacks: &[MotorHighSpeedFeedback; 6]) -> Self {
        let q: [f64; 6] = std::array::from_fn(|i| feedbacks[i].position as f64 * 0.001);
        let dq: [f64; 6] = std::array::from_fn(|i| feedbacks[i].motor_speed as f64 * 0.001);
        let current: [f64; 6] = std::array::from_fn(|i| feedbacks[i].current as f64 * 0.001);
        let effort: [f64; 6] = std::array::from_fn(|i| feedbacks[i].effort as f64 * 0.001);
        let timestamp: f64 = feedbacks[5].timestamp;
        Self { q, dq, current, effort, timestamp }
    }
}

impl MotorHighSpeedFeedback {
    /// Parse from CAN data
    pub fn from_bytes(can_id: u32, data: &[u8]) -> Result<Self> {
        if data.len() < 8 {
            return Err(Error::InvalidMessage("Invalid high-speed feedback data length".to_string()));
        }
        
        let motor_speed = i16::from_be_bytes([data[0], data[1]]);
        let current = i16::from_be_bytes([data[2], data[3]]);
        let position = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        
        // Calculate effort based on motor (joints 1-3 use coefficient 1.18125, joints 4-6 use 0.95844)
        let coefficient = match can_id {
            0x251 | 0x252 | 0x253 => 1.18125,
            0x254 | 0x255 | 0x256 => 0.95844,
            _ => 1.0,
        };
        let effort = (current as f32) * coefficient;
        
        Ok(Self {
            can_id,
            motor_speed,
            current,
            position,
            effort,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }
}

impl MotorLowSpeedFeedback {
    /// Parse from CAN data
    pub fn from_bytes(can_id: u32, data: &[u8]) -> Result<Self> {
        if data.len() < 8 {
            return Err(Error::InvalidMessage("Invalid low-speed feedback data length".to_string()));
        }
        
        let voltage = u16::from_be_bytes([data[0], data[1]]);
        let driver_temp = i16::from_be_bytes([data[2], data[3]]);
        let motor_temp = data[4] as i8;
        let driver_status = data[5];
        let bus_current = u16::from_be_bytes([data[6], data[7]]);
        
        Ok(Self {
            can_id,
            voltage,
            driver_temp,
            motor_temp,
            driver_status,
            bus_current,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("System time is before UNIX epoch")
                .as_secs_f64(),
        })
    }

    /// Check if voltage is too low (bit 0)
    pub fn is_voltage_too_low(&self) -> bool {
        (self.driver_status & (1 << 0)) != 0
    }
    
    /// Check if motor is overheating (bit 1)
    pub fn is_motor_overheating(&self) -> bool {
        (self.driver_status & (1 << 1)) != 0
    }
    
    /// Check if driver is overcurrent (bit 2)
    pub fn is_driver_overcurrent(&self) -> bool {
        (self.driver_status & (1 << 2)) != 0
    }
    
    /// Check if driver is overheating (bit 3)
    pub fn is_driver_overheating(&self) -> bool {
        (self.driver_status & (1 << 3)) != 0
    }
    
    /// Check collision status (bit 4)
    pub fn is_collision_triggered(&self) -> bool {
        (self.driver_status & (1 << 4)) != 0
    }
    
    /// Check driver error status (bit 5)
    pub fn is_driver_error(&self) -> bool {
        (self.driver_status & (1 << 5)) != 0
    }
    
    /// Check if driver is enabled (bit 6)
    pub fn is_driver_enabled(&self) -> bool {
        (self.driver_status & (1 << 6)) != 0
    }
    
    /// Check stall status (bit 7)
    pub fn is_stall_triggered(&self) -> bool {
        (self.driver_status & (1 << 7)) != 0
    }
}