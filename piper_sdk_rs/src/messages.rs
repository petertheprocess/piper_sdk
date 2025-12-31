//! Message types for Piper robot arm communication

use crate::error::{Error, Result};

/// Joint state information (6 joints + gripper)
#[derive(Debug, Clone)]
pub struct JointState {
    /// Joint angles in radians [J1, J2, J3, J4, J5, J6]
    pub angles: [f32; 6],
    /// Timestamp in seconds
    pub timestamp: f64,
}

impl JointState {
    /// Create a new JointState with zero angles
    pub fn new() -> Self {
        Self {
            angles: [0.0; 6],
            timestamp: 0.0,
        }
    }
    
    /// Parse joint state from CAN messages
    /// Joint data is split across 3 messages (0x2A5, 0x2A6, 0x2A7)
    pub fn from_can_data(data_12: &[u8], data_34: &[u8], data_56: &[u8]) -> Result<Self> {
        if data_12.len() < 8 || data_34.len() < 8 || data_56.len() < 8 {
            return Err(Error::InvalidMessage("Invalid joint data length".to_string()));
        }
        
        let j1 = i32::from_le_bytes([data_12[0], data_12[1], data_12[2], data_12[3]]) as f32 * 0.001;
        let j2 = i32::from_le_bytes([data_12[4], data_12[5], data_12[6], data_12[7]]) as f32 * 0.001;
        let j3 = i32::from_le_bytes([data_34[0], data_34[1], data_34[2], data_34[3]]) as f32 * 0.001;
        let j4 = i32::from_le_bytes([data_34[4], data_34[5], data_34[6], data_34[7]]) as f32 * 0.001;
        let j5 = i32::from_le_bytes([data_56[0], data_56[1], data_56[2], data_56[3]]) as f32 * 0.001;
        let j6 = i32::from_le_bytes([data_56[4], data_56[5], data_56[6], data_56[7]]) as f32 * 0.001;
        
        Ok(Self {
            angles: [j1, j2, j3, j4, j5, j6],
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        })
    }
}

impl Default for JointState {
    fn default() -> Self {
        Self::new()
    }
}

/// Gripper state information
#[derive(Debug, Clone)]
pub struct GripperState {
    /// Gripper position (0-1000)
    pub position: u16,
    /// Gripper status
    pub status: u8,
    /// Timestamp in seconds
    pub timestamp: f64,
}

impl GripperState {
    /// Create a new GripperState
    pub fn new() -> Self {
        Self {
            position: 0,
            status: 0,
            timestamp: 0.0,
        }
    }
    
    /// Parse gripper state from CAN message (0x2A8)
    pub fn from_can_data(data: &[u8]) -> Result<Self> {
        if data.len() < 3 {
            return Err(Error::InvalidMessage("Invalid gripper data length".to_string()));
        }
        
        let position = u16::from_le_bytes([data[0], data[1]]);
        let status = data[2];
        
        Ok(Self {
            position,
            status,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        })
    }
}

impl Default for GripperState {
    fn default() -> Self {
        Self::new()
    }
}

/// End effector pose information
#[derive(Debug, Clone)]
pub struct EndPose {
    /// Position [X, Y, Z] in meters
    pub position: [f32; 3],
    /// Orientation [RX, RY, RZ] in radians
    pub orientation: [f32; 3],
    /// Timestamp in seconds
    pub timestamp: f64,
}

impl EndPose {
    /// Create a new EndPose
    pub fn new() -> Self {
        Self {
            position: [0.0; 3],
            orientation: [0.0; 3],
            timestamp: 0.0,
        }
    }
    
    /// Parse end pose from CAN messages
    /// Pose data is split across 3 messages (0x2A2, 0x2A3, 0x2A4)
    pub fn from_can_data(data_1: &[u8], data_2: &[u8], data_3: &[u8]) -> Result<Self> {
        if data_1.len() < 8 || data_2.len() < 8 || data_3.len() < 8 {
            return Err(Error::InvalidMessage("Invalid end pose data length".to_string()));
        }
        
        let x = i32::from_le_bytes([data_1[0], data_1[1], data_1[2], data_1[3]]) as f32 * 0.001;
        let y = i32::from_le_bytes([data_1[4], data_1[5], data_1[6], data_1[7]]) as f32 * 0.001;
        let z = i32::from_le_bytes([data_2[0], data_2[1], data_2[2], data_2[3]]) as f32 * 0.001;
        let rx = i32::from_le_bytes([data_2[4], data_2[5], data_2[6], data_2[7]]) as f32 * 0.001;
        let ry = i32::from_le_bytes([data_3[0], data_3[1], data_3[2], data_3[3]]) as f32 * 0.001;
        let rz = i32::from_le_bytes([data_3[4], data_3[5], data_3[6], data_3[7]]) as f32 * 0.001;
        
        Ok(Self {
            position: [x, y, z],
            orientation: [rx, ry, rz],
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        })
    }
}

impl Default for EndPose {
    fn default() -> Self {
        Self::new()
    }
}

/// Arm status information
#[derive(Debug, Clone)]
pub struct ArmStatus {
    /// Control mode
    pub ctrl_mode: u8,
    /// Arm mode
    pub arm_mode: u8,
    /// Error code
    pub err_code: u16,
    /// Timestamp in seconds
    pub timestamp: f64,
}

impl ArmStatus {
    /// Create a new ArmStatus
    pub fn new() -> Self {
        Self {
            ctrl_mode: 0,
            arm_mode: 0,
            err_code: 0,
            timestamp: 0.0,
        }
    }
    
    /// Parse arm status from CAN message (0x2A1)
    pub fn from_can_data(data: &[u8]) -> Result<Self> {
        if data.len() < 4 {
            return Err(Error::InvalidMessage("Invalid status data length".to_string()));
        }
        
        let ctrl_mode = data[0];
        let arm_mode = data[1];
        let err_code = u16::from_le_bytes([data[2], data[3]]);
        
        Ok(Self {
            ctrl_mode,
            arm_mode,
            err_code,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        })
    }
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// Joint control command
#[derive(Debug, Clone)]
pub struct JointControl {
    /// Target joint angles in radians [J1, J2, J3, J4, J5, J6]
    pub angles: [f32; 6],
}

impl JointControl {
    /// Create a new JointControl
    pub fn new(angles: [f32; 6]) -> Self {
        Self { angles }
    }
    
    /// Convert to CAN messages (3 messages for 6 joints)
    pub fn to_can_data(&self) -> [Vec<u8>; 3] {
        let j1 = (self.angles[0] * 1000.0) as i32;
        let j2 = (self.angles[1] * 1000.0) as i32;
        let j3 = (self.angles[2] * 1000.0) as i32;
        let j4 = (self.angles[3] * 1000.0) as i32;
        let j5 = (self.angles[4] * 1000.0) as i32;
        let j6 = (self.angles[5] * 1000.0) as i32;
        
        let mut data_12 = Vec::with_capacity(8);
        data_12.extend_from_slice(&j1.to_le_bytes());
        data_12.extend_from_slice(&j2.to_le_bytes());
        
        let mut data_34 = Vec::with_capacity(8);
        data_34.extend_from_slice(&j3.to_le_bytes());
        data_34.extend_from_slice(&j4.to_le_bytes());
        
        let mut data_56 = Vec::with_capacity(8);
        data_56.extend_from_slice(&j5.to_le_bytes());
        data_56.extend_from_slice(&j6.to_le_bytes());
        
        [data_12, data_34, data_56]
    }
}

/// Gripper control command
#[derive(Debug, Clone)]
pub struct GripperControl {
    /// Target gripper position (0-1000)
    pub position: u16,
    /// Gripper speed (0-1000)
    pub speed: u16,
}

impl GripperControl {
    /// Create a new GripperControl
    pub fn new(position: u16, speed: u16) -> Self {
        Self { position, speed }
    }
    
    /// Convert to CAN message data
    pub fn to_can_data(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(4);
        data.extend_from_slice(&self.position.to_le_bytes());
        data.extend_from_slice(&self.speed.to_le_bytes());
        data
    }
}
