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
                .expect("System time is before UNIX epoch")
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
                .expect("System time is before UNIX epoch")
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
                .expect("System time is before UNIX epoch")
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
                .expect("System time is before UNIX epoch")
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

/// MIT control parameters for a single joint
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

impl JointMitControl {
    /// Create a new MIT control command
    ///
    /// # Arguments
    /// * `motor_num` - Motor index (1-6)
    /// * `pos_ref` - Target position in radians (-12.5 to 12.5)
    /// * `vel_ref` - Target velocity in rad/s (-45.0 to 45.0)
    /// * `kp` - Proportional gain (0.0 to 500.0, typical: 10.0)
    /// * `kd` - Derivative gain (-5.0 to 5.0, typical: 0.8)
    /// * `t_ref` - Target torque in Nm (-18.0 to 18.0)
    pub fn new(motor_num: u8, pos_ref: f32, vel_ref: f32, kp: f32, kd: f32, t_ref: f32) -> Self {
        Self {
            motor_num,
            pos_ref,
            vel_ref,
            kp,
            kd,
            t_ref,
        }
    }
    
    /// Convert to CAN message data with proper encoding
    pub fn to_can_data(&self) -> Vec<u8> {
        // Encode parameters with proper bit packing as per protocol
        let pos_tmp = float_to_uint(self.pos_ref, -12.5, 12.5, 16);
        let vel_tmp = float_to_uint(self.vel_ref, -45.0, 45.0, 12);
        let kp_tmp = float_to_uint(self.kp, 0.0, 500.0, 12);
        let kd_tmp = float_to_uint(self.kd, -5.0, 5.0, 12);
        let t_tmp = float_to_uint(self.t_ref, -18.0, 18.0, 8);
        
        let mut data = vec![0u8; 8];
        
        // Byte 0-1: pos_ref (16 bits)
        data[0] = ((pos_tmp >> 8) & 0xFF) as u8;
        data[1] = (pos_tmp & 0xFF) as u8;
        
        // Byte 2: vel_ref[11:4]
        data[2] = ((vel_tmp >> 4) & 0xFF) as u8;
        
        // Byte 3: vel_ref[3:0] | kp[11:8]
        data[3] = (((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F)) as u8;
        
        // Byte 4: kp[7:0]
        data[4] = (kp_tmp & 0xFF) as u8;
        
        // Byte 5: kd[11:4]
        data[5] = ((kd_tmp >> 4) & 0xFF) as u8;
        
        // Byte 6: kd[3:0] | t_ref[7:4]
        data[6] = (((kd_tmp & 0x0F) << 4) | ((t_tmp >> 4) & 0x0F)) as u8;
        
        // Byte 7: t_ref[3:0] | CRC[3:0]
        // CRC is XOR of bytes 0-6, masked to 4 bits
        let crc = (data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]) & 0x0F;
        data[7] = ((((t_tmp & 0x0F) as u8) << 4) | crc) as u8;
        
        data
    }
}

/// Helper function to convert float to uint with range mapping
fn float_to_uint(value: f32, min: f32, max: f32, bits: u32) -> u16 {
    let value = value.clamp(min, max);
    let normalized = (value - min) / (max - min);
    let max_val = (1u32 << bits) - 1;
    (normalized * max_val as f32) as u16
}

/// Motion control mode settings
#[derive(Debug, Clone)]
pub struct MotionCtrl2 {
    /// Control mode
    pub ctrl_mode: u8,
    /// Move mode
    pub move_mode: u8,
    /// Movement speed percentage (0-100)
    pub move_spd_rate: u8,
    /// MIT mode enable
    pub is_mit_mode: u8,
    /// Residence time for offline trajectory
    pub residence_time: u8,
    /// Installation position
    pub installation_pos: u8,
}

impl MotionCtrl2 {
    /// Create a new motion control command
    ///
    /// # Arguments
    /// * `ctrl_mode` - Control mode (0x00=standby, 0x01=CAN, 0x03=Ethernet, 0x04=WiFi, 0x07=offline)
    /// * `move_mode` - Move mode (0x00=P, 0x01=J, 0x02=L, 0x03=C, 0x04=M/MIT)
    /// * `move_spd_rate` - Speed percentage (0-100)
    /// * `is_mit_mode` - MIT mode (0x00=pos/vel, 0xAD=MIT, 0xFF=invalid)
    pub fn new(ctrl_mode: u8, move_mode: u8, move_spd_rate: u8, is_mit_mode: u8) -> Self {
        Self {
            ctrl_mode,
            move_mode,
            move_spd_rate,
            is_mit_mode,
            residence_time: 0,
            installation_pos: 0,
        }
    }
    
    /// Convert to CAN message data
    pub fn to_can_data(&self) -> Vec<u8> {
        vec![
            self.ctrl_mode,
            self.move_mode,
            self.move_spd_rate,
            self.is_mit_mode,
            self.residence_time,
            self.installation_pos,
            0,
            0,
        ]
    }
}

/// End effector pose control (Cartesian coordinates)
#[derive(Debug, Clone)]
pub struct EndPoseControl {
    /// Position [X, Y, Z] in millimeters
    pub position: [i32; 3],
    /// Orientation [RX, RY, RZ] in milliradians
    pub orientation: [i32; 3],
}

impl EndPoseControl {
    /// Create a new end pose control command
    pub fn new(x: i32, y: i32, z: i32, rx: i32, ry: i32, rz: i32) -> Self {
        Self {
            position: [x, y, z],
            orientation: [rx, ry, rz],
        }
    }
    
    /// Convert to CAN messages (3 messages for 6 DOF)
    pub fn to_can_data(&self) -> [Vec<u8>; 3] {
        let mut data_1 = Vec::with_capacity(8);
        data_1.extend_from_slice(&self.position[0].to_le_bytes()); // X
        data_1.extend_from_slice(&self.position[1].to_le_bytes()); // Y
        
        let mut data_2 = Vec::with_capacity(8);
        data_2.extend_from_slice(&self.position[2].to_le_bytes()); // Z
        data_2.extend_from_slice(&self.orientation[0].to_le_bytes()); // RX
        
        let mut data_3 = Vec::with_capacity(8);
        data_3.extend_from_slice(&self.orientation[1].to_le_bytes()); // RY
        data_3.extend_from_slice(&self.orientation[2].to_le_bytes()); // RZ
        
        [data_1, data_2, data_3]
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
