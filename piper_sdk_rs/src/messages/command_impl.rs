//! Message sending and encoding implementations
//!
//! This module contains implementations for encoding structured data types into CAN messages

use super::{JointControl, JointMitControl, MotionCtrl2, EndPoseControl, GripperControl, CtrlMode, MoveMode, MitMode};

impl JointControl {
    /// Create a new JointControl
    pub fn new(angles: [f32; 6]) -> Self {
        Self { angles }
    }
    
    /// Convert to bytes representing CAN messages (3 messages for 6 joints)
    /// Returns three fixed-size 8-byte frames
    pub fn to_bytes(&self) -> [[u8; 8]; 3] {
        let j1 = (self.angles[0] * 1000.0) as i32;
        let j2 = (self.angles[1] * 1000.0) as i32;
        let j3 = (self.angles[2] * 1000.0) as i32;
        let j4 = (self.angles[3] * 1000.0) as i32;
        let j5 = (self.angles[4] * 1000.0) as i32;
        let j6 = (self.angles[5] * 1000.0) as i32;
        
        let mut data_12 = [0u8; 8];
        data_12[0..4].copy_from_slice(&j1.to_be_bytes());
        data_12[4..8].copy_from_slice(&j2.to_be_bytes());
        
        let mut data_34 = [0u8; 8];
        data_34[0..4].copy_from_slice(&j3.to_be_bytes());
        data_34[4..8].copy_from_slice(&j4.to_be_bytes());
        
        let mut data_56 = [0u8; 8];
        data_56[0..4].copy_from_slice(&j5.to_be_bytes());
        data_56[4..8].copy_from_slice(&j6.to_be_bytes());
        
        [data_12, data_34, data_56]
    }
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
    /// * `t_ref` - Target torque in Nm (-8.0 to 8.0)
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
    
    /// Convert to bytes for CAN message data with proper encoding
    /// Returns an 8-byte frame encoded per protocol
    pub fn to_bytes(&self) -> [u8; 8] {
        // Encode parameters with proper bit packing as per protocol
        let pos_tmp = float_to_uint(self.pos_ref, -12.5, 12.5, 16);
        let vel_tmp = float_to_uint(self.vel_ref, -45.0, 45.0, 12);
        let kp_tmp = float_to_uint(self.kp, 0.0, 500.0, 12);
        let kd_tmp = float_to_uint(self.kd, -5.0, 5.0, 12);
        let t_tmp = float_to_uint(self.t_ref, -8.0, 8.0, 8);
        
        let mut data = [0u8; 8];
        
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
        data[7] = (((t_tmp & 0x0F) << 4) as u8) | crc;
        
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

impl MotionCtrl2 {
    /// Create a new motion control command
    ///
    /// # Arguments
    /// * `ctrl_mode` - Control mode (use `CtrlMode` enum)
    /// * `move_mode` - Move mode (use `MoveMode` enum)
    /// * `move_spd_rate` - Speed percentage (0-100)
    /// * `is_mit_mode` - MIT mode (use `MitMode` enum: `PosVel`=0x00, `MIT`=0xAD, `Invalid`=0xFF)
    pub fn new(ctrl_mode: CtrlMode, move_mode: MoveMode, move_spd_rate: u8, is_mit_mode: MitMode) -> Self {
        Self {
            ctrl_mode,
            move_mode,
            move_spd_rate,
            is_mit_mode,
            residence_time: 0,
            installation_pos: 0,
        }
    }
    
    /// Convert to bytes for CAN message data (8 bytes)
    pub fn to_bytes(&self) -> [u8; 8] {
        [
            self.ctrl_mode as u8,
            self.move_mode as u8,
            self.move_spd_rate,
            self.is_mit_mode as u8,
            self.residence_time,
            self.installation_pos,
            0,
            0,
        ]
    }
}

impl EndPoseControl {
    /// Create a new end pose control command
    pub fn new(x: i32, y: i32, z: i32, rx: i32, ry: i32, rz: i32) -> Self {
        Self {
            position: [x, y, z],
            orientation: [rx, ry, rz],
        }
    }
    
    /// Convert to bytes representing CAN messages (3 messages for 6 DOF)
    /// Returns three fixed-size 8-byte frames
    pub fn to_bytes(&self) -> [[u8; 8]; 3] {
        let mut data_1 = [0u8; 8];
        data_1[0..4].copy_from_slice(&self.position[0].to_be_bytes()); // X
        data_1[4..8].copy_from_slice(&self.position[1].to_be_bytes()); // Y
        
        let mut data_2 = [0u8; 8];
        data_2[0..4].copy_from_slice(&self.position[2].to_be_bytes()); // Z
        data_2[4..8].copy_from_slice(&self.orientation[0].to_be_bytes()); // RX
        
        let mut data_3 = [0u8; 8];
        data_3[0..4].copy_from_slice(&self.orientation[1].to_be_bytes()); // RY
        data_3[4..8].copy_from_slice(&self.orientation[2].to_be_bytes()); // RZ
        
        [data_1, data_2, data_3]
    }
}

impl GripperControl {
    /// Create a new GripperControl
    pub fn new(position: u16, speed: u16) -> Self {
        Self { position, speed }
    }
    
    /// Convert to bytes for CAN message data (4 bytes)
    pub fn to_bytes(&self) -> [u8; 4] {
        let mut data = [0u8; 4];
        data[0..2].copy_from_slice(&self.position.to_be_bytes());
        data[2..4].copy_from_slice(&self.speed.to_be_bytes());
        data
    }
}
