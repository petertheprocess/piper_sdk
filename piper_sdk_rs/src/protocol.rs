//! Protocol layer for CAN message parsing and encoding

use crate::can_id::CanId;
use crate::error::Result;
use crate::messages::*;
use std::sync::{Arc, Mutex};

/// Protocol parser for Piper CAN messages
pub struct PiperProtocol {
    /// Buffer for multi-frame messages
    message_buffer: Arc<Mutex<MessageBuffer>>,
}

/// Buffer for storing partial messages
struct MessageBuffer {
    /// Joint feedback data
    joint_data_12: Option<Vec<u8>>,
    joint_data_34: Option<Vec<u8>>,
    joint_data_56: Option<Vec<u8>>,
    
    /// End pose feedback data
    end_pose_data_1: Option<Vec<u8>>,
    end_pose_data_2: Option<Vec<u8>>,
    end_pose_data_3: Option<Vec<u8>>,
    
    /// Latest complete messages
    joint_state: Option<JointState>,
    gripper_state: Option<GripperState>,
    end_pose: Option<EndPose>,
    arm_status: Option<ArmStatus>,
    
    /// Motor high-speed feedback (6 motors)
    motor_high_speed: [Option<MotorHighSpeedFeedback>; 6],
    
    /// Motor low-speed feedback (6 motors)
    motor_low_speed: [Option<MotorLowSpeedFeedback>; 6],
}

impl MessageBuffer {
    fn new() -> Self {
        Self {
            joint_data_12: None,
            joint_data_34: None,
            joint_data_56: None,
            end_pose_data_1: None,
            end_pose_data_2: None,
            end_pose_data_3: None,
            joint_state: None,
            gripper_state: None,
            end_pose: None,
            arm_status: None,
            motor_high_speed: [None, None, None, None, None, None],
            motor_low_speed: [None, None, None, None, None, None],
        }
    }
}

impl PiperProtocol {
    /// Create a new protocol parser
    pub fn new() -> Self {
        Self {
            message_buffer: Arc::new(Mutex::new(MessageBuffer::new())),
        }
    }
    
    /// Process incoming CAN message
    pub fn process_message(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let id = CanId::from_u32(can_id);
        
        let mut buffer = self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer");
        
        match id {
            Some(CanId::ArmJointFeedback12) => {
                buffer.joint_data_12 = Some(data.to_vec());
                self.try_complete_joint_state(&mut buffer)?;
            }
            Some(CanId::ArmJointFeedback34) => {
                buffer.joint_data_34 = Some(data.to_vec());
                self.try_complete_joint_state(&mut buffer)?;
            }
            Some(CanId::ArmJointFeedback56) => {
                buffer.joint_data_56 = Some(data.to_vec());
                self.try_complete_joint_state(&mut buffer)?;
            }
            Some(CanId::ArmGripperFeedback) => {
                buffer.gripper_state = Some(GripperState::from_bytes(data)?);
            }
            Some(CanId::ArmEndPoseFeedback1) => {
                buffer.end_pose_data_1 = Some(data.to_vec());
                self.try_complete_end_pose(&mut buffer)?;
            }
            Some(CanId::ArmEndPoseFeedback2) => {
                buffer.end_pose_data_2 = Some(data.to_vec());
                self.try_complete_end_pose(&mut buffer)?;
            }
            Some(CanId::ArmEndPoseFeedback3) => {
                buffer.end_pose_data_3 = Some(data.to_vec());
                self.try_complete_end_pose(&mut buffer)?;
            }
            Some(CanId::ArmStatusFeedback) => {
                buffer.arm_status = Some(ArmStatus::from_bytes(data)?);
            }
            // High-speed motor feedback (0x251-0x256)
            Some(CanId::ArmInfoHighSpdFeedback1) => {
                buffer.motor_high_speed[0] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoHighSpdFeedback2) => {
                buffer.motor_high_speed[1] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoHighSpdFeedback3) => {
                buffer.motor_high_speed[2] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoHighSpdFeedback4) => {
                buffer.motor_high_speed[3] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoHighSpdFeedback5) => {
                buffer.motor_high_speed[4] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoHighSpdFeedback6) => {
                buffer.motor_high_speed[5] = Some(MotorHighSpeedFeedback::from_bytes(can_id, data)?);
            }
            // Low-speed motor feedback (0x261-0x266)
            Some(CanId::ArmInfoLowSpdFeedback1) => {
                buffer.motor_low_speed[0] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoLowSpdFeedback2) => {
                buffer.motor_low_speed[1] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoLowSpdFeedback3) => {
                buffer.motor_low_speed[2] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoLowSpdFeedback4) => {
                buffer.motor_low_speed[3] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoLowSpdFeedback5) => {
                buffer.motor_low_speed[4] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            Some(CanId::ArmInfoLowSpdFeedback6) => {
                buffer.motor_low_speed[5] = Some(MotorLowSpeedFeedback::from_bytes(can_id, data)?);
            }
            _ => {
                // Unknown or unhandled message ID
                log::debug!("Unhandled CAN ID: 0x{:X}", can_id);
            }
        }
        
        Ok(())
    }
    
    /// Try to complete joint state from buffered data
    fn try_complete_joint_state(&self, buffer: &mut MessageBuffer) -> Result<()> {
        if let (Some(data_12), Some(data_34), Some(data_56)) = (
            &buffer.joint_data_12,
            &buffer.joint_data_34,
            &buffer.joint_data_56,
        ) {
            buffer.joint_state = Some(JointState::from_bytes(data_12, data_34, data_56)?);
        }
        Ok(())
    }
    
    /// Try to complete end pose from buffered data
    fn try_complete_end_pose(&self, buffer: &mut MessageBuffer) -> Result<()> {
        if let (Some(data_1), Some(data_2), Some(data_3)) = (
            &buffer.end_pose_data_1,
            &buffer.end_pose_data_2,
            &buffer.end_pose_data_3,
        ) {
            buffer.end_pose = Some(EndPose::from_bytes(data_1, data_2, data_3)?);
        }
        Ok(())
    }
    
    /// Get the latest joint state
    pub fn get_joint_state(&self) -> Option<JointState> {
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .joint_state.clone()
    }
    
    /// Get the latest gripper state
    pub fn get_gripper_state(&self) -> Option<GripperState> {
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .gripper_state.clone()
    }
    
    /// Get the latest end pose
    pub fn get_end_pose(&self) -> Option<EndPose> {
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .end_pose.clone()
    }
    
    /// Get the latest arm status
    pub fn get_arm_status(&self) -> Option<ArmStatus> {
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .arm_status.clone()
    }

    /// Get JointHighSpeedState for all joints
    pub fn get_joint_high_speed_states(&self) -> Option<JointHighSpeedStates> {
        let mut motor_feedbacks: [MotorHighSpeedFeedback; 6] = Default::default();

        for motor_num in 1..=6 {
            if let Some(motor_feedback) = self.get_motor_high_speed(motor_num) {
                motor_feedbacks[motor_num - 1] = motor_feedback;
            } else {
                log::warn!("Motor high-speed feedback for motor {} is missing", motor_num);
                return None;
            }
        }
        Some(JointHighSpeedStates::from_motor_high_speed_feedbacks(&motor_feedbacks))
    }
    
    /// Get high-speed feedback for a specific motor (1-6)
    pub fn get_motor_high_speed(&self, motor_num: usize) -> Option<MotorHighSpeedFeedback> {
        if motor_num < 1 || motor_num > 6 {
            return None;
        }
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .motor_high_speed[motor_num - 1].clone()
    }
    
    /// Get low-speed feedback for a specific motor (1-6)
    pub fn get_motor_low_speed(&self, motor_num: usize) -> Option<MotorLowSpeedFeedback> {
        if motor_num < 1 || motor_num > 6 {
            return None;
        }
        self.message_buffer.lock()
            .expect("Mutex poisoned - cannot access message buffer")
            .motor_low_speed[motor_num - 1].clone()
    }
}

impl Default for PiperProtocol {
    fn default() -> Self {
        Self::new()
    }
}
