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
                buffer.gripper_state = Some(GripperState::from_can_data(data)?);
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
                buffer.arm_status = Some(ArmStatus::from_can_data(data)?);
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
            buffer.joint_state = Some(JointState::from_can_data(data_12, data_34, data_56)?);
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
            buffer.end_pose = Some(EndPose::from_can_data(data_1, data_2, data_3)?);
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
}

impl Default for PiperProtocol {
    fn default() -> Self {
        Self::new()
    }
}
