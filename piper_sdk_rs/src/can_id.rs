//! CAN ID definitions for Piper robot arm
//!
//! This module defines all CAN message IDs used for communication with the Piper robot arm.

/// CAN IDs for Piper robot arm communication
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum CanId {
    // Active feedback instructions
    /// Arm status feedback
    ArmStatusFeedback = 0x2A1,
    /// Arm end pose feedback 1
    ArmEndPoseFeedback1 = 0x2A2,
    /// Arm end pose feedback 2
    ArmEndPoseFeedback2 = 0x2A3,
    /// Arm end pose feedback 3
    ArmEndPoseFeedback3 = 0x2A4,
    /// Arm joint feedback 1-2
    ArmJointFeedback12 = 0x2A5,
    /// Arm joint feedback 3-4
    ArmJointFeedback34 = 0x2A6,
    /// Arm joint feedback 5-6
    ArmJointFeedback56 = 0x2A7,
    /// Arm gripper feedback
    ArmGripperFeedback = 0x2A8,
    
    // Motion control instructions
    /// Motion control 1 (stop, track, drag teach)
    ArmMotionCtrl1 = 0x150,
    /// Motion control 2 (mode control, speed rate)
    ArmMotionCtrl2 = 0x151,
    /// Cartesian motion control 1 (X & Y)
    ArmMotionCtrlCartesian1 = 0x152,
    /// Cartesian motion control 2 (Z & RX)
    ArmMotionCtrlCartesian2 = 0x153,
    /// Cartesian motion control 3 (RY & RZ)
    ArmMotionCtrlCartesian3 = 0x154,
    /// Joint control 1-2 (J1 & J2)
    ArmJointCtrl12 = 0x155,
    /// Joint control 3-4 (J3 & J4)
    ArmJointCtrl34 = 0x156,
    /// Joint control 5-6 (J5 & J6)
    ArmJointCtrl56 = 0x157,
    /// Circular pattern coordinate update
    ArmCircularPatternCoordUpdate = 0x158,
    /// Gripper control
    ArmGripperCtrl = 0x159,
    
    // MIT mode control (V1.5-2+)
    /// Joint MIT control 1
    ArmJointMitCtrl1 = 0x15A,
    /// Joint MIT control 2
    ArmJointMitCtrl2 = 0x15B,
    /// Joint MIT control 3
    ArmJointMitCtrl3 = 0x15C,
    /// Joint MIT control 4
    ArmJointMitCtrl4 = 0x15D,
    /// Joint MIT control 5
    ArmJointMitCtrl5 = 0x15E,
    /// Joint MIT control 6
    ArmJointMitCtrl6 = 0x15F,
    
    // Parameter configuration instructions
    /// Master-slave mode configuration
    ArmMasterSlaveModeConfig = 0x470,
    /// Motor enable/disable
    ArmMotorEnableDisable = 0x471,
    /// Search motor max speed/acceleration limit
    ArmSearchMotorMaxSpdAccLimit = 0x472,
    /// Feedback current motor angle limit and max speed
    ArmFeedbackCurrentMotorAngleLimitMaxSpd = 0x473,
    /// Motor angle limit and max speed set
    ArmMotorAngleLimitMaxSpdSet = 0x474,
    /// Joint configuration
    ArmJointConfig = 0x475,
    /// Instruction response configuration
    ArmInstructionResponseConfig = 0x476,
    /// Parameter enquiry and configuration
    ArmParamEnquiryAndConfig = 0x477,
    /// Feedback current end velocity/acceleration parameters
    ArmFeedbackCurrentEndVelAccParam = 0x478,
    /// End velocity/acceleration parameter configuration
    ArmEndVelAccParamConfig = 0x479,
    /// Crash protection rating configuration
    ArmCrashProtectionRatingConfig = 0x47A,
    /// Crash protection rating feedback
    ArmCrashProtectionRatingFeedback = 0x47B,
    /// Feedback current motor max acceleration limit
    ArmFeedbackCurrentMotorMaxAccLimit = 0x47C,
    /// Gripper teaching pendant parameter configuration (V1.5-2+)
    ArmGripperTeachingPendantParamConfig = 0x47D,
    /// Gripper teaching pendant parameter feedback (V1.5-2+)
    ArmGripperTeachingPendantParamFeedback = 0x47E,
    
    // Joint velocity/acceleration feedback
    /// Joint 1 velocity/acceleration feedback
    ArmFeedbackJointVelAcc1 = 0x481,
    /// Joint 2 velocity/acceleration feedback
    ArmFeedbackJointVelAcc2 = 0x482,
    /// Joint 3 velocity/acceleration feedback
    ArmFeedbackJointVelAcc3 = 0x483,
    /// Joint 4 velocity/acceleration feedback
    ArmFeedbackJointVelAcc4 = 0x484,
    /// Joint 5 velocity/acceleration feedback
    ArmFeedbackJointVelAcc5 = 0x485,
    /// Joint 6 velocity/acceleration feedback
    ArmFeedbackJointVelAcc6 = 0x486,
    
    // Light control
    /// Light control
    ArmLightCtrl = 0x121,
    
    // High-speed driver feedback
    /// High-speed feedback motor 1
    ArmInfoHighSpdFeedback1 = 0x251,
    /// High-speed feedback motor 2
    ArmInfoHighSpdFeedback2 = 0x252,
    /// High-speed feedback motor 3
    ArmInfoHighSpdFeedback3 = 0x253,
    /// High-speed feedback motor 4
    ArmInfoHighSpdFeedback4 = 0x254,
    /// High-speed feedback motor 5
    ArmInfoHighSpdFeedback5 = 0x255,
    /// High-speed feedback motor 6
    ArmInfoHighSpdFeedback6 = 0x256,
    
    // Low-speed driver feedback
    /// Low-speed feedback motor 1
    ArmInfoLowSpdFeedback1 = 0x261,
    /// Low-speed feedback motor 2
    ArmInfoLowSpdFeedback2 = 0x262,
    /// Low-speed feedback motor 3
    ArmInfoLowSpdFeedback3 = 0x263,
    /// Low-speed feedback motor 4
    ArmInfoLowSpdFeedback4 = 0x264,
    /// Low-speed feedback motor 5
    ArmInfoLowSpdFeedback5 = 0x265,
    /// Low-speed feedback motor 6
    ArmInfoLowSpdFeedback6 = 0x266,
    
    // CAN update mode
    /// CAN update silent mode configuration
    ArmCanUpdateSilentModeConfig = 0x422,
    
    // Firmware read
    /// Firmware version read
    ArmFirmwareRead = 0x4AF,
}

impl CanId {
    /// Convert CAN ID to u32
    pub fn as_u32(self) -> u32 {
        self as u32
    }
    
    /// Try to convert u32 to CanId
    pub fn from_u32(id: u32) -> Option<Self> {
        match id {
            0x2A1 => Some(CanId::ArmStatusFeedback),
            0x2A2 => Some(CanId::ArmEndPoseFeedback1),
            0x2A3 => Some(CanId::ArmEndPoseFeedback2),
            0x2A4 => Some(CanId::ArmEndPoseFeedback3),
            0x2A5 => Some(CanId::ArmJointFeedback12),
            0x2A6 => Some(CanId::ArmJointFeedback34),
            0x2A7 => Some(CanId::ArmJointFeedback56),
            0x2A8 => Some(CanId::ArmGripperFeedback),
            0x150 => Some(CanId::ArmMotionCtrl1),
            0x151 => Some(CanId::ArmMotionCtrl2),
            0x152 => Some(CanId::ArmMotionCtrlCartesian1),
            0x153 => Some(CanId::ArmMotionCtrlCartesian2),
            0x154 => Some(CanId::ArmMotionCtrlCartesian3),
            0x155 => Some(CanId::ArmJointCtrl12),
            0x156 => Some(CanId::ArmJointCtrl34),
            0x157 => Some(CanId::ArmJointCtrl56),
            0x158 => Some(CanId::ArmCircularPatternCoordUpdate),
            0x159 => Some(CanId::ArmGripperCtrl),
            0x15A => Some(CanId::ArmJointMitCtrl1),
            0x15B => Some(CanId::ArmJointMitCtrl2),
            0x15C => Some(CanId::ArmJointMitCtrl3),
            0x15D => Some(CanId::ArmJointMitCtrl4),
            0x15E => Some(CanId::ArmJointMitCtrl5),
            0x15F => Some(CanId::ArmJointMitCtrl6),
            0x470 => Some(CanId::ArmMasterSlaveModeConfig),
            0x471 => Some(CanId::ArmMotorEnableDisable),
            0x472 => Some(CanId::ArmSearchMotorMaxSpdAccLimit),
            0x473 => Some(CanId::ArmFeedbackCurrentMotorAngleLimitMaxSpd),
            0x474 => Some(CanId::ArmMotorAngleLimitMaxSpdSet),
            0x475 => Some(CanId::ArmJointConfig),
            0x476 => Some(CanId::ArmInstructionResponseConfig),
            0x477 => Some(CanId::ArmParamEnquiryAndConfig),
            0x478 => Some(CanId::ArmFeedbackCurrentEndVelAccParam),
            0x479 => Some(CanId::ArmEndVelAccParamConfig),
            0x47A => Some(CanId::ArmCrashProtectionRatingConfig),
            0x47B => Some(CanId::ArmCrashProtectionRatingFeedback),
            0x47C => Some(CanId::ArmFeedbackCurrentMotorMaxAccLimit),
            0x47D => Some(CanId::ArmGripperTeachingPendantParamConfig),
            0x47E => Some(CanId::ArmGripperTeachingPendantParamFeedback),
            0x481 => Some(CanId::ArmFeedbackJointVelAcc1),
            0x482 => Some(CanId::ArmFeedbackJointVelAcc2),
            0x483 => Some(CanId::ArmFeedbackJointVelAcc3),
            0x484 => Some(CanId::ArmFeedbackJointVelAcc4),
            0x485 => Some(CanId::ArmFeedbackJointVelAcc5),
            0x486 => Some(CanId::ArmFeedbackJointVelAcc6),
            0x121 => Some(CanId::ArmLightCtrl),
            0x251 => Some(CanId::ArmInfoHighSpdFeedback1),
            0x252 => Some(CanId::ArmInfoHighSpdFeedback2),
            0x253 => Some(CanId::ArmInfoHighSpdFeedback3),
            0x254 => Some(CanId::ArmInfoHighSpdFeedback4),
            0x255 => Some(CanId::ArmInfoHighSpdFeedback5),
            0x256 => Some(CanId::ArmInfoHighSpdFeedback6),
            0x261 => Some(CanId::ArmInfoLowSpdFeedback1),
            0x262 => Some(CanId::ArmInfoLowSpdFeedback2),
            0x263 => Some(CanId::ArmInfoLowSpdFeedback3),
            0x264 => Some(CanId::ArmInfoLowSpdFeedback4),
            0x265 => Some(CanId::ArmInfoLowSpdFeedback5),
            0x266 => Some(CanId::ArmInfoLowSpdFeedback6),
            0x422 => Some(CanId::ArmCanUpdateSilentModeConfig),
            0x4AF => Some(CanId::ArmFirmwareRead),
            _ => None,
        }
    }
}
