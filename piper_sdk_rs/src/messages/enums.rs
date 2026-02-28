//! Additional enums for arm message fields
//!
//! These enums map numeric protocol values to typed variants for clarity.

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CtrlMode {
    Standby = 0x00,
    CAN = 0x01,
    TeachingMode = 0x02,
    EthernetControlMode = 0x03,
    WifiControlMode = 0x04,
    RemoteControlMode = 0x05,
    LinkageTeachingInputMode = 0x06,
    OfflineTrajectoryMode = 0x07,
    Unknown = 0xFF,
}

impl Default for CtrlMode {
    fn default() -> Self { CtrlMode::Standby }
}

impl From<CtrlMode> for u8 { fn from(v: CtrlMode) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for CtrlMode {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(CtrlMode::Standby),
            0x01 => Ok(CtrlMode::CAN),
            0x02 => Ok(CtrlMode::TeachingMode),
            0x03 => Ok(CtrlMode::EthernetControlMode),
            0x04 => Ok(CtrlMode::WifiControlMode),
            0x05 => Ok(CtrlMode::RemoteControlMode),
            0x06 => Ok(CtrlMode::LinkageTeachingInputMode),
            0x07 => Ok(CtrlMode::OfflineTrajectoryMode),
            0xFF => Ok(CtrlMode::Unknown),
            _ => Err(v),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmStatusCode {
    Normal = 0x00,
    EmergencyStop = 0x01,
    NoSolution = 0x02,
    SingularityPoint = 0x03,
    TargetPosExceedsLimit = 0x04,
    JointCommunicationErr = 0x05,
    JointBrakeNotReleased = 0x06,
    CollisionOccurred = 0x07,
    OverspeedDuringTeachingDrag = 0x08,
    JointStatusErr = 0x09,
    OtherErr = 0x0A,
    TeachingRecord = 0x0B,
    TeachingExecution = 0x0C,
    TeachingPause = 0x0D,
    MainControllerNtcOverTemperature = 0x0E,
    ReleaseResistorNtcOverTemperature = 0x0F,
    Unknown = 0xFF,
}

impl Default for ArmStatusCode { fn default() -> Self { ArmStatusCode::Unknown } }
impl From<ArmStatusCode> for u8 { fn from(v: ArmStatusCode) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for ArmStatusCode {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(ArmStatusCode::Normal),
            0x01 => Ok(ArmStatusCode::EmergencyStop),
            0x02 => Ok(ArmStatusCode::NoSolution),
            0x03 => Ok(ArmStatusCode::SingularityPoint),
            0x04 => Ok(ArmStatusCode::TargetPosExceedsLimit),
            0x05 => Ok(ArmStatusCode::JointCommunicationErr),
            0x06 => Ok(ArmStatusCode::JointBrakeNotReleased),
            0x07 => Ok(ArmStatusCode::CollisionOccurred),
            0x08 => Ok(ArmStatusCode::OverspeedDuringTeachingDrag),
            0x09 => Ok(ArmStatusCode::JointStatusErr),
            0x0A => Ok(ArmStatusCode::OtherErr),
            0x0B => Ok(ArmStatusCode::TeachingRecord),
            0x0C => Ok(ArmStatusCode::TeachingExecution),
            0x0D => Ok(ArmStatusCode::TeachingPause),
            0x0E => Ok(ArmStatusCode::MainControllerNtcOverTemperature),
            0x0F => Ok(ArmStatusCode::ReleaseResistorNtcOverTemperature),
            0xFF => Ok(ArmStatusCode::Unknown),
            _ => Err(v),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveMode {
    P = 0x00,
    J = 0x01,
    L = 0x02,
    C = 0x03,
    M = 0x04,
    CPV = 0x05,
    Unknown = 0xFF,
}

impl Default for MoveMode { fn default() -> Self { MoveMode::P } }
impl From<MoveMode> for u8 { fn from(v: MoveMode) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for MoveMode {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(MoveMode::P),
            0x01 => Ok(MoveMode::J),
            0x02 => Ok(MoveMode::L),
            0x03 => Ok(MoveMode::C),
            0x04 => Ok(MoveMode::M),
            0x05 => Ok(MoveMode::CPV),
            0xFF => Ok(MoveMode::Unknown),
            _ => Err(v),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TeachingState {
    Disabled = 0x00,
    StartRecording = 0x01,
    StopRecording = 0x02,
    ExecuteTrajectory = 0x03,
    PauseExecution = 0x04,
    ResumeExecution = 0x05,
    TerminateExecution = 0x06,
    MoveToStart = 0x07,
    Unknown = 0xFF,
}

impl Default for TeachingState { fn default() -> Self { TeachingState::Disabled } }
impl From<TeachingState> for u8 { fn from(v: TeachingState) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for TeachingState {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(TeachingState::Disabled),
            0x01 => Ok(TeachingState::StartRecording),
            0x02 => Ok(TeachingState::StopRecording),
            0x03 => Ok(TeachingState::ExecuteTrajectory),
            0x04 => Ok(TeachingState::PauseExecution),
            0x05 => Ok(TeachingState::ResumeExecution),
            0x06 => Ok(TeachingState::TerminateExecution),
            0x07 => Ok(TeachingState::MoveToStart),
            0xFF => Ok(TeachingState::Unknown),
            _ => Err(v),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionStatus {
    ReachTargetPosSuccessfully = 0x00,
    ReachTargetPosFailed = 0x01,
    Unknown = 0xFF,
}

impl Default for MotionStatus { fn default() -> Self { MotionStatus::Unknown } }
impl From<MotionStatus> for u8 { fn from(v: MotionStatus) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for MotionStatus {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(MotionStatus::ReachTargetPosSuccessfully),
            0x01 => Ok(MotionStatus::ReachTargetPosFailed),
            0xFF => Ok(MotionStatus::Unknown),
            _ => Err(v),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorEnableStatus {
    /// Motor disabled 0x01
    Disabled = 0x01,
    /// Motor enabled 0x02
    Enabled = 0x02,
}
impl Default for MotorEnableStatus { fn default() -> Self { MotorEnableStatus::Disabled } }
impl From<MotorEnableStatus> for u8 { fn from(v: MotorEnableStatus) -> u8 { v as u8 } }
impl std::convert::TryFrom<u8> for MotorEnableStatus {
    type Error = u8;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x01 => Ok(MotorEnableStatus::Disabled),
            0x02 => Ok(MotorEnableStatus::Enabled),
            _ => Err(v),
        }
    }
}
