//! Piper Robot Arm SDK for Rust
//!
//! This SDK provides a Rust interface for controlling Piper robot arms via CAN bus
//! using the socketcan crate.
//!
//! # Example
//!
//! ```no_run
//! use piper_sdk_rs::{PiperInterface, Result};
//!
//! fn main() -> Result<()> {
//!     let piper = PiperInterface::new("can0")?;
//!     
//!     // Read joint states
//!     if let Some(joint_state) = piper.get_joint_state()? {
//!         println!("Joint angles: {:?}", joint_state.angles);
//!     }
//!     
//!     Ok(())
//! }
//! ```

pub mod can_id;
pub mod error;
pub mod interface;
pub mod messages;
pub mod protocol;

pub use error::{Error, Result};
pub use interface::PiperInterface;
pub use messages::{
    ArmStatus, EndPose, EndPoseControl, GripperControl, GripperState, 
    JointControl, JointMitControl, JointState, MotionCtrl2, CtrlMode, MoveMode, MitMode,
    MotorHighSpeedFeedback, MotorLowSpeedFeedback
};
