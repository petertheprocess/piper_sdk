//! Example: Read joint states from Piper robot arm
//!
//! This example demonstrates how to connect to the robot arm and read joint angles.
//!
//! Usage:
//!   cargo run --example read_joint_state

use piper_sdk_rs::{PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    // Initialize logger
    env_logger::init();
    
    println!("Piper SDK - Read Joint State Example");
    println!("=====================================\n");
    
    // Create interface to CAN bus
    // Note: Make sure can0 is configured and active
    // Run: sudo ip link set can0 up type can bitrate 1000000
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    println!("Reading joint states... (Press Ctrl+C to exit)\n");
    
    // Loop and read joint states
    loop {
        if let Some(joint_state) = piper.get_joint_state()? {
            println!("Joint Angles (rad): [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
                joint_state.angles[0],
                joint_state.angles[1],
                joint_state.angles[2],
                joint_state.angles[3],
                joint_state.angles[4],
                joint_state.angles[5]
            );
            println!("Timestamp: {:.3}\n", joint_state.timestamp);
        } else {
            println!("Waiting for joint state data...");
        }
        
        // Also read gripper state if available
        if let Some(gripper_state) = piper.get_gripper_state()? {
            println!("Gripper Position: {}, Status: 0x{:02X}",
                gripper_state.position,
                gripper_state.status
            );
        }
        
        // Sleep for 200Hz update rate
        thread::sleep(Duration::from_millis(5));
    }
}
