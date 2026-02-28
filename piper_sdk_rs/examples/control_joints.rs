//! Example: Control robot arm joints
//!
//! This example demonstrates how to send joint control commands to move the robot.
//!
//! Usage:
//!   cargo run --example control_joints

use piper_sdk_rs::{JointControl, PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - Joint Control Example");
    println!("==================================\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Enable motors first
    println!("Enabling motors...");
    piper.set_motor_enable(true)?;
    thread::sleep(Duration::from_millis(100));
    println!("Motors enabled.\n");
    
    // Move to home position (all zeros)
    println!("Moving to home position...");
    let home_position = JointControl::new([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    piper.send_joint_control(&home_position)?;
    thread::sleep(Duration::from_secs(2));
    
    // Move to a sample position
    println!("Moving to sample position...");
    let sample_position = JointControl::new([0.5, -0.3, 0.2, 0.0, 0.4, 0.0]);
    piper.send_joint_control(&sample_position)?;
    thread::sleep(Duration::from_secs(2));
    
    // Return to home
    println!("Returning to home position...");
    piper.send_joint_control(&home_position)?;
    thread::sleep(Duration::from_secs(2));
    
    println!("\nControl sequence complete!");
    
    Ok(())
}
