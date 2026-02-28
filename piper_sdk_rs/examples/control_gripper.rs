//! Example: Control gripper
//!
//! This example demonstrates how to control the robot arm gripper.
//!
//! Usage:
//!   cargo run --example control_gripper

use piper_sdk_rs::{GripperControl, PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - Gripper Control Example");
    println!("====================================\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Enable motors first
    println!("Enabling motors...");
    piper.set_motor_enable(true)?;
    thread::sleep(Duration::from_millis(100));
    
    // Open gripper
    println!("Opening gripper...");
    let open_cmd = GripperControl::new(1000, 500); // position: 1000 (open), speed: 500
    piper.send_gripper_control(&open_cmd)?;
    thread::sleep(Duration::from_secs(2));
    
    // Close gripper
    println!("Closing gripper...");
    let close_cmd = GripperControl::new(0, 500); // position: 0 (closed), speed: 500
    piper.send_gripper_control(&close_cmd)?;
    thread::sleep(Duration::from_secs(2));
    
    // Half open
    println!("Setting gripper to half open...");
    let half_cmd = GripperControl::new(500, 500); // position: 500 (half), speed: 500
    piper.send_gripper_control(&half_cmd)?;
    thread::sleep(Duration::from_secs(2));
    
    println!("\nGripper control sequence complete!");
    
    Ok(())
}
