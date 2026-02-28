//! Example: End effector pose control (Cartesian coordinates)
//!
//! This example demonstrates how to control the robot arm using
//! Cartesian coordinates (X, Y, Z, RX, RY, RZ).
//!
//! Usage:
//!   cargo run --example end_pose_control

use piper_sdk_rs::{EndPoseControl, PiperInterface, Result, CtrlMode, MoveMode};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - End Pose Control Example");
    println!("=====================================\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Enable motors
    println!("Enabling motors...");
    piper.set_motor_enable(true)?;
    thread::sleep(Duration::from_millis(100));
    
    // Set to position control mode (MOVE P)
    println!("Setting mode to Cartesian position control...");
    piper.set_mode(CtrlMode::CAN, MoveMode::P, 50)?; // CAN mode, MOVE P, 50% speed
    thread::sleep(Duration::from_millis(100));
    
    // Move to home position
    println!("Moving to home position...");
    let home = EndPoseControl::new(
        300,    // X: 300 mm
        0,      // Y: 0 mm
        200,    // Z: 200 mm
        0,      // RX: 0 mrad
        0,      // RY: 0 mrad
        0,      // RZ: 0 mrad
    );
    piper.send_end_pose_control(&home)?;
    thread::sleep(Duration::from_secs(3));
    
    // Move forward (increase Y)
    println!("Moving forward...");
    let forward = EndPoseControl::new(300, 100, 200, 0, 0, 0);
    piper.send_end_pose_control(&forward)?;
    thread::sleep(Duration::from_secs(3));
    
    // Move up (increase Z)
    println!("Moving up...");
    let up = EndPoseControl::new(300, 100, 300, 0, 0, 0);
    piper.send_end_pose_control(&up)?;
    thread::sleep(Duration::from_secs(3));
    
    // Rotate end effector (change RZ)
    println!("Rotating end effector...");
    let rotated = EndPoseControl::new(300, 100, 300, 0, 0, 1000); // 1000 mrad = ~57 degrees
    piper.send_end_pose_control(&rotated)?;
    thread::sleep(Duration::from_secs(3));
    
    // Return to home
    println!("Returning to home...");
    piper.send_end_pose_control(&home)?;
    thread::sleep(Duration::from_secs(3));
    
    println!("\nEnd pose control sequence complete!");
    
    Ok(())
}
