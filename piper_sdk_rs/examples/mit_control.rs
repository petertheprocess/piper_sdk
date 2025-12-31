//! Example: MIT control mode
//!
//! This example demonstrates how to use MIT control mode for advanced
//! torque control of individual joints.
//!
//! WARNING: MIT mode is an advanced feature. Incorrect use can damage the robot!
//!
//! Usage:
//!   cargo run --example mit_control

use piper_sdk_rs::{JointMitControl, PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - MIT Control Mode Example");
    println!("=====================================");
    println!("WARNING: MIT mode is advanced. Use with caution!\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Enable motors
    println!("Enabling motors...");
    piper.set_motor_enable(true)?;
    thread::sleep(Duration::from_millis(100));
    
    // Enable MIT mode
    println!("Enabling MIT mode...");
    piper.enable_mit_mode(true)?;
    thread::sleep(Duration::from_millis(100));
    println!("MIT mode enabled.\n");
    
    // Example 1: Position control with MIT mode
    println!("Example 1: Position control for motor 1");
    let mit_ctrl = JointMitControl::new(
        1,      // motor_num: Motor 1
        0.5,    // pos_ref: Target position 0.5 rad
        0.0,    // vel_ref: No velocity command
        10.0,   // kp: Proportional gain (typical value)
        0.8,    // kd: Derivative gain (typical value)
        0.0,    // t_ref: No additional torque
    );
    piper.send_joint_mit_control(&mit_ctrl)?;
    thread::sleep(Duration::from_secs(2));
    
    // Example 2: Velocity control
    println!("Example 2: Velocity control for motor 2");
    let mit_ctrl = JointMitControl::new(
        2,      // motor_num: Motor 2
        0.0,    // pos_ref: No position command
        2.0,    // vel_ref: Target velocity 2 rad/s
        0.0,    // kp: No position control
        1.0,    // kd: Derivative gain for damping
        0.0,    // t_ref: No additional torque
    );
    piper.send_joint_mit_control(&mit_ctrl)?;
    thread::sleep(Duration::from_secs(2));
    
    // Example 3: Torque control (compliance/force control)
    println!("Example 3: Torque control for motor 3");
    let mit_ctrl = JointMitControl::new(
        3,      // motor_num: Motor 3
        0.0,    // pos_ref: No position command
        0.0,    // vel_ref: No velocity command
        0.0,    // kp: No position control
        0.5,    // kd: Light damping
        2.0,    // t_ref: Target torque 2 Nm
    );
    piper.send_joint_mit_control(&mit_ctrl)?;
    thread::sleep(Duration::from_secs(2));
    
    // Example 4: Impedance control (position + compliance)
    println!("Example 4: Impedance control for motor 4");
    let mit_ctrl = JointMitControl::new(
        4,      // motor_num: Motor 4
        0.3,    // pos_ref: Target position 0.3 rad
        0.0,    // vel_ref: No velocity command
        5.0,    // kp: Lower stiffness for compliance
        0.5,    // kd: Light damping
        0.0,    // t_ref: No additional torque
    );
    piper.send_joint_mit_control(&mit_ctrl)?;
    thread::sleep(Duration::from_secs(2));
    
    // Return to home with MIT control
    println!("Returning all motors to home position...");
    for motor_num in 1..=6 {
        let mit_ctrl = JointMitControl::new(
            motor_num,
            0.0,    // Home position
            0.0,
            10.0,   // Standard gains
            0.8,
            0.0,
        );
        piper.send_joint_mit_control(&mit_ctrl)?;
    }
    thread::sleep(Duration::from_secs(3));
    
    // Disable MIT mode
    println!("Disabling MIT mode...");
    piper.enable_mit_mode(false)?;
    thread::sleep(Duration::from_millis(100));
    
    println!("\nMIT control sequence complete!");
    println!("\nNOTE: Always test MIT parameters carefully in a safe environment.");
    
    Ok(())
}
