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
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - MIT Control Mode Example");
    println!("=====================================");
    println!("WARNING: MIT mode is advanced. Use with caution!\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Setup signal handler for graceful shutdown (handles both Ctrl+C and Ctrl+D)
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    
    ctrlc::set_handler(move || {
        println!("\n\n🛑 Shutdown signal received, exiting gracefully...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting signal handler");
    
    // Enable motors
    piper.emergency_stop()?;
    println!("Enabling motors...");
    // disable and re-enable to clear any faults
    piper.set_motor_enable(false)?;
    thread::sleep(Duration::from_millis(100));
    piper.set_motor_enable(true)?;
    thread::sleep(Duration::from_millis(100));
    
    // Check and loop enable until all drivers are enabled
    println!("Checking driver status...");
    let mut all_enabled = false;
    while !all_enabled {
        thread::sleep(Duration::from_millis(500));
        
        all_enabled = true;
        for motor_num in 1..=6 {
            if let Some(feedback) = piper.get_motor_low_speed(motor_num)? {
                if !feedback.is_driver_enabled() {
                    println!("Motor {} driver is disabled, enabling...", motor_num);
                    piper.set_motor_enable(true)?;
                    all_enabled = false;
                    thread::sleep(Duration::from_millis(200));
                    break;
                }
            } else {
                // No feedback yet, wait and retry
                all_enabled = false;
                break;
            }
        }
    }
    println!("✓ All motor drivers enabled!\n");
    
    // Enable MIT mode
    println!("Enabling MIT mode...");
    piper.enable_mit_mode(true)?;
    thread::sleep(Duration::from_millis(100));
    println!("MIT mode enabled.\n");
    
    let mit_ctrl = JointMitControl::new(
        6,      // motor_num: Motor 6
        0.0,    // pos_ref: No position command
        0.0,    // vel_ref: No velocity command
        0.0,    // kp: No position control
        0.0,    // kd: Light damping
        0.4,    // t_ref: Target torque 0.4 Nm
    );

    // keep sending torque command for 5 seconds
    println!("Sending MIT torque command to Motor 6 (0.4 Nm) for 5 seconds..., in 100Hz");
    println!("Press Ctrl+C to stop gracefully.\n");

    let mut count = 0;
    while count < 500 && running.load(Ordering::SeqCst) {
        piper.enable_mit_mode(true)?;
        piper.send_joint_mit_control(&mit_ctrl)?;
        thread::sleep(Duration::from_millis(10));
        count += 1;
    }

    // Graceful shutdown: Disable MIT mode
    println!("\nDisabling MIT mode...");
    piper.enable_mit_mode(false)?;
    thread::sleep(Duration::from_millis(100));
    
    if !running.load(Ordering::SeqCst) {
        println!("\n✓ MIT control stopped gracefully by user interrupt.");
    } else {
        println!("\n✓ MIT control sequence complete!");
    }
    println!("\nNOTE: Always test MIT parameters carefully in a safe environment.");
    
    Ok(())
}
