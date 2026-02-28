//! Example: Read motor high-speed and low-speed feedback
//!
//! This example demonstrates how to read detailed motor feedback including
//! speed, current, temperature, and status information.
//!
//! Usage:
//!   cargo run --example read_motor_feedback

use piper_sdk_rs::{PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - Read Motor Feedback Example");
    println!("========================================\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Set to slave mode to receive feedback
    piper.set_master_slave_mode(0xFC, 0, 0, 0)?;
    thread::sleep(Duration::from_millis(100));
    
    println!("Reading motor feedback... (Press Ctrl+C to exit)\n");
    
    loop {
        // Read high-speed feedback for all 6 motors
        println!("=== High-Speed Feedback ===");
        for motor_num in 1..=6 {
            if let Some(feedback) = piper.get_motor_high_speed(motor_num)? {
                println!(
                    "Motor {}: Speed={:.3} rad/s, Current={:.3} A, Pos={:.6} rad, Effort={:.3} N/m",
                    motor_num,
                    feedback.motor_speed as f32 * 0.001,
                    feedback.current as f32 * 0.001,
                    feedback.position as f32 * 0.001,
                    feedback.effort * 0.001
                );
            } else {
                println!("Motor {}: No feedback yet", motor_num);
            }
        }
        
        println!("\n=== Low-Speed Feedback ===");
        for motor_num in 1..=6 {
            if let Some(feedback) = piper.get_motor_low_speed(motor_num)? {
                println!(
                    "Motor {}: Voltage={:.1} V, Driver Temp={}°C, Motor Temp={}°C",
                    motor_num,
                    feedback.voltage as f32 * 0.1,
                    feedback.driver_temp,
                    feedback.motor_temp
                );
                
                // Check for any error conditions
                let mut warnings = Vec::new();
                if feedback.is_voltage_too_low() {
                    warnings.push("LOW VOLTAGE");
                }
                if feedback.is_motor_overheating() {
                    warnings.push("MOTOR OVERHEAT");
                }
                if feedback.is_driver_overcurrent() {
                    warnings.push("OVERCURRENT");
                }
                if feedback.is_driver_overheating() {
                    warnings.push("DRIVER OVERHEAT");
                }
                if feedback.is_collision_triggered() {
                    warnings.push("COLLISION");
                }
                if feedback.is_driver_error() {
                    warnings.push("DRIVER ERROR");
                }
                if feedback.is_stall_triggered() {
                    warnings.push("STALL");
                }
                
                if !warnings.is_empty() {
                    println!("  ⚠️  Warnings: {}", warnings.join(", "));
                }
                
                if feedback.is_driver_enabled() {
                    println!("  ✓ Driver enabled");
                } else {
                    println!("  ✗ Driver disabled");
                }
            } else {
                println!("Motor {}: No feedback yet", motor_num);
            }
        }
        
        println!("\n");
        thread::sleep(Duration::from_millis(500));
    }
}
