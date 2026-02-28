//! Example: Set robot to slave mode
//!
//! This example demonstrates how to configure the robot to slave mode,
//! which is necessary for reading feedback data.
//!
//! Usage:
//!   cargo run --example set_slave_mode

use piper_sdk_rs::{PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Piper SDK - Set Slave Mode Example");
    println!("===================================\n");
    
    let piper = PiperInterface::new("can0")?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());
    
    // Set to slave mode: 0xFC, 0, 0, 0
    println!("Setting robot to slave mode...");
    piper.set_master_slave_mode(0xFC, 0, 0, 0)?;
    println!("Command sent successfully!\n");
    
    // Wait a bit for the command to take effect
    thread::sleep(Duration::from_millis(100));
    
    println!("Robot should now be in slave mode and sending feedback.");
    println!("You can now run other examples to read feedback data.");
    
    Ok(())
}
