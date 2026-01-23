//! Example: Impedance Control with Gravity Compensation using MuJoCo
//!
//! This example demonstrates how to use impedance control combined with gravity
//! compensation torques for the robot arm. The impedance control allows the arm
//! to be compliant while gravity compensation prevents it from falling.
//!
//! The example uses MuJoCo physics engine to:
//! 1. Load the robot mjcf model
//! 2. Calculate gravity-induced torques at each joint using inverse dynamics
//! 3. Implement impedance control with gravity compensation via MIT mode
//!
//! Impedance control combines position tracking with compliance:
//! - kp: Position stiffness (spring constant)
//! - kd: Damping coefficient (viscous damping)
//! - t_ff: Feedforward torque (gravity compensation)
//!
//! Usage:
//!   cargo run --example gravity_compensation -- [can_interface] [xml_path]
//!
//! Prerequisites:
//!   - MJCF model file (e.g., piper_no_gripper_description.xml)
//!   - CAN interface configured and robot powered on
//!   - Set environment variable: export MUJOCO_DOWNLOAD_DIR=/path/to/download/dir

use piper_sdk_rs::{JointMitControl, PiperInterface, Result};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::{array, thread};
use std::time::Duration;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use mujoco_rs::prelude::*;

// Motor torque reduction factors (empirically determined for safety)
pub const MOTOR_REDUCE_FACTORS: [f32; 6] = [0.25, 0.25, 0.25, 1.25, 1.25, 1.25];

/// Graceful shutdown with damping control
/// 
/// Parameters:
/// - piper: PiperInterface reference
/// - shutdown_duration: Duration to apply damping (in seconds)
/// - damping: Array of damping values for 6 joints [kd1, kd2, kd3, kd4, kd5, kd6]
fn graceful_shutdown(piper: &PiperInterface, shutdown_duration: u64, damping: [f32; 6]) -> Result<()> {
    println!("\n\nApplying damping control for safe shutdown... waiting {} seconds.", shutdown_duration);
    let shutdown_start = std::time::Instant::now();    
    piper.enable_mit_mode(true)?;
    
    while shutdown_start.elapsed() < Duration::from_secs(shutdown_duration) {    
        for motor_num in 1..=6 {
            let mit_ctrl = JointMitControl::new(
                motor_num,
                0.0,              // pos_ref: no position control
                0.0,              // vel_ref: no velocity control
                0.0,              // kp: no position stiffness
                damping[(motor_num - 1) as usize],  // kd: damping
                0.0,              // t_ref: no torque control
            );
            piper.send_joint_mit_control(&mit_ctrl)?;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    
    // Disable MIT mode and motors on exit
    println!("\n\nDisabling MIT mode...");
    piper.enable_mit_mode(false)?;
    thread::sleep(Duration::from_millis(100));

    println!("Disabling motors...");
    piper.set_motor_enable(false)?;
    thread::sleep(Duration::from_millis(100));

    println!("\n✓ Graceful shutdown completed!");
    
    Ok(())
}

/// Expand tilde (~) to home directory path
fn expand_tilde(path: &str) -> PathBuf {
    if path.starts_with("~") {
        if let Ok(home) = std::env::var("HOME") {
            let path_without_tilde = &path[1..];  // Remove ~
            PathBuf::from(format!("{}{}", home, path_without_tilde))
        } else {
            PathBuf::from(path)
        }
    } else {
        PathBuf::from(path)
    }
}

/// MuJoCo Gravity Compensation Calculator
/// This struct handles gravity compensation torque calculations using MuJoCo
pub struct GravityCompensationCalculator {
    data: MjData<Rc<MjModel>>,
}

impl GravityCompensationCalculator {
    /// Create a new gravity compensation calculator from an MJCF/URDF file
    pub fn new(xml_path: &Path) -> Result<Self> {
        // Load the model
        let model = Rc::new(MjModel::from_xml(xml_path)
            .map_err(|e| std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to load MuJoCo model: {}", e)
            ))?);
        let data = MjData::new(model.clone());

        Ok(GravityCompensationCalculator { data })
    }

    /// Compute gravity compensation torques for given joint angles and velocities
    /// 
    /// Parameters:
    /// - angles_rad: Joint angles in radians [q1, q2, q3, q4, q5, q6]
    /// - velocities_rad: Joint velocities in rad/s [dq1, dq2, dq3, dq4, dq5, dq6]
    /// 
    /// Returns:
    /// - Vec of gravity compensation torques (one per joint)
    pub fn compute_torques(&mut self, angles_rad: &[f64; 6], velocities_rad: &[f64; 6]) -> [f64; 6] {

        // Set joint positions to the input angles
        self.data.qpos_mut()[0..6].copy_from_slice(angles_rad);

        // Set joint velocities
        self.data.qvel_mut()[0..6].copy_from_slice(velocities_rad);

        // Zero out accelerations for gravity-only computation
        self.data.qacc_mut()[0..6].fill(0.0);

        // Step the simulation to update kinematics and compute gravity effects
        self.data.step();

        // Extract gravity compensation forces from qfrc_bias
        // qfrc_bias contains gravity and constraint forces computed at the given state
        let gravity_torques: [f64; 6] = array::from_fn(|i| self.data.qfrc_bias()[i]);

        gravity_torques
    }
}

fn main() -> Result<()> {
    env_logger::init();

    println!("Piper SDK - Gravity Compensation Example");
    println!("=========================================");
    println!("Using MuJoCo for physics simulation\n");

    let default_xml_path = "~/tans_ws/AgileX/piper_ros/src/piper_description/mujoco_model/piper_no_gripper_description_new.xml";

    // Get CAN interface name from command line or use default
    let can_interface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "can0".to_string());

    // Get xml path from command line or use default
    let xml_path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| default_xml_path.to_string());

    // Expand tilde in the path
    let expanded_xml_path = expand_tilde(&xml_path);

    // Check if XML file exists
    if !expanded_xml_path.exists() {
        eprintln!("Error: XML file not found: {}", xml_path);
        eprintln!("Expanded path: {:?}", expanded_xml_path);
        eprintln!("Usage: cargo run --example gravity_compensation -- [can_interface] [xml_path]");
        eprintln!("\nExample:");
        eprintln!("  cargo run --example gravity_compensation -- can0 ~/path/to/piper_no_gripper_description.xml");
        return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "XML file not found").into());
    }

    println!("Loading robot model from: {}\n", expanded_xml_path.display());

    let piper = PiperInterface::new(&can_interface)?;
    println!("Connected to CAN interface: {}\n", piper.interface_name());

    // Create gravity compensation calculator using MuJoCo
    println!("Initializing gravity compensation calculator with model: {}", expanded_xml_path.display());
    let mut gravity_calc = GravityCompensationCalculator::new(&expanded_xml_path)?;
    println!("✓ Gravity compensation calculator initialized!\n");

    // Setup signal handler for graceful shutdown
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        println!("\n\n🛑 Shutdown signal received, exiting gracefully...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting signal handler");

    // Initialize robot
    println!("Initializing robot...");

    // Check driver status
    println!("Checking driver status...");
    let mut all_enabled = false;
    let mut check_count = 0;
    while !all_enabled && check_count < 20 {
        thread::sleep(Duration::from_millis(500));
        check_count += 1;

        all_enabled = true;
        for motor_num in 1..=6 {
            match piper.get_motor_low_speed(motor_num) {
                Ok(feedback) => {
                    if !feedback.is_driver_enabled() {
                        println!("Motor {} driver is disabled, enabling...", motor_num);
                        piper.set_motor_enable(true)?;
                        all_enabled = false;
                        break;
                    }
                }
                Err(_) => {
                    all_enabled = false;
                    break;
                }
            }
        }
    }

    if !all_enabled {
        println!("⚠️  Warning: Not all drivers enabled, proceeding anyway...");
    } else {
        println!("✓ All motor drivers enabled!\n");
    }

    // Enable MIT mode for torque control
    println!("Enabling MIT mode for gravity compensation...");
    piper.enable_mit_mode(true)?;
    thread::sleep(Duration::from_millis(100));
    println!("MIT mode enabled.\n");

    println!("Starting gravity compensation loop...");
    println!("Press Ctrl+C to exit.\n");

    let loop_rate = Duration::from_millis(5); // 200 Hz control loop
    let mut iteration = 0;

    // Control loop
    while running.load(Ordering::SeqCst) {
        let loop_start = std::time::Instant::now();

        // Read current joint high-speed state
        let joint_state = match piper.get_joint_high_speed_states() {
            Ok(state) => state,
            Err(e) => {
                println!("Error reading joint high-speed state: {}", e);
                continue;
            }
        };

        iteration += 1;

        // Convert angles from degrees to radians
        // Note: q is already in radians from high-speed feedback
        let angles_rad = joint_state.q;

        // Joint velocities
        let velocities_rad = joint_state.dq;

        // torques feedback
        let efforts: [f32; 6] = array::from_fn(|i| {
            joint_state.effort[i] as f32 / MOTOR_REDUCE_FACTORS[i]
        });

        // Compute gravity compensation torques using MuJoCo
        let torques = gravity_calc.compute_torques(&angles_rad, &velocities_rad);            

        let torques_reduced: [f32; 6] = array::from_fn(|i| {
            torques[i] as f32 * MOTOR_REDUCE_FACTORS[i]
        });

        // piper.enable_mit_mode(true)?;
        for (motor_num, _torque) in torques.iter().enumerate() {
            let motor_id = (motor_num + 1) as u8;
            let kp = {
                match motor_num {
                    2 | 5 => 0.4,    // Higher stiffness for joints 4-6
                    _ => 0.0,
                }
            };
            let pos_ref = {
                match motor_num {
                    2 => -1.340,
                    _ => 0.0,
                }
            };

            // MIT control parameters with impedance control:
            // - pos_ref: Current joint position (feedback control point)
            // - vel_ref: Target velocity (currently 0)
            // - kp: Position stiffness (impedance)
            // - kd: Velocity damping (impedance)
            // - t_ref: Gravity compensation torque feedforward
            let mit_ctrl = JointMitControl::new(
                motor_id,
                pos_ref,
                0.0,             // vel_ref: no velocity control
                kp,
                0.03,             // kd: velocity damping
                torques_reduced[motor_num],  // t_ref: gravity compensation torque (scaled)
                // 0.0,             // t_ref: no torque feedforward for testing
            );
            piper.send_joint_mit_control(&mit_ctrl)?;
        }

        // Print status every 100 iterations ( 200/100 = 2Hz )
        if iteration % 100 == 0 {
            println!(
                "[{:04}] Angles: [{:.2}°, {:.2}°, {:.2}°, {:.2}°, {:.2}°, {:.2}°]",
                iteration,
                joint_state.q[0].to_degrees(), joint_state.q[1].to_degrees(),
                joint_state.q[2].to_degrees(), joint_state.q[3].to_degrees(),
                joint_state.q[4].to_degrees(), joint_state.q[5].to_degrees()
            );
            println!(
                "       Efforts: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] N·m",
                efforts[0], efforts[1], efforts[2],
                efforts[3], efforts[4], efforts[5]
            );
            println!(
                "       Torques sent: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] N·m",
                torques[0], torques[1], torques[2],
                torques[3], torques[4], torques[5]
            );
        }

        // Maintain control loop rate
        let elapsed = loop_start.elapsed();
        if elapsed < loop_rate {
            thread::sleep(loop_rate - elapsed);
        }
    }
    
    // Graceful shutdown with damping
    let shutdown_duration = 5;  // 5 seconds
    let shutdown_damping = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4];  // 0.4 damping for all joints
    graceful_shutdown(&piper, shutdown_duration, shutdown_damping)?;

    println!("\nGravity compensation stopped!");
    println!("Total iterations: {}", iteration);

    Ok(())
}

