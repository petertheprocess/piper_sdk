//! High-level interface for Piper robot arm control

use crate::can_id::CanId;
use crate::error::{Error, Result};
use crate::messages::*;
use crate::protocol::PiperProtocol;
use socketcan::{CanDataFrame, CanFrame, CanSocket, Socket, EmbeddedFrame, Frame};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Main interface for Piper robot arm
pub struct PiperInterface {
    /// CAN socket
    socket: Arc<Mutex<CanSocket>>,
    /// Protocol parser
    protocol: Arc<PiperProtocol>,
    /// CAN interface name
    interface_name: String,
    /// Receive thread handle
    _rx_thread: Option<thread::JoinHandle<()>>,
}

impl PiperInterface {
    /// Create a new Piper interface
    ///
    /// # Arguments
    ///
    /// * `interface_name` - Name of the CAN interface (e.g., "can0")
    ///
    /// # Example
    ///
    /// ```no_run
    /// use piper_sdk_rs::PiperInterface;
    ///
    /// let piper = PiperInterface::new("can0").unwrap();
    /// ```
    pub fn new(interface_name: &str) -> Result<Self> {
        // Open CAN socket
        let socket = CanSocket::open(interface_name).map_err(|e| {
            Error::CanError(format!("Failed to open CAN interface '{}': {}", interface_name, e))
        })?;
        
        // Set read timeout
        socket
            .set_read_timeout(Duration::from_millis(100))
            .map_err(|e| Error::IoError(e))?;
        
        let socket = Arc::new(Mutex::new(socket));
        let protocol = Arc::new(PiperProtocol::new());
        
        // Start receive thread
        let rx_thread = {
            let socket = Arc::clone(&socket);
            let protocol = Arc::clone(&protocol);
            
            thread::spawn(move || {
                loop {
                    let frame = {
                        let sock = socket.lock()
                            .expect("Mutex poisoned - cannot access CAN socket");
                        sock.read_frame()
                    };
                    
                    match frame {
                        Ok(frame) => {
                            // Extract ID and data based on frame type
                            match frame {
                                CanFrame::Data(data_frame) => {
                                    let id = data_frame.raw_id();
                                    let data = data_frame.data();
                                    
                                    if let Err(e) = protocol.process_message(id, data) {
                                        log::debug!("Error processing message: {}", e);
                                    }
                                }
                                _ => {
                                    // Ignore remote and error frames
                                }
                            }
                        }
                        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            // Timeout, continue
                            thread::sleep(Duration::from_millis(1));
                        }
                        Err(e) => {
                            log::error!("Error reading CAN frame: {}", e);
                            thread::sleep(Duration::from_millis(10));
                        }
                    }
                }
            })
        };
        
        Ok(Self {
            socket,
            protocol,
            interface_name: interface_name.to_string(),
            _rx_thread: Some(rx_thread),
        })
    }
    
    /// Get the latest joint state
    ///
    /// Returns None if no joint state has been received yet
    pub fn get_joint_state(&self) -> Result<Option<JointState>> {
        Ok(self.protocol.get_joint_state())
    }
    
    /// Get the latest gripper state
    ///
    /// Returns None if no gripper state has been received yet
    pub fn get_gripper_state(&self) -> Result<Option<GripperState>> {
        Ok(self.protocol.get_gripper_state())
    }
    
    /// Get the latest end pose
    ///
    /// Returns None if no end pose has been received yet
    pub fn get_end_pose(&self) -> Result<Option<EndPose>> {
        Ok(self.protocol.get_end_pose())
    }
    
    /// Get the latest arm status
    ///
    /// Returns None if no arm status has been received yet
    pub fn get_arm_status(&self) -> Result<Option<ArmStatus>> {
        Ok(self.protocol.get_arm_status())
    }
    
    /// Get high-speed feedback for a specific motor
    ///
    /// # Arguments
    ///
    /// * `motor_num` - Motor number (1-6)
    ///
    /// Returns None if no feedback has been received yet or motor_num is invalid
    pub fn get_motor_high_speed(&self, motor_num: usize) -> Result<Option<MotorHighSpeedFeedback>> {
        Ok(self.protocol.get_motor_high_speed(motor_num))
    }
    
    /// Get low-speed feedback for a specific motor
    ///
    /// # Arguments
    ///
    /// * `motor_num` - Motor number (1-6)
    ///
    /// Returns None if no feedback has been received yet or motor_num is invalid
    pub fn get_motor_low_speed(&self, motor_num: usize) -> Result<Option<MotorLowSpeedFeedback>> {
        Ok(self.protocol.get_motor_low_speed(motor_num))
    }
    
    /// Send a joint control command
    ///
    /// # Arguments
    ///
    /// * `control` - Joint control command with target angles
    pub fn send_joint_control(&self, control: &JointControl) -> Result<()> {
        let can_data = control.to_can_data();
        
        // Send three CAN frames for 6 joints
        self.send_frame(CanId::ArmJointCtrl12, &can_data[0])?;
        self.send_frame(CanId::ArmJointCtrl34, &can_data[1])?;
        self.send_frame(CanId::ArmJointCtrl56, &can_data[2])?;
        
        Ok(())
    }
    
    /// Send MIT control command for a single joint
    ///
    /// # Arguments
    ///
    /// * `control` - MIT control parameters for one joint
    ///
    /// # Example
    ///
    /// ```no_run
    /// use piper_sdk_rs::{PiperInterface, JointMitControl};
    ///
    /// let piper = PiperInterface::new("can0").unwrap();
    /// 
    /// // Enable MIT mode first
    /// piper.enable_mit_mode(true).unwrap();
    ///
    /// // Control motor 1 with MIT parameters
    /// let mit_ctrl = JointMitControl::new(1, 0.5, 0.0, 10.0, 0.8, 0.0);
    /// piper.send_joint_mit_control(&mit_ctrl).unwrap();
    /// ```
    pub fn send_joint_mit_control(&self, control: &JointMitControl) -> Result<()> {
        if control.motor_num < 1 || control.motor_num > 6 {
            return Err(Error::InvalidMessage(
                format!("Motor number {} out of range (1-6)", control.motor_num)
            ));
        }
        
        let data = control.to_can_data();
        let can_id = match control.motor_num {
            1 => CanId::ArmJointMitCtrl1,
            2 => CanId::ArmJointMitCtrl2,
            3 => CanId::ArmJointMitCtrl3,
            4 => CanId::ArmJointMitCtrl4,
            5 => CanId::ArmJointMitCtrl5,
            6 => CanId::ArmJointMitCtrl6,
            _ => unreachable!(),
        };
        
        self.send_frame(can_id, &data)?;
        Ok(())
    }
    
    /// Send end pose control command (Cartesian coordinates)
    ///
    /// # Arguments
    ///
    /// * `control` - End pose control with position and orientation
    pub fn send_end_pose_control(&self, control: &EndPoseControl) -> Result<()> {
        let can_data = control.to_can_data();
        
        // Send three CAN frames for X, Y, Z, RX, RY, RZ
        self.send_frame(CanId::ArmMotionCtrlCartesian1, &can_data[0])?;
        self.send_frame(CanId::ArmMotionCtrlCartesian2, &can_data[1])?;
        self.send_frame(CanId::ArmMotionCtrlCartesian3, &can_data[2])?;
        
        Ok(())
    }
    
    /// Send motion control 2 command
    ///
    /// # Arguments
    ///
    /// * `control` - Motion control settings including mode and MIT enable
    pub fn send_motion_ctrl_2(&self, control: &MotionCtrl2) -> Result<()> {
        let data = control.to_can_data();
        self.send_frame(CanId::ArmMotionCtrl2, &data)?;
        Ok(())
    }
    
    /// Enable or disable MIT mode
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enable MIT mode, false for position/velocity mode
    pub fn enable_mit_mode(&self, enable: bool) -> Result<()> {
        let is_mit_mode = if enable { 0xAD } else { 0x00 };
        let ctrl = MotionCtrl2::new(0x01, 0x04, 50, is_mit_mode);
        self.send_motion_ctrl_2(&ctrl)?;
        Ok(())
    }
    
    /// Set control mode
    ///
    /// # Arguments
    ///
    /// * `ctrl_mode` - Control mode (0x00=standby, 0x01=CAN)
    /// * `move_mode` - Move mode (0x00=P, 0x01=J, 0x02=L, 0x03=C, 0x04=M)
    /// * `speed_rate` - Speed percentage (0-100)
    pub fn set_mode(&self, ctrl_mode: u8, move_mode: u8, speed_rate: u8) -> Result<()> {
        let ctrl = MotionCtrl2::new(ctrl_mode, move_mode, speed_rate, 0x00);
        self.send_motion_ctrl_2(&ctrl)?;
        Ok(())
    }
    
    /// Emergency stop
    pub fn emergency_stop(&self) -> Result<()> {
        let data = vec![0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.send_frame(CanId::ArmMotionCtrl1, &data)?;
        Ok(())
    }
    
    /// Reset the robot arm
    pub fn reset(&self) -> Result<()> {
        let data = vec![0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.send_frame(CanId::ArmMotionCtrl1, &data)?;
        Ok(())
    }
    
    /// Send a gripper control command
    ///
    /// # Arguments
    ///
    /// * `control` - Gripper control command with target position and speed
    pub fn send_gripper_control(&self, control: &GripperControl) -> Result<()> {
        let data = control.to_can_data();
        self.send_frame(CanId::ArmGripperCtrl, &data)?;
        Ok(())
    }
    
    /// Enable or disable motors
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enable, false to disable
    pub fn set_motor_enable(&self, enable: bool) -> Result<()> {
        let data = if enable {
            vec![0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        } else {
            vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        };
        self.send_frame(CanId::ArmMotorEnableDisable, &data)?;
        Ok(())
    }
    
    /// Configure master-slave mode
    ///
    /// # Arguments
    ///
    /// * `mode` - Mode byte (0xFC for slave mode)
    /// * `param1` - First parameter
    /// * `param2` - Second parameter
    /// * `param3` - Third parameter
    pub fn set_master_slave_mode(&self, mode: u8, param1: u8, param2: u8, param3: u8) -> Result<()> {
        let data = vec![mode, param1, param2, param3, 0x00, 0x00, 0x00, 0x00];
        self.send_frame(CanId::ArmMasterSlaveModeConfig, &data)?;
        Ok(())
    }
    
    /// Send a CAN frame
    /// 
    /// Optimized for real-time performance by minimizing allocations
    fn send_frame(&self, can_id: CanId, data: &[u8]) -> Result<()> {
        // Create a socketcan ID from u32
        // All Piper CAN IDs are standard 11-bit IDs (< 0x800), safe to cast to u16
        let socketcan_id = socketcan::StandardId::new(can_id.as_u32() as u16)
            .ok_or_else(|| Error::CanError("Invalid CAN ID".to_string()))?;
        
        // Create data frame directly without intermediate variables for better performance
        let data_frame = CanDataFrame::new(socketcan_id, data)
            .ok_or_else(|| Error::CanError("Failed to create CAN frame".to_string()))?;
        
        // Acquire lock and send in single operation
        let socket = self.socket.lock()
            .expect("Mutex poisoned - cannot access CAN socket");
        socket
            .write_frame(&CanFrame::Data(data_frame))
            .map_err(|e| Error::CanError(format!("Failed to send CAN frame: {}", e)))?;
        
        Ok(())
    }
    
    /// Get the CAN interface name
    pub fn interface_name(&self) -> &str {
        &self.interface_name
    }
}

impl Drop for PiperInterface {
    fn drop(&mut self) {
        // The receive thread will be automatically terminated when the struct is dropped
        log::info!("Closing Piper interface for {}", self.interface_name);
    }
}
