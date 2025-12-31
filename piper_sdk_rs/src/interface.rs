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
                        let sock = socket.lock().unwrap();
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
    fn send_frame(&self, can_id: CanId, data: &[u8]) -> Result<()> {
        // Create a socketcan ID from u32
        let socketcan_id = socketcan::StandardId::new(can_id.as_u32() as u16)
            .ok_or_else(|| Error::CanError("Invalid CAN ID".to_string()))?;
        
        let data_frame = CanDataFrame::new(socketcan_id, data)
            .ok_or_else(|| Error::CanError("Failed to create CAN frame".to_string()))?;
        
        let frame = CanFrame::Data(data_frame);
        
        let socket = self.socket.lock().unwrap();
        socket
            .write_frame(&frame)
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
