//! High-level interface for Piper robot arm control

use crate::can_id::CanId;
use crate::error::{Error, Result};
use crate::messages::*;
use crate::protocol::PiperProtocol;
use socketcan::{CanDataFrame, CanFrame, CanSocket, Socket, EmbeddedFrame, Frame};
use std::sync::Arc;
use std::sync::mpsc::{channel, Sender};
use std::thread;
use std::time::Duration;

/// Send command for the send thread
struct SendCommand {
    can_id: CanId,
    data: [u8; 8],
    len: usize,
}

/// Main interface for Piper robot arm
pub struct PiperInterface {
    /// Protocol parser
    protocol: Arc<PiperProtocol>,
    /// CAN interface name
    interface_name: String,
    /// Receive thread handle
    _rx_thread: Option<thread::JoinHandle<()>>,
    /// Send command channel
    send_tx: Sender<SendCommand>,
    /// Send thread handle
    _tx_thread: Option<thread::JoinHandle<()>>,
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
        let protocol = Arc::new(PiperProtocol::new());
        
        // Create send thread with dedicated socket
        let (send_tx, send_rx) = channel::<SendCommand>();
        let tx_interface_name = interface_name.to_string();
        let tx_thread = thread::spawn(move || {
            // Open dedicated socket for sending in this thread
            let tx_socket = match CanSocket::open(&tx_interface_name) {
                Ok(s) => s,
                Err(e) => {
                    log::error!("Failed to open send socket: {}", e);
                    return;
                }
            };
            
            loop {
                match send_rx.recv() {
                    Ok(cmd) => {
                        // Create CAN frame and send
                        if let Some(socketcan_id) = socketcan::StandardId::new(cmd.can_id.as_u32() as u16) {
                            if let Some(data_frame) = CanDataFrame::new(socketcan_id, &cmd.data[..cmd.len]) {
                                let frame = CanFrame::Data(data_frame);
                                if let Err(e) = tx_socket.write_frame(&frame) {
                                    log::error!("Failed to send CAN frame: {}", e);
                                }
                            } else {
                                log::error!("Failed to create CAN data frame");
                            }
                        } else {
                            log::error!("Invalid CAN ID: {}", cmd.can_id.as_u32());
                        }
                    }
                    Err(_) => {
                        // Channel closed, exit thread
                        break;
                    }
                }
            }
        });
        
        // Start receive thread with its own socket
        let rx_interface_name = interface_name.to_string();
        let rx_protocol = Arc::clone(&protocol);
        let rx_thread = thread::spawn(move || {
            // Open dedicated socket for reading in this thread
            let rx_socket = match CanSocket::open(&rx_interface_name) {
                Ok(s) => {
                    // Set read timeout
                    if let Err(e) = s.set_read_timeout(Duration::from_millis(100)) {
                        log::error!("Failed to set read timeout: {}", e);
                        return;
                    }
                    s
                }
                Err(e) => {
                    log::error!("Failed to open receive socket: {}", e);
                    return;
                }
            };
            
            loop {
                match rx_socket.read_frame() {
                    Ok(frame) => {
                        // Extract ID and data based on frame type
                        match frame {
                            CanFrame::Data(data_frame) => {
                                let id = data_frame.raw_id();
                                let data = data_frame.data();
                                
                                if let Err(e) = rx_protocol.process_message(id, data) {
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
        });
        
        Ok(Self {
            protocol,
            interface_name: interface_name.to_string(),
            _rx_thread: Some(rx_thread),
            send_tx,
            _tx_thread: Some(tx_thread),
        })
    }
    
    /// Get the latest joint state
    ///
    /// Returns error if no joint state has been received yet
    pub fn get_joint_state(&self) -> Result<JointState> {
        self.protocol.get_joint_state()
            .ok_or_else(|| Error::CanError("Joint state not available".to_string()))
    }
    
    /// Get the latest gripper state
    ///
    /// Returns error if no gripper state has been received yet
    pub fn get_gripper_state(&self) -> Result<GripperState> {
        self.protocol.get_gripper_state()
            .ok_or_else(|| Error::CanError("Gripper state not available".to_string()))
    }
    
    /// Get the latest end pose
    ///
    /// Returns error if no end pose has been received yet
    pub fn get_end_pose(&self) -> Result<EndPose> {
        self.protocol.get_end_pose()
            .ok_or_else(|| Error::CanError("End pose not available".to_string()))
    }
    
    /// Get the latest arm status
    ///
    /// Returns error if no arm status has been received yet
    pub fn get_arm_status(&self) -> Result<ArmStatus> {
        self.protocol.get_arm_status()
            .ok_or_else(|| Error::CanError("Arm status not available".to_string()))
    }
    
    /// Get high-speed feedback for a specific motor
    ///
    /// # Arguments
    ///
    /// * `motor_num` - Motor number (1-6)
    ///
    /// Returns error if no feedback has been received yet or motor_num is invalid
    pub fn get_motor_high_speed(&self, motor_num: usize) -> Result<MotorHighSpeedFeedback> {
        if motor_num < 1 || motor_num > 6 {
            return Err(Error::CanError(format!("Invalid motor number: {}", motor_num)));
        }
        self.protocol.get_motor_high_speed(motor_num)
            .ok_or_else(|| Error::CanError(format!("Motor {} high-speed feedback not available", motor_num)))
    }

    /// Get joint high-speed states for all joints
    ///
    /// Returns error if not all motor feedbacks are available
    pub fn get_joint_high_speed_states(&self) -> Result<JointHighSpeedStates> {
        self.protocol.get_joint_high_speed_states()
            .ok_or_else(|| Error::CanError("Joint high-speed states not available (incomplete motor feedbacks)".to_string()))
    }

    /// Get low-speed feedback for a specific motor
    ///
    /// # Arguments
    ///
    /// * `motor_num` - Motor number (1-6)
    ///
    /// Returns error if no feedback has been received yet or motor_num is invalid
    pub fn get_motor_low_speed(&self, motor_num: usize) -> Result<MotorLowSpeedFeedback> {
        if motor_num < 1 || motor_num > 6 {
            return Err(Error::CanError(format!("Invalid motor number: {}", motor_num)));
        }
        self.protocol.get_motor_low_speed(motor_num)
            .ok_or_else(|| Error::CanError(format!("Motor {} low-speed feedback not available", motor_num)))
    }
    
    /// Send a joint control command
    ///
    /// # Arguments
    ///
    /// * `control` - Joint control command with target angles
    pub fn send_joint_control(&self, control: &JointControl) -> Result<()> {
        let can_data = control.to_bytes();
        
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
        
        let data = control.to_bytes();
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
        let can_data = control.to_bytes();
        
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
        let data = control.to_bytes();
        self.send_frame(CanId::ArmMotionCtrl2, &data)?;
        Ok(())
    }
    
    /// Enable or disable MIT mode
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enable MIT mode, false for position/velocity mode
    pub fn enable_mit_mode(&self, enable: bool) -> Result<()> {
        let mit_mode = if enable { MitMode::MIT } else { MitMode::PosVel };
        let ctrl = MotionCtrl2::new(CtrlMode::CAN, MoveMode::M, 0, mit_mode);
        self.send_motion_ctrl_2(&ctrl)?;
        Ok(())
    }
    
    /// Set control mode
    ///
    /// # Arguments
    ///
    /// * `ctrl_mode` - Control mode (use `CtrlMode` enum)
    /// * `move_mode` - Move mode (use `MoveMode` enum)
    /// * `speed_rate` - Speed percentage (0-100)
    pub fn set_mode(&self, ctrl_mode: CtrlMode, move_mode: MoveMode, speed_rate: u8) -> Result<()> {
        let ctrl = MotionCtrl2::new(ctrl_mode, move_mode, speed_rate, MitMode::PosVel);
        self.send_motion_ctrl_2(&ctrl)?;
        Ok(())
    }
    
    /// Emergency stop
    pub fn emergency_stop(&self) -> Result<()> {
        // let data = vec![MotorEnableStatus::Disabled.into(), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let mut data = [0u8; 8];
        data[0] = MotorEnableStatus::Disabled.into();
        self.send_frame(CanId::ArmMotionCtrl1, &data)?;
        Ok(())
    }
    
    /// Reset the robot arm
    pub fn reset(&self) -> Result<()> {
        let mut data = [0u8; 8];
        data[0] = MotorEnableStatus::Disabled.into();
        self.send_frame(CanId::ArmMotionCtrl1, &data)?;
        Ok(())
    }
    
    /// Send a gripper control command
    ///
    /// # Arguments
    ///
    /// * `control` - Gripper control command with target position and speed
    pub fn send_gripper_control(&self, control: &GripperControl) -> Result<()> {
        let data = control.to_bytes();
        self.send_frame(CanId::ArmGripperCtrl, &data)?;
        Ok(())
    }
    
    /// Enable or disable motors
    ///
    /// # Arguments
    /// 0x02 enable; 0x01 disable
    /// 0x07 for all motors
    ///
    /// * `enable` - true to enable, false to disable
    pub fn set_motor_enable(&self, enable: bool) -> Result<()> {
        let data = if enable {
            [0x07, MotorEnableStatus::Enabled.into(), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        } else {
            [0x07, MotorEnableStatus::Disabled.into(), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
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
        let data = [mode, param1, param2, param3, 0x00, 0x00, 0x00, 0x00];
        self.send_frame(CanId::ArmMasterSlaveModeConfig, &data)?;
        Ok(())
    }
    
    /// Send a CAN frame
    /// 
    /// Sends frames via dedicated send thread for better real-time performance.
    /// The send operation is non-blocking.
    fn send_frame(&self, can_id: CanId, data: &[u8]) -> Result<()> {
        let mut buf = [0u8; 8];
        let len = data.len().min(8);
        buf[..len].copy_from_slice(&data[..len]);
        
        let cmd = SendCommand {
            can_id,
            data: buf,
            len,
        };
        
        self.send_tx.send(cmd)
            .map_err(|_| Error::CanError("Send thread disconnected".to_string()))?;
        
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
