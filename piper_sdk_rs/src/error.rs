//! Error types for the Piper SDK

use thiserror::Error;

/// Result type for Piper SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// Errors that can occur in the Piper SDK
#[derive(Error, Debug)]
pub enum Error {
    /// CAN bus error
    #[error("CAN bus error: {0}")]
    CanError(String),
    
    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    
    /// Invalid message format
    #[error("Invalid message format: {0}")]
    InvalidMessage(String),
    
    /// Timeout error
    #[error("Operation timed out")]
    Timeout,
    
    /// CAN interface not found
    #[error("CAN interface '{0}' not found or not configured")]
    InterfaceNotFound(String),
    
    /// Message send failed
    #[error("Failed to send CAN message")]
    SendFailed,
    
    /// Message receive failed
    #[error("Failed to receive CAN message")]
    ReceiveFailed,
}
