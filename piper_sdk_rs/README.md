# Piper Robot Arm SDK - Rust Edition

This is a Rust implementation of the Piper robot arm SDK using the `socketcan` crate for CAN bus communication. It provides a safe and efficient interface for controlling Piper robot arms on Linux systems.

## Features

- **CAN Bus Communication**: Uses socketcan for native Linux CAN support
- **Type-Safe Message Handling**: Rust's type system ensures message correctness
- **Asynchronous Reading**: Background thread continuously reads CAN messages
- **Easy-to-Use API**: Simple interface matching the Python SDK functionality
- **Zero-Cost Abstractions**: Efficient Rust implementation with minimal overhead

## Prerequisites

- Rust 1.70 or later
- Linux system with SocketCAN support
- CAN interface (e.g., USB-to-CAN adapter)
- can-utils package (for CAN configuration)

## Installation

### 1. Install Rust

If you don't have Rust installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### 2. Install CAN Utilities

```bash
sudo apt update && sudo apt install can-utils
```

### 3. Add to Your Project

Add this to your `Cargo.toml`:

```toml
[dependencies]
piper_sdk_rs = { path = "path/to/piper_sdk_rs" }
```

Or build the SDK:

```bash
cd piper_sdk_rs
cargo build --release
```

## CAN Interface Setup

Before using the SDK, configure your CAN interface:

### Single CAN Module

```bash
# Set up can0 with 1Mbps bitrate
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify configuration
ifconfig can0
```

### Check CAN Interface

```bash
# Find connected CAN modules
ip link show | grep can

# Monitor CAN traffic
candump can0
```

## Quick Start

### 1. Set Robot to Slave Mode

Before reading feedback, set the robot to slave mode:

```rust
use piper_sdk_rs::{PiperInterface, Result};

fn main() -> Result<()> {
    let piper = PiperInterface::new("can0")?;
    
    // Set to slave mode (0xFC, 0, 0, 0)
    piper.set_master_slave_mode(0xFC, 0, 0, 0)?;
    
    Ok(())
}
```

Or run the example:

```bash
cargo run --example set_slave_mode
```

### 2. Read Joint States

```rust
use piper_sdk_rs::{PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    let piper = PiperInterface::new("can0")?;
    
    loop {
        if let Some(joint_state) = piper.get_joint_state()? {
            println!("Joint angles: {:?}", joint_state.angles);
        }
        thread::sleep(Duration::from_millis(5)); // 200Hz
    }
}
```

### 3. Control Joints

```rust
use piper_sdk_rs::{JointControl, PiperInterface, Result};

fn main() -> Result<()> {
    let piper = PiperInterface::new("can0")?;
    
    // Enable motors
    piper.set_motor_enable(true)?;
    
    // Move to target position (angles in radians)
    let target = JointControl::new([0.5, -0.3, 0.2, 0.0, 0.4, 0.0]);
    piper.send_joint_control(&target)?;
    
    Ok(())
}
```

### 4. Control Gripper

```rust
use piper_sdk_rs::{GripperControl, PiperInterface, Result};

fn main() -> Result<()> {
    let piper = PiperInterface::new("can0")?;
    
    // Open gripper (position: 1000, speed: 500)
    let cmd = GripperControl::new(1000, 500);
    piper.send_gripper_control(&cmd)?;
    
    Ok(())
}
```

## Examples

The SDK includes several example programs:

```bash
# Read joint states continuously
cargo run --example read_joint_state

# Set robot to slave mode
cargo run --example set_slave_mode

# Control robot joints
cargo run --example control_joints

# Control gripper
cargo run --example control_gripper
```

## API Documentation

Generate and view the API documentation:

```bash
cargo doc --open
```

## Architecture

The SDK is organized into several modules:

- **`can_id`**: CAN message ID definitions
- **`error`**: Error types and Result type alias
- **`interface`**: High-level PiperInterface for robot control
- **`messages`**: Message types (JointState, GripperState, etc.)
- **`protocol`**: Protocol layer for message parsing

### Message Flow

1. **Receive**: Background thread reads CAN frames from socketcan
2. **Parse**: Protocol layer parses frames into message types
3. **Buffer**: Multi-frame messages are assembled
4. **Query**: User code queries latest messages via interface methods

### Thread Safety

The SDK uses `Arc<Mutex<>>` for thread-safe access to shared data:
- CAN socket is shared between send and receive operations
- Message buffer is protected by mutex for concurrent access

## Comparison with Python SDK

| Feature | Python SDK | Rust SDK |
|---------|-----------|----------|
| CAN Library | python-can | socketcan |
| Performance | ~200 Hz | ~1000+ Hz |
| Memory Safety | Runtime checks | Compile-time checks |
| Concurrency | Threading | Native threads |
| Type Safety | Dynamic typing | Static typing |
| Error Handling | Exceptions | Result type |

## Troubleshooting

### "Failed to open CAN interface"

- Ensure CAN interface is configured: `sudo ip link set can0 up type can bitrate 1000000`
- Check interface exists: `ip link show can0`
- Verify permissions: Add user to dialout group or run with sudo

### No Data Received

- Verify robot is in slave mode (run `set_slave_mode` example)
- Check CAN connection and wiring
- Monitor CAN bus: `candump can0`
- Ensure robot is powered on

### Build Errors

- Update Rust: `rustup update`
- Check dependencies: `cargo check`
- Clean build: `cargo clean && cargo build`

## Performance Tips

1. **Polling Rate**: The SDK can handle 1000+ Hz reading rates, but typical robot feedback is 200 Hz
2. **Message Processing**: The background thread handles all CAN I/O, minimizing latency
3. **Memory Usage**: Messages are cloned only when requested, reducing overhead

## Safety Notes

- Always verify robot workspace before sending commands
- Start with low speeds when testing
- Implement emergency stop mechanisms
- The MIT control mode is advanced - incorrect use can damage the robot

## Contributing

Contributions are welcome! Please ensure:

1. Code follows Rust best practices
2. All examples compile and run
3. Documentation is updated
4. Tests pass: `cargo test`

## License

MIT License - See LICENSE file for details

## Related Projects

- [Python SDK](../piper_sdk) - Original Python implementation
- [socketcan-rs](https://github.com/socketcan-rs/socketcan-rs) - Rust SocketCAN library

## Contact

- GitHub Issues: Report bugs and feature requests
- Discord: Join the Agilex Robotics community

## Version History

- **0.1.0** (2024-12-31): Initial Rust SDK release
  - Basic CAN communication
  - Joint and gripper control
  - Message reading and parsing
  - Example programs
