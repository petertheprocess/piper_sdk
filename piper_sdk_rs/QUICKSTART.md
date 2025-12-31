# Quick Start Guide - Piper Rust SDK

This guide will get you up and running with the Piper Rust SDK in minutes.

## Prerequisites

1. **Linux System** with SocketCAN support (Ubuntu 18.04+, Debian, etc.)
2. **Rust** 1.70 or later
3. **CAN Hardware** (USB-to-CAN adapter)
4. **Piper Robot Arm**

## Step 1: Install Rust

If you don't have Rust installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

Verify installation:
```bash
cargo --version
```

## Step 2: Install CAN Tools

```bash
sudo apt update
sudo apt install can-utils
```

## Step 3: Configure CAN Interface

Connect your USB-to-CAN adapter and configure it:

```bash
# Find your CAN interface
ip link show | grep can

# Configure can0 with 1Mbps bitrate (required for Piper)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
ifconfig can0
```

Expected output:
```
can0: flags=193<UP,RUNNING,NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
        RX packets 0  bytes 0 (0.0 B)
        TX packets 0  bytes 0 (0.0 B)
```

## Step 4: Clone and Build

```bash
# Clone the repository
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk/piper_sdk_rs

# Build the SDK
cargo build --release

# Build examples
cargo build --release --examples
```

## Step 5: First Test - Set Slave Mode

Before reading feedback, configure the robot to slave mode:

```bash
cargo run --release --example set_slave_mode
```

You should see:
```
Piper SDK - Set Slave Mode Example
===================================

Connected to CAN interface: can0

Setting robot to slave mode...
Command sent successfully!

Robot should now be in slave mode and sending feedback.
```

## Step 6: Read Joint States

Now read the joint angles:

```bash
cargo run --release --example read_joint_state
```

You should see continuous output:
```
Piper SDK - Read Joint State Example
=====================================

Connected to CAN interface: can0

Reading joint states... (Press Ctrl+C to exit)

Joint Angles (rad): [0.0000, -0.1234, 0.5678, 0.0000, 0.3456, 0.0000]
Timestamp: 1704067200.123

Gripper Position: 500, Status: 0x01
```

Press Ctrl+C to exit.

## Step 7: Control the Robot (Optional)

⚠️ **Safety First**: Ensure the robot has clear workspace before running control commands!

```bash
# Control joints
cargo run --release --example control_joints

# Control gripper
cargo run --release --example control_gripper
```

## Your First Program

Create a new Rust project:

```bash
cargo new my_piper_app
cd my_piper_app
```

Edit `Cargo.toml`:
```toml
[dependencies]
piper_sdk_rs = { path = "../piper_sdk/piper_sdk_rs" }
env_logger = "0.11"
```

Edit `src/main.rs`:
```rust
use piper_sdk_rs::{PiperInterface, Result};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    env_logger::init();
    
    println!("Connecting to Piper...");
    let piper = PiperInterface::new("can0")?;
    
    // Set to slave mode
    piper.set_master_slave_mode(0xFC, 0, 0, 0)?;
    thread::sleep(Duration::from_millis(100));
    
    // Read joint states for 5 seconds
    println!("Reading joint states for 5 seconds...");
    let start = std::time::Instant::now();
    
    while start.elapsed().as_secs() < 5 {
        if let Some(state) = piper.get_joint_state()? {
            println!("Joints: {:?}", state.angles);
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("Done!");
    Ok(())
}
```

Run it:
```bash
cargo run --release
```

## Common Issues

### "Failed to open CAN interface"

**Solution**: Ensure CAN is configured and up:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### "No data received"

**Solutions**:
1. Verify robot is in slave mode: `cargo run --example set_slave_mode`
2. Check CAN wiring and connections
3. Verify robot is powered on
4. Monitor CAN bus: `candump can0`

### "Permission denied"

**Solution**: Add user to dialout group or run with sudo:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Build errors

**Solution**: Update Rust and clean build:
```bash
rustup update
cargo clean
cargo build --release
```

## Next Steps

1. Read the [full README](README.md) for detailed API documentation
2. Study the [examples](examples/) for more use cases
3. Check [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) for architecture details
4. Generate API docs: `cargo doc --open`

## Getting Help

- **Issues**: https://github.com/agilexrobotics/piper_sdk/issues
- **Discord**: https://discord.gg/wrKYTxwDBd
- **Documentation**: `cargo doc --open`

## Safety Reminders

⚠️ **Always**:
- Verify clear workspace before moving robot
- Start with low speeds when testing
- Keep emergency stop accessible
- Never leave robot unattended during operation

---

Happy coding! 🦀🤖
