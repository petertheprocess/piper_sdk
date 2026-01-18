use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use std::time::Duration;
use std::path::Path;
use std::env;

fn main() {
    /* Load the model and create data */
    let home = env!("HOME");
    let mjcf_path = Path::new(&home).join("tans_ws/AgileX/piper_ros/src/piper_description/mujoco_model/piper_no_gripper_description.xml");
    let model = MjModel::from_xml(mjcf_path).expect("could not load the model");
    let mut data = model.make_data();  // or MjData::new(&model);

    // randomely initialize qpos
    {
        let qpos = data.qpos_mut();
        for pos in qpos {
            *pos = rand::random::<f64>() * 20.0 - 10.0; // Random values between -1.0 and 1.0
        }
    }
    
    println!("Model loaded successfully!");

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");


    /* Obtain the timestep through the wrapped mjModel */
    let timestep = model.opt().timestep;

    while viewer.running() {
        /* Step the simulation and sync the viewer */
        viewer.sync_data(&mut data);
        data.step();
        viewer.render();

        // Calculate gravity compensation torques using inverse dynamics
        let gravity_torques = compute_gravity_compensation(&model, &data);

        // print the gravity compensation torques
        println!("Gravity Compensation Torques: {:?}", gravity_torques);
        

        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}

// Helper function to compute gravity compensation torques
fn compute_gravity_compensation(model: &MjModel, data: &MjData<&MjModel>) -> Vec<f64> {
    // Clone the data to work with a copy
    
    let qfrc_gravcomp = data.qfrc_bias();
    let gravity_torques: Vec<f64> = qfrc_gravcomp.to_vec();

    println!("qfrc_gravcomp length: {}, values: {:?}", gravity_torques.len(), gravity_torques);
    
    gravity_torques
}