use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use nalgebra::{SVector, SMatrix, Vector3};
use std::time::Duration;
use std::path::Path;

fn main() {
    /* Load the model and create data */
    let mjcf_path = Path::new("/home/tans/tans_ws/AgileX/piper_sdk/piper_description/mujoco_model/piper_no_gripper_sim_scene.xml");
    let model = MjModel::from_xml(mjcf_path).expect("could not load the model");
    let mut data = model.make_data();  // or MjData::new(&model);
    
    println!("Model loaded successfully!");

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");


    /* Obtain the timestep through the wrapped mjModel */
    let timestep = model.opt().timestep;

    let site_name = "target_site";
    let joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ];
    let ee_body_id = model.body("link6")
        .unwrap_or_else(|| panic!("Body link6 not found")).id;

    let site_id = model.site(site_name)
                                .unwrap_or_else(|| panic!("Site {} not found", site_name)).id;
    let actuator_ids: Vec<usize> = joint_names.iter()
        .map(|&name|
            model.actuator(name)
                .unwrap_or_else(|| panic!("Actuator for joint {} not found", name))
                .id
        ).collect();

    let dof_ids: Vec<usize> = joint_names.iter()
        .map(|&name|
            model.joint(name)
                .unwrap_or_else(|| panic!("Joint {} not found", name)).id
        ).collect();

    while viewer.running() {
        /* Step the simulation and sync the viewer */
        viewer.sync_data(&mut data);
        data.step();
        viewer.render();

        // --- 实时碰撞检测并打印 ---
        let ncon = data.ncon();
        if ncon > 0 {
            println!("Contacts detected: {}", ncon);
            // 遍历并打印每个 contact 的关键信息
            let contacts = data.contact();
            for i in 0..ncon as usize {
                if let Some(c) = contacts.get(i) {
                    // 大多数 wrapper 会以字段名直接暴露这些值；若编译失败我可以再调整
                    let geom1 = c.geom1;
                    let geom2 = c.geom2;
                    let pos = c.pos; // 期望是 [f64; 3] 或类似切片
                    let dist = c.dist;
                    println!("  contact {}: geom{} <-> geom{} pos=[{:.3}, {:.3}, {:.3}] dist={:.6}",
                        i, geom1, geom2, pos[0], pos[1], pos[2], dist
                    );
                }
            }
        } else {
            // 可选：只在需要时取消注释以减少日志
            // println!("No contacts");
        }
        // --- 碰撞检测结束 ---

        // get target position
        let target_pos = data.site_xpos()[site_id];
        let ee_pos = data.xpos()[ee_body_id];
        // to nalgebra SVector for compile-time fixed size
        let target_pos: Vector3<f64> = Vector3::from_row_slice(&target_pos[..3]);
        let ee_pos: Vector3<f64> = Vector3::from_row_slice(&ee_pos[..3]);
        let position_error: Vector3<f64> = target_pos - ee_pos;

        // print .3f positions
        println!("End-Effector Position: [{:.3}, {:.3}, {:.3}]", ee_pos[0], ee_pos[1], ee_pos[2]);
        println!("Target Position: [{:.3}, {:.3}, {:.3}]", target_pos[0], target_pos[1], target_pos[2]);

        // Calculate gravity compensation torques using inverse dynamics
        let gravity_torques = compute_gravity_compensation(&data);

        let mut tau_to_apply = vec![0.0; actuator_ids.len()];

        let (jacp,_ ) = data.jac_body(true, false, ee_body_id as i32);

        let jacp_nd: SMatrix<f64, 3, 6> = SMatrix::from_row_slice(&jacp[..]);

        // PD control parameters
        let kp = 800.0;
        
        // Compute desired end-effector force
        let desired_force: Vector3<f64> = kp * position_error;

        // Compute desired joint torques using the Jacobian (3x6)^T * 3x1 => 6x1
        let joint_torques: SVector<f64, 6> = jacp_nd.transpose() * desired_force;

        // apply the gravity compensation torques to the actuators
        for (i, &act_id) in actuator_ids.iter().enumerate() {
            tau_to_apply[act_id] += gravity_torques[dof_ids[i]];
            tau_to_apply[act_id] += joint_torques[dof_ids[i]];
        }

        data.ctrl_mut().copy_from_slice(&tau_to_apply);

        // print the gravity compensation torques
        // println!("Gravity Compensation Torques: {:?}", gravity_torques);
        

        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}

// Helper function to compute gravity compensation torques
fn compute_gravity_compensation(data: &MjData<&MjModel>) -> Vec<f64> {
    // Clone the data to work with a copy
    
    let qfrc_gravcomp = data.qfrc_bias();
    let gravity_torques: Vec<f64> = qfrc_gravcomp.to_vec();
    
    gravity_torques
}