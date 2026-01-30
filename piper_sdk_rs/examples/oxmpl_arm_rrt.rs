use std::{f64::consts::PI, sync::Arc, time::Duration};
use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::{RealVectorStateSpace, StateSpace},
    state::RealVectorState,
    validity::StateValidityChecker,
};
use oxmpl::geometric::RRT;

use rand::Rng;
use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;

/// MuJoCo-based obstacle checker that performs forward kinematics and collision detection.
/// 使用 MuJoCo 的 contact 检测来判断给定关节角是否会发生碰撞
struct MujocoObstacleChecker {
    /// MuJoCo 数据（使用 RefCell 以支持内部可变性）
    data: RefCell<MjData<Rc<MjModel>>>,
}

impl MujocoObstacleChecker {
    /// 创建新的 MujocoObstacleChecker
    /// - `xml_path`: MJCF 模型文件路径
    pub fn new(xml_path: &Path) -> Self {
        let model = Rc::new(MjModel::from_xml(xml_path).expect("Could not load MuJoCo model"));
        let data = MjData::new(model);
        Self {
            data: RefCell::new(data),
        }
    }

    /// 检查给定关节角配置是否发生碰撞
    /// 返回 true 表示无碰撞（有效），false 表示有碰撞（无效）
    fn check_collision(&self, joint_angles: &[f64]) -> bool {
        let mut data = self.data.borrow_mut();

        // 设置关节位置 (qpos)
        let qpos = data.qpos_mut();
        let n = joint_angles.len().min(qpos.len());
        qpos[..n].copy_from_slice(&joint_angles[..n]);

        // 执行正向运动学和碰撞检测（mj_forward 会更新 contact 信息）
        data.forward();

        // 检查碰撞数量，ncon > 0 表示有碰撞
        data.ncon() == 0
    }
}

impl StateValidityChecker<RealVectorState> for MujocoObstacleChecker {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        // state.values 包含关节角（弧度）
        self.check_collision(&state.values)
    }
}

/// A Goal definition where success is being within a certain radius of a target state.
struct CircularGoalRegion {
    target: RealVectorState,
    radius: f64,
    space: Arc<RealVectorStateSpace>,
}

impl Goal<RealVectorState> for CircularGoalRegion {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<RealVectorState> for CircularGoalRegion {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<RealVectorState> for CircularGoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        // 在目标状态附近采样（球形区域）
        let mut values = self.target.values.clone();
        for v in values.iter_mut() {
            // 在每个维度上添加随机扰动
            let perturbation = (rng.random::<f64>() - 0.5) * 2.0 * self.radius;
            *v += perturbation;
        }
        Ok(RealVectorState { values })
    }
}

fn main() {
    // 6 自由度机械臂的关节空间（关节角限制，单位：弧度）
    // Piper 机械臂的典型关节限制
    let joint_limits = vec![
        (-PI, PI),      // joint1
        (-PI / 2.0, PI / 2.0),  // joint2
        (-PI, PI),      // joint3
        (-PI / 2.0, PI / 2.0),  // joint4
        (-PI, PI),      // joint5
        (-PI, PI),      // joint6
    ];
    
    let new_rvss_result = RealVectorStateSpace::new(6, Some(joint_limits));
    let space = Arc::new(new_rvss_result.expect("Error creating new RealVectorState!"));

    // 起始关节配置（零位）
    let start_state = RealVectorState {
        values: vec![1.8, 2.0, -1.2, 0.0, 0.1, 0.0],
    };
    
    // 目标关节配置
    let goal_definition = Arc::new(CircularGoalRegion {
        target: RealVectorState {
            values: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        radius: 0.1, // 关节空间中的距离容差（弧度）
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    // 使用 MuJoCo 碰撞检测器
    let mjcf_path = Path::new("/home/tans/tans_ws/AgileX/piper_sdk/piper_description/mujoco_model/piper_no_gripper_sim_scene.xml");
    let validity_checker = Arc::new(MujocoObstacleChecker::new(mjcf_path));

    let mut planner = RRT::new(0.01, 0.2, &PlannerConfig { seed: Some(123) });

    planner.setup(problem_definition, validity_checker.clone());

    let timeout = Duration::from_secs(4);
    let result = planner.solve(timeout);

    let path = result.expect("Planner failed to find a solution");
    println!("Found path with {} states.", path.0.len());

    // 在 MuJoCo viewer 中显示规划路径动画
    let model = MjModel::from_xml(mjcf_path).expect("Could not load MuJoCo model");
    let mut data = model.make_data();
    
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("Could not launch the viewer");
    
    let timestep = model.opt().timestep;
    
    // 动画播放参数
    let skip_step = 5; // 每隔多少个路径点播放一次（跳着播）
    let animation_delay = 0.05; // 每帧延迟（秒）
    
    println!("Playing path animation ({} states, skip={})... Press Ctrl+C or close viewer to exit.", 
             path.0.len(), skip_step);
    
    while viewer.running() {
        // 循环播放路径，跳着播
        let mut i = 0;
        while i < path.0.len() {
            if !viewer.running() {
                break;
            }
            
            let state = &path.0[i].values;
            
            // 设置关节位置
            let qpos = data.qpos_mut();
            for j in 0..6 {
                qpos[j] = state[j];
            }
            
            // 执行正向运动学
            data.forward();
            
            // 同步并渲染
            viewer.sync_data(&mut data);
            viewer.render();
            
            std::thread::sleep(Duration::from_secs_f64(animation_delay));
            
            i += skip_step;
        }
        
        // 确保播放最后一个点
        if path.0.len() > 0 {
            let last = &path.0[path.0.len() - 1].values;
            let qpos = data.qpos_mut();
            for j in 0..6 {
                qpos[j] = last[j];
            }
            data.forward();
            viewer.sync_data(&mut data);
            viewer.render();
        }
        
        // 在最后一个路径点停留一会儿
        std::thread::sleep(Duration::from_millis(1000));
    }
    
    println!("Viewer closed.");
}