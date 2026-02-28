use std::{f64::consts::PI, sync::Arc, time::Duration};
use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;
use std::sync::{Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::io::{self, Write};

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

// 引入路径平滑模块
#[path = "path_smoother.rs"]
mod path_smoother;
use path_smoother::PathSmoother;

/// 共享的路径数据，用于主线程和规划线程之间通信
struct SharedPathData {
    /// 当前要播放的路径
    path: Vec<RealVectorState>,
    /// 路径是否已更新（viewer 需要重新开始播放）
    updated: bool,
}

impl SharedPathData {
    fn new() -> Self {
        Self {
            path: Vec::new(),
            updated: false,
        }
    }
}

/// 共享的命令数据，用于触发规划
struct SharedCommand {
    /// 是否需要规划新路径
    plan_requested: bool,
    /// 规划是否正在进行
    planning_in_progress: bool,
}

impl SharedCommand {
    fn new() -> Self {
        Self {
            plan_requested: false,
            planning_in_progress: false,
        }
    }
}

/// MuJoCo-based obstacle checker that performs forward kinematics and collision detection.
/// 使用 MuJoCo 的 contact 检测来判断给定关节角是否会发生碰撞
/// 
/// 注意：这个结构体使用 RefCell，因此不是 Send/Sync，每个线程需要创建自己的实例
struct MujocoObstacleChecker {
    /// MuJoCo 数据（使用 RefCell 以支持内部可变性）
    data: RefCell<MjData<Rc<MjModel>>>,
    /// 关节限制 (min, max) for each joint
    joint_limits: Vec<(f64, f64)>,
}

impl MujocoObstacleChecker {
    /// 创建新的 MujocoObstacleChecker
    /// - `xml_path`: MJCF 模型文件路径
    /// - `joint_limits`: 各关节的角度限制 (min, max)
    pub fn new(xml_path: &Path, joint_limits: Vec<(f64, f64)>) -> Self {
        let model = Rc::new(MjModel::from_xml(xml_path).expect("Could not load MuJoCo model"));
        let data = MjData::new(model);
        Self {
            data: RefCell::new(data),
            joint_limits,
        }
    }

    /// 检查关节角是否在限制范围内
    /// 返回 true 表示在范围内，false 表示超出限制
    fn check_joint_limits(&self, joint_angles: &[f64]) -> bool {
        for (i, &angle) in joint_angles.iter().enumerate() {
            if i < self.joint_limits.len() {
                let (min, max) = self.joint_limits[i];
                if angle < min || angle > max {
                    return false;
                }
            }
        }
        true
    }

    /// 检查给定关节角配置是否发生碰撞
    /// 返回 true 表示无碰撞（有效），false 表示有碰撞（无效）
    fn check_collision(&self, joint_angles: &[f64]) -> bool {
        // 首先检查关节限制（节省 MuJoCo 计算）
        if !self.check_joint_limits(joint_angles) {
            return false;
        }

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

/// 生成随机可行状态（无碰撞的关节配置）
fn generate_random_valid_state(
    validity_checker: &Arc<MujocoObstacleChecker>,
    space: &Arc<RealVectorStateSpace>,
    rng: &mut impl Rng,
    max_attempts: usize,
) -> Option<RealVectorState> {
    for _ in 0..max_attempts {
        // 在关节空间中均匀采样
        let state = space.sample_uniform(rng).ok()?;
        
        // 检查是否无碰撞
        if validity_checker.is_valid(&state) {
            return Some(state);
        }
    }
    None
}

/// 规划路径并返回平滑后的路径数据
fn plan_path(
    start_state: &RealVectorState,
    goal_state: &RealVectorState,
    validity_checker: &Arc<MujocoObstacleChecker>,
    space: &Arc<RealVectorStateSpace>,
) -> Result<Vec<RealVectorState>, Box<dyn std::error::Error + Send + Sync>> {
    println!("[Planner] Planning path...");

    // 创建目标区域
    let goal_definition = Arc::new(CircularGoalRegion {
        target: goal_state.clone(),
        radius: 0.1, // 关节空间中的距离容差（弧度）
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let mut planner = RRT::new(0.01, 0.2, &PlannerConfig { seed: Some(rand::random()) });
    planner.setup(problem_definition, validity_checker.clone());

    let timeout = Duration::from_secs(2);
    let result = planner.solve(timeout);

    let path = result.map_err(|_| "Planner failed to find a solution")?;
    println!("[Planner] Found path with {} states.", path.0.len());

    // 对路径进行剪枝和平滑
    let smoother = PathSmoother::new(validity_checker.clone(), 0.02);
    let smoothed_path = smoother.process(&path.0, 0.5, 3, 10);
    println!("[Planner] Smoothed path: {} states", smoothed_path.len());

    Ok(smoothed_path)
}

/// 规划线程函数
/// 在子线程中进行路径规划，有自己独立的 MuJoCo 碰撞检测器
fn planner_thread_fn(
    mjcf_path: &'static str,
    joint_limits: Vec<(f64, f64)>,
    space: Arc<RealVectorStateSpace>,
    shared_path: Arc<Mutex<SharedPathData>>,
    shared_command: Arc<Mutex<SharedCommand>>,
    running: Arc<AtomicBool>,
) {
    // 规划线程有自己独立的 MuJoCo 碰撞检测器
    let validity_checker = Arc::new(MujocoObstacleChecker::new(Path::new(mjcf_path), joint_limits));
    let mut rng = rand::rng();

    println!("[Planner] Planner thread started.");

    while running.load(Ordering::Relaxed) {
        // 检查是否有规划请求
        let should_plan = {
            let mut cmd = shared_command.lock().unwrap();
            if cmd.plan_requested {
                cmd.plan_requested = false;
                cmd.planning_in_progress = true;
                true
            } else {
                false
            }
        };

        if should_plan {
            println!("[Planner] Generating random valid start and goal states...");

            // 生成随机初始状态
            let start_state = match generate_random_valid_state(&validity_checker, &space, &mut rng, 100) {
                Some(state) => state,
                None => {
                    println!("[Planner] Failed to generate valid start state");
                    shared_command.lock().unwrap().planning_in_progress = false;
                    continue;
                }
            };

            // 生成随机目标状态
            let goal_state = match generate_random_valid_state(&validity_checker, &space, &mut rng, 100) {
                Some(state) => state,
                None => {
                    println!("[Planner] Failed to generate valid goal state");
                    shared_command.lock().unwrap().planning_in_progress = false;
                    continue;
                }
            };

            // 规划路径
            match plan_path(&start_state, &goal_state, &validity_checker, &space) {
                Ok(path) => {
                    // 更新共享路径数据
                    let mut shared = shared_path.lock().unwrap();
                    shared.path = path;
                    shared.updated = true;
                    println!("[Planner] Path sent to viewer!");
                }
                Err(e) => {
                    eprintln!("[Planner] Planning failed: {} - showing start and goal only", e);
                    // 失败时也发送路径，只包含起点和终点
                    let mut shared = shared_path.lock().unwrap();
                    shared.path = vec![start_state.clone(), goal_state.clone()];
                    shared.updated = true;
                }
            }

            shared_command.lock().unwrap().planning_in_progress = false;
        } else {
            // 没有规划请求时，短暂休眠以避免忙等
            thread::sleep(Duration::from_millis(100));
        }
    }

    println!("[Planner] Planner thread exiting.");
}

/// 终端输入线程函数
/// 在子线程中处理用户的终端输入
fn input_thread_fn(
    shared_command: Arc<Mutex<SharedCommand>>,
    running: Arc<AtomicBool>,
) {
    println!("[Input] Input thread started.");

    while running.load(Ordering::Relaxed) {
        print!("Command (r/q): ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        if io::stdin().read_line(&mut input).is_err() {
            continue;
        }

        let command = input.trim().to_lowercase();

        match command.as_str() {
            "r" => {
                let mut cmd = shared_command.lock().unwrap();
                if cmd.planning_in_progress {
                    println!("[Input] Planning already in progress, please wait...");
                } else {
                    cmd.plan_requested = true;
                    println!("[Input] Planning request sent!");
                }
            }
            "q" => {
                println!("[Input] Quit requested. Shutting down...");
                running.store(false, Ordering::Relaxed);
                break;
            }
            "" => {
                // 空输入，继续
            }
            _ => {
                println!("[Input] Unknown command. Use 'r' to plan new path or 'q' to quit.");
            }
        }
    }

    println!("[Input] Input thread exiting.");
}

fn main() {
    // MJCF 路径（使用静态字符串以便传递给线程）
    const MJCF_PATH: &str = "/home/tans/tans_ws/AgileX/piper_sdk/piper_description/mujoco_model/piper_no_gripper_sim_scene.xml";

    // 6 自由度机械臂的关节空间（关节角限制，单位：弧度）
    // Piper 机械臂实际关节限制
    let joint_limits = vec![
        (-2.6179, 2.6179),   // joint1: [-150°, 150°]
        (0.0, 3.14),         // joint2: [0°, 180°]
        (-2.967, 0.0),       // joint3: [-170°, 0°]
        (-1.745, 1.745),     // joint4: [-100°, 100°]
        (-1.22, 1.22),       // joint5: [-70°, 70°]
        (-2.09439, 2.09439), // joint6: [-120°, 120°]
    ];
    
    // 保存一份用于规划线程
    let planner_joint_limits = joint_limits.clone();
    
    let new_rvss_result = RealVectorStateSpace::new(6, Some(joint_limits));
    let space = Arc::new(new_rvss_result.expect("Error creating new RealVectorState!"));

    // 共享的路径数据
    let shared_path = Arc::new(Mutex::new(SharedPathData::new()));
    
    // 共享的命令数据
    let shared_command = Arc::new(Mutex::new(SharedCommand::new()));
    
    // 运行标志
    let running = Arc::new(AtomicBool::new(true));

    // 启动规划线程
    let planner_space = Arc::clone(&space);
    let planner_shared_path = Arc::clone(&shared_path);
    let planner_shared_command = Arc::clone(&shared_command);
    let planner_running = Arc::clone(&running);
    let planner_handle = thread::spawn(move || {
        planner_thread_fn(MJCF_PATH, planner_joint_limits, planner_space, planner_shared_path, planner_shared_command, planner_running);
    });

    // 启动终端输入线程
    let input_shared_command = Arc::clone(&shared_command);
    let input_running = Arc::clone(&running);
    let _input_handle = thread::spawn(move || {
        input_thread_fn(input_shared_command, input_running);
    });

    // 主线程：运行 MuJoCo viewer（必须在主线程，因为 GUI 事件循环要求）
    let model = MjModel::from_xml(Path::new(MJCF_PATH)).expect("Could not load MuJoCo model");
    let mut viewer = MjViewer::launch_passive(&model, 0).expect("Could not launch the viewer");
    let mut data = model.make_data();

    let animation_delay = Duration::from_secs_f64(0.1);
    let mut local_path: Vec<RealVectorState> = Vec::new();
    let mut path_index = 0;

    println!();
    println!("RRT Motion Planning for Piper Robot Arm");
    println!("========================================");
    println!("Controls (in terminal):");
    println!("  r - Generate new random start/goal states and plan path");
    println!("  q - Quit");
    println!();
    println!("You can also rotate/zoom the MuJoCo viewer with mouse.");
    println!();

    while viewer.running() && running.load(Ordering::Relaxed) {
        // 检查是否有新路径
        {
            let mut shared = shared_path.lock().unwrap();
            if shared.updated {
                local_path = shared.path.clone();
                shared.updated = false;
                path_index = 0;
                println!("[Viewer] New path received ({} states). Starting playback...", local_path.len());
            }
        }

        // 播放路径动画
        if !local_path.is_empty() {
            let state = &local_path[path_index];
            
            // 设置关节位置
            let qpos = data.qpos_mut();
            for j in 0..6 {
                qpos[j] = state.values[j];
            }

            // 执行正向运动学
            data.forward();

            // 同步并渲染
            viewer.sync_data(&mut data);
            viewer.render();

            // 更新路径索引
            path_index = (path_index + 1) % local_path.len();

            // 如果播放完一轮，停顿一下
            if path_index == 0 {
                thread::sleep(Duration::from_millis(500));
            } else {
                thread::sleep(animation_delay);
            }
        } else {
            // 没有路径时，保持 viewer 响应
            viewer.sync_data(&mut data);
            viewer.render();
            thread::sleep(Duration::from_millis(50));
        }
    }

    // 通知其他线程退出
    running.store(false, Ordering::Relaxed);

    // 等待规划线程结束
    println!("Waiting for planner thread to finish...");
    let _ = planner_handle.join();
    
    // 注意：input_handle 可能会阻塞在 stdin，这里不等待它
    // 程序会在主线程结束时自动终止
    println!("Program finished.");
}