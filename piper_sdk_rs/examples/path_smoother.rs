//! 路径平滑和剪枝工具
//! 
//! 提供对 RRT 规划出的路径进行后处理的功能：
//! - 路径剪枝（shortcutting）：移除不必要的中间点
//! - 路径平滑（smoothing）：使路径更加连续平滑

use oxmpl::base::{
    state::RealVectorState,
    validity::StateValidityChecker,
};
use std::sync::Arc;

/// 路径平滑器
pub struct PathSmoother<C: StateValidityChecker<RealVectorState>> {
    /// 碰撞检测器
    validity_checker: Arc<C>,
    /// 插值步长（用于检测两点之间是否有碰撞）
    interpolation_step: f64,
}

impl<C: StateValidityChecker<RealVectorState>> PathSmoother<C> {
    /// 创建新的路径平滑器
    /// - `validity_checker`: 碰撞检测器
    /// - `interpolation_step`: 插值步长（弧度），越小越精确但越慢
    pub fn new(validity_checker: Arc<C>, interpolation_step: f64) -> Self {
        Self {
            validity_checker,
            interpolation_step,
        }
    }

    /// 检查两个状态之间的直线路径是否无碰撞
    fn is_path_valid(&self, from: &RealVectorState, to: &RealVectorState) -> bool {
        let n = from.values.len();
        
        // 计算两点之间的距离
        let dist: f64 = from.values.iter()
            .zip(to.values.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        
        // 计算需要插值的步数
        let steps = (dist / self.interpolation_step).ceil() as usize;
        if steps == 0 {
            return true;
        }
        
        // 检查插值点是否都有效
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let interpolated: Vec<f64> = (0..n)
                .map(|j| from.values[j] + t * (to.values[j] - from.values[j]))
                .collect();
            
            let state = RealVectorState { values: interpolated };
            if !self.validity_checker.is_valid(&state) {
                return false;
            }
        }
        
        true
    }

    /// 路径剪枝（Shortcutting）
    /// 
    /// 尝试移除不必要的中间点，直接连接更远的点
    /// 这会显著减少路径点数量
    pub fn shortcut(&self, path: &[RealVectorState]) -> Vec<RealVectorState> {
        if path.len() <= 2 {
            return path.to_vec();
        }
        
        let mut result = vec![path[0].clone()];
        let mut current_idx = 0;
        
        while current_idx < path.len() - 1 {
            // 尝试找到最远的可直接到达的点
            let mut farthest_valid = current_idx + 1;
            
            for next_idx in (current_idx + 2)..path.len() {
                if self.is_path_valid(&path[current_idx], &path[next_idx]) {
                    farthest_valid = next_idx;
                }
            }
            
            result.push(path[farthest_valid].clone());
            current_idx = farthest_valid;
        }
        
        result
    }

    /// 路径平滑（B-spline 风格的平滑）
    /// 
    /// 在相邻点之间插入平滑的中间点
    /// - `path`: 输入路径
    /// - `smoothing_factor`: 平滑因子（0-1），越大越平滑
    /// - `iterations`: 平滑迭代次数
    pub fn smooth(
        &self, 
        path: &[RealVectorState], 
        smoothing_factor: f64,
        iterations: usize,
    ) -> Vec<RealVectorState> {
        if path.len() <= 2 {
            return path.to_vec();
        }
        
        let mut smoothed: Vec<RealVectorState> = path.to_vec();
        let n = path[0].values.len();
        
        for _ in 0..iterations {
            let mut new_path = vec![smoothed[0].clone()];
            
            for i in 1..smoothed.len() - 1 {
                // 计算平滑后的位置（向相邻点的中点移动）
                let prev = &smoothed[i - 1];
                let curr = &smoothed[i];
                let next = &smoothed[i + 1];
                
                let smoothed_values: Vec<f64> = (0..n)
                    .map(|j| {
                        let midpoint = (prev.values[j] + next.values[j]) / 2.0;
                        curr.values[j] + smoothing_factor * (midpoint - curr.values[j])
                    })
                    .collect();
                
                let new_state = RealVectorState { values: smoothed_values };
                
                // 只有当新位置有效时才使用
                if self.validity_checker.is_valid(&new_state) {
                    // 同时检查与前后点的连接是否有效
                    if self.is_path_valid(&new_path.last().unwrap(), &new_state) {
                        new_path.push(new_state);
                        continue;
                    }
                }
                
                // 否则保持原位置
                new_path.push(curr.clone());
            }
            
            new_path.push(smoothed.last().unwrap().clone());
            smoothed = new_path;
        }
        
        smoothed
    }

    /// 完整的路径后处理流程
    /// 
    /// 1. 先剪枝减少点数
    /// 2. 再平滑使路径更连续
    /// 3. 最后插值生成执行用的密集路径
    pub fn process(
        &self,
        path: &[RealVectorState],
        smoothing_factor: f64,
        smoothing_iterations: usize,
        points_per_segment: usize,
    ) -> Vec<RealVectorState> {
        println!("Original path: {} states", path.len());
        
        // 第一步：剪枝
        let shortcut_path = self.shortcut(path);
        println!("After shortcut: {} states", shortcut_path.len());
        
        // 第二步：平滑
        let smoothed_path = self.smooth(&shortcut_path, smoothing_factor, smoothing_iterations);
        println!("After smooth: {} states", smoothed_path.len());
        
        // 第三步：插值（生成执行用的密集路径）
        let interpolated_path = self.interpolate(&smoothed_path, points_per_segment);
        println!("After interpolation: {} states", interpolated_path.len());
        
        interpolated_path
    }

    /// 在路径点之间进行插值，生成更密集的路径
    /// 用于最终执行时的平滑运动
    pub fn interpolate(&self, path: &[RealVectorState], points_per_segment: usize) -> Vec<RealVectorState> {
        if path.len() < 2 {
            return path.to_vec();
        }
        
        let n = path[0].values.len();
        let mut result = Vec::new();
        
        for i in 0..path.len() - 1 {
            let from = &path[i];
            let to = &path[i + 1];
            
            for step in 0..points_per_segment {
                let t = step as f64 / points_per_segment as f64;
                let interpolated: Vec<f64> = (0..n)
                    .map(|j| from.values[j] + t * (to.values[j] - from.values[j]))
                    .collect();
                result.push(RealVectorState { values: interpolated });
            }
        }
        
        // 添加最后一个点
        result.push(path.last().unwrap().clone());
        
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 简单的无碰撞检测器（总是返回 true）
    struct NoObstacleChecker;
    
    impl StateValidityChecker<RealVectorState> for NoObstacleChecker {
        fn is_valid(&self, _state: &RealVectorState) -> bool {
            true
        }
    }

    #[test]
    fn test_shortcut() {
        let checker = Arc::new(NoObstacleChecker);
        let smoother = PathSmoother::new(checker, 0.01);
        
        // 创建一条直线上的多个点
        let path: Vec<RealVectorState> = (0..10)
            .map(|i| RealVectorState { 
                values: vec![i as f64 * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0] 
            })
            .collect();
        
        let shortcut = smoother.shortcut(&path);
        
        // 应该被剪枝到只有起点和终点
        assert_eq!(shortcut.len(), 2);
    }
}
