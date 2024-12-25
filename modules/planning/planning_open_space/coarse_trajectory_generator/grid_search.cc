/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "modules/planning/planning_open_space/coarse_trajectory_generator/grid_search.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

GridSearch::GridSearch(const PlannerOpenSpaceConfig& open_space_conf) {
  xy_grid_resolution_ =
      open_space_conf.warm_start_config().grid_a_star_xy_resolution();
  node_radius_ =
      open_space_conf.warm_start_config().node_radius();
}

double GridSearch::EuclidDistance(  // 计算两个点之间的欧氏距离
    const double x1, const double y1, const double x2, const double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();  // 获取待检查节点的 x 格点坐标
  const double node_grid_y = node->GetGridY();  // 获取待检查节点的 y 格点坐标
  if (node_grid_x > max_grid_x_ ||  // 如果 x 和 y 格点坐标超出了边界范围
      node_grid_x < 0  ||
      node_grid_y > max_grid_y_ ||
      node_grid_y < 0) {
    return false;  // 则约束检查不通过
  }
  if (obstacles_linesegments_vec_.empty()) {  // 如果障碍物线段数组是空的
    return true;  // 则约束检查通过
  }
  for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {  // 遍历每一个障碍物
    for (const common::math::LineSegment2d& linesegment :  // 遍历该障碍物的每一个线段
         obstacle_linesegments) {
      if (linesegment.DistanceTo({node->GetGridX(), node->GetGridY()})
          < node_radius_) {  // 如果当前节点到这条线段的距离小于节点半径
        return false;  // 则约束检查不通过
      }
    }
  }
  return true;  // 约束都通过, 最终约束检查通过
}

std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(  // 生成当前节点下一个节点的函数
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();  // 获取当前节点的 x 格点坐标
  double current_node_y = current_node->GetGridY();  // 获取当前节点的 y 格点坐标
  double current_node_path_cost = current_node->GetPathCost();  // 获取当前节点路径代价
  double diagonal_distance = std::sqrt(2.0);  // 对角距离为 根2
  std::vector<std::shared_ptr<Node2d>> next_nodes;  // 维护一个 Node2d 指针的数组来存储后继的所有节点
  std::shared_ptr<Node2d> up =  // [x, y+1] 节点
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);  // 上方节点的轨迹代价 + 1
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(  // [x+1, y+1] 节点
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);  // 右上方的节点路径代价 + 根2
  std::shared_ptr<Node2d> right =  // [x+1, y] 节点
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);  // 右侧的节点路径代价 + 1
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(  // [x+1, y-1] 节点
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);  // 右下方的节点路径代价 + 根2
  std::shared_ptr<Node2d> down =  // [x, y-1] 节点
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);  // 下方节点路径代价 + 1
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(  // [x-1, y-1] 节点
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);  // 左下节点路径代价 + 根2
  std::shared_ptr<Node2d> left =  // 【x-1, y] 节点
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);  // 左侧节点路径代价 + 1
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(  // [x-1, y+1] 节点
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);  // 左上节点路径代价 + 1

  next_nodes.emplace_back(up);  // 将当前节点周边的 8 个后继节点的指针都存储在数组中
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridSearch::GenerateAStarPath(
    const double sx, const double sy, const double ex, const double ey,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::LineSegment2d>>&
        obstacles_linesegments_vec,
    GridAStartResult* result) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
  XYbounds_ = XYbounds;
  std::shared_ptr<Node2d> start_node =
      std::make_shared<Node2d>(sx, sy, xy_grid_resolution_, XYbounds_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  std::shared_ptr<Node2d> final_node_ = nullptr;
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;
  open_set.emplace(start_node->GetIndex(), start_node);
  open_pq.emplace(start_node->GetIndex(), start_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    // Check destination
    if (*(current_node) == *(end_node)) {
      final_node_ = current_node;
      break;
    }
    close_set.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (close_set.find(next_node->GetIndex()) != close_set.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetHeuristic(
            EuclidDistance(next_node->GetGridX(), next_node->GetGridY(),
                           end_node->GetGridX(), end_node->GetGridY()));
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }

  if (final_node_ == nullptr) {
    AERROR << "Grid A searching return null ptr(open_set ran out)";
    return false;
  }
  LoadGridAStarResult(result);
  ADEBUG << "explored node num is " << explored_node_num;
  return true;
}

bool GridSearch::GenerateDpMap(
        const double ex,
        const double ey,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::LineSegment2d>>&
            obstacles_linesegments_vec,
        const std::vector<std::vector<common::math::LineSegment2d>>&
            soft_boundary_linesegments_vec) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;
  // XYbounds with xmin, xmax, ymin, ymax
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    dp_map_.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  ADEBUG << "explored node num is " << explored_node_num;
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
  std::string index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void GridSearch::LoadGridAStarResult(GridAStartResult* result) {  // 加载A*算法结果
  (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;  // 最后一个节点的路径代价 经过分辨率加权 得到最终的路径代价
  std::shared_ptr<Node2d> current_node = final_node_;  // 初始化当前节点为最后一个节点
  std::vector<double> grid_a_x;  // 存储路径点 x 坐标
  std::vector<double> grid_a_y;  // 存储路径点 y 坐标
  while (current_node->GetPreNode() != nullptr) {  // 如果当前节点不是第一个节点
    grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ +
                       XYbounds_[0]);  // 添加当前节点的实际 x 坐标到列表
    grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ +
                       XYbounds_[2]);  // 添加当前节点的实际 y 坐标到列表
    current_node = current_node->GetPreNode();  // 更新到前一个结点
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());  // 将路径点坐标顺序逆序，得到从起点到终点的路径
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);  // 更新原结果中的从终点到起点的逆序路径
  (*result).y = std::move(grid_a_y);
}
}  // namespace planning
}  // namespace apollo
