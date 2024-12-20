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

#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <unordered_set>
#include <vector>

#include "absl/strings/str_cat.h"

#include "modules/planning/planning_open_space/proto/planner_open_space_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"

namespace apollo {
namespace planning {

class Node2d {
 public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double cost) { cost_ = cost; }
  void SetDistanceToObstacle(const double dist) {
      distance_to_obstacle_ = dist;
  }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  double GetGridX() const { return grid_x_; }  // 获取节点的 x 方向格点坐标
  double GetGridY() const { return grid_y_; }  // 获取节点的 y 方向格点坐标
  double GetPathCost() const { return path_cost_; }  // 获取节点的路径代价
  double GetHeuCost() const { return heuristic_; }  // 获取节点的启发式代价
  double GetCost() const { return cost_; }  // 获取节点的总代价
  double GetDistanceToObstacle() const {  // 获取节点与障碍物的距离
      return distance_to_obstacle_;
  }
  const std::string& GetIndex() const { return index_; }  // 获取字符串类型的格子索引值
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }  // 获取此节点的上一个节点的指针
  static std::string CalcIndex(const double x, const double y,
                               const double xy_resolution,
                               const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid) {  // 私有成员函数, 输入两个整型, 拼接为类似 3_5 的形式
    return absl::StrCat(x_grid, "_", y_grid);  // 调用了另一个库的字符串拼接函数
  }

 private:  // 私有成员变量
  int grid_x_ = 0;  // 整型变量, 节点在 x 方向上的格点坐标
  int grid_y_ = 0;  // 整型变量, 节点在 y 方向上的格点坐标
  double path_cost_ = 0.0;  // 双精度浮点型, 路径代价
  double heuristic_ = 0.0;  // 双精度浮点型, 启发式代价
  double cost_ = 0.0;  // 节点整体代价
  double distance_to_obstacle_ = std::numeric_limits<double>::max();  // 双精度浮点型, 数据类型的最大值
  std::string index_;  // 节点的格点坐标的字符串表示
  std::shared_ptr<Node2d> pre_node_ = nullptr;  // 指向前一个节点的智能指针
};

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~GridSearch() = default;
  bool GenerateAStarPath(
      const double sx, const double sy, const double ex, const double ey,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::LineSegment2d>>&
          obstacles_linesegments_vec,
      GridAStartResult* result);
  bool GenerateDpMap(
          const double ex,
          const double ey,
          const std::vector<double>& XYbounds,
          const std::vector<std::vector<common::math::LineSegment2d>>&
              obstacles_linesegments_vec,
          const std::vector<std::vector<common::math::LineSegment2d>>&
              soft_boundary_linesegments_vec = {{}});
  double CheckDpMap(const double sx, const double sy);

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
  void LoadGridAStarResult(GridAStartResult* result);

 private:
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp {
      bool operator()(const std::pair<std::string,
                      double>& left,
                      const std::pair<std::string, double>& right) const {
          return left.second >= right.second;
      }
  };
  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;

  // park generic
 public:
  explicit GridSearch(const WarmStartConfig& warm_start_conf);
  double GetObstacleDistance(const double x, const double y);

 private:
  void AddSoftCost(
      const std::vector<std::vector<common::math::LineSegment2d>>&
      soft_boundary);
  void ExtendNode(
      const int& x,
      const int& y,
      common::math::Vec2d origin_index,
      std::unordered_set<std::string>& close_set);
  std::vector<std::vector<common::math::LineSegment2d>>
      soft_boundary_linesegments_vec_;
  double esdf_range_ = 0.0;
  bool use_esdf_ = false;
  std::vector<int> dx_{1, 0, -1, 0};
  std::vector<int> dy_{0, -1, 0, 1};
};
}  // namespace planning
}  // namespace apollo
