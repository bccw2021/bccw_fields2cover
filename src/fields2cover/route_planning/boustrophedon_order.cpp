//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include "fields2cover/route_planning/boustrophedon_order.h"
#include <algorithm>
#include <vector>
#include <utility>
#include <map>
#include <set>
#include <limits>

namespace f2c::rp {

void BoustrophedonOrder::sortSwaths(F2CSwaths& swaths) const {}

////===============================================
//// 路径规划图数据结构
//// 定义节点类，表示条带的起点或终点
//class Node {
//public:
//  size_t swath_index;  // 条带索引
//  bool is_start;       // 是否是起点
//
//  Node() : swath_index(0), is_start(true) {}
//  Node(size_t idx, bool start) : swath_index(idx), is_start(start) {}
//
//  // 重载相等运算符，用于作为map的键
//  bool operator==(const Node& other) const {
//    return swath_index == other.swath_index && is_start == other.is_start;
//  }
//
//  // 重载小于运算符，用于作为map的键
//  bool operator<(const Node& other) const {
//    if (swath_index == other.swath_index) {
//      return is_start < other.is_start;
//    }
//    return swath_index < other.swath_index;
//  }
//};
//
//// 定义路径图类
//class Graph {
//public:
//  // 添加节点
//  void addNode(size_t swath_idx, bool is_start, const F2CPoint& position) {
//    Node node(swath_idx, is_start);
//    nodes[node] = position;
//  }
//
//  // 添加有向边
//  void addEdge(const Node& from, const Node& to, double distance) {
//    edges[from][to] = distance;
//  }
//
//  // 广度优先遍历找到从起点到终点的最短路径
//  std::vector<size_t> findOptimalPath() {
//    // 如果没有节点，返回空
//    if (nodes.empty()) {
//      return {};
//    }
//
//    // 用于记录最短路径的数组
//    std::vector<size_t> result;
//    std::set<size_t> included_swaths;
//
//    // 使用贪心算法找到自然合理的路径
//    Node current;
//    bool first = true;
//
//    while (included_swaths.size() < nodes.size() / 2) {
//      // 找到距离当前节点最近的下一个节点
//      double min_dist = std::numeric_limits<double>::max();
//      Node next;
//      bool found = false;
//
//      // 第一次循环，选择任意一个起点
//      if (first) {
//        for (const auto& entry : nodes) {
//          if (entry.first.is_start) {
//            current = entry.first;
//            included_swaths.insert(current.swath_index);
//            result.push_back(current.swath_index);
//            first = false;
//            break;
//          }
//        }
//        continue;
//      }
//
//      // 已经选择了起点，现在往前移动
//      for (const auto& edge_entry : edges[current]) {
//        const Node& candidate = edge_entry.first;
//        double dist = edge_entry.second;
//
//        // 如果这个条带还没有被包含
//        if (included_swaths.find(candidate.swath_index) == included_swaths.end()) {
//          if (dist < min_dist) {
//            min_dist = dist;
//            next = candidate;
//            found = true;
//          }
//        }
//      }
//
//      if (!found) {
//        // 如果没有找到连接的节点，选择任何未访问的条带
//        for (const auto& entry : nodes) {
//          const Node& candidate = entry.first;
//          if (candidate.is_start && 
//              included_swaths.find(candidate.swath_index) == included_swaths.end()) {
//            next = candidate;
//            found = true;
//            break;
//          }
//        }
//      }
//
//      if (found) {
//        current = next;
//        included_swaths.insert(current.swath_index);
//        result.push_back(current.swath_index);
//      } else {
//        // 如果所有节点都被访问了或没有可用连接，退出
//        break;
//      }
//    }
//
//    return result;
//  }
//
//private:
//  std::map<Node, F2CPoint> nodes;                     // 节点及其位置
//  std::map<Node, std::map<Node, double>> edges;      // 边及其权重
//};
//
////void BoustrophedonOrder::sortSwaths(F2CSwaths& swaths) const {
////  if (swaths.size() <= 1) {
////    return;
////  }
////  
////  // 第1步：将swaths按X坐标分组
////  // 首先，收集所有swath中心的X坐标
////  std::vector<std::pair<double, size_t>> swath_x_centers;
////  for (size_t i = 0; i < swaths.size(); ++i) {
////    double x_center = (swaths[i].startPoint().getX() + swaths[i].endPoint().getX()) / 2.0;
////    swath_x_centers.emplace_back(x_center, i);
////  }
////  
////  // 按X坐标对swaths排序
////  std::sort(swath_x_centers.begin(), swath_x_centers.end(),
////    [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
////      return a.first < b.first;
////    });
////  
////  // 第2步：估算条带间距得到X坐标组
////  double avg_x_spacing = 0.0;
////  if (swath_x_centers.size() > 1) {
////    for (size_t i = 1; i < swath_x_centers.size(); ++i) {
////      avg_x_spacing += std::abs(swath_x_centers[i].first - swath_x_centers[i-1].first);
////    }
////    avg_x_spacing /= (swath_x_centers.size() - 1);
////  }
////  
////  // 使用小的阈值以捕获微小的差异
////  double threshold = avg_x_spacing * 0.3;  // 使用平均间距的30%作为阈值
////  
////  // 第3步：将swaths分组
////  std::vector<std::vector<size_t>> x_groups;
////  double current_x = swath_x_centers[0].first;
////  std::vector<size_t> current_group;
////  
////  for (const auto& p : swath_x_centers) {
////    if (std::abs(p.first - current_x) > threshold) {
////      // 新组
////      if (!current_group.empty()) {
////        x_groups.push_back(current_group);
////        current_group.clear();
////      }
////      current_x = p.first;
////    }
////    current_group.push_back(p.second);
////  }
////  
////  // 添加最后一组
////  if (!current_group.empty()) {
////    x_groups.push_back(current_group);
////  }
////  
////  // 第4步：对每一组内部的swaths按Y坐标排序并确定每个swath的方向
////  F2CSwaths sorted_swaths;
////  // 注意: F2CSwaths 不支持 reserve
////  
////  // 用于存储每个swath排序后的索引和方向（正/反）
////  std::vector<std::pair<size_t, bool>> sorted_with_direction;
////  sorted_with_direction.reserve(swaths.size());
////  
////  // 遍历每个x坐标组
////  for (size_t g = 0; g < x_groups.size(); ++g) {
////    auto& group = x_groups[g];
////    
////    // 为条带组创建(y_center, swath_idx)对
////    std::vector<std::pair<double, size_t>> group_y_centers;
////    for (size_t swath_idx : group) {
////      double y_center = (swaths[swath_idx].startPoint().getY() + 
////                         swaths[swath_idx].endPoint().getY()) / 2.0;
////      group_y_centers.emplace_back(y_center, swath_idx);
////    }
////    
////    // 组内使用蛇形S型排序 - 确定正向/反向的排序方向
////    bool downward = (g % 2 == 0);  // 偶数组从下往上，奇数组从上往下
////    
////    // 对该组内的swaths按Y坐标排序
////    std::sort(group_y_centers.begin(), group_y_centers.end(),
////      [downward](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
////        if (downward) {
////          return a.first < b.first;  // 从下到上
////        } else {
////          return a.first > b.first;  // 从上到下
////        }
////      });
////    
////    // 确定当前组内每个swath的方向（考虑交替方向）
////    for (size_t i = 0; i < group_y_centers.size(); ++i) {
////      size_t swath_idx = group_y_centers[i].second;
////      
////      // 确定swath的起点和终点Y坐标
////      F2CSwath& current_swath = swaths[swath_idx];
////      double start_y = current_swath.startPoint().getY();
////      double end_y = current_swath.endPoint().getY();
////      
////      // 判断方向（正向/反向）
////      bool should_reverse = false;
////      
////      // 对于每个swath，确保其方向与该组内的行进方向一致
////      if (downward) {  // 从下往上
////        // 希望起点Y比终点Y小，这样才能符合从下往上的方向
////        should_reverse = (start_y > end_y);
////      } else {  // 从上往下
////        // 希望起点Y比终点Y大，这样才能符合从上往下的方向
////        should_reverse = (start_y < end_y);
////      }
////      
////      // 保存此swath索引和其是否需要翻转的信息
////      sorted_with_direction.emplace_back(swath_idx, should_reverse);
////    }
////  }
////  
////  // 第5步：根据确定的顺序和方向创建最终的swaths列表
////  for (const auto& [swath_idx, should_reverse] : sorted_with_direction) {
////    F2CSwath swath = swaths[swath_idx];
////    if (should_reverse) {
////      swath.reverse();
////    }
////    sorted_swaths.push_back(swath);
////  }
////  
////  // 第6步：确保相邻swaths连接的顺序性
////  // 当前已按照从左到右的顺序排列了swath，并且确保了每组内的方向一致性
////  // 交替行的连接点应该在相邻X组的边界上
////  
////  // 替换原始的swaths列表
////  swaths = std::move(sorted_swaths);
////}
//
//void BoustrophedonOrder::sortSwaths(F2CSwaths& swaths) const {
//  if (swaths.size() <= 1) {
//    return;
//  }
//  
//  // Step 1: 计算每个swath的坐标和判断其主要方向
//  std::vector<std::tuple<double, double, size_t, bool>> swath_data; // x, y, index, is_vertical
//  for (size_t i = 0; i < swaths.size(); ++i) {
//    const F2CSwath& swath = swaths[i];
//    double x1 = swath.startPoint().getX();
//    double y1 = swath.startPoint().getY();
//    double x2 = swath.endPoint().getX();
//    double y2 = swath.endPoint().getY();
//    
//    // 计算中心点
//    double x_center = (x1 + x2) / 2.0;
//    double y_center = (y1 + y2) / 2.0;
//    
//    // 判断是否是垂直主导方向的条带
//    bool is_vertical = std::abs(y2 - y1) > std::abs(x2 - x1);
//    
//    swath_data.emplace_back(x_center, y_center, i, is_vertical);
//  }
//  
//  // Step 2: 计算X方向的平均间距
//  std::vector<double> x_values;
//  for (const auto& [x, y, idx, is_vert] : swath_data) {
//    x_values.push_back(x);
//  }
//  std::sort(x_values.begin(), x_values.end());
//  
//  double avg_x_spacing = 0.0;
//  if (x_values.size() > 1) {
//    for (size_t i = 1; i < x_values.size(); ++i) {
//      avg_x_spacing += std::abs(x_values[i] - x_values[i-1]);
//    }
//    avg_x_spacing /= (x_values.size() - 1);
//  }
//  
//  // 使用30%平均X间距作为阈值
//  double x_threshold = avg_x_spacing * 0.3;
//  
//  // Step 3: 按X坐标将swaths分组
//  std::vector<std::vector<size_t>> x_groups;
//  std::sort(swath_data.begin(), swath_data.end(),
//    [](const auto& a, const auto& b) {
//      return std::get<0>(a) < std::get<0>(b);
//    });
//  
//  if (!swath_data.empty()) {
//    double current_x = std::get<0>(swath_data[0]);
//    std::vector<size_t> current_group;
//    
//    for (const auto& [x, y, idx, is_vert] : swath_data) {
//      if (std::abs(x - current_x) > x_threshold) {
//        if (!current_group.empty()) {
//          x_groups.push_back(current_group);
//          current_group.clear();
//        }
//        current_x = x;
//      }
//      current_group.push_back(idx);
//    }
//    
//    // 添加最后一组
//    if (!current_group.empty()) {
//      x_groups.push_back(current_group);
//    }
//  }
//  
//  // Step 4: 对每组内部严格按Y坐标排序
//  for (size_t g = 0; g < x_groups.size(); ++g) {
//    auto& group = x_groups[g];
//    
//    // 按Y坐标排序
//    std::sort(group.begin(), group.end(), 
//      [&swath_data, g](size_t idx1, size_t idx2) {
//        auto it1 = std::find_if(swath_data.begin(), swath_data.end(),
//          [idx1](const auto& item) { return std::get<2>(item) == idx1; });
//        auto it2 = std::find_if(swath_data.begin(), swath_data.end(),
//          [idx2](const auto& item) { return std::get<2>(item) == idx2; });
//        
//        double y1 = std::get<1>(*it1);
//        double y2 = std::get<1>(*it2);
//        
//        // 偶数组从下到上，奇数组从上到下
//        return (g % 2 == 0) ? y1 < y2 : y1 > y2;
//      });
//  }
//  
//  // Step 5: 创建全新的排序结果
//  F2CSwaths sorted_swaths;
//  std::vector<F2CSwath> temp_swaths;
//  
//  // 首先确定每个swath的方向
//  for (size_t g = 0; g < x_groups.size(); ++g) {
//    bool up_direction = (g % 2 == 0);  // 偶数组从下往上，奇数组从上往下
//    
//    for (size_t i = 0; i < x_groups[g].size(); ++i) {
//      size_t idx = x_groups[g][i];
//      F2CSwath swath = swaths[idx];
//      
//      // 确定是否需要翻转swath方向
//      double start_y = swath.startPoint().getY();
//      double end_y = swath.endPoint().getY();
//      bool swath_goes_up = (end_y > start_y);
//      
//      // 如果要向上走但swath向下，或者要向下走但swath向上，则需要翻转
//      if ((up_direction && !swath_goes_up) || (!up_direction && swath_goes_up)) {
//        swath.reverse();
//      }
//      
//      temp_swaths.push_back(swath);
//    }
//  }
//  
//  // Step 6: 连接swaths确保首尾相连
//  if (!temp_swaths.empty()) {
//    sorted_swaths.push_back(temp_swaths[0]);
//    
//    for (size_t i = 1; i < temp_swaths.size(); ++i) {
//      F2CSwath& prev = sorted_swaths.back();
//      F2CSwath current = temp_swaths[i];
//      
//      // 计算前一个swath的终点与当前swath的起点终点的距离
//      double dist_to_start = prev.endPoint().distance(current.startPoint());
//      double dist_to_end = prev.endPoint().distance(current.endPoint());
//      
//      // 如果到终点的距离更小，需要翻转swath
//      if (dist_to_end < dist_to_start) {
//        current.reverse();
//      }
//      
//      sorted_swaths.push_back(current);
//    }
//  }
//  
//  // 用排序后的swaths替换原始swaths
//  swaths = sorted_swaths;
//
//  ////// 删除不需要的swath
//  ////if (swaths.size() > 1) {
//  //    // 创建一个新的F2CSwaths对象，仅包含我们想要保留的元素
//  //    F2CSwaths filtered_swaths;
//
//  //    // 仅保留中间的swath（跳过第一条和最后一条）
//  //    for (size_t i = 5; i < swaths.size() - 5; ++i) {
//  //        filtered_swaths.emplace_back(swaths[i]);
//  //    }
//
//  //    // 替换原来的swaths
//  //    swaths = std::move(filtered_swaths);
//  ////}
//
//}
////=============================================


}  // namespace f2c::rp

