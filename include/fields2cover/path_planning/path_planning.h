//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#pragma once
#ifndef FIELDS2COVER_PATH_PLANNING_PATH_PLANNING_H_
#define FIELDS2COVER_PATH_PLANNING_PATH_PLANNING_H_

#include <utility>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <limits>

#include "fields2cover/types.h"
#include "fields2cover/path_planning/turning_base.h"

// 直接包含OMPL头文件
// 注意: 已在CMakeLists.txt中添加编译选项处理可变参数宏警告
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

//// 定义OMPL命名空间别名
//namespace ob = ompl::base;
//namespace og = ompl::geometric;

namespace f2c::pp {

/// Path planning class to connect a path using a TurningBase class method
class PathPlanning {
 public:
  /// Compute the coverage path using a route as a reference.
  static F2CPath planPath(const F2CRobot& robot, const F2CRoute& route,
      TurningBase& turn);

  static F2CPath planPath(const F2CField& field,const F2CRobot& robot, const F2CRoute& route,
      TurningBase& turn);

  /// Compute path that covers the swaths in order,
  /// using a path planner to connect swaths.
  static F2CPath planPath(const F2CRobot& robot, const F2CSwaths& swaths,
      TurningBase& turn);
      
  /// Compute path that covers the swaths in order with obstacle avoidance
  /// using a path planner to connect swaths.
  static F2CPath planPath(const F2CField& field, const F2CRobot& robot, const F2CSwaths& swaths,
      TurningBase& turn);

  /// Compute path that goes from the end of the last swath in a F2CSwaths
  /// to the beginning of the first one in another F2CSwaths.
  static F2CPath planPathForConnection(
      const F2CRobot& robot,
      const F2CSwaths& s1, const F2CMultiPoint& mp, const F2CSwaths& s2,
      TurningBase& turn);

  static F2CPath planPathForConnection(
      const F2CField& field,
      const F2CRobot& robot,
      const F2CSwaths& s1, const F2CMultiPoint& mp, const F2CSwaths& s2,
      TurningBase& turn);

  /// Compute path that connects two points with angles and visit middle points.
  static F2CPath planPathForConnection(const F2CRobot& robot,
      const F2CPoint& p1, double ang1,
      const F2CMultiPoint& mp,
      const F2CPoint& p2, double ang2,
      TurningBase& turn);

  static F2CPath planPathForConnection(
      const F2CField& field,
      const F2CRobot& robot,
      const F2CPoint& p1, double ang1,
      const F2CMultiPoint& mp,
      const F2CPoint& p2, double ang2,
      TurningBase& turn);

  /// 检测OMPL状态是否有效
  //static bool isStateValidOMPL(const ompl::base::State* state, const std::vector<F2CCell>& obstacles, double robot_width);
  static bool isStateValidOMPL(const ompl::base::State* state,
                     const F2CCell& shrunken_field_boundary,
                     const std::vector<F2CCell>& inflated_obstacles);


  /// 使用OMPL规划避障路径
  //static std::vector<F2CPoint> planPathWithOMPL(
  //    const F2CPoint& start_point,
  //    const F2CPoint& end_point,
  //    const std::vector<F2CCell>& obstacles,
  //    double robot_width,
  //    double planning_time = 1.0);

  static std::vector<F2CPoint> planPathWithOMPL(
      const F2CPoint& start_point,
      const F2CPoint& end_point,
      const F2CCell& field_boundary,
      const std::vector<F2CCell>& obstacles,
      double robot_width,
      double planning_time = 1.0);


 private:
  static double getSmoothTurningRadius(const F2CRobot& robot);
  static std::vector<std::pair<F2CPoint, double>> simplifyConnection(
      const F2CRobot& robot,
      const F2CPoint& p1, double ang1,
      const F2CMultiPoint& mp,
      const F2CPoint& p2, double ang2);

  /// 检测路径是否与障碍物相交
  static bool isPathIntersectObstacle(
      const F2CPoint& start_point, 
      const F2CPoint& end_point,
      const std::vector<F2CCell>& obstacles);
      
  /// 检测两条线段是否相交
  static bool doLineSegmentsIntersect(
      const F2CPoint& p1, 
      const F2CPoint& p2,
      const F2CPoint& p3, 
      const F2CPoint& p4);

  /// 使用A*算法规划绕过障碍物的路径
  static std::vector<F2CPoint> planPathWithAStar(
      const F2CPoint& start_point,
      const F2CPoint& end_point,
      const std::vector<F2CCell>& obstacles,
      double grid_size);

  /// 检测点是否在障碍物内部或太接近障碍物
  static bool isPointInObstacles(
      const F2CPoint& point,
      const std::vector<F2CCell>& obstacles,
      double safety_margin);
};

}  // namespace f2c::pp

#endif  // FIELDS2COVER_PATH_PLANNING_PATH_PLANNING_H_
