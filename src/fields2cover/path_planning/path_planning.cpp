//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

// 包含转向函数工具库
#include <steering_functions/utilities/utilities.hpp>
// 包含路径规划头文件
#include "fields2cover/path_planning/path_planning.h"
// 包含几何计算库
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include "clipper2/clipper.h"



/*
  // turn_point_dist已在Field2Cover 2.0 API中删除，并改用静态方法
  return f2c::pp::PathPlanning::planPath(robot_params_->getRobot(), swaths, *curve);
*/

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 CGALPoint;
typedef K::Segment_2 CGALSegment;


namespace f2c::pp {

F2CPath PathPlanning::planPath(const F2CRobot& robot,
    const F2CRoute& route, TurningBase& turn) {
  F2CPath path;
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    auto prev_swaths = (i >0) ? route.getSwaths(i-1) : F2CSwaths();
    path += planPathForConnection(robot, prev_swaths, route.getConnection(i), route.getSwaths(i), turn);
    path += planPath(robot, route.getSwaths(i), turn);
  }

  if (route.sizeConnections() > route.sizeVectorSwaths()) {
    path += planPathForConnection(robot, route.getLastSwaths(), route.getLastConnection(), F2CSwaths(), turn);
  }
  
  return path;
}

// 使用OMPL规划避障路径
std::vector<F2CPoint> PathPlanning::planPathWithOMPL(
    const F2CPoint& start_point,
    const F2CPoint& end_point,
    const std::vector<F2CCell>& obstacles,
    double robot_width,
    double planning_time) {  // 规划时间
    
    std::cout << "[OMPL规划] 开始使用OMPL规划避障路径" << std::endl;
    std::cout << "[OMPL规划] 起点: (" << start_point.getX() << ", " << start_point.getY() << ")" << std::endl;
    std::cout << "[OMPL规划] 终点: (" << end_point.getX() << ", " << end_point.getY() << ")" << std::endl;
    
    // 创建OMPL状态空间 (2D)
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
    
    // 计算状态空间边界 (基于起点和终点位置，加上足够的边界)
    double min_x = std::min(start_point.getX(), end_point.getX()) - 20.0;
    double max_x = std::max(start_point.getX(), end_point.getX()) + 20.0;
    double min_y = std::min(start_point.getY(), end_point.getY()) - 20.0;
    double max_y = std::max(start_point.getY(), end_point.getY()) + 20.0;
    
    // 设置状态空间边界
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);
    space->setBounds(bounds);
    
    // 创建简单设置对象
    ompl::geometric::SimpleSetup ss(space);
    
    // 设置状态有效性检查器
    double safety_margin = robot_width * 1.5;  // 使用与A*相同的安全距离计算
    ss.setStateValidityChecker(
        [&obstacles, safety_margin](const ompl::base::State* state) {
            return PathPlanning::isStateValidOMPL(state, obstacles, safety_margin);
        }
    );
    
    // 定义起点
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = start_point.getX();
    start[1] = start_point.getY();
    
    // 定义终点
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = end_point.getX();
    goal[1] = end_point.getY();
    
    // 设置起点和终点
    ss.setStartAndGoalStates(start, goal);
    
    // 设置规划器 (使用RRT*，更优质的路径)
    auto planner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
    ss.setPlanner(planner);
    
    // 尝试解决问题
    std::cout << "[OMPL规划] 开始规划 (最大允许" << planning_time << "秒)" << std::endl;
    ompl::base::PlannerStatus solved = ss.solve(planning_time);
    
    // 检查是否找到解决方案
    if (solved) {
        std::cout << "[OMPL规划] 成功找到路径!" << std::endl;
        
        // 获取路径
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        
        // 优化路径，使其更平滑
        std::cout << "[OMPL规划] 优化路径..." << std::endl;
        ss.simplifySolution();
        path = ss.getSolutionPath();
        
        // 对路径进行插值，使其更平滑
        path.interpolate(50);
        
        // 将OMPL路径转换为F2CPoint的向量
        std::vector<F2CPoint> result_path;
        
        // 确保第一个点是准确的起点
        result_path.push_back(start_point);
        
        // 添加中间点
        for (size_t i = 1; i < path.getStateCount() - 1; ++i) {
            const auto* state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            result_path.push_back(F2CPoint(state->values[0], state->values[1]));
        }
        
        // 确保最后一个点是准确的终点
        result_path.push_back(end_point);
        
        std::cout << "[OMPL规划] 生成路径共有" << result_path.size() << "个点" << std::endl;
        return result_path;
    }
    else {
        std::cout << "[OMPL规划] 未能找到路径" << std::endl;
        return {};
    }
}


// OMPL状态有效性检查函数（用于避开障碍物）
bool PathPlanning::isStateValidOMPL(const ompl::base::State* state, const std::vector<F2CCell>& obstacles, double robot_width) {
  // 获取状态空间中的位置
  const ompl::base::RealVectorStateSpace::StateType* pos = 
      state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  // 提取坐标
  double x = pos->values[0];
  double y = pos->values[1];
  
  // 创建一个点
  F2CPoint point(x, y);
  
  // 表示当前机器人位置的点
  Clipper2Lib::PointD robot_point(x, y);
  
  // 创建以机器人为中心的圆形区域（用多边形近似）
  Clipper2Lib::PathD robot_circle;
  double safe_distance = robot_width / 2.0; // 安全距离为机器人半宽度
  const int circle_points = 12; // 圆的近似多边形点数
  
  // 创建多边形近似的圆
  for (int i = 0; i < circle_points; ++i) {
    double angle = 2.0 * M_PI * i / circle_points;
    double px = x + safe_distance * std::cos(angle);
    double py = y + safe_distance * std::sin(angle);
    robot_circle.push_back(Clipper2Lib::PointD(px, py));
  }
  // 确保闭合
  robot_circle.push_back(robot_circle[0]);
  
  // 将圆形区域转换为路径
  Clipper2Lib::PathsD subject = {robot_circle};
  
  // 检查每个障碍物
  for (const auto& obstacle : obstacles) {
    // 检查点是否在障碍物内部（使用F2CCell自带的包含检测）
    //if (obstacle.contains(point)) {
    if (obstacle.isPointIn(point)) {
      return false;
    }
    
    // 使用Clipper2创建障碍物的表示
    Clipper2Lib::PathsD clip;
    Clipper2Lib::PathD exterior_points;
    
    // 提取障碍物外环坐标
    F2CLinearRing exterior_ring = obstacle.getExteriorRing();
    
    for (size_t i = 0; i < exterior_ring.size(); ++i) {
      F2CPoint p = exterior_ring.getGeometry(i);
      exterior_points.push_back({p.getX(), p.getY()});
    }
    
    // 确保多边形闭合（首尾点相同）
    if (exterior_ring.size() > 0 && 
        (exterior_points[0].x != exterior_points.back().x || 
         exterior_points[0].y != exterior_points.back().y)) {
      exterior_points.push_back(exterior_points[0]);
    }
    
    clip.push_back(exterior_points);
    
    // 处理障碍物的内环（洞）
    for (size_t j = 0; j < obstacle.size() - 1; ++j) {
      Clipper2Lib::PathD interior_points;
      F2CLinearRing interior_ring = obstacle.getInteriorRing(j);
      
      for (size_t i = 0; i < interior_ring.size(); ++i) {
        F2CPoint p = interior_ring.getGeometry(i);
        interior_points.push_back({p.getX(), p.getY()});
      }
      
      // 确保内环闭合
      if (interior_ring.size() > 0 && 
          (interior_points[0].x != interior_points.back().x || 
           interior_points[0].y != interior_points.back().y)) {
        interior_points.push_back(interior_points[0]);
      }
      
      if (!interior_points.empty()) {
        clip.push_back(interior_points);
      }
    }
    
    // 计算机器人圆形区域与障碍物的交集
    Clipper2Lib::PathsD solution = 
      Clipper2Lib::Intersect(
        subject,                // 机器人圆形区域
        clip,                   // 障碍物多边形
        Clipper2Lib::FillRule::NonZero  // 非零填充规则
      );
    
    // 如果存在交集，表示机器人太靠近障碍物
    if (!solution.empty()) {
      return false;
    }
  }
  
  return true;
}

F2CPath PathPlanning::planPath(const F2CField& field, const F2CRobot& robot, const F2CRoute& route, TurningBase& turn) {
  std::cout << "=========================================="  << std::endl;
  std::cout << "[路径规划] 开始为路线生成完整路径" << std::endl;
  std::cout << "[路径规划] 路线包含" << route.sizeVectorSwaths() << "组作业带，" 
            << route.sizeConnections() << "个连接" << std::endl;
            
  // 创建空路径对象用于存储结果
  F2CPath path;
  
  // 提取字段中的障碍物
  std::vector<F2CCell> obstacles;
  for (size_t i = 0; i < field.getField().size(); ++i) {
    const F2CCell& cell = field.getField().getCell(i);
    // 处理内部环（障碍物）
    int ring_count = cell.size() - 1;  // 总环数减去外环
    
    for (size_t j = 0; j < ring_count; ++j) {
      F2CLinearRing ring = cell.getInteriorRing(j);
      F2CCell obstacle;
      obstacle.addRing(ring);
      obstacles.push_back(obstacle);
    }
  }
  
  if (!obstacles.empty()) {
    std::cout << "[路径规划] 检测到" << obstacles.size() << "个障碍物" << std::endl;
  }
  
  // 遍历路线中的所有作业带向量
  for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
    // 获取前一组作业带，如果是第一组则为空
    auto prev_swaths = (i >0) ? route.getSwaths(i-1) : F2CSwaths();
    
    // 添加从前一组作业带到当前作业带的连接路径
    F2CPath connectionPath = planPathForConnection(field, robot, prev_swaths, route.getConnection(i), route.getSwaths(i), turn);
    
    // 如果存在障碍物，检查连接路径是否与障碍物相交
    if (!obstacles.empty() && connectionPath.size() >= 2) {
      std::cout << "[路径规划] 检查连接路径" << i << "是否与障碍物相交..." << std::endl;
      
      bool hasIntersection = false;
      
      // 定义参数
      const double MIN_SEGMENT_LENGTH = 0.1;   // 小线段的定义阈值
      const double MAX_SEGMENT_LENGTH = 100.0;  // 合并后最大线段长度
      const double DIRECTION_THRESHOLD = 0.1;  // 方向相似性阈值（张角的余弦值）
      
      // 简化的路径段
      std::vector<std::pair<F2CPoint, F2CPoint>> optimizedSegments;
      
      // 先判断是否需要详细检测
      bool skipDetailed = false;
      
      // 先尝试直接连接路径的起点和终点
      F2CPoint path_start = connectionPath[0].point;
      F2CPoint path_end = connectionPath[connectionPath.size()-1].atEnd();
      double total_path_length = path_start.distance(path_end);
      
      // 如果总路径长度满足最小阈值，尝试这种简化方式
      if (total_path_length >= MIN_SEGMENT_LENGTH && total_path_length <= MAX_SEGMENT_LENGTH) {
        std::cout << "[障碍检测] 尝试使用路径的起终点直接检测: "
                  << "(" << path_start.getX() << ", " << path_start.getY() << ") -> "
                  << "(" << path_end.getX() << ", " << path_end.getY() << ")"
                  << " 长度: " << total_path_length << std::endl;
        
        if (isPathIntersectObstacle(path_start, path_end, obstacles)) {
          hasIntersection = true;
          std::cout << "[障碍检测] 路径起终点连线与障碍物相交" << std::endl;
        } else {
          // 如果没有交叉，直接使用这一条线段
          std::cout << "[障碍检测] 路径起终点连线没有与障碍物相交，跳过详细检测" << std::endl;
          skipDetailed = true;
        }
      }
      
      // 详细检测 - 合并相似方向的小线段
      if (!skipDetailed) {
        std::cout << "[障碍检测] 开始合并相似方向的小线段..." << std::endl;
        
        // 初始化当前线段
        F2CPoint currentStart = connectionPath[0].atEnd();
        F2CPoint currentEnd = connectionPath[1].point;
        double currentAngle = connectionPath[0].angle;
        
        // 如果第一个线段无效，直接使用第一个点
        if (currentStart.distance(currentEnd) < 1e-6) {
          currentStart = connectionPath[0].point;
        }
      
        for (size_t j = 1; j < connectionPath.size() - 1; ++j) {
          // 使用PathState中的现有属性获取起点、终点和角度
          F2CPoint nextStart = connectionPath[j].atEnd();
          F2CPoint nextEnd = connectionPath[j+1].point;
          double nextAngle = connectionPath[j].angle;
          
          // 检查是否为有效线段
          double nextSegLength = nextStart.distance(nextEnd);
          if (nextSegLength < 1e-6) continue;
          
          // 根据角度差异计算方向相似性
          double angleDiff = std::abs(nextAngle - currentAngle);
          while (angleDiff > M_PI) angleDiff = 2*M_PI - angleDiff; // 角度差值标准化到 [0, PI]
          bool isSimilarDirection = angleDiff < DIRECTION_THRESHOLD * M_PI;
          
          // 如果方向相似且合并后的线段长度不超过最大限制
          if (isSimilarDirection && currentStart.distance(nextEnd) <= MAX_SEGMENT_LENGTH) {
            // 更新当前线段的终点
            currentEnd = nextEnd;
          } else {
            // 方向变化较大或达到最大长度，保存当前线段
            double segmentLength = currentStart.distance(currentEnd);
            if (segmentLength >= MIN_SEGMENT_LENGTH) {
              optimizedSegments.push_back(std::make_pair(currentStart, currentEnd));
              std::cout << "[障碍检测] 添加合并线段: "
                        << "(" << currentStart.getX() << ", " << currentStart.getY() << ") -> "
                        << "(" << currentEnd.getX() << ", " << currentEnd.getY() << ")"
                        << " 长度: " << segmentLength << std::endl;
            }
            
            // 开始新线段
            currentStart = nextStart;
            currentEnd = nextEnd;
            currentAngle = nextAngle;
          }
        }
        // 处理最后一个线段
        double finalSegmentLength = currentStart.distance(currentEnd);
        if (finalSegmentLength >= MIN_SEGMENT_LENGTH) {
          optimizedSegments.push_back(std::make_pair(currentStart, currentEnd));
          std::cout << "[障碍检测] 添加最后的合并线段: "
                    << "(" << currentStart.getX() << ", " << currentStart.getY() << ") -> "
                    << "(" << currentEnd.getX() << ", " << currentEnd.getY() << ")"
                    << " 长度: " << finalSegmentLength << std::endl;
        }
        
        // 如果没有生成任何有效线段，使用原始的起终点
        if (optimizedSegments.empty()) {
          F2CPoint first = connectionPath[0].point;
          F2CPoint last = connectionPath[connectionPath.size()-1].atEnd();
          double fullPathLength = first.distance(last);
          
          if (fullPathLength >= MIN_SEGMENT_LENGTH) {
            std::cout << "[障碍检测] 没有生成有效线段，使用原始起终点: "
                      << "(" << first.getX() << ", " << first.getY() << ") -> "
                      << "(" << last.getX() << ", " << last.getY() << ")"
                      << " 长度: " << fullPathLength << std::endl;
            optimizedSegments.push_back(std::make_pair(first, last));
          }
        }
        
        // 检查合并后的线段是否与障碍物相交
        std::cout << "[障碍检测] 共有" << optimizedSegments.size() << "个合并线段需要检测" << std::endl;
        
        for (size_t j = 0; j < optimizedSegments.size(); ++j) {
          F2CPoint start_point = optimizedSegments[j].first;
          F2CPoint end_point = optimizedSegments[j].second;
          double length = start_point.distance(end_point);
          
          std::cout << "[障碍检测] 检查合并线段 " << j << ": "
                    << "(" << start_point.getX() << ", " << start_point.getY() << ") -> "
                    << "(" << end_point.getX() << ", " << end_point.getY() << ")"
                    << " 长度: " << length << std::endl;
          
          if (isPathIntersectObstacle(start_point, end_point, obstacles)) {
            hasIntersection = true;
            std::cout << "[障碍检测] 合并线段" << j << "与障碍物相交" << std::endl;
            break;
          }
        }
      } // 关闭 !skipDetailed 分支
      
      // 如果连接路径与障碍物相交，使用OMPL算法重新规划
      if (hasIntersection && connectionPath.size() >= 2) {
        std::cout << "[障碍检测] 使用OMPL算法重新规划连接路径" << std::endl;
        
        // 获取连接路径的起点和终点
        F2CPoint start_point = connectionPath[0].point;
        F2CPoint end_point = connectionPath[connectionPath.size()-1].atEnd();
        
        // 使用OMPL规划避障路径
        auto ompl_points = planPathWithOMPL(start_point, end_point, obstacles, robot.getWidth(), 2.0);
        
        // 检查OMPL是否成功找到路径
        if (!ompl_points.empty()) {
          std::cout << "[重规划] OMPL算法成功找到" << ompl_points.size() << "个安全点" << std::endl;
          
          // 使用OMPL生成的安全点创建新的连接路径
          F2CPath newConnectionPath;
          
          // 获取原始连接路径的起始和结束角度
          double start_angle = connectionPath[0].angle;
          double end_angle = connectionPath[connectionPath.size()-1].angle;
          
          // 添加起点
          F2CPathState start_state;
          start_state.point = start_point;
          start_state.angle = start_angle;
          start_state.type = connectionPath[0].type;
          start_state.dir = connectionPath[0].dir;
          newConnectionPath.addState(start_state);
          
          // 添加中间点
          for (size_t j = 1; j < ompl_points.size() - 1; ++j) {
            // 计算当前点到下一个点的角度，作为当前点的角度
            double angle = std::atan2(ompl_points[j+1].getY() - ompl_points[j].getY(), 
                                   ompl_points[j+1].getX() - ompl_points[j].getX());
            
            F2CPathState state;
            state.point = ompl_points[j];
            state.angle = angle;
            //state.type = f2c::types::PathType::RP_CUSTOM;
            //state.dir = f2c::types::PathDirection::FORWARD;
            state.type = connectionPath[j].type;
            state.dir = connectionPath[j].dir;
            
            // 计算长度
            if (j > 0) {
              double len = ompl_points[j].distance(ompl_points[j-1]);
              state.len = len;
            }
            
            newConnectionPath.addState(state);
          }
          
          // 添加终点
          F2CPathState end_state;
          end_state.point = end_point;
          end_state.angle = end_angle;
          end_state.type = connectionPath[connectionPath.size()-1].type;
          end_state.dir = connectionPath[connectionPath.size()-1].dir;
          
          // 计算最后一段长度
          if (ompl_points.size() > 1) {
            double len = end_point.distance(ompl_points[ompl_points.size()-2]);
            end_state.len = len;
          }
          
          newConnectionPath.addState(end_state);
          
          // 使用新的连接路径替换原有的连接路径
          connectionPath = newConnectionPath;
          std::cout << "[重规划] 成功使用OMPL替换原有连接路径" << i << std::endl;
        } else {
          std::cout << "[重规划] OMPL规划失败，使用原始连接路径" << std::endl;
        }
      }
    }
    
    // 将处理后的连接路径添加到总路径中
    path += connectionPath;
    
    // 添加当前作业带内的路径
    F2CPath swathPath = planPath(field, robot, route.getSwaths(i), turn);
    path += swathPath;
  }
  
  // 如果连接数量大于作业带向量数量，添加最后一个连接
  if (route.sizeConnections() > route.sizeVectorSwaths()) {
    F2CPath lastConnectionPath = planPathForConnection(field, robot, route.getLastSwaths(), route.getLastConnection(), F2CSwaths(), turn);
    
    // 如果存在障碍物，检查最后一个连接路径是否与障碍物相交
    if (!obstacles.empty() && lastConnectionPath.size() >= 2) {
      std::cout << "[路径规划] 检查最后一个连接路径是否与障碍物相交..." << std::endl;
      
      bool hasIntersection = false;
      
      // 检查连接路径中的每个相邻点对形成的线段
      for (size_t j = 0; j < lastConnectionPath.size() - 1; ++j) {
        F2CPoint start_point = lastConnectionPath[j].atEnd();
        F2CPoint end_point = lastConnectionPath[j+1].point;
        
        // 如果起点和终点几乎重合，跳过检查
        if (start_point.distance(end_point) < 1e-6) {
          continue;
        }
        
        // 检查线段是否与任何障碍物相交
        if (isPathIntersectObstacle(start_point, end_point, obstacles)) {
          hasIntersection = true;
          std::cout << "[障碍检测] 最后一个连接路径与障碍物相交" << std::endl;
          std::cout << "[障碍检测] 路径段" << j << "-" << j+1 
                    << "从(" << start_point.getX() << ", " << start_point.getY() << ")" 
                    << "到(" << end_point.getX() << ", " << end_point.getY() << ")" << std::endl;
          break;
        }
      }
      
      // 如果连接路径与障碍物相交，使用OMPL算法重新规划
      if (hasIntersection && lastConnectionPath.size() >= 2) {
        std::cout << "[障碍检测] 使用OMPL算法重新规划最后一个连接路径" << std::endl;
        
        // 获取连接路径的起点和终点
        F2CPoint start_point = lastConnectionPath[0].point;
        F2CPoint end_point = lastConnectionPath[lastConnectionPath.size()-1].atEnd();
        
        // 使用OMPL规划避障路径
        auto ompl_points = planPathWithOMPL(start_point, end_point, obstacles, robot.getWidth(), 2.0);
        
        if (!ompl_points.empty()) {
          std::cout << "[重规划] OMPL算法成功找到" << ompl_points.size() << "个安全点" << std::endl;
          
          // 使用OMPL生成的安全点创建新的连接路径
          F2CPath newLastConnectionPath;
          
          // 获取原始连接路径的起始和结束角度
          double start_angle = lastConnectionPath[0].angle;
          double end_angle = lastConnectionPath[lastConnectionPath.size()-1].angle;
          
          // 添加起点
          F2CPathState start_state;
          start_state.point = start_point;
          start_state.angle = start_angle;
          start_state.type = lastConnectionPath[0].type;
          start_state.dir = lastConnectionPath[0].dir;
          newLastConnectionPath.addState(start_state);
          
          // 添加中间点
          for (size_t j = 1; j < ompl_points.size() - 1; ++j) {
            // 计算当前点到下一个点的角度，作为当前点的角度
            double angle = std::atan2(ompl_points[j+1].getY() - ompl_points[j].getY(), 
                                   ompl_points[j+1].getX() - ompl_points[j].getX());
            
            F2CPathState state;
            state.point = ompl_points[j];
            state.angle = angle;
            //state.type = f2c::types::PathType::RP_CUSTOM;
            //state.dir = f2c::types::PathDirection::FORWARD;
            state.type = lastConnectionPath[j].type;
            state.dir = lastConnectionPath[j].dir;
            
            // 计算长度
            if (j > 0) {
              double len = ompl_points[j].distance(ompl_points[j-1]);
              state.len = len;
            }
            
            newLastConnectionPath.addState(state);
          }
          
          // 添加终点
          F2CPathState end_state;
          end_state.point = end_point;
          end_state.angle = end_angle;
          end_state.type = lastConnectionPath[lastConnectionPath.size()-1].type;
          end_state.dir = lastConnectionPath[lastConnectionPath.size()-1].dir;
          
          // 计算最后一段长度
          if (ompl_points.size() > 1) {
            double len = end_point.distance(ompl_points[ompl_points.size()-2]);
            end_state.len = len;
          }
          
          newLastConnectionPath.addState(end_state);
          
          // 使用新的连接路径替换原有的连接路径
          lastConnectionPath = newLastConnectionPath;
          std::cout << "[重规划] 成功使用OMPL替换原有最后一个连接路径" << std::endl;
        } else {
          std::cout << "[重规划] OMPL规划失败，使用原始连接路径" << std::endl;
        }
      }
    }
    
    // 将处理后的最后一个连接路径添加到总路径中
    path += lastConnectionPath;
  }
  
  std::cout << "[路径规划] 完整路径生成完成，共" << path.size() << "个路径点" << std::endl;
  std::cout << "=========================================="  << std::endl;
  
  return path;
}



// 为一组作业带规划路径，包括作业带之间的转弯
// 这个函数处理单个作业带组内的路径规划
F2CPath PathPlanning::planPath(const F2CRobot& robot, const F2CSwaths& swaths, TurningBase& turn) {
  // 创建空路径对象
  F2CPath path;
  // 如果有多个作业带，需要处理作业带之间的转弯
  if (swaths.size() > 1) {
    // 遍历所有作业带（除了最后一个）
    for (size_t i = 0; i < swaths.size()-1; ++i) {
      // 将当前作业带添加到路径中，使用巡航速度
      path.appendSwath(swaths[i], robot.getCruiseVel());
      // 创建从当前作业带到下一个作业带的转弯路径
      F2CPath turn_path = turn.createTurn(robot,
          swaths[i].endPoint(), swaths[i].getOutAngle(),  // 当前作业带的终点和出口角度
          swaths[i + 1].startPoint(), swaths[i + 1].getInAngle());  // 下一个作业带的起点和入口角度
      // 将离散化的转弯路径添加到总路径中，离散化步长为0.1
      path += turn_path.discretize(0.1);
    }
  }
  // 添加最后一个作业带（如果存在）
  if (swaths.size() > 0) {
    path.appendSwath(swaths.back(), robot.getCruiseVel());
  }
  // 返回完整路径
  return path;
}

// 为一组作业带规划路径，包括作业带之间的转弯
// 并考虑避开障碍物
// 这个函数是planPath的重载版本，增加了field参数来提供障碍物信息
F2CPath PathPlanning::planPath(const F2CField& field, const F2CRobot& robot, const F2CSwaths& swaths, TurningBase& turn) {
  // 调试日志：记录函数被调用
  //std::cout << "[DEBUG] 调用带Field参数的planPath函数" << std::endl;
  //std::cout << "[DEBUG] 作业带数量: " << swaths.size() << std::endl;
  
  // 创建空路径对象
  F2CPath path;
  
  // 从字段中提取障碍物信息
  std::vector<F2CCell> obstacles;
  //std::cout << "[DEBUG] 字段中单元数量: " << field.getField().size() << std::endl;
  
  for (size_t i = 0; i < field.getField().size(); ++i) {
    F2CCell cell = field.getField().getCell(i);
    // 只处理内部环（障碍皉）
    int ring_count = cell.size() - 1;  // 总环数减去外环
    //std::cout << "[DEBUG] 单元 " << i << " 内环数量: " << ring_count << std::endl;
    
    for (size_t j = 0; j < ring_count; ++j) {
      F2CLinearRing ring = cell.getInteriorRing(j);
      F2CCell obstacle;
      obstacle.addRing(ring);
      obstacles.push_back(obstacle);
      //std::cout << "[DEBUG] 提取到障碍物: 单元 " << i << " 内环 " << j << std::endl;
    }
  }
  
  // 是否找到障碍物
  //std::cout << "[DEBUG] 提取到障碍物总数: " << obstacles.size() << std::endl;
  
  // 如果有多个作业带，需要处理作业带之间的转弯
  if (swaths.size() > 1) {
    //std::cout << "[DEBUG] 开始处理" << swaths.size() << "个作业带之间的转弯" << std::endl;
    // 遍历所有作业带（除了最后一个）
    for (size_t i = 0; i < swaths.size()-1; ++i) {
      //std::cout << "[DEBUG] 处理作业带 " << i << " 到 " << i+1 << " 的连接" << std::endl;
      // 将当前作业带添加到路径中，使用巡航速度
      path.appendSwath(swaths[i], robot.getCruiseVel());
      
      // 获取相邻作业带的端点
      F2CPoint start_point = swaths[i].endPoint();
      double start_angle = swaths[i].getOutAngle();
      F2CPoint end_point = swaths[i + 1].startPoint();
      double end_angle = swaths[i + 1].getInAngle();
      
      //std::cout << "[DEBUG] 从 (" << start_point.getX() << ", " << start_point.getY() 
      //          << ") 到 (" << end_point.getX() << ", " << end_point.getY() << ")" << std::endl;
      
      // 检查转弯路径是否会穿过障碍物
      bool path_has_obstacle = !obstacles.empty() && isPathIntersectObstacle(start_point, end_point, obstacles);
      //std::cout << "[DEBUG] 路径是否穿过障碍物: " << (path_has_obstacle ? "是" : "否") << std::endl;
      
      if (path_has_obstacle) {
        // 如果穿过障碍物，使用A*算法规划绕过障碍物的路径
        //std::cout << "[DEBUG] 开始使用A*算法规划避障路径" << std::endl;
        double grid_size = robot.getWidth() / 2.0;  // 网格大小为机器人宽度的一半
        //std::cout << "[DEBUG] 网格大小: " << grid_size << std::endl;
        auto safe_points = planPathWithAStar(start_point, end_point, obstacles, grid_size);
        //std::cout << "[DEBUG] A*算法找到" << safe_points.size() << "个安全点" << std::endl;
        
        // 如果A*算法成功找到路径
        if (!safe_points.empty()) {
          F2CPoint last_point = start_point;
          double last_angle = start_angle;
          
          // 逐段创建路径
          for (size_t j = 0; j < safe_points.size(); ++j) {
            // 计算当前点到下一个点的角度
            double next_angle;
            if (j < safe_points.size() - 1) {
              // 如果不是最后一个点，计算到下一个点的角度
              double dx = safe_points[j+1].getX() - safe_points[j].getX();
              double dy = safe_points[j+1].getY() - safe_points[j].getY();
              next_angle = std::atan2(dy, dx);
            } else {
              // 如果是最后一个点，使用终点的入口角度
              next_angle = end_angle;
            }
            
            // 创建从上一个点到当前点的转弯
            F2CPath segment_path = turn.createTurn(robot, last_point, last_angle, safe_points[j], next_angle);
            path += segment_path.discretize(0.1);
            
            // 更新上一个点和角度
            last_point = safe_points[j];
            last_angle = next_angle;
          }
          
          // 创建从最后一个安全点到目标终点的转弯
          F2CPath final_segment = turn.createTurn(robot, last_point, last_angle, end_point, end_angle);
          path += final_segment.discretize(0.1);
        } else {
          // 如果A*算法无法找到路径，使用原始转弯
          F2CPath turn_path = turn.createTurn(robot, start_point, start_angle, end_point, end_angle);
          path += turn_path.discretize(0.1);
        }
      } else {
        // 如果没有障碍物或路径不会穿过障碍物，使用原始转弯
        F2CPath turn_path = turn.createTurn(robot, start_point, start_angle, end_point, end_angle);
        path += turn_path.discretize(0.1);
      }
    }
  }
  
  // 添加最后一个作业带（如果存在）
  if (swaths.size() > 0) {
    path.appendSwath(swaths.back(), robot.getCruiseVel());
  }
  
  return path;
}


F2CPath PathPlanning::planPathForConnection(const F2CRobot& robot,
    const F2CSwaths& s1,
    const F2CMultiPoint& mp,
    const F2CSwaths& s2,
    TurningBase& turn) {
  F2CPoint p1, p2;
  double ang1, ang2;

  if (s1.size() > 0) {
    p1 = s1.back().endPoint();
    ang1 = s1.back().getOutAngle();
  } else if (mp.size() > 0) {
    p1 = mp[0];
    ang1 = mp.getOutAngle(0);
  } else {
    return {};
  }
  if (s2.size() > 0) {
    p2 = s2[0].startPoint();
    ang2 = s2[0].getInAngle();
  } else if (mp.size() > 0) {
    p2 = mp.getLastPoint();
    ang2 = mp.getInAngle(mp.size()-1);
  } else {
    return {};
  }
  return planPathForConnection(robot, p1, ang1, mp, p2, ang2, turn);
}


// 规划两组作业带之间的连接路径
// 这个函数处理作业带组之间的连接路径规划
F2CPath PathPlanning::planPathForConnection(
    const F2CField& field,
    const F2CRobot& robot,
    const F2CSwaths& s1,     // 第一组作业带
    const F2CMultiPoint& mp, // 连接点
    const F2CSwaths& s2,     // 第二组作业带
    TurningBase& turn) {     // 转弯生成器
  // 定义起点、终点及其角度
  F2CPoint p1, p2;
  double ang1, ang2;

  // 确定起点和起始角度
  if (s1.size() > 0) {
    // 如果第一组作业带非空，使用其最后一个作业带的终点和出口角度
    p1 = s1.back().endPoint();
    ang1 = s1.back().getOutAngle();
  } else if (mp.size() > 0) {
    // 否则，如果连接点非空，使用第一个连接点和其出口角度
    p1 = mp[0];
    ang1 = mp.getOutAngle(0);
  } else {
    // 如果都为空，返回空路径
    return {};
  }
  // 确定终点和终止角度
  if (s2.size() > 0) {
    // 如果第二组作业带非空，使用其第一个作业带的起点和入口角度
    p2 = s2[0].startPoint();
    ang2 = s2[0].getInAngle();
  } else if (mp.size() > 0) {
    // 否则，如果连接点非空，使用最后一个连接点和其入口角度
    p2 = mp.getLastPoint();
    ang2 = mp.getInAngle(mp.size()-1);
  } else {
    // 如果都为空，返回空路径
    return {};
  }
  // 调用重载版本的planPathForConnection函数
  return planPathForConnection(field, robot, p1, ang1, mp, p2, ang2, turn);
}


F2CPath PathPlanning::planPathForConnection(
    const F2CRobot& robot,
    const F2CPoint& p1, double ang1,
    const F2CMultiPoint& mp,
    const F2CPoint& p2, double ang2,
    TurningBase& turn) 
{
  auto v_con = simplifyConnection(robot, p1, ang1, mp, p2, ang2);

  F2CPath path;
  for (int i = 1; i < v_con.size(); ++i) 
  {
    path += turn.createTurn(robot,
        v_con[i-1].first, v_con[i-1].second,
        v_con[i].first, v_con[i].second);
  }

  return path;
}


// 使用具体的起点和终点规划连接路径
// 这是planPathForConnection的重载版本，接受具体的点和角度
F2CPath PathPlanning::planPathForConnection(
    const F2CField& field,
    const F2CRobot& robot,
    const F2CPoint& p1, double ang1,  // 起点和起始角度
    const F2CMultiPoint& mp,          // 连接点集合
    const F2CPoint& p2, double ang2,  // 终点和终止角度
    TurningBase& turn) {              // 转弯生成器
  // 获取字段中的障碍物（如果有）
  std::vector<F2CCell> obstacles;
  // 检查是否提供了字段对象
  if (!mp.isEmpty() && mp.size() > 0) {
    // 获取字段内部的障碍物（如果有）
    // 遍历字段中的每个单元格
    for (size_t i = 0; i < field.getField().size(); ++i) {
      F2CCell cell = field.getField().getCell(i);
      // 只处理内部环（障碍物）
      // 获取内环的数量
      int ring_count = cell.size() - 1;  // 总环数减去外环
      for (size_t j = 0; j < ring_count; ++j) {
        F2CLinearRing ring = cell.getInteriorRing(j);
        F2CCell obstacle;
        obstacle.addRing(ring);
        obstacles.push_back(obstacle);
      }
    }
  }

  // 简化连接点，生成一系列关键点和角度
  auto v_con = simplifyConnection(robot, p1, ang1, mp, p2, ang2);

  // 创建空路径
  F2CPath path;
  // 遍历简化后的连接点，创建相邻点之间的转弯
  for (int i = 1; i < v_con.size(); ++i) {
    // 前一个点的位置和角度
    F2CPoint prev_point = v_con[i-1].first;
    double prev_angle = v_con[i-1].second;
    // 当前点的位置和角度
    F2CPoint curr_point = v_con[i].first;
    double curr_angle = v_con[i].second;
    
    // 检查相邻点之间的路径是否穿过障碍物
    if (!obstacles.empty() && isPathIntersectObstacle(prev_point, curr_point, obstacles)) {
      // 如果穿过障碍物，使用A*算法规划绕过障碍物的路径
      double grid_size = robot.getWidth() / 2.0;  // 网格大小为机器人宽度的一半
      auto safe_points = planPathWithAStar(prev_point, curr_point, obstacles, grid_size);
      
      // 如果A*算法成功找到路径
      if (!safe_points.empty()) {
        F2CPoint last_point = prev_point;
        double last_angle = prev_angle;
        
        // 遍历A*生成的路径点
        for (size_t j = 1; j < safe_points.size(); ++j) {
          // 计算当前点到下一个点的角度
          double current_angle = (safe_points[j] - safe_points[j-1]).getAngleFromPoint();
          // 添加转弯路径
          path += turn.createTurn(robot, last_point, last_angle, safe_points[j], current_angle);
          // 更新上一个点和角度
          last_point = safe_points[j];
          last_angle = current_angle;
        }
        
        // 添加从最后一个安全点到目标点的转弯
        path += turn.createTurn(robot, last_point, last_angle, curr_point, curr_angle);
      } else {
        // 如果A*算法无法找到路径，退回到原始方法
        path += turn.createTurn(robot, prev_point, prev_angle, curr_point, curr_angle);
      }
    } else {
      // 如果没有障碍物或路径不穿过障碍物，使用原始方法
      path += turn.createTurn(robot, prev_point, prev_angle, curr_point, curr_angle);
    }
  }
  // 返回完整的连接路径
  return path;
}



// 计算平滑转弯半径
// 这个函数计算考虑机器人运动学约束的安全转弯半径
double PathPlanning::getSmoothTurningRadius(const F2CRobot& robot) {
  // 定义回旋曲线终点的坐标、角度和曲率
  double x, y, ang, k;
  // 计算回旋曲线的终点参数
  // robot.getMaxDiffCurv()：它表示机器人转向系统从直线行驶到达到最大转弯角度（或反之）所需的过渡距离。
  // 设置为 1e8 的影响：
  // - 表示机器人可以进行几乎无限的转弯
  // - 这意味着在规划路径时，可以避免不必要的转弯
  // - 有助于生成更平滑的路径
  // - 这是一个极大的值，接近无穷大
  // - 实际上意味着机器人可以几乎瞬间从直线行驶切换到最大曲率的转弯（或反之）
  // - 相当于假设机器人可以进行"理想化"的转弯，没有平滑过渡的限制
  //
  // 这表示车辆从原点出发，沿着一条逐渐向右弯曲的路径行驶10米，最终朝向约为5弧度（约287度），曲率为1.0（转弯半径为1米）。
  //
  // 参数依次为：起点x、起点y、起始角度、起始曲率、最大曲率变化率、缩放因子、最大曲率与最大曲率变化率的比值
  end_of_clothoid(0.0, 0.0, 0.0, 0.0, robot.getMaxDiffCurv(), 1.0,
      robot.getMaxCurv() / robot.getMaxDiffCurv(),
      &x, &y, &ang, &k);  // 输出参数：终点x、y、角度和曲率
  // 计算圆心坐标
  double xi = x - sin(ang) / robot.getMaxCurv();
  double yi = y + cos(ang) / robot.getMaxCurv();
  // 返回圆心到原点的距离，即平滑转弯半径
  return sqrt(xi*xi + yi*yi);
}

// 简化连接路径
// 这个函数将复杂的连接点序列简化为关键点，以生成更平滑的路径
std::vector<std::pair<F2CPoint, double>> PathPlanning::simplifyConnection(
    const F2CRobot& robot,            // 机器人参数
    const F2CPoint& p1, double ang1,  // 起点和起始角度
    const F2CMultiPoint& mp,          // 连接点集合
    const F2CPoint& p2, double ang2) { // 终点和终止角度
  // 计算安全距离（平滑转弯半径）
  const double safe_dist = getSmoothTurningRadius(robot);
  // 创建路径点和角度的向量
  std::vector<std::pair<F2CPoint, double>> path;
  // 添加起点和起始角度
  path.emplace_back(p1, ang1);

  // @@@ 如果起点和终点距离较近，或连接点太少，直接返回起点到终点的直接路径
  if (p1.distance(p2) < 6.0 * safe_dist || mp.size() < 2) {
    path.emplace_back(p2, ang2);
    return path;
  }

  // 筛选出角度变化明显的关键点
  std::vector<F2CPoint> vp;
  for (int i = 1; i < mp.size() - 1; ++i) {
    // 计算进入和离开当前点的角度
    double ang_in  = (mp[i] - mp[i-1]).getAngleFromPoint();
    double ang_out = (mp[i+1] - mp[i]).getAngleFromPoint();
    // 如果角度变化超过阈值，将该点视为关键点
    if (fabs(ang_in - ang_out) > 0.1) {
      vp.emplace_back(mp[i]);
    }
  }

  // 如果关键点太少，直接返回起点到终点的直接路径
  if (vp.size() < 2) {
    path.emplace_back(p2, ang2);
    return path;
  }

  // 处理每个关键点，生成平滑的入口点和出口点
  for (int i = 1; i < vp.size() - 1; ++i) {
    // 计算当前点到前后点的距离
    double dist_in  = vp[i].distance(vp[i-1]);
    double dist_out  = vp[i].distance(vp[i+1]);
    // 如果距离为零，跳过该点
    if (dist_in == 0.0 || dist_out == 0.0) {
      continue;
    }
    // 计算入口和出口点的距离，取距离的一半和安全距离的较小值
    double d_in  = min(0.5 * dist_in,  safe_dist);
    double d_out  = min(0.5 * dist_out,  safe_dist);
    // 计算入口点和出口点的位置
    F2CPoint p_in =  (vp[i-1] - vp[i]) * (d_in  / dist_in)  + vp[i];
    F2CPoint p_out = (vp[i+1] - vp[i]) * (d_out / dist_out) + vp[i];
    // 如果入口点与已有点的距离都足够大，添加入口点
    if (p_in.distance(path.back().first) > 3.0 * safe_dist &&
        p_in.distance(p1)                > 3.0 * safe_dist &&
        p_in.distance(p2)                > 3.0 * safe_dist) {
      // 计算入口角度
      double ang_in   = (vp[i  ] - vp[i-1]).getAngleFromPoint();
      // 添加入口点和角度
      path.emplace_back(p_in, ang_in);
    }
    // 如果出口点与已有点的距离都足够大，添加出口点
    if (p_out.distance(vp[i+1]) > 3.0 * safe_dist &&
        p_out.distance(p1)                > 3.0 * safe_dist &&
        p_out.distance(p2)                > 3.0 * safe_dist &&
        p_out.distance(p_in)              > 3.0 * safe_dist) {
      // 计算出口角度
      double ang_out  = (vp[i+1] - vp[i]).getAngleFromPoint();
      // 添加出口点和角度
      path.emplace_back(p_out, ang_out);
    }
  }
  // 添加终点和终止角度
  path.emplace_back(p2, ang2);
  // 返回简化后的路径点和角度
  return path;
}


bool PathPlanning::isPathIntersectObstacle(
    const F2CPoint& start_point, 
    const F2CPoint& end_point,
    const std::vector<F2CCell>& obstacles) {
    
        // 如果起点和终点几乎重合，则返回false（零长度路径不判定为相交）
        if (start_point.distance(end_point) < 1e-6) {
            std::cout << "[DEBUG] isPathIntersectObstacle - 零长度路径，跳过检查" << std::endl;
            return false;
        }
        
        std::cout << "[DEBUG] isPathIntersectObstacle - 检查路径：(" 
                  << start_point.getX() << ", " << start_point.getY() << ") -> (" 
                  << end_point.getX() << ", " << end_point.getY() << ")" << std::endl;
        std::cout << "[DEBUG] 障碍物数量: " << obstacles.size() << std::endl;
    
        // =============== 1. 定义线段（路径） ===============
        // 创建路径为一条线段（从起点到终点）
        Clipper2Lib::PathsD subject = {{
            {start_point.getX(), start_point.getY()},   // 线段起点坐标
            {end_point.getX(), end_point.getY()}      // 线段终点坐标
        }};
        
        std::cout << "[DEBUG] 路径数据: {{" 
                  << start_point.getX() << ", " << start_point.getY() << "}, {" 
                  << end_point.getX() << ", " << end_point.getY() << "}}" << std::endl;
        
        // =============== 2. 将线段膨胀为细长多边形 ===============
        // 因线段本身是零宽度，需通过膨胀生成微小宽度多边形才能进行面积交集检测
        Clipper2Lib::PathsD subject_inflated = 
            Clipper2Lib::InflatePaths(
                subject,                    // 原始路径
                0.01,                       // 膨胀宽度为0.01
                Clipper2Lib::JoinType::Square,  // 连接处直角处理
                Clipper2Lib::EndType::Square    // 端点直角处理
            );
            
        std::cout << "[DEBUG] 膨胀后的路径点数: " << subject_inflated[0].size() << std::endl;
            
        // =============== 3. 检查每个障碍物 ===============
        for (size_t obs_idx = 0; obs_idx < obstacles.size(); ++obs_idx) {
            const auto& obstacle = obstacles[obs_idx];
            
            // 创建障碍物多边形
            Clipper2Lib::PathsD clip;
            Clipper2Lib::PathD exterior_points;
            
            // 提取障碍物外环坐标
            F2CLinearRing exterior_ring = obstacle.getExteriorRing();
            std::cout << "[DEBUG] 障碍物#" << obs_idx << "外环点数: " << exterior_ring.size() << std::endl;
            
            for (size_t i = 0; i < exterior_ring.size(); ++i) {
                F2CPoint p = exterior_ring.getGeometry(i);
                exterior_points.push_back({p.getX(), p.getY()});
            }
            
            // 确保多边形闭合（首尾点相同）
            if (exterior_ring.size() > 0 && 
                (exterior_points[0].x != exterior_points.back().x || 
                 exterior_points[0].y != exterior_points.back().y)) {
                exterior_points.push_back(exterior_points[0]);
            }
            
            clip.push_back(exterior_points);
            
            // 打印障碍物外环数据（示例格式）
            std::cout << "[DEBUG] 障碍物外环数据示例: {{" << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), exterior_points.size()); ++i) {
                std::cout << "  {" << exterior_points[i].x << ", " << exterior_points[i].y << "}," << std::endl;
            }
            if (exterior_points.size() > 5) {
                std::cout << "  ... 共" << exterior_points.size() << "个点" << std::endl;
            }
            std::cout << "}}" << std::endl;
            
            // 处理障碍物的内环（洞）- 添加到clip中
            for (size_t j = 0; j < obstacle.size() - 1; ++j) {
                Clipper2Lib::PathD interior_points;
                F2CLinearRing interior_ring = obstacle.getInteriorRing(j);
                
                for (size_t i = 0; i < interior_ring.size(); ++i) {
                    F2CPoint p = interior_ring.getGeometry(i);
                    interior_points.push_back({p.getX(), p.getY()});
                }
                
                // 确保内环闭合
                if (interior_ring.size() > 0 && 
                    (interior_points[0].x != interior_points.back().x || 
                     interior_points[0].y != interior_points.back().y)) {
                    interior_points.push_back(interior_points[0]);
                }
                
                if (!interior_points.empty()) {
                    clip.push_back(interior_points);
                }
            }
            
            // =============== 4. 计算交集 ===============
            std::cout << "[DEBUG] 计算路径与障碍物#" << obs_idx << "的交集..." << std::endl;
            Clipper2Lib::PathsD solution = 
                Clipper2Lib::Intersect(
                    subject_inflated,    // 膨胀后的路径
                    clip,                // 障碍物多边形
                    Clipper2Lib::FillRule::NonZero  // 非零填充规则
                );
                
            // =============== 5. 判断结果 ===============
            // 检查交集结果 - 如果非空，则路径与障碍物相交
            bool intersects = !solution.empty();
            std::cout << "[DEBUG] 交集检测结果: " << (intersects ? "有交集" : "无交集") << std::endl;
            std::cout << "[DEBUG] 交集路径数量: " << solution.size() << std::endl;
            
            if (intersects) {
                // 打印一些交集点的信息用于调试
                for (size_t i = 0; i < std::min(size_t(2), solution.size()); ++i) {
                    std::cout << "[DEBUG] 交集路径 " << i << " 点数: " << solution[i].size() << std::endl;
                    for (size_t j = 0; j < std::min(size_t(3), solution[i].size()); ++j) {
                        std::cout << "[DEBUG] 点 " << j << ": (" << solution[i][j].x 
                                  << ", " << solution[i][j].y << ")" << std::endl;
                    }
                }
                return true; // 有交集，路径与障碍物相交
            }
            
            // =============== 6. 点在多边形内的检测（额外安全检查）===============
            // 检查起点是否在障碍物内部
            Clipper2Lib::PointInPolygonResult start_result = 
                Clipper2Lib::PointInPolygon({start_point.getX(), start_point.getY()}, exterior_points);
            std::cout << "[DEBUG] 起点在障碍物内? " << 
                (start_result == Clipper2Lib::PointInPolygonResult::IsInside ? "是" : 
                (start_result == Clipper2Lib::PointInPolygonResult::IsOutside ? "否" : "在边界上")) << std::endl;
                
            if (start_result == Clipper2Lib::PointInPolygonResult::IsInside) {
                return true;
            }
            
            // 检查终点是否在障碍物内部
            Clipper2Lib::PointInPolygonResult end_result = 
                Clipper2Lib::PointInPolygon({end_point.getX(), end_point.getY()}, exterior_points);
            std::cout << "[DEBUG] 终点在障碍物内? " << 
                (end_result == Clipper2Lib::PointInPolygonResult::IsInside ? "是" : 
                (end_result == Clipper2Lib::PointInPolygonResult::IsOutside ? "否" : "在边界上")) << std::endl;
                
            if (end_result == Clipper2Lib::PointInPolygonResult::IsInside) {
                return true;
            }
            
            // 检查路径中点是否在障碍物内部
            double mid_x = (start_point.getX() + end_point.getX()) / 2.0;
            double mid_y = (start_point.getY() + end_point.getY()) / 2.0;
            Clipper2Lib::PointInPolygonResult mid_result = 
                Clipper2Lib::PointInPolygon({mid_x, mid_y}, exterior_points);
            std::cout << "[DEBUG] 中点(" << mid_x << ", " << mid_y << ")在障碍物内? " << 
                (mid_result == Clipper2Lib::PointInPolygonResult::IsInside ? "是" : 
                (mid_result == Clipper2Lib::PointInPolygonResult::IsOutside ? "否" : "在边界上")) << std::endl;
                
            if (mid_result == Clipper2Lib::PointInPolygonResult::IsInside) {
                return true;
            }
        }
        
        // 所有障碍物都检查完毕且没有相交
        std::cout << "[DEBUG] 所有障碍物检查完毕，未发现相交，返回false" << std::endl;
        return false;
}

// 检测点是否在障碍物内部或太接近障碍物
bool PathPlanning::isPointInObstacles(
    const F2CPoint& point,
    const std::vector<F2CCell>& obstacles,
    double safety_margin) {
  // 检查点是否在任何障碍物内部
  for (const auto& obstacle : obstacles) {
    // 检查点是否在障碍物内部
    if (obstacle.isPointIn(point)) {
      return true;
    }
    
    // 检查点是否太接近障碍物边界
    F2CLinearRing exterior_ring = obstacle.getExteriorRing();
    for (size_t i = 0; i < exterior_ring.size(); ++i) {
      F2CPoint p1 = exterior_ring.getGeometry(i);
      F2CPoint p2 = exterior_ring.getGeometry((i + 1) % exterior_ring.size());
      
      // 计算点到线段的距离
      CGALPoint cgal_point(point.getX(), point.getY());
      CGALSegment cgal_segment(CGALPoint(p1.getX(), p1.getY()),
                              CGALPoint(p2.getX(), p2.getY()));
      
      double distance = std::sqrt(CGAL::squared_distance(cgal_point, cgal_segment));
      if (distance < safety_margin) {
        return true;  // 点太接近障碍物
      }
    }
  }
  
  return false;  // 点不在任何障碍物内部且不太接近障碍物
}

// 使用A*算法规划绕过障碍物的路径
std::vector<F2CPoint> PathPlanning::planPathWithAStar(
    const F2CPoint& start_point,
    const F2CPoint& end_point,
    const std::vector<F2CCell>& obstacles,
    double grid_size) {
  //std::cout << "[DEBUG] 开始A*路径规划" << std::endl;
  //std::cout << "[DEBUG] 起点: (" << start_point.getX() << ", " << start_point.getY() << ")" << std::endl;
  //std::cout << "[DEBUG] 终点: (" << end_point.getX() << ", " << end_point.getY() << ")" << std::endl;
  //std::cout << "[DEBUG] 障碍物数量: " << obstacles.size() << std::endl;
  //std::cout << "[DEBUG] 网格大小: " << grid_size << std::endl;
  
  // 定义一个网格地图边界，扩展一些余量
  double min_x = std::min(start_point.getX(), end_point.getX()) - 20.0 * grid_size;
  double max_x = std::max(start_point.getX(), end_point.getX()) + 20.0 * grid_size;
  double min_y = std::min(start_point.getY(), end_point.getY()) - 20.0 * grid_size;
  double max_y = std::max(start_point.getY(), end_point.getY()) + 20.0 * grid_size;
  //std::cout << "[DEBUG] 规划区域边界: [左下(" << min_x << ", " << min_y << "), 右上(" << max_x << ", " << max_y << ")]" << std::endl;
  
  // 安全距离为网格大小的1.5倍
  double safety_margin = 1.5 * grid_size;
  //std::cout << "[DEBUG] 安全距离: " << safety_margin << std::endl;
  
  // 定义网格坐标的哈希函数
  struct GridCoordHash {
    std::size_t operator()(const std::pair<int, int>& k) const {
      return std::hash<int>()(k.first) ^ (std::hash<int>()(k.second) << 1);
    }
  };
  
  // 将连续坐标转换为网格坐标
  auto toGridCoord = [grid_size, min_x, min_y](const F2CPoint& p) -> std::pair<int, int> {
    int grid_x = static_cast<int>((p.getX() - min_x) / grid_size);
    int grid_y = static_cast<int>((p.getY() - min_y) / grid_size);
    return {grid_x, grid_y};
  };
  
  // 将网格坐标转换为连续坐标（网格中心）
  auto toContinuousCoord = [grid_size, min_x, min_y](const std::pair<int, int>& grid) -> F2CPoint {
    double x = min_x + (grid.first + 0.5) * grid_size;
    double y = min_y + (grid.second + 0.5) * grid_size;
    return F2CPoint(x, y);
  };
  
  // 启发式函数：使用欧几里得距离
  auto heuristic = [](const std::pair<int, int>& a, const std::pair<int, int>& b) -> double {
    return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
  };
  
  // 转换起点和终点为网格坐标
  std::pair<int, int> start_grid = toGridCoord(start_point);
  std::pair<int, int> end_grid = toGridCoord(end_point);
  
  // 定义相邻方向（8个方向）
  std::vector<std::pair<int, int>> directions = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}
  };
  
  // 定义开集（待探索的节点）和闭集（已探索的节点）
  std::priority_queue<std::pair<double, std::pair<int, int>>,
                      std::vector<std::pair<double, std::pair<int, int>>>,
                      std::greater<std::pair<double, std::pair<int, int>>>> open_set;
  std::unordered_set<std::pair<int, int>, GridCoordHash> closed_set;
  
  // 记录每个节点的父节点，用于重建路径
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, GridCoordHash> came_from;
  
  // 记录从起点到每个节点的代价
  std::unordered_map<std::pair<int, int>, double, GridCoordHash> g_score;
  g_score[start_grid] = 0.0;
  
  // 记录从起点经过每个节点到目标的估计总代价
  std::unordered_map<std::pair<int, int>, double, GridCoordHash> f_score;
  f_score[start_grid] = heuristic(start_grid, end_grid);
  
  // 将起点加入开集
  open_set.push({f_score[start_grid], start_grid});
  
  // A*算法主循环
  int iterations = 0;
  int max_iterations = 10000; // 防止无限循环
  //std::cout << "[DEBUG] 开始A*算法主循环" << std::endl;
  
  while (!open_set.empty()) {
    // 迭代计数
    iterations++;
    if (iterations % 1000 == 0) {
      //std::cout << "[DEBUG] A*算法已迭代 " << iterations << " 次" << std::endl;
    }
    
    // 检查是否超过最大迭代次数
    if (iterations > max_iterations) {
      //std::cout << "[DEBUG] A*算法超过最大迭代次数，放弃寻路" << std::endl;
      break;
    }
    
    // 获取f值最小的节点
    auto current = open_set.top().second;
    open_set.pop();
    
    // 如果到达终点，重建路径并返回
    if (current == end_grid) {
      //std::cout << "[DEBUG] 成功找到路径！开始重建路径" << std::endl;
      //std::cout << "[DEBUG] 总共探索节点数: " << closed_set.size() << std::endl;
      
      std::vector<F2CPoint> path;
      // 重建路径
      auto node = current;
      while (node != start_grid) {
        path.push_back(toContinuousCoord(node));
        node = came_from[node];
      }
      // 添加起点
      path.push_back(start_point);
      // 反转路径，使其从起点到终点
      std::reverse(path.begin(), path.end());
      // 最后一个点使用确切的终点坐标
      path.back() = end_point;
      
      //std::cout << "[DEBUG] 重建的路径点数: " << path.size() << std::endl;
      //std::cout << "[DEBUG] 路径起点: (" << path.front().getX() << ", " << path.front().getY() << ")" << std::endl;
      //std::cout << "[DEBUG] 路径终点: (" << path.back().getX() << ", " << path.back().getY() << ")" << std::endl;
      
      return path;
    }
    
    // 将当前节点加入闭集
    closed_set.insert(current);
    
    // 探索相邻节点
    for (const auto& dir : directions) {
      std::pair<int, int> neighbor = {current.first + dir.first, current.second + dir.second};
      
      // 检查是否超出地图边界
      F2CPoint neighbor_point = toContinuousCoord(neighbor);
      if (neighbor_point.getX() < min_x || neighbor_point.getX() > max_x ||
          neighbor_point.getY() < min_y || neighbor_point.getY() > max_y) {
        continue;
      }
      
      // 检查是否在障碍物内部或太接近障碍物
      if (isPointInObstacles(neighbor_point, obstacles, safety_margin)) {
        continue;
      }
      
      // 如果节点已经在闭集中，跳过
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      
      // 计算从起点到相邻节点的新代价
      double tentative_g_score = g_score[current] + heuristic(current, neighbor);
      
      // 如果相邻节点不在开集中，或者新路径更好，更新路径
      if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
        // 更新父节点和代价
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_grid);
        
        // 将相邻节点加入开集
        open_set.push({f_score[neighbor], neighbor});
      }
    }
  }
  
  // 如果无法找到路径，返回空向量
  return {};
}

}  // namespace f2c::pp

