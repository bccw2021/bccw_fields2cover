//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "fields2cover.h"
#include <iostream>

int main() {

  std::cout << "####### Tutorial 5.1 Route planning for all swaths ######" << std::endl;

  F2CRobot robot_c (1.0);
  robot_c.setMinTurningRadius(0.0);  // m
  robot_c.setMaxDiffCurv(1e8);
  //F2CCells cells_c {
  //  F2CCell(F2CLinearRing({
  //        F2CPoint(0,0), F2CPoint(2,0),F2CPoint(2,2),F2CPoint(0,2), F2CPoint(0,0)
  //  }))
  //};
  //cells_c.addRing(0, F2CLinearRing({
  //      F2CPoint(.4,.4), F2CPoint(.4,.6),F2CPoint(.6,.6),F2CPoint(.6,.4), F2CPoint(.4,.4)
  //      }));
  //cells_c.addRing(0, F2CLinearRing({
  //      F2CPoint(1.2,1.2), F2CPoint(1.2,1.6),F2CPoint(1.6,1.6),F2CPoint(1.6,1.2), F2CPoint(1.2,    1.2)
  //      }));
  //cells_c *= 3e1;

 
/*
field_data = [ 
    [[0.0, 0.0], [43.457, -29.696], [31.722, -47.145], [9.225, -31.503], [-1.065, -43.405], [-18.112, -29.07], [0.0, 0.0]],
    [[-14.662, -28.127], [-14.251, -27.699], [-0.783, -38.755], [-1.078, -39.298], [-14.662, -28.127]],
    [[0.944, -36.15], [0.715, -36.557], [-12.905, -25.509], [-12.556, -24.996], [0.944, -36.15]],
    [[-10.035, -21.98], [-9.645, -21.356], [4.081, -32.604], [3.592, -33.302], [-10.035, -21.98]],
    #[[7.223, -26.933], [6.698, -27.724], [-6.254, -19.486], [-5.891, -18.811], [7.223, -26.933]],
    #[[-3.597, -14.747], [-3.17, -14.142], [8.963, -23.706], [8.661, -24.376], [-3.597, -14.747]],
    #[[11.715, -18.393], [11.318, -19.042], [-1.939, -9.72], [-1.419, -9.03], [11.715, -18.393]],
    #[[27.675, -20.594], [28.009, -20.011], [41.139, -28.871], [40.658, -29.563], [27.675, -20.594]],
    #[[38.711, -32.522], [38.368, -33.133], [25.724, -23.426], [26.124, -22.777], [38.711, -32.522]],
    #[[23.823, -26.171], [24.324, -25.604], [37.281, -35.213], [36.773, -35.975], [23.823, -26.171]],
    #[[35.306, -37.902], [34.863, -38.696], [21.581, -29.717], [21.87, -29.068], [35.306, -37.902]],
    #[[19.913, -32.0], [20.292, -31.502], [33.381, -40.896], [33.036, -41.576], [19.913, -32.0]],
    #[[31.721, -43.776], [31.361, -44.521], [20.142, -36.294], [20.604, -35.559], [31.721, -43.776]]
]
*/


  // 构建地图field
  // 第一个数组是外部轮廓，其余的数组是内部孔洞（障碍物）
  std::vector<std::vector<std::vector<double>>> field_data = {
      {{0.0, 0.0}, {43.457, -29.696}, {31.722, -47.145}, {9.225, -31.503}, {-1.065, -43.405}, {-18.112, -29.07}, {0.0, 0.0}},
      {{-14.662, -28.127}, {-14.251, -27.699}, {-0.783, -38.755}, {-1.078, -39.298}, {-14.662, -28.127}},
      {{0.944, -36.15}, {0.715, -36.557}, {-12.905, -25.509}, {-12.556, -24.996}, {0.944, -36.15}},
      {{-10.035, -21.98}, {-9.645, -21.356}, {4.081, -32.604}, {3.592, -33.302}, {-10.035, -21.98}},
      {{7.223,  -26.933}, {6.698,  -27.724}, {-6.254, -19.486}, {-5.891, -18.811}, {7.223,  -26.933}},
      {{-3.597, -14.747}, {-3.17,  -14.142}, {8.963,  -23.706}, {8.661,  -24.376}, {-3.597, -14.747}},
      {{11.715, -18.393}, {11.318, -19.042}, {-1.939,   -9.72}, {-1.419,   -9.03}, {11.715, -18.393}},
      {{27.675, -20.594}, {28.009, -20.011}, {41.139, -28.871}, {40.658, -29.563}, {27.675, -20.594}},
      {{38.711, -32.522}, {38.368, -33.133}, {25.724, -23.426}, {26.124, -22.777}, {38.711, -32.522}},
      {{23.823, -26.171}, {24.324, -25.604}, {37.281, -35.213}, {36.773, -35.975}, {23.823, -26.171}},
      {{35.306, -37.902}, {34.863, -38.696}, {21.581, -29.717}, {21.87,  -29.068}, {35.306, -37.902}},
      {{19.913,   -32.0}, {20.292, -31.502}, {33.381, -40.896}, {33.036, -41.576}, {19.913,   -32.0}},
      {{31.721, -43.776}, {31.361, -44.521}, {20.142, -36.294}, {20.604, -35.559}, {31.721, -43.776}}
  };

  // 创建一个单元格 (Cell)
  F2CCell cell;
  
  // 第一步：添加外部边界
  F2CLinearRing outer_ring;
  for (const auto& point : field_data[0]) {
      outer_ring.addPoint(point[0], point[1]);
  }
  cell.addRing(outer_ring);
  
  // 第二步：添加内部孔洞（障碍物）
  for (size_t i = 1; i < field_data.size(); ++i) {
      F2CLinearRing inner_ring;
      for (const auto& point : field_data[i]) {
          inner_ring.addPoint(point[0], point[1]);
      }
      cell.addRing(inner_ring);
  }
  
  // 第三步：创建 Cells 对象
  F2CCells cells_d;
  cells_d.addGeometry(cell);
  
  // 第四步：创建 Field 对象
  F2CField field(cells_d, "my_field");
  
  // 可选：设置坐标系统
  field.setCRS("EPSG:4326");  // WGS84 坐标系
  
  // 现在您可以使用 field 对象进行后续操作
  //std::cout << "Field area: " << field.getArea() << " square units" << std::endl;
  
  // 例如：转换为 UTM 坐标系进行操作
  //f2c::Transform::transformToUTM(field);

  F2CCells cells = field.getField();
  
  f2c::hg::ConstHL const_hl;
  //F2CCells mid_hl_c = const_hl.generateHeadlands(cells_c, 1.5 * robot_c.getWidth());
  //F2CCells no_hl_c = const_hl.generateHeadlands(cells_c, 3.0 * robot_c.getWidth());
  F2CCells mid_hl = const_hl.generateHeadlands(cells, 0.1 * robot_c.getWidth());
  F2CCells no_hl = const_hl.generateHeadlands(cells, 0.3 * robot_c.getWidth());

  f2c::sg::BruteForce bf;
  F2CSwathsByCells swaths_c = bf.generateSwaths(M_PI/2.0, robot_c.getCovWidth(), no_hl);
  //F2CSwathsByCells swaths_c = bf.generateSwaths(M_PI/5.2, robot_c.getCovWidth(), no_hl);
  //f2c::obj::NSwath n_swaths_obj;
  //f2c::obj::SwathLength swath_length;
  //F2CSwathsByCells swaths_c = bf.generateBestSwaths(n_swaths_obj, robot_c.getCovWidth(), no_hl);

  f2c::rp::RoutePlannerBase route_planner;
  F2CRoute route = route_planner.genRoute(mid_hl, swaths_c);

  f2c::pp::PathPlanning path_planner;
  f2c::pp::DubinsCurves dubins;
  //F2CPath path = path_planner.planPath(robot_c, route, dubins);
  F2CPath path = path_planner.planPath(field, robot_c, route, dubins);
  
  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  //f2c::Visualizer::xlim(-5,65);
  //f2c::Visualizer::ylim(-5,65);
  f2c::Visualizer::save("Tutorial_5_0_field.svg");

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  //f2c::Visualizer::plot(field.getField());
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(route);
  //f2c::Visualizer::xlim(-5,65);
  //f2c::Visualizer::ylim(-5,65);
  f2c::Visualizer::save("Tutorial_5_0_route.svg");

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(path);
  f2c::Visualizer::save("Tutorial_5_0_path.svg");


  std::cout << "####### Tutorial 5.2 Known Patterns ######" << std::endl;


  f2c::Random rand(42);
  F2CRobot robot (1.0, 1.0);
  //F2CCells cells = rand.generateRandField(1e4, 5).getField();
  //F2CCells no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth());
  F2CSwaths swaths = bf.generateSwaths(M_PI/5.2, robot.getCovWidth(), no_hl.getGeometry(0));

  std::cout << "####### Tutorial 5.2.1 Boustrophedon ######" << std::endl;
  f2c::rp::BoustrophedonOrder boustrophedon_sorter;
  auto boustrophedon_swaths = boustrophedon_sorter.genSortedSwaths(swaths);

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(boustrophedon_swaths);
  f2c::Visualizer::save("Tutorial_5_1_Boustrophedon_1.svg");


  for (int i = 1; i < 4; ++i) {
    f2c::Visualizer::figure();
    f2c::Visualizer::plot(cells);
    f2c::Visualizer::plot(no_hl);
    f2c::Visualizer::plot(boustrophedon_sorter.genSortedSwaths(swaths, i));
    f2c::Visualizer::save("Tutorial_5_1_Boustrophedon_" + std::to_string(i+1) + ".svg" );
  }


  std::cout << "####### Tutorial 5.2.2 Snake order ######" << std::endl;
  f2c::rp::SnakeOrder snake_sorter;
  auto snake_swaths = snake_sorter.genSortedSwaths(swaths);

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(snake_swaths);
  f2c::Visualizer::save("Tutorial_5_2_Snake_1.svg");

  for (int i = 1; i < 4; ++i) {
    f2c::Visualizer::figure();
    f2c::Visualizer::plot(cells);
    f2c::Visualizer::plot(no_hl);
    f2c::Visualizer::plot(snake_sorter.genSortedSwaths(swaths, i));
    f2c::Visualizer::save("Tutorial_5_2_Snake_" + std::to_string(i+1) + ".svg");
  }

  swaths = bf.generateSwaths(M_PI, robot.getCovWidth(), no_hl.getGeometry(0));
  std::cout << "####### Tutorial 5.2.3 Spiral order ######" << std::endl;
  f2c::rp::SpiralOrder spiral_sorter(6);
  auto spiral_swaths = spiral_sorter.genSortedSwaths(swaths);

  f2c::Visualizer::figure();
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(spiral_swaths);
  f2c::Visualizer::save("Tutorial_5_3_Spiral_1.svg");

  for (int i = 1; i < 4; ++i) {
    f2c::Visualizer::figure();
    f2c::Visualizer::plot(cells);
    f2c::Visualizer::plot(no_hl);
    f2c::Visualizer::plot(spiral_sorter.genSortedSwaths(swaths, i));
    f2c::Visualizer::save("Tutorial_5_3_Spiral_" + std::to_string(i+1) + ".svg");
  }

  return 0;
}

