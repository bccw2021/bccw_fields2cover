//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "fields2cover.h"
#include <iostream>

int main() {
  // Import field
  F2CField field = f2c::Parser::importFieldGml(std::string(DATA_PATH) + "test1.xml");
  F2CField orig_field = field.clone();
  // Transform into UTM to work in meters
  f2c::Transform::transformToUTM(field);

  F2CCells cells_c {
    F2CCell(F2CLinearRing({
          F2CPoint(0,0), F2CPoint(14,0),F2CPoint(14,20),F2CPoint(0,20), F2CPoint(0,0)
    }))
  };
  cells_c.addRing(0, F2CLinearRing({
        F2CPoint(3,2), F2CPoint(4,2),F2CPoint(4,18),F2CPoint(3,18), F2CPoint(3,2)
        }));
  cells_c.addRing(0, F2CLinearRing({
        F2CPoint(7,2), F2CPoint(8,2),F2CPoint(8,18),F2CPoint(7,18), F2CPoint(7,2)
        }));
  //cells_c *= 3e1;

  F2CRobot robot (0.8, 0.8);
  f2c::hg::ConstHL const_hl;
  //F2CCells no_hl = const_hl.generateHeadlands(field.getField(), 3.0 * robot.getWidth());
  F2CCells no_hl = const_hl.generateHeadlands(cells_c, 0.5*robot.getWidth() );
  f2c::sg::BruteForce bf;
  f2c::obj::NSwath n_swaths_obj;
  f2c::obj::SwathLength swath_length;
//   F2CSwaths swaths = bf.generateBestSwaths(n_swaths_obj, robot.getCovWidth(), no_hl.getGeometry(0));
//   f2c::rp::SnakeOrder snake_sorter;
//   swaths = snake_sorter.genSortedSwaths(swaths);
//   f2c::rp::BoustrophedonOrder boustrophedon_sorter;
//   swaths = boustrophedon_sorter.genSortedSwaths(swaths);

F2CSwathsByCells swaths = bf.generateBestSwaths(n_swaths_obj, robot.getCovWidth(), no_hl);
//F2CSwathsByCells swaths = bf.generateSwaths(M_PI*0.5, robot.getCovWidth(), cells_c);
f2c::rp::RoutePlannerBase route_planner;
 F2CRoute route = route_planner.genRoute(cells_c,swaths);

  f2c::pp::PathPlanning path_planner;
  robot.setMinTurningRadius(0.0);  // m
  f2c::pp::DubinsCurves dubins;
  F2CPath path = path_planner.planPath(robot, route, dubins);
  path.saveToFile("path.csv");
//f2c::Visualizer::axis_equal();
  f2c::Visualizer::figure();
  
  //f2c::Visualizer::plot(field);
  f2c::Visualizer::plotFilled(cells_c,{10,10,10},{100,100,100});
  //f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(route);
  f2c::Visualizer::plot(path);
  //f2c::Visualizer::show();
  //f2c::Visualizer::figure_size(10000,20000);//f2c::Visualizer::save("Tutorial_8_1_UTM.png");
  f2c::Visualizer::save("jds_test.svg");

  // // Transform the generated path back to the previousa CRS.
  // F2CPath path_gps = f2c::Transform::transformToPrevCRS(path, field);
  // f2c::Transform::transformToPrevCRS(field);

  // f2c::Visualizer::figure();
  // f2c::Visualizer::plot(orig_field.getCellsAbsPosition());
  // f2c::Visualizer::plot(path_gps);
  // f2c::Visualizer::save("Tutorial_8_1_GPS.png");

  return 0;
}


