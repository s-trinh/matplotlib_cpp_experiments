/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Code modified from tutorial-ibvs-4pts-plotter-continuous-gain-adaptive.cpp.
 */
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

// https://github.com/alandefreitas/matplotplusplus
// Doc: https://alandefreitas.github.io/matplotplusplus/
#include <matplot/matplot.h>
// https://github.com/lava/matplotlib-cpp
#include <matplotlibcpp.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    // Matplotlib-cpp
    namespace plt = matplotlibcpp;
    plt::suptitle("lava/matplotlib-cpp backend");

    plt::subplot(2, 1, 1);
    plt::title("Visual features error");
    plt::Plot plot_x0("x1");
    plt::Plot plot_y0("y1");
    plt::Plot plot_x1("x2");
    plt::Plot plot_y1("y2");
    plt::Plot plot_x2("x3");
    plt::Plot plot_y2("y3");
    plt::Plot plot_x3("x4");
    plt::Plot plot_y3("y4");

    plt::subplot(2, 1, 2);
    plt::title("Camera velocities");
    plt::Plot plot_vx("v_x", "r"); // Test setting manual color
    plt::Plot plot_vy("v_y", "g");
    plt::Plot plot_vz("v_z", "b");
    plt::Plot plot_wx("w_x");
    plt::Plot plot_wy("w_y");
    plt::Plot plot_wz("w_z");

    double global_min_xy = 1e9, global_max_xy = -1e9;
    double global_min_v = 1e9, global_max_v = -1e9;

    std::vector<double> vec_times;

    // Features errors
    std::vector<std::vector<double>> mat_times;
    std::vector<std::vector<double>> mat_features_errors;
    std::vector<double> vec_x0, vec_x1, vec_x2, vec_x3;
    std::vector<double> vec_y0, vec_y1, vec_y2, vec_y3;

    // Camera velocity
    std::vector<double> vec_vx, vec_vy, vec_vz;
    std::vector<double> vec_wx, vec_wy, vec_wz;

    // Matplot / Gnuplot
    auto f = matplot::figure(true);

    auto ax1 = matplot::subplot(2, 1, 0);
    matplot::grid(true);
    // matplot::legend({"x_1", "y_1", "x_2", "y_2", "x_3", "y_3", "x_4", "y_4"});
    ax1->legend({"x_1", "y_1", "x_2", "y_2", "x_3", "y_3", "x_4", "y_4"});
    ax1->title("Visual features error");

    auto ax2 = matplot::subplot(2, 1, 1);
    matplot::grid(true);
    // matplot::legend({"v_x", "v_y", "v_z", "w_x", "w_y", "w_z"});
    ax2->legend({"v_x", "v_y", "v_z", "w_x", "w_y", "w_z"});
    ax2->title("Camera velocities");

    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    vpAdaptiveGain lambda(4, 0.4, 30);
    task.setLambda(lambda);

    double time = 0;
    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
    }

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

#ifdef VISP_HAVE_DISPLAY
    vpPlot plotter(2, 250 * 2, 500, 100, 200, "Real time curves plotter");
    plotter.setTitle(0, "Visual features error");
    plotter.setTitle(1, "Camera velocities");

    plotter.initGraph(0, 8);
    plotter.initGraph(1, 6);

    plotter.setLegend(0, 0, "x1");
    plotter.setLegend(0, 1, "y1");
    plotter.setLegend(0, 2, "x2");
    plotter.setLegend(0, 3, "y2");
    plotter.setLegend(0, 4, "x3");
    plotter.setLegend(0, 5, "y3");
    plotter.setLegend(0, 6, "x4");
    plotter.setLegend(0, 7, "y4");

    plotter.setLegend(1, 0, "v_x");
    plotter.setLegend(1, 1, "v_y");
    plotter.setLegend(1, 2, "v_z");
    plotter.setLegend(1, 3, "w_x");
    plotter.setLegend(1, 4, "w_y");
    plotter.setLegend(1, 5, "w_z");
#endif

    unsigned int iter = 0;
    while (1) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
      vpColVector v = task.computeControlLaw(iter * robot.getSamplingTime());
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

#ifdef VISP_HAVE_DISPLAY
      plotter.plot(0, iter, task.getError());
      plotter.plot(1, iter, v);

      vec_times.push_back(time);

      // Features errors
      vec_x0.push_back(task.getError()[0]);
      vec_y0.push_back(task.getError()[1]);
      vec_x1.push_back(task.getError()[2]);
      vec_y1.push_back(task.getError()[3]);
      vec_x2.push_back(task.getError()[4]);
      vec_y2.push_back(task.getError()[5]);
      vec_x3.push_back(task.getError()[6]);
      vec_y3.push_back(task.getError()[7]);

      // Matplot / Gnuplot
      // Without subplot:
      // matplot::plot(
      //   vec_times, vec_x0, vec_times, vec_y0,
      //   vec_times, vec_x1, vec_times, vec_y1,
      //   vec_times, vec_x2, vec_times, vec_y2,
      //   vec_times, vec_x3, vec_times, vec_y3
      // );
      //
      // Works:
      // ax1->plot(
      //   vec_times, vec_x0, vec_times, vec_y0,
      //   vec_times, vec_x1, vec_times, vec_y1,
      //   vec_times, vec_x2, vec_times, vec_y2,
      //   vec_times, vec_x3, vec_times, vec_y3
      // );
      //
      // Works also:
      matplot::plot(ax1,
        vec_times, vec_x0, vec_times, vec_y0,
        vec_times, vec_x1, vec_times, vec_y1,
        vec_times, vec_x2, vec_times, vec_y2,
        vec_times, vec_x3, vec_times, vec_y3
      );

      // Camera velocity
      vec_vx.push_back(v[0]);
      vec_vy.push_back(v[1]);
      vec_vz.push_back(v[2]);
      vec_wx.push_back(v[3]);
      vec_wy.push_back(v[4]);
      vec_wz.push_back(v[5]);

      // // matplot::plot(vec_times, vec_vx, "r", vec_times, vec_vy, "g", vec_times, vec_vz, "b",
      // //     vec_times, vec_wx, "", vec_times, vec_wy, "", vec_times, vec_wz, "");
      // // Following works:
      // matplot::plot(vec_times, vec_vx, vec_times, vec_vy, vec_times, vec_vz,
      //     vec_times, vec_wx, vec_times, vec_wy, vec_times, vec_wz);
      ax2->plot(
        vec_times, vec_vx, vec_times, vec_vy, vec_times, vec_vz,
        vec_times, vec_wx, vec_times, vec_wy, vec_times, vec_wz
      );

      // matplot::show();
      f->draw();


      // Matplotlib-cpp
      plt::subplot(2, 1, 1);
      plt::grid(true);
      plt::xlim(0.0, time + 1);
      std::vector<double> vec_xy_min {
        *std::min_element(vec_x0.begin(), vec_x0.end()), *std::min_element(vec_y0.begin(), vec_y0.end()),
        *std::min_element(vec_x1.begin(), vec_x1.end()), *std::min_element(vec_y1.begin(), vec_y1.end()),
        *std::min_element(vec_x2.begin(), vec_x2.end()), *std::min_element(vec_y2.begin(), vec_y2.end()),
        *std::min_element(vec_x3.begin(), vec_x3.end()), *std::min_element(vec_y3.begin(), vec_y3.end())
      };
      std::vector<double> vec_xy_max {
        *std::max_element(vec_x0.begin(), vec_x0.end()), *std::max_element(vec_y0.begin(), vec_y0.end()),
        *std::max_element(vec_x1.begin(), vec_x1.end()), *std::max_element(vec_y1.begin(), vec_y1.end()),
        *std::max_element(vec_x2.begin(), vec_x2.end()), *std::max_element(vec_y2.begin(), vec_y2.end()),
        *std::max_element(vec_x3.begin(), vec_x3.end()), *std::max_element(vec_y3.begin(), vec_y3.end())
      };
      double global_min = *std::min_element(vec_xy_min.begin(), vec_xy_min.end());
      global_min_xy = std::min(global_min_xy, global_min);
      double global_max = *std::max_element(vec_xy_min.begin(), vec_xy_min.end());
      global_max_xy = std::max(global_max_xy, global_max);
      double range = global_max_xy - global_min_xy;
      plt::ylim(global_min_xy - 0.1*range, global_max_xy + 0.1*range);
      plt::legend();

      plot_x0.update(vec_times, vec_x0);  plot_y0.update(vec_times, vec_y0);
      plot_x1.update(vec_times, vec_x1);  plot_y1.update(vec_times, vec_y1);
      plot_x2.update(vec_times, vec_x2);  plot_y2.update(vec_times, vec_y2);
      plot_x3.update(vec_times, vec_x3);  plot_y3.update(vec_times, vec_y3);

      // Velocities
      plt::subplot(2, 1, 2);
      // // plt::axis("equal");
      plt::grid(true);
      plt::xlim(0.0, time + 1);
      std::vector<double> vec_v_min {
        *std::min_element(vec_vx.begin(), vec_vx.end()),
        *std::min_element(vec_vy.begin(), vec_vy.end()),
        *std::min_element(vec_vz.begin(), vec_vz.end()),
        //
        *std::min_element(vec_wx.begin(), vec_wx.end()),
        *std::min_element(vec_wy.begin(), vec_wy.end()),
        *std::min_element(vec_wz.begin(), vec_wz.end())
      };
      std::vector<double> vec_v_max {
        *std::max_element(vec_vx.begin(), vec_vx.end()),
        *std::max_element(vec_vy.begin(), vec_vy.end()),
        *std::max_element(vec_vz.begin(), vec_vz.end()),
        //
        *std::max_element(vec_wx.begin(), vec_wx.end()),
        *std::max_element(vec_wy.begin(), vec_wy.end()),
        *std::max_element(vec_wz.begin(), vec_wz.end())
      };
      global_min = *std::min_element(vec_v_min.begin(), vec_v_min.end());
      global_min_v = std::min(global_min_v, global_min);
      global_max = *std::max_element(vec_v_max.begin(), vec_v_max.end());
      global_max_v = std::max(global_max_v, global_max);
      range = global_max_v - global_min_v;
      plt::ylim(global_min_v - 0.1*range, global_max_v + 0.1*range);
      plt::legend();

      // plt::plot(vec_times, vec_vx, "r-");
      // plt::draw();
      plot_vx.update(vec_times, vec_vx);
      plot_vy.update(vec_times, vec_vy);
      plot_vz.update(vec_times, vec_vz);
      //
      plot_wx.update(vec_times, vec_wx);
      plot_wy.update(vec_times, vec_wy);
      plot_wz.update(vec_times, vec_wz);

      plt::pause(0.1);
#endif
      if ((task.getError()).sumSquare() < 0.0001)
        break;

      vpTime::wait(100);

      time += 0.1;
      iter++;
    }
    plt::show();

    std::cout << "Convergence in " << iter << " iterations" << std::endl;

    std::cout << "vec_times.size()=" << vec_times.size() << std::endl;
    std::cout << "vec_times range: " << *std::min_element(vec_times.begin(), vec_times.end()) << " ; "
        << *std::max_element(vec_times.begin(), vec_times.end()) << std::endl;

    std::cout << "vec_vx range: " << *std::min_element(vec_vx.begin(), vec_vx.end()) << " ; "
        << *std::max_element(vec_vx.begin(), vec_vx.end()) << std::endl;

    std::cout << "vec_vy range: " << *std::min_element(vec_vy.begin(), vec_vy.end()) << " ; "
        << *std::max_element(vec_vy.begin(), vec_vy.end()) << std::endl;

    std::cout << "vec_vz range: " << *std::min_element(vec_vz.begin(), vec_vz.end()) << " ; "
        << *std::max_element(vec_vz.begin(), vec_vz.end()) << std::endl;

#ifdef VISP_HAVE_DISPLAY
    plotter.saveData(0, "error.dat");
    plotter.saveData(1, "vc.dat");

    vpDisplay::getClick(plotter.I);
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
