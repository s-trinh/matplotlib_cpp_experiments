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
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    // Matplot / Gnuplot
    auto f = matplot::figure(true);

    vpPoint obj0(-0.1, -0.1, 0);
    vpPoint obj1(0.1, -0.1, 0);
    vpPoint obj2(0.1, 0.1, 0);
    vpPoint obj3(-0.1, 0.1, 0);
    std::cout << "obj0=" << obj0.get_oP().t() << std::endl;
    std::cout << "obj1=" << obj1.get_oP().t() << std::endl;
    std::cout << "obj2=" << obj2.get_oP().t() << std::endl;
    std::cout << "obj3=" << obj3.get_oP().t() << std::endl;

    std::cout << "\ncdMo:\n" << cdMo << std::endl;

    vpPoint c_obj0 = cdMo * obj0;
    vpPoint c_obj1 = cdMo * obj1;
    vpPoint c_obj2 = cdMo * obj2;
    vpPoint c_obj3 = cdMo * obj3;
    std::cout << "\nc_obj0=" << c_obj0.get_cP().t() << std::endl;
    std::cout << "c_obj1=" << c_obj1.get_cP().t() << std::endl;
    std::cout << "c_obj2=" << c_obj2.get_cP().t() << std::endl;
    std::cout << "c_obj3=" << c_obj3.get_cP().t() << std::endl;

    obj0.changeFrame(cdMo);
    obj1.changeFrame(cdMo);
    obj2.changeFrame(cdMo);
    obj3.changeFrame(cdMo);
    std::cout << "\nobj0=" << obj0.get_cP().t() << std::endl;
    std::cout << "obj1=" << obj1.get_cP().t() << std::endl;
    std::cout << "obj2=" << obj2.get_cP().t() << std::endl;
    std::cout << "obj3=" << obj3.get_cP().t() << std::endl;

    // std::vector<double> obj_X {c_obj0.get_oX(), c_obj1.get_oX(), c_obj2.get_oX(), c_obj3.get_oX()};
    // std::vector<double> obj_Y {c_obj0.get_oY(), c_obj1.get_oY(), c_obj2.get_oY(), c_obj3.get_oY()};
    // std::vector<double> obj_Z {c_obj0.get_oZ(), c_obj1.get_oZ(), c_obj2.get_oZ(), c_obj3.get_oZ()};
    std::vector<double> obj_X {obj0.get_X(), obj1.get_X(), obj2.get_X(), obj3.get_X(), obj0.get_X()};
    std::vector<double> obj_Y {obj0.get_Y(), obj1.get_Y(), obj2.get_Y(), obj3.get_Y(), obj0.get_Y()};
    std::vector<double> obj_Z {obj0.get_Z(), obj1.get_Z(), obj2.get_Z(), obj3.get_Z(), obj0.get_Z()};

    for (size_t i = 0; i < obj_X.size(); i++) {
      std::cout << "pt" << i << "=" << obj_X[i] << ", " << obj_Y[i] << ", " << obj_Z[i] << std::endl;
    }

    auto ax1 = matplot::nexttile();
    // auto ax2 = matplot::nexttile(); // equivalent to subplot
    // matplot::xlim({-0.05, 0.2});
    // matplot::ylim({-0.2, 0.05});
    // matplot::zlim({1.5, 0.5});
    matplot::xlabel("X");
    matplot::ylabel("Y");
    matplot::zlabel("Z");

    // matplot::show();


    // https://github.com/lava/matplotlib-cpp/blob/master/examples/lines3d.cpp
    // Matplotlib-cpp
    namespace plt = matplotlibcpp;
    // plt::suptitle("lava/matplotlib-cpp backend");
    // plt::Plot plot_3d("x1");
    // plt::plot3(obj_X, obj_Y, obj_Z);
    // plt::show();


    std::vector<double> cam_X, cam_Y, cam_Z;
    cam_X.push_back(cMo.getTranslationVector()[0]);
    cam_Y.push_back(cMo.getTranslationVector()[1]);
    cam_Z.push_back(cMo.getTranslationVector()[2]);

    std::vector<double> vec_tx, vec_ty, vec_tz;

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

      vec_tx.push_back(cMo.getTranslationVector()[0]);
      vec_ty.push_back(cMo.getTranslationVector()[1]);
      vec_tz.push_back(cMo.getTranslationVector()[2]);

      cam_X.push_back(cMo.getTranslationVector()[0]);
      cam_Y.push_back(cMo.getTranslationVector()[1]);
      cam_Z.push_back(cMo.getTranslationVector()[2]);

      // Matplotlib-cpp
      // plt::plot3(cam_X, cam_Y, cam_Z);
      // // plot_3d.update(cam_X, cam_Y, cam_Z);
      // // plt::plot3(obj_X, obj_Y, obj_Z, std::map<std::string, std::string>(), 0);
      // // plt::clf();
      // plt::plot3(cam_X, cam_Y, cam_Z, std::map<std::string, std::string>(), 0);
      // plt::show();


      // Matplot / Gnuplot
      matplot::hold(matplot::on); // Needed
      auto lO = matplot::plot3(ax1, obj_X, obj_Y, obj_Z, "b");
      lO->color({1, 1, 0});
      lO->marker_color({1, 1, 0});
      auto lC = matplot::plot3(ax1, cam_X, cam_Y, cam_Z, "r+");
      lC->color({1, 0, 1});
      lC->marker_color({1, 0, 1});

      matplot::axis(matplot::equal);
      f->draw();
#endif
      if ((task.getError()).sumSquare() < 0.0001)
        break;

      vpTime::wait(100);

      time += 0.1;
      iter++;
    }

    plt::plot3(obj_X, obj_Y, obj_Z, std::map<std::string, std::string>(), 0);
    plt::plot3(cam_X, cam_Y, cam_Z, std::map<std::string, std::string>(), 0);
    plt::show();

    double min_tx = *std::min_element(vec_tx.begin(), vec_tx.end());
    double max_tx = *std::max_element(vec_tx.begin(), vec_tx.end());

    double min_ty = *std::min_element(vec_ty.begin(), vec_ty.end());
    double max_ty = *std::max_element(vec_ty.begin(), vec_ty.end());

    double min_tz = *std::min_element(vec_tz.begin(), vec_tz.end());
    double max_tz = *std::max_element(vec_tz.begin(), vec_tz.end());

    std::cout << "min_tx=" << min_tx << " ; max_tx=" << max_tx << std::endl;
    std::cout << "min_ty=" << min_ty << " ; max_ty=" << max_ty << std::endl;
    std::cout << "min_tz=" << min_tz << " ; max_tz=" << max_tz << std::endl;

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
