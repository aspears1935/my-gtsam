//TODO: 
//need to change size of x,y,z, arrays to actual size of frames from 256.
//Need to change error model for camera - if missing sonar data, should continue at same speed in the cam direction.
//Need to change the initial estimate array from length to max node because might skip some nodes
//Need to fix yaw marginal covariance growing unbounded. Might just have to not use yaw for examples
//Need to figure out noise model - this one weights sonar heavily so output looks just like sonar. Should also consider that sonar doesn't have all 1000 detected points - might consider having max_detected_points or matches/detected points as a metric in the calculation of noise.

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file OdometryExample.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements
 * @author Frank Dellaert
 */

/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>


//*************AMS ADDED**************//
#define PI 3.14159265
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

using namespace std;
using namespace gtsam;

#define VERBOSE false
#define CAM_CORNERS_WEIGHT 0.3333333
#define CAM_MATCHES_WEIGHT 0.3333333
#define CAM_INLIERS_WEIGHT 0.3333333
#define MAX_CAM_CORNERS 1000 //Should most likely be 1000
#define MAX_SON_CORNERS 1000 //Should most likely be 1000
#define ZERO_NOISE 0.000001//0.00001
#define SON_INLIERS_THRESH 200 //Number of inliers considered to hit the low noise plateau  _______
#define CAM_INLIERS_THRESH 200 //Number of inliers considered to hit the low noise plateau /

int main(int argc, char** argv) 
{
  char sonInputFileName[256];
  char camInputFileName[256];

  double x_sum_cam = 0;
  double y_sum_cam = 0;
  double z_sum_cam = 0;
  double roll_sum_cam = 0;
  double pitch_sum_cam = 0;
  double yaw_sum_cam = 0;

  double x_sum_son = 0;
  double y_sum_son = 0;
  double z_sum_son = 0;
  double roll_sum_son = 0;
  double pitch_sum_son = 0;
  double yaw_sum_son = 0;

  int firstSonNode, firstCamNode, lastSonNode, lastCamNode;

  if(argc < 1)
    {
      cout << "Not Enough Args. Usage: ./gtsam" << endl << "Example: ./gtsam" << endl;
      return 0;
    }

  //Write the column headings on the output file:
  ofstream outfile("outputGTSAM.csv");
  outfile << "frame/time?;x;y;z;roll;pitch;yaw;";
  outfile << "xInit;yInit;zInit;rollInit;pitchInit;yawInit;";
  outfile << "xson;yson;zson;rollson;pitchson;yawson;";
  outfile << "xsonSum;ysonSum;zsonSum;rollsonSum;pitchsonSum;yawsonSum;";
  outfile << "xcam;ycam;zcam;rollcam;pitchcam;yawcam;";
  outfile << "xcamSum;ycamSum;zcamSum;rollcamSum;pitchcamSum;yawcamSum;";
  outfile << "covx;covy;covz;covroll;covpitch;covyaw;";
  outfile << "covxson;covyson;covzson;covrollson;covpitchson;covyawson;";
  outfile << "covxcam;covycam;covzcam;covrollcam;covpitchcam;covyawcam;" << endl; 


  // Create an empty nonlinear factor graph
  NonlinearFactorGraph graph;
  NonlinearFactorGraph graphSonOnly;
  NonlinearFactorGraph graphCamOnly;

  /*Rot3 priorMeanRot3 = Rot3::ypr(0,0,0);
  //  Rot3 priorMeanRot3 = Rot3::ypr(yaw_arr[0], pitch_arr[0], roll_arr[0]);
  //  Rot3 priorMeanRot3 = Rot3::ypr(PI/3, PI/2, PI); //RADIANS!

  cout << "Prior Rot Matrix:" << endl;
  priorMeanRot3.print();

  Vector3 current_ypr;
  current_ypr = priorMeanRot3.ypr();
  cout << "Prior Yaw = " << current_ypr[0]*180/PI << endl;
  cout << "Prior Pitch = " << current_ypr[1]*180/PI << endl;
  cout << "Prior Roll = " << current_ypr[2]*180/PI << endl;

  Point3 priorMeanPoint3(0,0,0);
  priorMeanPoint3.print("Prior 3D Point = ");
  */

  //  Pose3 priorMean(priorMeanRot3,priorMeanPoint3);
  Pose3 priorMean(Rot3::ypr(0,0,0), Point3(0,0,0));
  cout << "priorMean=" << endl;
  priorMean.print();

  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  //  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

  graph.add(PriorFactor<Pose3>(0, priorMean, priorNoise));
  graphSonOnly.add(PriorFactor<Pose3>(0, priorMean, priorNoise));
  graphCamOnly.add(PriorFactor<Pose3>(0, priorMean, priorNoise));

  Values initial;
  Values initialSon;
  Values initialSonFuse;
  Values initialCam;
  Values initialCamSon;

  //NOTE: addedErr - This is confusing. When both before and after prior addedErr is 7 or less, it has no effect - all the same. When 8 before and after, it begins to change the output. When it is 10, it changes a lot. When 150, doesn't change at all and back to no effect like 0. This is for the simulated data input rectangle200x100 and the sonar only data (the fused data performs differently).
  double addedErr = 0; //Debug: should be 0
   
  initial.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
  initialSon.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
  initialSonFuse.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
  initialCam.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
  initialCamSon.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));

  //Note: changed just this and didn't effect output of sonar results 0-10
  //addedErr = 0; //Debug: should be 0

  // Add odometry factors
  //Pose3 odometry_0(Rot3::ypr(0,0,0), Point3(0,0,0));
  //Pose3 odometry_x(Rot3::ypr(0,0,0), Point3(1,0,0));
  //Pose3 odometry_y(Rot3::ypr(0,0,0), Point3(0,1,0));
  //Pose3 odometry_z(Rot3::ypr(0,0,0), Point3(0,0,1));
  //Pose3 odometry_yaw(Rot3::ypr(PI/160,0,0), Point3(0,0,0));
  //  odometry.print("Odometry=");

  // For simplicity, we will use the same noise model for each odometry factor
  //  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  //noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << 1, 1, 1, 1, 1, 1));

  //EssentialMatrix e_matrix(zeroRot3, xUnit3);
  //  EssentialMatrix e_matrix(Rot3::ypr(yaw_arr[0],pitch_arr[0],roll_arr[0]), Unit3(xunit_arr[0],yunit_arr[0],zunit_arr[0]));

  //noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));


  /*********************** Add Sonar Nodes *************************/
  int i;
  double sonNoiseMult = 0.01;
  double sonNoiseTransl = 0.01*sonNoiseMult; //allInliers=>0.0001m, noInliers=>0.01m
  //double sonNoiseTransl = 10*sonNoiseMult; //allInliers=>0.1m, noInliers=>10m
  double sonNoiseRot = 0.001*sonNoiseMult; //allInliers=>0.00001rad, noInliers=>0.001 deg
  //double sonNoiseRot = 10*sonNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
  double sonFuseNoiseTransl = sonNoiseMult*100*sonNoiseTransl; //*sonNoiseMult*100 to reduce effect 
  //double sonFuseNoiseRot = sonNoiseMult*100*sonNoiseRot; //*sonNoiseMult*100 to reduce effect 
  
  noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << sonNoiseTransl, sonNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, sonNoiseRot));
  noiseModel::Diagonal::shared_ptr sonarFuseNoise = noiseModel::Diagonal::Variances((Vector(6) << sonFuseNoiseTransl, sonFuseNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, sonNoiseRot));
  
  //NOTE: If make sonar noise 0.1, causes indeterminate solution! 
  //...Problem with yaw growing unbounded. For now, set to ZERO_NOISE!!! 
  noiseModel::Diagonal::shared_ptr constSonarNoise2dec = noiseModel::Diagonal::Variances((Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01));
  noiseModel::Diagonal::shared_ptr constSonarNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  noiseModel::Diagonal::shared_ptr constSonarNoise1 = noiseModel::Diagonal::Variances((Vector(6) << 1,1,1,1,1,1));
  noiseModel::Diagonal::shared_ptr constSonarNoise2 = noiseModel::Diagonal::Variances((Vector(6) << 2,2,2,2,2,2));
  noiseModel::Diagonal::shared_ptr constSonarNoise3 = noiseModel::Diagonal::Variances((Vector(6) << 3,3,3,3,3,3));
  noiseModel::Diagonal::shared_ptr constSonarNoise4 = noiseModel::Diagonal::Variances((Vector(6) << 4,4,4,4,4,4));
  noiseModel::Diagonal::shared_ptr constSonarNoise5 = noiseModel::Diagonal::Variances((Vector(6) << 5,5,5,5,5,5));
  noiseModel::Diagonal::shared_ptr constSonarNoise10 = noiseModel::Diagonal::Variances((Vector(6) << 10,10,10,10,10,10));
  noiseModel::Diagonal::shared_ptr zeroSonarNoise = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

  /*  for(i=0; i<10; i++)
    {
      graph.add(BetweenFactor<Pose3>(i*2, i*2+2, Pose3(Rot3::ypr(0,0,0), Point3(2,0,0)), constSonarNoise1));
      graphSonOnly.add(BetweenFactor<Pose3>(i*2, i*2+2, Pose3(Rot3::ypr(0,0,0), Point3(2,0,0)), zeroSonarNoise));
      initialSon.insert(i*2+2, Pose3(Rot3::ypr(addedErr,0,0), Point3(2+addedErr,addedErr,0)));
      initialSonFuse.insert(i*2+2, Pose3(Rot3::ypr(addedErr,0,0), Point3(2+addedErr,addedErr,0)));
    }
  */

  for(i=0; i<20; i++)
    {
      /*      if(i<10)
	graph.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(0,0,0), Point3(1,0,0)), constSonarNoise1));
	else*/
	graph.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(0,0,0), Point3(1,1,0)), constSonarNoise2));
    }
  

  //  graph.print();
  graphSonOnly.print();

  //  graphSonOnly.print();
  //  initialSon.print();

  /*********************** Add Camera Nodes *************************/

      //double camNoiseTransl = camNoiseMult; //allInliers=>0.01 , noInliers=>1. transl is a unit vector
      //      double camNoiseTransl = 10*camNoiseMult; //allInliers=>0.1 , noInliers=>10. transl is a unit vector
  double camNoiseMult = 0.01;
      double camNoiseTransl = 0.01*camNoiseMult; //allInliers=>0.0001 , noInliers=>0.01. transl is a unit vector
      double camNoiseRot = 0.001*camNoiseMult; //allInliers=>0.00001deg, noInliers=>0.001 deg
      //double camNoiseRot = 10*camNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
      double camSonNoiseTransl = 0.01;///camNoiseMult*100*sqrt(prevSonNoiseTransl*camNoiseTransl); //Geometric Mean * camNoiseMult*100 to reduce effect 


      noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << camNoiseTransl, camNoiseTransl, ZERO_NOISE/*camNoiseTransl*/, ZERO_NOISE/*camNoiseRot*/, camNoiseRot)); //4th (2nd from last) was ZERO_NOISE but changed.??

      noiseModel::Diagonal::shared_ptr constCameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));//ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr cameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camNoiseTransl, camNoiseTransl, camNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));

      noiseModel::Diagonal::shared_ptr cameraSonarNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camSonNoiseTransl, camSonNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, camNoiseRot));

      noiseModel::Diagonal::shared_ptr constCameraNoise6_2dec = noiseModel::Diagonal::Variances((Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
      noiseModel::Diagonal::shared_ptr constCameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
      noiseModel::Diagonal::shared_ptr constCameraNoise6_1 = noiseModel::Diagonal::Variances((Vector(6) << 1,1,1,1,1,1)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
      noiseModel::Diagonal::shared_ptr constCameraNoise6_2 = noiseModel::Diagonal::Variances((Vector(6) << 2,2,2,2,2,2)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
      noiseModel::Diagonal::shared_ptr constCameraNoise6_5 = noiseModel::Diagonal::Variances((Vector(6) << 5,5,5,5,5,5)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
      noiseModel::Diagonal::shared_ptr constCameraNoise6_10 = noiseModel::Diagonal::Variances((Vector(6) << 10,10,10,10,10,10)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr zeroNoise6 = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr bigNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 10000,10000,10000,10000,10000,10000));
      
      /*NOTE: The 4th variance (Roll?) in the output is constant at the maximum, which seems to be the estimated input value for the first guess. The 5 values in the noise input vector seem to correspond to: 1-1, 2-2, 3-3, 4-5, 5-6 to the output covariance 6 item vector. This was determined by changing one input noise value at a time and looking at the resultant noise vector at the output. Seems to be a problem with the GTSAM code? So, for now, since we aren't using roll at all, just ignore it. BUT make sure to remember the correspondences - so set first three in the Noise(5) vector to translNoise and the last two to rotNoise. */

      for(i=0; i< 20; i++)
	{
	  //graph.add(EssentialMatrixConstraint(i, i+1, EssentialMatrix(Rot3::ypr(0,0,0), Unit3(1,0,0)), cameraNoise));
	  graph.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(0,0,0), Point3(3,5,0)), constCameraNoise6_5));

	  graphCamOnly.add(EssentialMatrixConstraint(i, i+1, EssentialMatrix(Rot3::ypr(0,0,0), Unit3(1,0,0)), cameraNoise));
	  graphCamOnly.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(0,0,0), Point3(1,0,0)), cameraNoise6)); //Have to add this or else underconstrained

	  initialCam.insert(i+1, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(1+addedErr,addedErr,addedErr)));
      //Below is for the fused estimates with sonar magnitudes
	  initialCamSon.insert(i+1, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(1+addedErr,addedErr,addedErr)));

	}

      //graphCamOnly.print();
      //initialCamSon.print();
      //  graph.print();

  /********************Create Fused Initial Guess*******************/
  for(i=0; i<20; i++)
    {
      initial.insert(i+1, Pose3(Rot3::ypr(0,0,0), Point3(i+1,0,0)));
    }
  initial.print();

  // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  cout << "Optimized Fusion Graph" << endl;
  Values resultSonOnly = LevenbergMarquardtOptimizer(graphSonOnly, initialSon).optimize();
  cout << "Optimized Sonar Graph" << endl;
  Values resultCamOnly = LevenbergMarquardtOptimizer(graphCamOnly, initialCam).optimize();
  cout << "Optimized Camera Graph" << endl;

  // result.print("Final Result:\n");

  //resultSonOnly.print();

  Marginals marginals(graph, result);
  cout << "Calculated Fusion Marginals" << endl;
  Marginals marginalsSonOnly(graphSonOnly, resultSonOnly);
  cout << "Calculated Sonar Marginals" << endl;
  Marginals marginalsCamOnly(graphCamOnly, resultCamOnly);
  cout << "Calculated Camera Marginals" << endl;

  for(i = 0; i< 21; i++)
    {
      cout << i << " Result: x,y,yaw = " << result.at<Pose3>(i).x() << "," << result.at<Pose3>(i).y() << "," << result.at<Pose3>(i).rotation().yaw() << endl;
      
      /*      if(i%2 == 0) //only even nodes
	cout << i << " Son Result: x,y,yaw = " << resultSonOnly.at<Pose3>(i).x() << "," << resultSonOnly.at<Pose3>(i).y() << "," << resultSonOnly.at<Pose3>(i).rotation().yaw() << endl;
      
	cout << i << " Cam Result: x,y,yaw = " << resultCamOnly.at<Pose3>(i).x() << "," << resultCamOnly.at<Pose3>(i).y() << "," << resultCamOnly.at<Pose3>(i).rotation().yaw() << endl;*/
    }

  
  for(i=0;i<21;i++)
    {
      cout << "Result Node " << i << endl;

      double xprint = result.at<Pose3>(i).x();
      double yprint = result.at<Pose3>(i).y();
      double zprint = result.at<Pose3>(i).z();
      double rollprint = result.at<Pose3>(i).rotation().roll();
      double pitchprint = result.at<Pose3>(i).rotation().pitch();
      double yawprint = result.at<Pose3>(i).rotation().yaw();

      //Initial Guess: 
      double xinitprint = initial.at<Pose3>(i).x();
      double yinitprint = initial.at<Pose3>(i).y();
      double zinitprint = initial.at<Pose3>(i).z();
      double rollinitprint = initial.at<Pose3>(i).rotation().roll();
      double pitchinitprint = initial.at<Pose3>(i).rotation().pitch();
      double yawinitprint = initial.at<Pose3>(i).rotation().yaw();

      if(abs(xprint) < 0.001)
	xprint = 0;
      if(abs(yprint) < 0.001)
	yprint = 0;
      if(abs(zprint) < 0.001)
	zprint = 0;
      if(abs(rollprint) < 0.001)
	rollprint = 0;
      if(abs(pitchprint) < 0.001)
	pitchprint = 0;
      if(abs(yawprint) < 0.001)
	yawprint = 0;

      if(abs(xinitprint) < 0.001)
	xinitprint = 0;
      if(abs(yinitprint) < 0.001)
	yinitprint = 0;
      if(abs(zinitprint) < 0.001)
	zinitprint = 0;
      if(abs(rollinitprint) < 0.001)
	rollinitprint = 0;
      if(abs(pitchinitprint) < 0.001)
	pitchinitprint = 0;
      if(abs(yawinitprint) < 0.001)
	yawinitprint = 0;

      cout.precision(3); 
      cout << i << " Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      cout << i << " Initial: x,y,yaw = " << xinitprint << ";" << yinitprint << ";" << yawinitprint << endl;
      //cout << marginals.marginalCovariance(i) << ",";

      // Calculate and print marginal covariances for all variables
      //cout.precision(2); 
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;

      cout << "X" << i << " covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginals.marginalCovariance(i)(i1,i1) << ",";
	}
      cout << endl;   
 }

  return 0; // DEBUG DELETE!!!

      //-------------SonOnly-------------//
  for(i=0;i<11;i++)
    {
      cout << "SONAR Result Node " << i << endl;

      double xprint = resultSonOnly.at<Pose3>(i).x();
      double yprint = resultSonOnly.at<Pose3>(i).y();
      double zprint = resultSonOnly.at<Pose3>(i).z();
      double rollprint = resultSonOnly.at<Pose3>(i).rotation().roll();
      double pitchprint = resultSonOnly.at<Pose3>(i).rotation().pitch();
      double yawprint = resultSonOnly.at<Pose3>(i).rotation().yaw();
      
      //Initial Guess: 
      double xinitprint = initialSon.at<Pose3>(i).x();
      double yinitprint = initialSon.at<Pose3>(i).y();
      double zinitprint = initialSon.at<Pose3>(i).z();
      double rollinitprint = initialSon.at<Pose3>(i).rotation().roll();
      double pitchinitprint = initialSon.at<Pose3>(i).rotation().pitch();
      double yawinitprint = initialSon.at<Pose3>(i).rotation().yaw();
      
      if(abs(xprint) < 0.001)
	xprint = 0;
      if(abs(yprint) < 0.001)
	yprint = 0;
      if(abs(zprint) < 0.001)
	zprint = 0;
      if(abs(rollprint) < 0.001)
	rollprint = 0;
      if(abs(pitchprint) < 0.001)
	pitchprint = 0;
      if(abs(yawprint) < 0.001)
	yawprint = 0;
      
      if(abs(xinitprint) < 0.001)
	xinitprint = 0;
      if(abs(yinitprint) < 0.001)
	yinitprint = 0;
      if(abs(zinitprint) < 0.001)
	zinitprint = 0;
      if(abs(rollinitprint) < 0.001)
	rollinitprint = 0;
      if(abs(pitchinitprint) < 0.001)
	pitchinitprint = 0;
      if(abs(yawinitprint) < 0.001)
	yawinitprint = 0;
      
      cout << i << " Sonar Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      
      cout << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
      cout << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";

       // Calculate and print marginal covariances for all variables
      cout.precision(2); 
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;

      cout << "X" << i << " Son covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  //Note: If problem here, probably yaw covariance too big
	  cout << marginalsSonOnly.marginalCovariance(i)(i1,i1) << ",";
	}
      cout << endl;

      
    }
    
      //-------------CamOnly----------------//
  for(i=0;i<21;i++)
    {
      cout << "CAM Result Node " << i << endl;

      double xprint = resultCamOnly.at<Pose3>(i).x();
      double yprint = resultCamOnly.at<Pose3>(i).y();
      double zprint = resultCamOnly.at<Pose3>(i).z();
      double rollprint = resultCamOnly.at<Pose3>(i).rotation().roll();
      double pitchprint = resultCamOnly.at<Pose3>(i).rotation().pitch();
      double yawprint = resultCamOnly.at<Pose3>(i).rotation().yaw();
      
      //Initial Guess: 
      double xinitprint = initialCam.at<Pose3>(i).x();
      double yinitprint = initialCam.at<Pose3>(i).y();
      double zinitprint = initialCam.at<Pose3>(i).z();
      double rollinitprint = initialCam.at<Pose3>(i).rotation().roll();
      double pitchinitprint = initialCam.at<Pose3>(i).rotation().pitch();
      double yawinitprint = initialCam.at<Pose3>(i).rotation().yaw();
      
      if(abs(xprint) < 0.001)
	xprint = 0;
      if(abs(yprint) < 0.001)
	yprint = 0;
      if(abs(zprint) < 0.001)
	zprint = 0;
      if(abs(rollprint) < 0.001)
	rollprint = 0;
      if(abs(pitchprint) < 0.001)
	pitchprint = 0;
      if(abs(yawprint) < 0.001)
	yawprint = 0;
      
      if(abs(xinitprint) < 0.001)
	xinitprint = 0;
      if(abs(yinitprint) < 0.001)
	yinitprint = 0;
      if(abs(zinitprint) < 0.001)
	zinitprint = 0;
      if(abs(rollinitprint) < 0.001)
	rollinitprint = 0;
      if(abs(pitchinitprint) < 0.001)
	pitchinitprint = 0;
      if(abs(yawinitprint) < 0.001)
	yawinitprint = 0;
      
      cout << i << " Cam Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      cout << i << " Cam Only Initial: x,y,yaw = " << xinitprint << "," << yinitprint << "," << yawinitprint << endl;

       // Calculate and print marginal covariances for all variables
      cout.precision(2); 
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;

      cout << "X" << i << " Cam covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginalsCamOnly.marginalCovariance(i)(i1,i1) << ",";
	  outfile << marginalsCamOnly.marginalCovariance(i)(i1,i1) << ";";
	}
      cout << endl;
    }
    
  cout << "Initial Estimate Error (Fused): " <<  graph.error(initial) << endl;
  cout << "Result Error (Fused): " << graph.error(result) << endl;

  //--------------------- Son Only -----------------------//
  cout << "Initial Estimate Error (Sonar): " <<  graphSonOnly.error(initial) << endl;
  cout << "Result Error (Sonar): " << graphSonOnly.error(resultSonOnly) << endl;

  //--------------------- Cam Only -----------------------//
  cout << "Initial Estimate Error (Camera): " <<  graphCamOnly.error(initial) << endl;
  cout << "Result Error (Camera): " << graphCamOnly.error(resultCamOnly) << endl;

  return 0;
}
