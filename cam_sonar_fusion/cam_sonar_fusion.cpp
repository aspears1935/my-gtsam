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
#define ZERO_NOISE 0.00001
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

  if(argc < 2)
    {
      cout << "Not Enough Args. Usage: ./gtsam <sonar input CSV file with x,y,theta> <camera input CSV file with x,y,z,roll,pitch,yaw>" << endl << "Example: ./gtsam soninput.csv caminput.csv" << endl;
      return 0;
    }
  else
    {
      strcpy(sonInputFileName, argv[1]);  
      strcpy(camInputFileName, argv[2]);  
    }

  ifstream inFileSon(sonInputFileName);
  ifstream inFileCam(camInputFileName);

  string tmpstring;
  
  int lengthSon, lengthCam;

  //Get number of rows for each file input:
  getline(inFileSon,tmpstring,';'); //First line should just have length listed
  if(strcmp(tmpstring.c_str(),"length=")==0) //Found correct string
    cout << "Found length" << endl;
  else
    {
      cout << "Couldn't find length." << endl;
      return 0;
    }
  getline(inFileSon,tmpstring,';'); //First line should just have length listed
  lengthSon = (int)(atoi(tmpstring.c_str()));
  cout << "lengthSon=" << lengthSon << endl;
  getline(inFileSon,tmpstring,'\n'); //Discard rest of line


  getline(inFileCam,tmpstring,';'); //First line should just have length listed
  if(strcmp(tmpstring.c_str(),"length=")==0) //Found correct string
    cout << "Found length" << endl;
  else
    {
      cout << "Couldn't find length." << endl;
      return 0;
    }
  getline(inFileCam,tmpstring,';'); //First line should just have length listed
  lengthCam = (int)(atoi(tmpstring.c_str()));
  cout << "lengthCam=" << lengthCam << endl;
  getline(inFileCam,tmpstring,'\n'); //Discard rest of line

  //--------------Look for headings:----------------------//
  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t1")==0) //FOUND t1 HEADING
    cout << "FOUND t1 HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND t1 HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t2")==0) //FOUND t2 HEADING
    cout << "FOUND t2 HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND t2 HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"x")==0) //FOUND son x HEADING
    cout << "FOUND X HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND X HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"y")==0) //FOUND son y HEADING
    cout << "FOUND Y HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND Y HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"yaw")==0) //FOUND son yaw HEADING
    cout << "FOUND YAW HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND YAW HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numCorners")==0) //FOUND son numCorners HEADING
    cout << "FOUND NUMCORNERS HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMCORNERS HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numMatches")==0) //FOUND son numMatches HEADING
    cout << "FOUND NUMMATCHES HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMMATCHES HEADING! - " << tmpstring << endl;

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numInliers")==0) //FOUND son numInliers HEADING
    cout << "FOUND NUMINLIERS HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMINLIERS HEADING! - " << tmpstring << endl;

  //Read in camera input file headings:
  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t1")==0) //FOUND cam t1 HEADING
    cout << "FOUND t1 HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND t1 HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t2")==0) //FOUND cam t2 HEADING
    cout << "FOUND t2 HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND t2 HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"x")==0) //FOUND cam x HEADING
    cout << "FOUND x HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND x HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"y")==0) //FOUND cam y HEADING
    cout << "FOUND y HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND y HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"z")==0) //FOUND cam z HEADING
    cout << "FOUND z HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND z HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"roll")==0) //FOUND cam roll HEADING
    cout << "FOUND roll HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND roll HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"pitch")==0) //FOUND cam pitch HEADING
    cout << "FOUND pitch HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND pitch HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"yaw")==0) //FOUND cam yaw HEADING
    cout << "FOUND yaw HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND yaw HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numCorners")==0) //FOUND cam numCorners HEADING
    cout << "FOUND NUMCORNERS HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMCORNERS HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numMatches")==0) //FOUND cam numMatches HEADING
    cout << "FOUND NUMMATCHES HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMMATCHES HEADING! - " << tmpstring << endl;

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numInliers")==0) //FOUND cam numInliers HEADING
    cout << "FOUND NUMINLIERS HEADING! - " << tmpstring << endl;
  else
    cout << "DIDN'T FIND NUMINLIERS HEADING! - " << tmpstring << endl;

  //Create input data arrays:
  float * t1son_arr;
  float * t2son_arr;
  float * x_son_arr;
  float * y_son_arr;
  float * yaw_son_arr;
  float * numCorners_son_arr;
  float * numMatches_son_arr;
  float * numInliers_son_arr;
  double * sonNoiseTranslArr;
  sonNoiseTranslArr = new double[lengthSon];
  t1son_arr = new float[lengthSon];
  t2son_arr = new float[lengthSon];
  x_son_arr = new float[lengthSon];
  y_son_arr = new float[lengthSon];
  yaw_son_arr = new float[lengthSon];
  numCorners_son_arr = new float[lengthSon];
  numMatches_son_arr = new float[lengthSon];
  numInliers_son_arr = new float[lengthSon];

  float * t1cam_arr;
  float * t2cam_arr;
  float * xunit_arr;
  float * yunit_arr;
  float * zunit_arr;
  float * roll_arr;
  float * pitch_arr;
  float * yaw_arr;
  float * numCorners_arr;
  float * numMatches_arr;
  float * numInliers_arr;
  t1cam_arr = new float[lengthCam];
  t2cam_arr = new float[lengthCam];
  xunit_arr = new float[lengthCam];
  yunit_arr = new float[lengthCam];
  zunit_arr = new float[lengthCam];
  roll_arr = new float[lengthCam];
  pitch_arr = new float[lengthCam];
  yaw_arr = new float[lengthCam];
  numCorners_arr = new float[lengthCam];
  numMatches_arr = new float[lengthCam];
  numInliers_arr = new float[lengthCam];

  //Read in the input data:
  for(int i=0; i<lengthSon; i++)
    {
      //Sonar Data:
      getline(inFileSon,tmpstring,';');
      t1son_arr[i] = 10*(float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      t2son_arr[i] = 10*(float)(atof(tmpstring.c_str()));
      cout << "son t1,t2 = " << t1son_arr[i] << "," << t2son_arr[i] << endl;

      getline(inFileSon,tmpstring,';');
      x_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      y_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      yaw_son_arr[i] = (float)(atof(tmpstring.c_str()));

      getline(inFileSon,tmpstring,';');
      numCorners_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      numMatches_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      numInliers_son_arr[i] = (float)(atof(tmpstring.c_str()));
    }

  for(int i=0; i<lengthCam; i++)
    {
      //Camera Data:
      getline(inFileCam,tmpstring,';');
      t1cam_arr[i] = 10*(float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      t2cam_arr[i] = 10*(float)(atof(tmpstring.c_str()));
      cout << "cam t1,t2 = " << t1cam_arr[i] << "," << t2cam_arr[i] << endl;

      getline(inFileCam,tmpstring,';');
      xunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      yunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      zunit_arr[i] = (float)(atof(tmpstring.c_str()));
    
      //Convert from degrees to radians:
      getline(inFileCam,tmpstring,';');
      roll_arr[i] = (float)(atof(tmpstring.c_str()))*PI/180;
      getline(inFileCam,tmpstring,';');
      pitch_arr[i] = (float)(atof(tmpstring.c_str()))*PI/180;
      getline(inFileCam,tmpstring,';');
      yaw_arr[i] = (float)(atof(tmpstring.c_str()))*PI/180;

      getline(inFileCam,tmpstring,';');
      numCorners_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      numMatches_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      numInliers_arr[i] = (float)(atof(tmpstring.c_str()));
    }    

  //Find the first and last nodes:
  firstSonNode = t1son_arr[0];
  firstCamNode = t1cam_arr[0];
  lastSonNode = t2son_arr[lengthSon-1];
  lastCamNode = t2cam_arr[lengthCam-1];
  cout << "Sonar Nodes: " << firstSonNode << ":" << lastSonNode << endl;
  cout << "Camera Nodes: " << firstCamNode << ":" << lastCamNode << endl;

  //Print out the data that was read in:
  if(VERBOSE)
    {
      for(int i=0; i<lengthSon; i++)
	{
	  cout << "xson,yson,yawson=" << x_son_arr[i] << "," << y_son_arr[i] << "," << yaw_son_arr[i] << endl;
	}
      for(int i=0; i<lengthCam; i++)
	{
	  cout << "x,y,z=" <<  xunit_arr[i] << "," << yunit_arr[i] << "," << zunit_arr[i] << endl;
	  cout << "roll,pitch,yaw=" << roll_arr[i] << "," << pitch_arr[i] << "," << yaw_arr[i] << endl;
	}
    }


  //Write the column headings on the output file:
  ofstream outfile("outputGTSAM.csv");
  outfile << "frame/time?;x;y;z;roll;pitch;yaw;";
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
  Values initialCam;
  Values initialCamSon;

  //NOTE: addedErr - This is confusing. When both before and after prior addedErr is 7 or less, it has no effect - all the same. When 8 before and after, it begins to change the output. When it is 10, it changes a lot. When 150, doesn't change at all and back to no effect like 0. This is for the simulated data input rectangle200x100 and the sonar only data (the fused data performs differently).
  double addedErr = 0; //Debug: should be 0
   
  initial.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
  initialSon.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));
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
  EssentialMatrix e_matrix(Rot3::ypr(yaw_arr[0],pitch_arr[0],roll_arr[0]), Unit3(xunit_arr[0],yunit_arr[0],zunit_arr[0]));

  //noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));


  /*********************** Add Sonar Nodes *************************/
  int i;
  x_sum_son = 0;
  y_sum_son = 0;
  yaw_sum_son = 0;
  for(i = 0; i<lengthSon; i++)    
    {
      cout << i+1 << "/" << lengthSon << endl;
      x_sum_son += x_son_arr[i];
      y_sum_son += y_son_arr[i];
      yaw_sum_son += yaw_son_arr[i];

      double sonNoiseMult;
      if(VERBOSE)
	cout << "son corners,matches,inliers=" << numCorners_son_arr[i] << "," << numMatches_son_arr[i] << "," << numInliers_son_arr[i] << endl;
      //      sonNoiseMult = 1-(numInliers_son_arr[i]/MAX_SON_CORNERS);
      if(numInliers_son_arr[i] < SON_INLIERS_THRESH)
	sonNoiseMult = 1-(numInliers_son_arr[i]/SON_INLIERS_THRESH);
      else
	sonNoiseMult = 0.01;

      if(sonNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
	sonNoiseMult = 0.01;
      
      if(VERBOSE)
	cout << "Son Noise multipler=" << sonNoiseMult << endl;
      double sonNoiseTransl = 0.01*sonNoiseMult; //allInliers=>0.0001m, noInliers=>0.01m
      //double sonNoiseTransl = 10*sonNoiseMult; //allInliers=>0.1m, noInliers=>10m
      double sonNoiseRot = 0.001*sonNoiseMult; //allInliers=>0.00001rad, noInliers=>0.001 deg
      //double sonNoiseRot = 10*sonNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
      sonNoiseTranslArr[i] = sonNoiseTransl;

      if(VERBOSE)
	cout << "Son Noise: " << sonNoiseTransl << "," << sonNoiseRot << endl;

      noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << sonNoiseTransl, sonNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, sonNoiseRot));

      //NOTE: If make sonar noise 0.1, causes indeterminate solution! 
      //...Problem with yaw growing unbounded. For now, set to ZERO_NOISE!!! 
      noiseModel::Diagonal::shared_ptr constSonarNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.01, 0.01, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      graph.add(BetweenFactor<Pose3>(t1son_arr[i], t2son_arr[i], Pose3(Rot3::ypr(yaw_son_arr[i],0,0), Point3(x_son_arr[i],y_son_arr[i],0)), sonarNoise));
      graphSonOnly.add(BetweenFactor<Pose3>(t1son_arr[i], t2son_arr[i], Pose3(Rot3::ypr(yaw_son_arr[i],0,0), Point3(x_son_arr[i],y_son_arr[i],0)), sonarNoise));

      cout << "Graph Node - Son (x,y,yaw): " << x_son_arr[i] << "," << y_son_arr[i] << "," << yaw_son_arr[i] << " - Noise (Transl, Rot): " << sonNoiseTransl << "," << sonNoiseRot << endl;

      initialSon.insert(t2son_arr[i], Pose3(Rot3::ypr(yaw_sum_son+addedErr,0,0), Point3(x_sum_son+addedErr,y_sum_son+addedErr,0)));
      
      if(VERBOSE)
	{
	  cout << "sonar sum: " << x_sum_son << "," << y_sum_son << "," << yaw_sum_son << endl;
	}

    }

  //  graphSonOnly.print();
  //  initialSon.print();

  /*********************** Add Camera Nodes *************************/
  x_sum_cam=0;
  y_sum_cam=0;
  z_sum_cam=0;
  roll_sum_cam=0;
  pitch_sum_cam=0;
  yaw_sum_cam=0;
  double x_sum_camSon = 0;
  double y_sum_camSon = 0;
  double z_sum_camSon = 0;  
  int iSonVel = 0;
  //initialize velocities from first sonar data:
  double prev_velx = x_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double prev_vely = y_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double prev_velz = 0;
  double prevSonNoiseTransl = sonNoiseTranslArr[0];
  double prev_unitx = xunit_arr[0];
  double prev_unity = yunit_arr[0];
  double prev_unitz = zunit_arr[0];
  double prev_vel_mag = 0;

  for(i = 0; i<lengthCam; i++)
    {
      cout << i+1 << "/" << lengthCam << endl;
      x_sum_cam += xunit_arr[i];
      y_sum_cam += yunit_arr[i];
      z_sum_cam += zunit_arr[i];
      roll_sum_cam += roll_arr[i];
      pitch_sum_cam += pitch_arr[i];
      yaw_sum_cam += yaw_arr[i];

      //Get previous velocities from sonar data and the corresponding cam unit vector:
      while(t2son_arr[iSonVel+1] <= t2cam_arr[i]) //want sonar t2 that is closest but less than or equal to camera t2
	{
	  if(numInliers_arr[i] >= CAM_INLIERS_THRESH)
	    {
	      prev_unitx = xunit_arr[i]; //get the closest cam unit vector to the sonar vector
	      prev_unity = yunit_arr[i];
	      prev_unitz = zunit_arr[i];
	    }

	  if(iSonVel >= lengthSon-1)
	    break;
	  else
	    iSonVel++;
	}

      if(VERBOSE)
	cout << "Sonar Node: " << t2son_arr[iSonVel] << " -- Using Velocity from Cam Node: " << t2cam_arr[i] << endl;

      if(numInliers_son_arr[iSonVel] >= SON_INLIERS_THRESH) //only update prev vel if sufficient inliers
	{
	  prev_velx = x_son_arr[iSonVel]/(t2son_arr[iSonVel]-t1son_arr[iSonVel]);
	  prev_vely = y_son_arr[iSonVel]/(t2son_arr[iSonVel]-t1son_arr[iSonVel]);
	}

      if((prev_unitx == 0)&&(prev_unity == 0)) //avoid div by zero
	prev_vel_mag = 1; //if unit vector only z (0,0,1) then give it a guess of 1 velocity in z direction
      else if(prev_unitx == 0)
	prev_vel_mag = abs(prev_vely/prev_unity); //get extimate of VEL magnitude from x estimate. 
      else if(prev_unity == 0)
	prev_vel_mag = abs(prev_velx/prev_unitx); //get extimate of VEL magnitude from y estimate. 
      else
	prev_vel_mag = (abs(prev_velx/prev_unitx)+abs(prev_vely/prev_unity))/2; //get extimate of VEL magnitude by averaging x and y estimates.

      prev_velz = prev_vel_mag*prev_unitz; //...then multiply by unitz to get estimated z component corresponding to that unit vector.
      prevSonNoiseTransl = sonNoiseTranslArr[iSonVel];

      cout << "prev_unit (x,y,z)=" << prev_unitx << "," << prev_unity << "," << prev_unitz << endl;
      cout << "prev_vel (x,y,z,mag)=" << prev_velx << "," << prev_vely << "," << prev_velz << "," << prev_vel_mag << endl;

      if(VERBOSE)
	cout << "prevSonNoiseTransl=" << prevSonNoiseTransl << endl;

      double camNoiseMult;
      if(VERBOSE)
	cout << "corners,matches,inliers=" << numCorners_arr[i] << "," << numMatches_arr[i] << "," << numInliers_arr[i] << endl;
      //DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:
      /*if(numInliers_arr[i] != 0)
	camNoiseMult = (CAM_MATCHES_WEIGHT*(numCorners_arr[i]/numMatches_arr[i]))+(CAM_INLIERS_WEIGHT*(numMatches_arr[i]/numInliers_arr[i]))+(CAM_CORNERS_WEIGHT*(MAX_CAM_CORNERS/numCorners_arr[i])); //Inversly related to matches/corners, inliers/matches, and corners. This reduces to just 1/numInliers?. 
      else
	camNoiseMult = 1;
      */
      //END DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:

      //Just use numInliers for noise calculation 
      //camNoiseMult = 1-(numInliers_arr[i]/MAX_CAM_CORNERS);
      if(numInliers_arr[i] < CAM_INLIERS_THRESH)
	camNoiseMult = 1-(numInliers_arr[i]/CAM_INLIERS_THRESH);
	//camNoiseMult = 100; //DEBUG DELETE!!!
      else
	camNoiseMult = 0.01;

      if(camNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
	camNoiseMult = 0.01;

      //double camNoiseTransl = camNoiseMult; //allInliers=>0.01 , noInliers=>1. transl is a unit vector
      //      double camNoiseTransl = 10*camNoiseMult; //allInliers=>0.1 , noInliers=>10. transl is a unit vector
      double camNoiseTransl = 0.01*camNoiseMult; //allInliers=>0.0001 , noInliers=>0.01. transl is a unit vector
      double camNoiseRot = 0.001*camNoiseMult; //allInliers=>0.00001deg, noInliers=>0.001 deg
      //double camNoiseRot = 10*camNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
      double camSonNoiseTransl = camNoiseMult*100*sqrt(prevSonNoiseTransl*camNoiseTransl); //Geometric Mean * camNoiseMult*100 to reduce effect 

      if(VERBOSE)
	cout << "Cam Noise multipler=" << camNoiseMult << endl;

      noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << camNoiseTransl, camNoiseTransl, camNoiseTransl, camNoiseRot, camNoiseRot)); //4th (2nd from last) was ZERO_NOISE but changed.??

      noiseModel::Diagonal::shared_ptr constCameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));//ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr cameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camNoiseTransl, camNoiseTransl, camNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));

      noiseModel::Diagonal::shared_ptr cameraSonarNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camSonNoiseTransl, camSonNoiseTransl, camSonNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));

      noiseModel::Diagonal::shared_ptr constCameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr zeroNoise6 = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

      noiseModel::Diagonal::shared_ptr bigNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 10000,10000,10000,10000,10000,10000));
      
      /*NOTE: The 4th variance (Roll?) in the output is constant at the maximum, which seems to be the estimated input value for the first guess. The 5 values in the noise input vector seem to correspond to: 1-1, 2-2, 3-3, 4-5, 5-6 to the output covariance 6 item vector. This was determined by changing one input noise value at a time and looking at the resultant noise vector at the output. Seems to be a problem with the GTSAM code? So, for now, since we aren't using roll at all, just ignore it. BUT make sure to remember the correspondences - so set first three in the Noise(5) vector to translNoise and the last two to rotNoise. */
      double totalEstShiftMag = prev_vel_mag*(t2cam_arr[i]-t1cam_arr[i]);
      cout << "totalEstShiftMag=" << totalEstShiftMag << endl;

      //Update Initial Guesses for Fused Graph
      if(numInliers_arr[i] >= CAM_INLIERS_THRESH)
	{
	  x_sum_camSon += xunit_arr[i]*totalEstShiftMag;
	  y_sum_camSon += yunit_arr[i]*totalEstShiftMag;
	  z_sum_camSon += zunit_arr[i]*totalEstShiftMag;
	}
      else //Use previous good sonar data
	{
	  x_sum_camSon += prev_unitx*totalEstShiftMag;
	  y_sum_camSon += prev_unity*totalEstShiftMag;
	  z_sum_camSon += 0.0001;
	}

      graph.add(EssentialMatrixConstraint(t1cam_arr[i], t2cam_arr[i], EssentialMatrix(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Unit3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise));
      //            graph.add(BetweenFactor<Pose3>(t1cam_arr[i], t2cam_arr[i], Pose3(Rot3::ypr(0,0,0), Point3(1,1,1)), bigNoise6)); //Have to add this or else underconstrained if no sonar. Doesn't seem to have much effect - GOOD!. Here, just add random vector because the large noise will essential weight it to zero.
      graph.add(BetweenFactor<Pose3>(t1cam_arr[i], t2cam_arr[i], Pose3(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Point3(xunit_arr[i]*totalEstShiftMag,yunit_arr[i]*totalEstShiftMag,zunit_arr[i]*totalEstShiftMag)), cameraSonarNoise6)); //Have to add this or else underconstrained     
      //      if(VERBOSE)
      cout << "Graph Node" << t1cam_arr[i] << "-" << t2cam_arr[i] << " - Cam (x,y,z,roll,pitch,yaw): " << xunit_arr[i]*totalEstShiftMag << "," << yunit_arr[i]*totalEstShiftMag << "," << zunit_arr[i]*totalEstShiftMag << "," << roll_arr[i] << "," << pitch_arr[i] << "," << yaw_arr[i] << " - Noise (Transl, Rot): " << camSonNoiseTransl << "," << camNoiseRot << endl;

      graphCamOnly.add(EssentialMatrixConstraint(t1cam_arr[i], t2cam_arr[i], EssentialMatrix(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Unit3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise));
      graphCamOnly.add(BetweenFactor<Pose3>(t1cam_arr[i], t2cam_arr[i], Pose3(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Point3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise6)); //Have to add this or else underconstrained

      initialCam.insert(t2cam_arr[i], Pose3(Rot3::ypr(yaw_sum_cam+addedErr,pitch_sum_cam+addedErr,roll_sum_cam+addedErr), Point3(x_sum_cam+addedErr,y_sum_cam+addedErr,z_sum_cam+addedErr)));
      //Below is for the fused estimates with sonar magnitudes
      initialCamSon.insert(t2cam_arr[i], Pose3(Rot3::ypr(yaw_sum_cam+addedErr,pitch_sum_cam+addedErr,roll_sum_cam+addedErr), Point3(x_sum_camSon+addedErr,y_sum_camSon+addedErr,z_sum_camSon+addedErr)));

      if(VERBOSE)
	{
	  cout << "camera sum (x,y,yaw): " << x_sum_cam << "," << y_sum_cam << "," << yaw_sum_cam << endl;
	}
 
    }

  //graphCamOnly.print();

  /********************Create Fused Initial Guess*******************/
  int iSon1 = 0;
  int iCam1 = 0;
  int offset = 1; //Used as a multiplier to avoid identical nodes

  for(int iFuse=1; iFuse<max(lastSonNode,lastCamNode)+1; iFuse++)
    {
      if(
	 ((t2son_arr[iSon1] == iFuse)&&(numInliers_son_arr[iSon1]>=SON_INLIERS_THRESH)) //If there is a GOOD sonar node here
	 || ((t2son_arr[iSon1] == iFuse)&&(t2cam_arr[iCam1] == iFuse)&&(numInliers_arr[iCam1]<CAM_INLIERS_THRESH)) //If bad sonar node AND bad cam node, use sonar node
	 || ((t2son_arr[iSon1] == iFuse)&&(t2cam_arr[iCam1] != iFuse))) //If sonar node, but no cam node
	{
	  cout << iFuse << "(sonar node)" << endl;
	  initial.insert(iFuse, Pose3(Rot3::ypr(initialSon.at<Pose3>(iFuse).rotation().yaw(),initialSon.at<Pose3>(iFuse).rotation().pitch(),initialSon.at<Pose3>(iFuse).rotation().roll()), Point3(initialSon.at<Pose3>(iFuse).x(),initialSon.at<Pose3>(iFuse).y(),initialSon.at<Pose3>(iFuse).z())));
	  //initial.insert(iFuse, initialSon.at<Pose3>(iFuse));
	  iSon1++;
	  if(t2cam_arr[iCam1] == iFuse) //If there is also a camera node here:
	    iCam1++;
	  offset = 1; //reset offset
	}
      else if(t2cam_arr[iCam1] == iFuse) //If there is a good camera node, but no good sonar node here, NEW: use camera estimate //OLD:so use last sonar estimate again:
	{ 
	  /***************OLD WAY: ****************************
	  //NOTE!!! MUST NOT BE EQUAL TO PREVIOUS NODE. CAUSES NAN TO BE OUTPUT FOR COVARIANCES. So, here I add a small amount to x. 
	  cout << iFuse << " (camonly)"<< endl;
	  if(iSon1>0) //Make sure not trying to access zero
	    initial.insert(iFuse, Pose3(Rot3::ypr(initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().yaw(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().pitch(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().roll()), Point3(initialSon.at<Pose3>(t2son_arr[iSon1-1]).x()+0.001*offset++,initialSon.at<Pose3>(t2son_arr[iSon1-1]).y(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).z())));
	  	  
	  else
	    initial.insert(iFuse, Pose3(Rot3::ypr(0,0,0), Point3(0.001*offset++,0,0))); //Must add small amount in for x or else optimize returns NAN for covariances. Also, must be different each time - hence the *(iFuse-iSon1)
	  //initial.insert(iFuse, initialSon.at<Pose3>(t2son_arr[iSon1-1]));
	  ****************************************************/

	  cout << iFuse << " (camonly)"<< endl;
	  initial.insert(iFuse, Pose3(Rot3::ypr(initialCamSon.at<Pose3>(iFuse).rotation().yaw(),initialCamSon.at<Pose3>(iFuse).rotation().pitch(),initialCamSon.at<Pose3>(iFuse).rotation().roll()), Point3(initialCamSon.at<Pose3>(iFuse).x(),initialCamSon.at<Pose3>(iFuse).y(),initialCamSon.at<Pose3>(iFuse).z())));	  

	  //Increment:
	  iCam1++;
	}
      else
	continue; //Neither cam nor son here. skip node.
    }
  /**************************************************************/
  

  //    int n = i-1;

    //graph.print("\nFactor Graph:\n"); // print

    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values

    /*    for(i=2; i<max(lengthSon,lengthCam)+1; i++) 
    {
      initial.insert(i, Pose3(Rot3::ypr(0,0,0), Point3(i-1+addedErr,addedErr,addedErr)));      
    }
  n=i-1;
    
  for(i=2; i<lengthSon+1; i++) 
    {
      initialSon.insert(i, Pose3(Rot3::ypr(0,0,0), Point3(i-1+addedErr,addedErr,addedErr)));
    }
    for(i=2; i<lengthCam+1; i++) 
    {
      initialCam.insert(i, Pose3(Rot3::ypr(0,0,0), Point3(i-1+addedErr,addedErr,addedErr)));
      }*/


  // cout << "Initial Estimate Nodes: " << n << endl;
     
  //  initial.print("\nInitial Estimate:\n"); // print

  //AMS Custom Print:
  cout << "Initial Size = " << initial.size() << endl;
  for(i=1;i<max(lastSonNode,lastCamNode)+1;i++) 
    {
      if(initial.exists(i))
	{
	  double xprint = initial.at<Pose3>(i).x();
	  double yprint = initial.at<Pose3>(i).y();
	  double yawprint = initial.at<Pose3>(i).rotation().yaw();
	  
	  if(abs(xprint) < 0.001)
	    xprint = 0;
	  if(abs(yprint) < 0.001)
	    yprint = 0;
	  if(abs(yawprint) < 0.001)
	    yawprint = 0;
	  cout << i << " Initial: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
	}
      else
	continue;
    }


  //  graph.print();
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
  //  Marginals marginals(graphSonOnly, resultSonOnly); //DEBUG DELETE!!!

  Marginals marginalsSonOnly(graphSonOnly, resultSonOnly);
  cout << "Calculated Sonar Marginals" << endl;
  Marginals marginalsCamOnly(graphCamOnly, resultCamOnly);
  cout << "Calculated Camera Marginals" << endl;


  cout << 0 << " Result: x,y,yaw = " << result.at<Pose3>(0).x() << "," << result.at<Pose3>(0).y() << "," << result.at<Pose3>(0).rotation().yaw() << endl;
  outfile << 0 << ";" << result.at<Pose3>(0).x() << ";" << result.at<Pose3>(0).y() << ";" << result.at<Pose3>(0).z() << ";" << result.at<Pose3>(0).rotation().roll() << ";" << result.at<Pose3>(0).rotation().pitch() << ";" << result.at<Pose3>(0).rotation().yaw() << ";";
  cout << 0 << " Son Result: x,y,yaw = " << resultSonOnly.at<Pose3>(0).x() << "," << resultSonOnly.at<Pose3>(0).y() << "," << resultSonOnly.at<Pose3>(0).rotation().yaw() << endl;
  outfile << resultSonOnly.at<Pose3>(0).x() << ";" << resultSonOnly.at<Pose3>(0).y() << ";" << resultSonOnly.at<Pose3>(0).z() << ";" << resultSonOnly.at<Pose3>(0).rotation().roll() << ";" << resultSonOnly.at<Pose3>(0).rotation().pitch() << ";" << resultSonOnly.at<Pose3>(0).rotation().yaw() << ";";
  outfile << initialSon.at<Pose3>(0).x() << ";" << initialSon.at<Pose3>(0).y() << ";" << initialSon.at<Pose3>(0).z() << ";" << initialSon.at<Pose3>(0).rotation().roll() << ";" << initialSon.at<Pose3>(0).rotation().pitch() << ";" << initialSon.at<Pose3>(0).rotation().yaw() << ";";
  cout << 0 << " Cam Result: x,y,yaw = " << resultCamOnly.at<Pose3>(0).x() << "," << resultCamOnly.at<Pose3>(0).y() << "," << resultCamOnly.at<Pose3>(0).rotation().yaw() << endl;
  outfile << resultCamOnly.at<Pose3>(0).x() << ";" << resultCamOnly.at<Pose3>(0).y() << ";" << resultCamOnly.at<Pose3>(0).z() << ";" << resultCamOnly.at<Pose3>(0).rotation().roll() << ";" << resultCamOnly.at<Pose3>(0).rotation().pitch() << ";" << resultCamOnly.at<Pose3>(0).rotation().yaw() << ";";
  outfile << initialCam.at<Pose3>(0).x() << ";" << initialCam.at<Pose3>(0).y() << ";" << initialCam.at<Pose3>(0).z() << ";" << initialCam.at<Pose3>(0).rotation().roll() << ";" << initialCam.at<Pose3>(0).rotation().pitch() << ";" << initialCam.at<Pose3>(0).rotation().yaw() << ";";
  outfile << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << endl;

  int iSon = 0;
  int iCam = 0;
  bool doneSon = false;
  bool doneCam = false; //Flag to tell when done with Cam data

  do  //do while not finished with BOTH files
    {
      int nodeNum;
      if((!doneSon)&&(!doneCam)) //Both files still have data
	nodeNum = min(t2son_arr[iSon],t2cam_arr[iCam]); // Get the next node. It will be the minimum of the next son and cam values.
      else if(doneSon)
	nodeNum = t2cam_arr[iCam];
      else if(doneCam)
	nodeNum = t2son_arr[iSon];
      else //Shouldn't ever get here.
	{
	  cout << "ERROR. nodeNum Problem" << endl;
	  return -1;
	}

      cout << "Node " << nodeNum << endl;

      double xprint = result.at<Pose3>(nodeNum).x();
      double yprint = result.at<Pose3>(nodeNum).y();
      double zprint = result.at<Pose3>(nodeNum).z();
      double rollprint = result.at<Pose3>(nodeNum).rotation().roll();
      double pitchprint = result.at<Pose3>(nodeNum).rotation().pitch();
      double yawprint = result.at<Pose3>(nodeNum).rotation().yaw();

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

      cout << nodeNum << " Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      
      outfile << nodeNum << ";" << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";

      //-------------SonOnly-------------//
      if(t2son_arr[iSon]==nodeNum)
	{
	  xprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).x();
	  yprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).y();
	  zprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).z();
	  rollprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().roll();
	  pitchprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().pitch();
	  yawprint = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().yaw();

	  //Initial Guess: 
	  double xinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).x();
	  double yinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).y();
	  double zinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).z();
	  double rollinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().roll();
	  double pitchinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().pitch();
	  double yawinitprint = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().yaw();
	  
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
	  
	  cout << t2son_arr[iSon] << " Sonar Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
	  
	  outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
	  outfile << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";
	  
	}
      else
	outfile << ";;;;;;;;;;;;";

      //-------------CamOnly----------------//
      if(t2cam_arr[iCam]==nodeNum)
	{
	  xprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).x();
	  yprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).y();
	  zprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).z();
	  rollprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().roll();
	  pitchprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().pitch();
	  yawprint = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().yaw();

	  //Initial Guess: 
	  double xinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).x();
	  double yinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).y();
	  double zinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).z();
	  double rollinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().roll();
	  double pitchinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().pitch();
	  double yawinitprint = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().yaw();
	  
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

	  cout << t2cam_arr[iCam] << " Cam Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
	  cout << t2cam_arr[iCam] << " Cam Only Initial: x,y,yaw = " << xinitprint << "," << yinitprint << "," << yawinitprint << endl;
	  
	  outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
	  outfile << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";

	}

      else
	outfile << ";;;;;;;;;;;;";

      //------------------------------------------------------------------------
      // Calculate and print marginal covariances for all variables
      cout.precision(2); 
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;

      cout << "X" << nodeNum << " covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginals.marginalCovariance(nodeNum)(i1,i1) << ",";
	  outfile << marginals.marginalCovariance(nodeNum)(i1,i1) << ";";
	}
      cout << endl;

      //------------SonOnly--------------//
      //      if(t2son_arr[iSon]==nodeNum)
      if(!doneSon)	
	{
	  cout << "X" << nodeNum << " Son covariance: ";
	  for(int i1=0;i1<6;i1++)
	    {
	      //Note: If problem here, probably yaw covariance too big
	      cout << marginalsSonOnly.marginalCovariance(t2son_arr[iSon])(i1,i1) << ",";
	      outfile << marginalsSonOnly.marginalCovariance(t2son_arr[iSon])(i1,i1) << ";";
	    }
	  cout << endl;
	}
      else
	outfile << ";;;;;;";

      //------------VidOnly------------//
      //      if(t2cam_arr[iCam]==nodeNum)
      if(!doneCam)	
	{
	  cout << "X" << nodeNum << " Cam covariance: ";
	  for(int i1=0;i1<6;i1++)
	    {
	      cout << marginalsCamOnly.marginalCovariance(t2cam_arr[iCam])(i1,i1) << ",";
	      outfile << marginalsCamOnly.marginalCovariance(t2cam_arr[iCam])(i1,i1) << ";";
	    }
	  cout << endl;
	}
      else
	outfile << ";;;;;;";

      outfile << endl;

      //Increment counters if needed:
      if((t2son_arr[iSon] == nodeNum)&&(nodeNum!=lastSonNode))
	iSon++;
      if((t2cam_arr[iCam] == nodeNum)&&(nodeNum!=lastCamNode))
	iCam++;
      
      //set done flags if needed:
      if(nodeNum==lastSonNode)
	doneSon = true;
      if(nodeNum==lastCamNode)
	doneCam = true;
    }
  while((!doneSon)||(!doneCam)); //do while not finished with BOTH files
    
  cout << "Initial Estimate Error (Fused): " <<  graph.error(initial) << endl;
  cout << "Result Error (Fused): " << graph.error(result) << endl;

  //--------------------- Son Only -----------------------//
  cout << "Initial Estimate Error (Sonar): " <<  graphSonOnly.error(initial) << endl;
  cout << "Result Error (Sonar): " << graphSonOnly.error(resultSonOnly) << endl;

  //--------------------- Cam Only -----------------------//
  cout << "Initial Estimate Error (Camera): " <<  graphCamOnly.error(initial) << endl;
  cout << "Result Error (Camera): " << graphCamOnly.error(resultCamOnly) << endl;

  outfile.close();
  return 0;
}
