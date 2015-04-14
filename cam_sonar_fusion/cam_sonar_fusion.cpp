//TODO: need to change size of x,y,z, arrays to actual size of frames from 256.


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
#define MAX_CAM_CORNERS 100 //Should most likely be 1000
#define MAX_SON_CORNERS 100 //Should most likely be 1000
#define ZERO_NOISE 0.00001

int main(int argc, char** argv) 
{
  char sonInputFileName[256];
  char camInputFileName[256];

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
  t1son_arr = new float[32];
  t2son_arr = new float[32];
  x_son_arr = new float[32];
  y_son_arr = new float[32];
  yaw_son_arr = new float[32];
  numCorners_son_arr = new float[32];
  numMatches_son_arr = new float[32];
  numInliers_son_arr = new float[32];

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
  t1cam_arr = new float[32];
  t2cam_arr = new float[32];
  xunit_arr = new float[32];
  yunit_arr = new float[32];
  zunit_arr = new float[32];
  roll_arr = new float[32];
  pitch_arr = new float[32];
  yaw_arr = new float[32];
  numCorners_arr = new float[32];
  numMatches_arr = new float[32];
  numInliers_arr = new float[32];

  //Read in the input data:
  for(int i=0; i<lengthSon; i++)
    {
      //Sonar Data:
      getline(inFileSon,tmpstring,';');
      t1son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      t2son_arr[i] = (float)(atof(tmpstring.c_str()));

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
      t1cam_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      t2cam_arr[i] = (float)(atof(tmpstring.c_str()));

      getline(inFileCam,tmpstring,';');
      xunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      yunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      zunit_arr[i] = (float)(atof(tmpstring.c_str()));
    
      getline(inFileCam,tmpstring,';');
      roll_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      pitch_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      yaw_arr[i] = (float)(atof(tmpstring.c_str()));

      getline(inFileCam,tmpstring,';');
      numCorners_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      numMatches_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      numInliers_arr[i] = (float)(atof(tmpstring.c_str()));
    }    

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
  outfile << "xcam;ycam;zcam;rollcam;pitchcam;yawcam;";
  outfile << "covx;covy;covz;covroll;covpitch;covyaw;";
  outfile << "covxson;covyson;covzson;covrollson;covpitchson;covyawson;";
  outfile << "covxcam;covycam;covzcam;covrollcam;covpitchcam;covyawcam;" << endl; 

  // Create an empty nonlinear factor graph
  NonlinearFactorGraph graph;
  NonlinearFactorGraph graphSonOnly;
  NonlinearFactorGraph graphCamOnly;

  Rot3 priorMeanRot3 = Rot3::ypr(yaw_arr[0], pitch_arr[0], roll_arr[0]);

  cout << "Prior Rot Matrix:" << endl;
  priorMeanRot3.print();

  Vector3 current_ypr;
  current_ypr = priorMeanRot3.ypr();
  cout << "Prior Yaw = " << current_ypr[0] << endl;
  cout << "Prior Pitch = " << current_ypr[1] << endl;
  cout << "Prior Roll = " << current_ypr[2] << endl;
  
  Point3 priorMeanPoint3(0,0,0);
  priorMeanPoint3.print("Prior 3D Point = ");

  //  Pose3 priorMean(priorMeanRot3,priorMeanPoint3);
  Pose3 priorMean(Rot3::ypr(0,0,0), Point3(0,0,0));
  cout << "priorMean=" << endl;
  priorMean.print();

  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  //  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));

  graph.add(PriorFactor<Pose3>(1, priorMean, priorNoise));
  graphSonOnly.add(PriorFactor<Pose3>(1, priorMean, priorNoise));
  graphCamOnly.add(PriorFactor<Pose3>(1, priorMean, priorNoise));


  // Add odometry factors
  Pose3 odometry_0(Rot3::ypr(0,0,0), Point3(0,0,0));
  Pose3 odometry_x(Rot3::ypr(0,0,0), Point3(1,0,0));
  Pose3 odometry_y(Rot3::ypr(0,0,0), Point3(0,1,0));
  Pose3 odometry_z(Rot3::ypr(0,0,0), Point3(0,0,1));
  Pose3 odometry_yaw(Rot3::ypr(PI/160,0,0), Point3(0,0,0));
  //  odometry.print("Odometry=");

  // For simplicity, we will use the same noise model for each odometry factor
  //  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  //noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << 1, 1, 1, 1, 1, 1));

  //EssentialMatrix e_matrix(zeroRot3, xUnit3);
  EssentialMatrix e_matrix(Rot3::ypr(yaw_arr[0],pitch_arr[0],roll_arr[0]), Unit3(xunit_arr[0],yunit_arr[0],zunit_arr[0]));

  //noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));


  /*********************** Add Sonar Nodes *************************/
  int i;
  for(i = 1; i<lengthSon; i++)    
    {
      double sonNoiseMult;
      cout << "son corners,matches,inliers=" << numCorners_son_arr[i] << "," << numMatches_son_arr[i] << "," << numInliers_son_arr[i] << endl;
      sonNoiseMult = 1-(numInliers_son_arr[i]/MAX_SON_CORNERS);
      if(sonNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
	sonNoiseMult = 0.01;
      
      cout << "Son Noise multipler=" << sonNoiseMult << endl;
      double sonNoiseTransl = 10*sonNoiseMult; //allInliers=>0.1m, noInliers=>10m
      double sonNoiseRot = 10*sonNoiseMult; //allInliers=>0.1deg, noInliers=>10deg

      noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << sonNoiseTransl, sonNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, sonNoiseRot));
      
      graph.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(yaw_son_arr[i],0,0), Point3(x_son_arr[i],y_son_arr[i],0)), sonarNoise));
      graphSonOnly.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(yaw_son_arr[i],0,0), Point3(x_son_arr[i],y_son_arr[i],0)), sonarNoise));
    }

  /*********************** Add Camera Nodes *************************/
  for(i = 1; i<lengthCam; i++)
    {
      double camNoiseMult;
      cout << "corners,matches,inliers=" << numCorners_arr[i] << "," << numMatches_arr[i] << "," << numInliers_arr[i] << endl;
      //DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:
      /*if(numInliers_arr[i] != 0)
	camNoiseMult = (CAM_MATCHES_WEIGHT*(numCorners_arr[i]/numMatches_arr[i]))+(CAM_INLIERS_WEIGHT*(numMatches_arr[i]/numInliers_arr[i]))+(CAM_CORNERS_WEIGHT*(MAX_CAM_CORNERS/numCorners_arr[i])); //Inversly related to matches/corners, inliers/matches, and corners. This reduces to just 1/numInliers?. 
      else
	camNoiseMult = 1;
      */
      //END DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:

      //Just use numInliers for noise calculation 
      camNoiseMult = 1-(numInliers_arr[i]/MAX_CAM_CORNERS);
      if(camNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
	camNoiseMult = 0.01;

      cout << "Cam Noise multipler=" << camNoiseMult << endl;
      double camNoiseTransl = camNoiseMult; //allInliers=>0.01 , noInliers=>1. transl is a unit vector
      double camNoiseRot = 10*camNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
      noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << camNoiseTransl, camNoiseTransl, camNoiseTransl, ZERO_NOISE, camNoiseRot));
      noiseModel::Diagonal::shared_ptr cameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camNoiseTransl, camNoiseTransl, camNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));
      
      /*NOTE: The 4th variance (Roll?) in the output is constant at the maximum, which seems to be the estimated input value for the first guess. The 5 values in the noise input vector seem to correspond to: 1-1, 2-2, 3-3, 4-5, 5-6 to the output covariance 6 item vector. This was determined by changing one input noise value at a time and looking at the resultant noise vector at the output. Seems to be a problem with the GTSAM code? So, for now, since we aren't using roll at all, just ignore it. BUT make sure to remember the correspondences - so set first three in the Noise(5) vector to translNoise and the last two to rotNoise. */
      
      graph.add(EssentialMatrixConstraint(i, i+1, EssentialMatrix(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Unit3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise));
      graphCamOnly.add(EssentialMatrixConstraint(i, i+1, EssentialMatrix(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Unit3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise));
      graphCamOnly.add(BetweenFactor<Pose3>(i, i+1, Pose3(Rot3::ypr(yaw_arr[i],pitch_arr[i],roll_arr[i]), Point3(xunit_arr[i],yunit_arr[i],zunit_arr[i])), cameraNoise6)); //Have to add this or else underconstrained
    }

    int n = i-1;

    //graph.print("\nFactor Graph:\n"); // print

    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    Values initial;
    double addedErr = 0.1;
 
    initial.insert(1, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));

    for(i=2; i<max(lengthSon,lengthCam)+1; i++) 
    {
      initial.insert(i, Pose3(Rot3::ypr(0,0,0), Point3(i-1+addedErr,addedErr,addedErr)));
    }
  n=i-1;

  cout << "Initial Estimate Nodes: " << n << endl;
     
  //  initial.print("\nInitial Estimate:\n"); // print

  //AMS Custom Print:
  for(i=1;i<max(lengthSon,lengthCam)+1;i++) 
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

  // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  Values resultSonOnly = LevenbergMarquardtOptimizer(graphSonOnly, initial).optimize();
  Values resultCamOnly = LevenbergMarquardtOptimizer(graphCamOnly, initial).optimize();

  // result.print("Final Result:\n");

  Marginals marginals(graph, result);
  Marginals marginalsSonOnly(graphSonOnly, resultSonOnly);
  Marginals marginalsCamOnly(graphCamOnly, resultCamOnly);

  for(i=1;i<max(lengthSon,lengthCam)+1;i++) //was n+1
    {
      double xprint = result.at<Pose3>(i).x();
      double yprint = result.at<Pose3>(i).y();
      double zprint = result.at<Pose3>(i).z();
      double rollprint = result.at<Pose3>(i).rotation().roll();
      double pitchprint = result.at<Pose3>(i).rotation().pitch();
      double yawprint = result.at<Pose3>(i).rotation().yaw();

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

      cout << i << " Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      
      outfile << i << ";" << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";

      //-------------SonOnly-------------//
      xprint = resultSonOnly.at<Pose3>(i).x();
      yprint = resultSonOnly.at<Pose3>(i).y();
      zprint = resultSonOnly.at<Pose3>(i).z();
      rollprint = resultSonOnly.at<Pose3>(i).rotation().roll();
      pitchprint = resultSonOnly.at<Pose3>(i).rotation().pitch();
      yawprint = resultSonOnly.at<Pose3>(i).rotation().yaw();

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

      cout << i << " Sonar Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      
      outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";

      //-------------CamOnly----------------//
      xprint = resultCamOnly.at<Pose3>(i).x();
      yprint = resultCamOnly.at<Pose3>(i).y();
      zprint = resultCamOnly.at<Pose3>(i).z();
      rollprint = resultCamOnly.at<Pose3>(i).rotation().roll();
      pitchprint = resultCamOnly.at<Pose3>(i).rotation().pitch();
      yawprint = resultCamOnly.at<Pose3>(i).rotation().yaw();

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

      cout << i << " Cam Only Result: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;
      
      outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
      
      // Calculate and print marginal covariances for all variables
      cout.precision(2); 
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;
      cout << "X" << i << " covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginals.marginalCovariance(i)(i1,i1) << ",";
	  outfile << marginals.marginalCovariance(i)(i1,i1) << ";";
	}
      cout << endl;

      //------------SonOnly--------------//
      cout << "X" << i << " Son covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginalsSonOnly.marginalCovariance(i)(i1,i1) << ",";
	  outfile << marginalsSonOnly.marginalCovariance(i)(i1,i1) << ";";
	}
      cout << endl;

      //------------VidOnly------------//
      cout << "X" << i << " Cam covariance: ";
      for(int i1=0;i1<6;i1++)
	{
	  cout << marginalsCamOnly.marginalCovariance(i)(i1,i1) << ",";
	  outfile << marginalsCamOnly.marginalCovariance(i)(i1,i1) << ";";
	}
      cout << endl;
      outfile << endl;
    }

  cout << "Initial Estimate Error: " <<  graph.error(initial) << endl;
  cout << "Result Error: " << graph.error(result) << endl;

  //--------------------- Son Only -----------------------//
  cout << "Initial Estimate Error: " <<  graphSonOnly.error(initial) << endl;
  cout << "Result Error: " << graphSonOnly.error(resultSonOnly) << endl;

  //--------------------- Cam Only -----------------------//
  cout << "Initial Estimate Error: " <<  graphCamOnly.error(initial) << endl;
  cout << "Result Error: " << graphCamOnly.error(resultCamOnly) << endl;

  outfile.close();
  return 0;
}
