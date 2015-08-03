//TODO: 
//investigate the noise models. For now, camonly is set to ZERO_NOISE because it was causing it to not solve. Should test with simple inputs.
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

#define VERBOSE true
//#define CAM_CORNERS_WEIGHT 0.3333333
//#define CAM_MATCHES_WEIGHT 0.3333333
//#define CAM_INLIERS_WEIGHT 0.3333333
//#define MAX_CAM_CORNERS 1000 //Should most likely be 1000
//#define MAX_SON_CORNERS 1000 //Should most likely be 1000
#define ZERO_NOISE 0.00001
#define SON_INLIERS_THRESH 200 //Number of inliers considered to hit the low noise plateau  _______
#define CAM_INLIERS_THRESH 200 //Number of inliers considered to hit the low noise plateau /
#define TIME_NODE_MULT 100 //Time to node number multiplier. ie t=0.1->node 1 if mult=10
#define PRINT_UNIX_TIMES false //Print abs unix timestamps instead of zero-based node numbers
#define ZERO_ZROLLPITCH true
#define AVERAGE_INITIAL_VALUES true

int main(int argc, char** argv) 
{
  char sonInputFileName[256];
  char camInputFileName[256];
  char truthInputFileName[256];

  int firstSonNode, firstCamNode, firstTruthNode, lastSonNode, lastCamNode, lastTruthNode;

  double SONAR_TIME0;
  
  //  cout << fixed;
  //cout << defaultfloat;

  double xinit = 0;
  double yinit = 0;
  double zinit = 0;
  double rollinit = 0;
  double pitchinit = 0;
  double yawinit = 0;

  if(argc < 3)
    {
      cout << "Not Enough Args. Usage: ./gtsam <sonar input CSV file with x,y,theta> <camera input CSV file with x,y,z,roll,pitch,yaw> <OPTIONAL truth CSV file with x,y,yaw> <OPTIONAL xinit> <OPTIONAL yinit> <OPTIONAL zinit> <OPTIONAL rollinit> <OPTIONAL pitchinit> <OPTIONAL yawinit>" << endl << "Example: ./gtsam soninput.csv caminput.csv truth.csv 0 0 0 0 0 90" << endl;
      return 0;
    }
  else
    {
      strcpy(sonInputFileName, argv[1]);  
      strcpy(camInputFileName, argv[2]);
      if(argc > 3)
	strcpy(truthInputFileName, argv[3]);  
    }
  //initial values:
  if(argc > 4) 
    xinit = atof(argv[4]);
  if(argc > 5) 
    xinit = atof(argv[5]);
  if(argc > 6) 
    xinit = atof(argv[6]);
  if(argc > 7) 
    xinit = atof(argv[7]);
  if(argc > 8) 
    xinit = atof(argv[8]);
  if(argc > 9) 
    xinit = atof(argv[9]);

  cout << "Initial Global Values (x,y,z,roll,pitch,yaw) = " << xinit << "," << yinit << "," << zinit << "," << rollinit << "," << pitchinit << "," << yawinit << endl;

  ifstream inFileSon(sonInputFileName);
  ifstream inFileCam(camInputFileName);
  ifstream inFileTruth(truthInputFileName);

  string tmpstring;
  
  int lengthSon, lengthCam, lengthTruth;

  //Get number of rows for each file input:
  getline(inFileSon,tmpstring,';'); //First line should just have length listed
  if(strcmp(tmpstring.c_str(),"length=")==0) //Found correct string
    cout << "Found length" << endl;
  else
    {
      cout << "Couldn't find son length." << endl;
      return 0;
    }
  getline(inFileSon,tmpstring,';'); //First line should just have length listed
  lengthSon = (int)(atoi(tmpstring.c_str()));
  cout << "lengthSon=" << lengthSon << endl;
  getline(inFileSon,tmpstring,'\n'); //Discard rest of line
  if(VERBOSE)
    cout << "Bad input data found, discarding: " << tmpstring << endl;

  getline(inFileCam,tmpstring,';'); //First line should just have length listed
  if(strcmp(tmpstring.c_str(),"length=")==0) //Found correct string
    cout << "Found length" << endl;
  else
    {
      cout << "Couldn't find cam length." << endl;
      return 0;
    }
  getline(inFileCam,tmpstring,';'); //First line should just have length listed
  lengthCam = (int)(atoi(tmpstring.c_str()));
  cout << "lengthCam=" << lengthCam << endl;
  getline(inFileCam,tmpstring,'\n'); //Discard rest of line
  if(VERBOSE)
    cout << "Bad input data found, discarding: " << tmpstring << endl;

  //Truth file length:
  if(argc > 3)
    {
      getline(inFileTruth,tmpstring,';'); //First line should just have length listed
      if(strcmp(tmpstring.c_str(),"length=")==0) //Found correct string
	cout << "Found length" << endl;
      else
	{
	  cout << "Couldn't find truth length." << endl;
	  return 0;
	}
      getline(inFileTruth,tmpstring,';'); //First line should just have length listed
      lengthTruth = (int)(atoi(tmpstring.c_str()));
      cout << "lengthTruth=" << lengthTruth << endl;
      getline(inFileTruth,tmpstring,'\n'); //Discard rest of line
      if(VERBOSE)
	cout << "Bad input data found, discarding: " << tmpstring << endl;
    }

  //--------------Look for son headings:----------------------//
  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t1")==0) //FOUND t1 HEADING
    cout << "FOUND t1 HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND t1 HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t2")==0) //FOUND t2 HEADING
    cout << "FOUND t2 HEADING! - " << tmpstring << endl;
  else
    {
      cout << "DIDN'T FIND t2 HEADING! - " << tmpstring << endl;
      return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"x")==0) //FOUND son x HEADING
    cout << "FOUND X HEADING! - " << tmpstring << endl;
  else
    {
      cout << "DIDN'T FIND X HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"y")==0) //FOUND son y HEADING
    cout << "FOUND Y HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND Y HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"yaw")==0) //FOUND son yaw HEADING
    cout << "FOUND YAW HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND YAW HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numCorners")==0) //FOUND son numCorners HEADING
    cout << "FOUND NUMCORNERS HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMCORNERS HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numMatches")==0) //FOUND son numMatches HEADING
    cout << "FOUND NUMMATCHES HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMMATCHES HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numInliers")==0) //FOUND son numInliers HEADING
    cout << "FOUND NUMINLIERS HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMINLIERS HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"estValid")==0) //FOUND cam numInliers HEADING
    cout << "FOUND ESTVALID HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND ESTVALID HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileSon,tmpstring,'\n'); //Discard rest of line
  if(VERBOSE)
    cout << "Bad input data found, discarding: " << tmpstring << endl;

  //Read in camera input file headings:
  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t1")==0) //FOUND cam t1 HEADING
    cout << "FOUND t1 HEADING! - " << tmpstring << endl;
  else
    {
      cout << "DIDN'T FIND t1 HEADING! - " << tmpstring << endl;
      return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"t2")==0) //FOUND cam t2 HEADING
    cout << "FOUND t2 HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND t2 HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"x")==0) //FOUND cam x HEADING
    cout << "FOUND x HEADING! - " << tmpstring << endl;
  else
        {
cout << "DIDN'T FIND x HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"y")==0) //FOUND cam y HEADING
    cout << "FOUND y HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND y HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"z")==0) //FOUND cam z HEADING
    cout << "FOUND z HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND z HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"roll")==0) //FOUND cam roll HEADING
    cout << "FOUND roll HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND roll HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"pitch")==0) //FOUND cam pitch HEADING
    cout << "FOUND pitch HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND pitch HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"yaw")==0) //FOUND cam yaw HEADING
    cout << "FOUND yaw HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND yaw HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numCorners")==0) //FOUND cam numCorners HEADING
    cout << "FOUND NUMCORNERS HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMCORNERS HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numMatches")==0) //FOUND cam numMatches HEADING
    cout << "FOUND NUMMATCHES HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMMATCHES HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"numInliers")==0) //FOUND cam numInliers HEADING
    cout << "FOUND NUMINLIERS HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND NUMINLIERS HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,';');
  if(strcmp(tmpstring.c_str(),"estValid")==0) //FOUND cam numInliers HEADING
    cout << "FOUND ESTVALID HEADING! - " << tmpstring << endl;
  else
    {
    cout << "DIDN'T FIND ESTVALID HEADING! - " << tmpstring << endl;
    return -1;
    }

  getline(inFileCam,tmpstring,'\n'); //Discard rest of line
  if(VERBOSE)
    cout << "Bad input data found, discarding: " << tmpstring << endl;

  //Read in truth input file headings:
  if(argc > 3)
    {
      getline(inFileTruth,tmpstring,';');
      if(strcmp(tmpstring.c_str(),"timestamp")==0) //FOUND truth timestamp HEADING
	cout << "FOUND timestamp HEADING! - " << tmpstring << endl;
      else
	{
	  cout << "DIDN'T FIND timestamp HEADING! - " << tmpstring << endl;
	  return -1;
	}
      
      getline(inFileTruth,tmpstring,';');
      if(strcmp(tmpstring.c_str(),"x")==0) //FOUND truth x HEADING
	cout << "FOUND x HEADING! - " << tmpstring << endl;
      else
	{
	  cout << "DIDN'T FIND x HEADING! - " << tmpstring << endl;
	  return -1;
	}
      
      getline(inFileTruth,tmpstring,';');
      if(strcmp(tmpstring.c_str(),"y")==0) //FOUND truth y HEADING
	cout << "FOUND y HEADING! - " << tmpstring << endl;
      else
	{
	  cout << "DIDN'T FIND y HEADING! - " << tmpstring << endl;
	  return -1;
	}
      
      getline(inFileTruth,tmpstring,';');
      if(strcmp(tmpstring.c_str(),"yaw")==0) //FOUND truth yaw HEADING
	cout << "FOUND yaw HEADING! - " << tmpstring << endl;
      else
	{
	  cout << "DIDN'T FIND yaw HEADING! - " << tmpstring << endl;
	  return -1;
	}
      
      getline(inFileTruth,tmpstring,'\n'); //Discard rest of line
      if(VERBOSE)
  	cout << "Bad input data found, discarding: " << tmpstring << endl;
    }

  //Create input data arrays:
  double * t1son_arr;
  double * t2son_arr;
  float * x_son_arr;
  float * y_son_arr;
  float * yaw_son_arr;
  float * numCorners_son_arr;
  float * numMatches_son_arr;
  float * numInliers_son_arr;
  int * estValid_son_arr;
  //  double * sonNoiseTranslArr;
  //  sonNoiseTranslArr = new double[lengthSon];
  t1son_arr = new double[lengthSon];
  t2son_arr = new double[lengthSon];
  x_son_arr = new float[lengthSon];
  y_son_arr = new float[lengthSon];
  yaw_son_arr = new float[lengthSon];
  numCorners_son_arr = new float[lengthSon];
  numMatches_son_arr = new float[lengthSon];
  numInliers_son_arr = new float[lengthSon];
  estValid_son_arr = new int[lengthSon];

  double * t1cam_arr;
  double * t2cam_arr;
  float * xunit_arr;
  float * yunit_arr;
  float * zunit_arr;
  float * roll_arr;
  float * pitch_arr;
  float * yaw_arr;
  float * numCorners_arr;
  float * numMatches_arr;
  float * numInliers_arr;
  int * estValid_arr;
  t1cam_arr = new double[lengthCam];
  t2cam_arr = new double[lengthCam];
  xunit_arr = new float[lengthCam];
  yunit_arr = new float[lengthCam];
  zunit_arr = new float[lengthCam];
  roll_arr = new float[lengthCam];
  pitch_arr = new float[lengthCam];
  yaw_arr = new float[lengthCam];
  numCorners_arr = new float[lengthCam];
  numMatches_arr = new float[lengthCam];
  numInliers_arr = new float[lengthCam];
  estValid_arr = new int[lengthCam];

  double * t2truth_arr;
  float * xtruth_arr;
  float * ytruth_arr;
  float * yawtruth_arr;
  if(argc > 3)
    {
      t2truth_arr = new double[lengthTruth];
      xtruth_arr = new float[lengthTruth];
      ytruth_arr = new float[lengthTruth];
      yawtruth_arr = new float[lengthTruth];
    }

  cout.precision(12); 

  //Read in the input data:
  for(int i=0; i<lengthSon; i++)
    {
      //Sonar Data:
      getline(inFileSon,tmpstring,';');
      t1son_arr[i] = round(TIME_NODE_MULT*(double)(atof(tmpstring.c_str())));
      getline(inFileSon,tmpstring,';');
      t2son_arr[i] = round(TIME_NODE_MULT*(double)(atof(tmpstring.c_str())));
      if(VERBOSE)
	cout << "son t1,t2 = " << t1son_arr[i] << "," << t2son_arr[i] << endl;

      getline(inFileSon,tmpstring,';');
      x_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      y_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      yaw_son_arr[i] = (float)(atof(tmpstring.c_str()))*PI/180;

      getline(inFileSon,tmpstring,';');
      numCorners_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      numMatches_son_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileSon,tmpstring,';');
      numInliers_son_arr[i] = (float)(atof(tmpstring.c_str()));

      getline(inFileSon,tmpstring,';');
      estValid_son_arr[i] = (int)(atoi(tmpstring.c_str()));

      getline(inFileSon,tmpstring,'\n'); //Discard rest of line
      if(VERBOSE)
	cout << "Bad input data found, discarding: " << tmpstring << endl;
    }
  //cout.precision(2); 

  SONAR_TIME0=t1son_arr[0]/TIME_NODE_MULT; //Initial time for sonar data in unix timestamp seconds
  for(int i=0; i<lengthSon; i++) //Subtract out the SONAR_TIME0 from all nodes
    {
      t1son_arr[i] = t1son_arr[i]-(SONAR_TIME0*TIME_NODE_MULT);
      t2son_arr[i] = t2son_arr[i]-(SONAR_TIME0*TIME_NODE_MULT);
      if(VERBOSE)
	cout << "son n1,n2 = " << t1son_arr[i] << "," << t2son_arr[i] << endl;
    }

  for(int i=0; i<lengthCam; i++)
    {
      //Camera Data:
      getline(inFileCam,tmpstring,';');
      t1cam_arr[i] = round(TIME_NODE_MULT*(double)(atof(tmpstring.c_str())));
      getline(inFileCam,tmpstring,';');
      t2cam_arr[i] = round(TIME_NODE_MULT*(double)(atof(tmpstring.c_str())));
      if(VERBOSE)
	cout << "cam t1,t2 = " << t1cam_arr[i] << "," << t2cam_arr[i] << endl;

      getline(inFileCam,tmpstring,';');
      xunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      yunit_arr[i] = (float)(atof(tmpstring.c_str()));
      getline(inFileCam,tmpstring,';');
      zunit_arr[i] = (float)(atof(tmpstring.c_str()));

      //Convert to unit vector if not
      //      cout << "x,y,z unit: " << xunit_arr[i] << "," << yunit_arr[i] << "," << zunit_arr[i] << endl;
      double unit_mag = sqrt(xunit_arr[i]*xunit_arr[i]+yunit_arr[i]*yunit_arr[i]+zunit_arr[i]*zunit_arr[i]);
      if(unit_mag!=0)
	{
	  xunit_arr[i] = xunit_arr[i]/unit_mag;
	  yunit_arr[i] = yunit_arr[i]/unit_mag;
	  zunit_arr[i] = zunit_arr[i]/unit_mag;
	}

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

      getline(inFileCam,tmpstring,';');
      estValid_arr[i] = (int)(atoi(tmpstring.c_str()));

      //Discard Z, Roll, Pitch if needed
      if(ZERO_ZROLLPITCH)
	{
	  zunit_arr[i] = 0;
	  roll_arr[i] = 0;
	  pitch_arr[i] = 0;
	}

      getline(inFileCam,tmpstring,'\n'); //Discard rest of line
      if(VERBOSE)
	cout << "Bad input data found, discarding: " << tmpstring << endl;
    }    

  //-------------Read in truth data------------
  if(argc > 3)
    {
      for(int i=0; i<lengthTruth; i++)
	{
	  //Truth Data:
	  getline(inFileTruth,tmpstring,';');
	  t2truth_arr[i] = (double)(atof(tmpstring.c_str()));
	  
	  getline(inFileTruth,tmpstring,';');
	  xtruth_arr[i] = (float)(atof(tmpstring.c_str()));
	  
	  getline(inFileTruth,tmpstring,';');
	  ytruth_arr[i] = (float)(atof(tmpstring.c_str()));
	  
	  getline(inFileTruth,tmpstring,';');
	  yawtruth_arr[i] = (float)(atof(tmpstring.c_str()));

	  cout << "truth (t,x,y,yaw): " << t2truth_arr[i] << "," << xtruth_arr[i] << "," << ytruth_arr[i] << "," << yawtruth_arr[i] << endl;
	  
	  getline(inFileTruth,tmpstring,'\n'); //Discard rest of line
	}    
    }

  //Find the first and last nodes:
  firstSonNode = t1son_arr[0];
  firstCamNode = t1cam_arr[0];
  if(argc > 3)
    firstTruthNode = t2truth_arr[0];

  lastSonNode = t2son_arr[lengthSon-1];
  lastCamNode = t2cam_arr[lengthCam-1];
  if(argc > 3)
    lastTruthNode = t2truth_arr[lengthTruth-1];

  cout << "Sonar Nodes: " << firstSonNode << ":" << lastSonNode << endl;
  cout << "Camera Nodes: " << firstCamNode << ":" << lastCamNode << endl;
  if(argc > 3)
    cout << "Truth Nodes: " << firstTruthNode << "(" << round((firstTruthNode-SONAR_TIME0)*TIME_NODE_MULT) << ")" << ":" << lastTruthNode << "(" << round((lastTruthNode-SONAR_TIME0)*TIME_NODE_MULT) << ")" << endl;

  //Print out the data that was read in:
  if(VERBOSE)
    {
      for(int i=0; i<lengthSon; i++)
	{
	  cout << "xson,yson,yawson=" << x_son_arr[i] << "," << y_son_arr[i] << "," << yaw_son_arr[i] << endl;
	}
      for(int i=0; i<lengthCam; i++)
	{
	  cout << "xcam,ycam,zcam=" <<  xunit_arr[i] << "," << yunit_arr[i] << "," << zunit_arr[i] << endl;
	  cout << "camera roll,pitch,yaw=" << roll_arr[i] << "," << pitch_arr[i] << "," << yaw_arr[i] << endl;
	}

      for(int i=0; i<lengthSon; i++)
	{
	  cout << "SonValid=" <<  estValid_son_arr[i] << endl;
	}
      for(int i=0; i<lengthCam; i++)
	{
	  cout << "CamValid=" << estValid_arr[i] << endl;
	}
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
  outfile << "covxcam;covycam;covzcam;covrollcam;covpitchcam;covyawcam;";
  //outfile << "noiseTranslSon;noiseRotSon;noiseTranslCam;noiseRotCam;" << endl; 
  outfile << "xtruth;ytruth;yawtruth;";
  outfile << endl;

  ofstream outfilexls("outputGTSAM.xls");
  outfilexls << "frame/time?\tx\ty\tz\troll\tpitch\tyaw\t";
  outfilexls << "xInit\tyInit\tzInit\trollInit\tpitchInit\tyawInit\t";
  outfilexls << "xson\tyson\tzson\trollson\tpitchson\tyawson\t";
  outfilexls << "xsonSum\tysonSum\tzsonSum\trollsonSum\tpitchsonSum\tyawsonSum\t";
  outfilexls << "xcam\tycam\tzcam\trollcam\tpitchcam\tyawcam\t";
  outfilexls << "xcamSum\tycamSum\tzcamSum\trollcamSum\tpitchcamSum\tyawcamSum\t";
  outfilexls << "covx\tcovy\tcovz\tcovroll\tcovpitch\tcovyaw\t";
  outfilexls << "covxson\tcovyson\tcovzson\tcovrollson\tcovpitchson\tcovyawson\t";
  outfilexls << "covxcam\tcovycam\tcovzcam\tcovrollcam\tcovpitchcam\tcovyawcam\t";
  //  outfilexls << "noiseTranslSon\tnoiseRotSon\tnoiseTranslCam\tnoiseRotCam" << endl; 
  outfilexls << "xtruth\tytruth\tyawtruth\t";
  outfilexls << endl; 

  ofstream outfileNoise("outputNoises.csv");
  outfileNoise << "camSonNoiseTransl;camNoiseRot;sonNoiseTransl;sonNoiseRot;" << endl;

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
  Pose3 priorMean(Rot3::ypr(yawinit,pitchinit,rollinit), Point3(xinit,yinit,zinit));
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
  Values initialSonRobust;
  Values initialCam;
  Values initialCamRobust;
  //Values initialCamSon;

  //NOTE: addedErr - This is confusing. When both before and after prior addedErr is 7 or less, it has no effect - all the same. When 8 before and after, it begins to change the output. When it is 10, it changes a lot. When 150, doesn't change at all and back to no effect like 0. This is for the simulated data input rectangle200x100 and the sonar only data (the fused data performs differently). Documentation says that if there is only one solution, it should be found no matter what initial guess - but needs to be close to soln if multiple solutions.
  double addedErr = 0; //Debug: should be 0
   
  initial.insert(0, Pose3(Rot3::ypr(yawinit+addedErr,pitchinit+addedErr,rollinit+addedErr), Point3(xinit+addedErr,yinit+addedErr,zinit+addedErr)));
  initialSon.insert(0, Pose3(Rot3::ypr(yawinit+addedErr,pitchinit+addedErr,rollinit+addedErr), Point3(xinit+addedErr,yinit+addedErr,zinit+addedErr)));
  initialSonRobust.insert(0, Pose3(Rot3::ypr(yawinit+addedErr,pitchinit+addedErr,rollinit+addedErr), Point3(xinit+addedErr,yinit+addedErr,zinit+addedErr)));
  initialCam.insert(0, Pose3(Rot3::ypr(yawinit+addedErr,pitchinit+addedErr,rollinit+addedErr), Point3(xinit+addedErr,yinit+addedErr,zinit+addedErr)));
  initialCamRobust.insert(0, Pose3(Rot3::ypr(yawinit+addedErr,pitchinit+addedErr,rollinit+addedErr), Point3(xinit+addedErr,yinit+addedErr,zinit+addedErr)));
  //initialCamSon.insert(0, Pose3(Rot3::ypr(addedErr,addedErr,addedErr), Point3(addedErr,addedErr,addedErr)));

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
  //EssentialMatrix e_matrix(Rot3::ypr(yaw_arr[0],pitch_arr[0],roll_arr[0]), Unit3(xunit_arr[0],yunit_arr[0],zunit_arr[0]));

  //noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));

  // i is main loop counter
  int i;

  //Sonar Inits:
  double x_sum_sonRobust = xinit;
  double y_sum_sonRobust = yinit;
  double yaw_sum_sonRobust = yawinit;
  double x_sonRobust = x_son_arr[0];
  double y_sonRobust = y_son_arr[0];
  double yaw_sonRobust = yaw_son_arr[0];
  double xvel_sonRobust = x_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double yvel_sonRobust = y_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double zvel_sonRobust = 0;
  double yawvel_sonRobust = yaw_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  //  double prev_vel_mag_sonRobust = sqrt(prev_xvel_sonRobust*prev_xvel_sonRobust + prev_yvel_sonRobust*prev_yvel_sonRobust);
  double sonNoiseTranslRobust = 0.0001;
  double sonNoiseRotRobust = 0.00001;
  double sonNoiseTransl = 0.0001;
  double sonNoiseRot = 0.00001;
  double x_sum_son = xinit;
  double y_sum_son = yinit;
  double yaw_sum_son = yawinit;
  //  double prev_x_son = x_son_arr[0];
  //  double prev_y_son = y_son_arr[0];
  //  double prev_yaw_son = yaw_son_arr[0];
  double xvel_son = x_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double yvel_son = y_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  double yawvel_son = yaw_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  //  double prev_vel_mag_son = sqrt(prev_xvel_son*prev_xvel_son + prev_yvel_son*prev_yvel_son);
  //  double prev_sonNoiseTransl = 0.0001;
  //  double prev_sonNoiseRot = 0.00001;

  //Camera Inits:
  double x_sum_cam=xinit;
  double y_sum_cam=yinit;
  double z_sum_cam=zinit;
  double roll_sum_cam=rollinit;
  double pitch_sum_cam=pitchinit;
  double yaw_sum_cam=yawinit;
  double x_sum_camSon = xinit;
  double y_sum_camSon = yinit;
  double z_sum_camSon = zinit;
  double x_sum_camRobust=xinit;
  double y_sum_camRobust=yinit;
  double z_sum_camRobust=zinit;
  double roll_sum_camRobust=rollinit;
  double pitch_sum_camRobust=pitchinit;
  double yaw_sum_camRobust=yawinit;  
  double timeDiffSon = 0;
  double camNoiseTransl = 0.0001;
  double camNoiseRot = 0.00001;
  double camSonNoiseTransl = 0.0001;

  //initialize velocities from first sonar data:
  //  double prev_velx = x_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  //  double prev_vely = y_son_arr[0]/(t2son_arr[0]-t1son_arr[0]);
  //double prevSonNoiseTransl = 1;
  // double prev_unitx = xunit_arr[0];
  //  double prev_unity = yunit_arr[0];
  //  double prev_unitz = zunit_arr[0];
  //  double prev_x_cam = 0;
  //  double prev_y_cam = 0;
  //  double prev_z_cam = 0;
  //  double prev_roll_cam = roll_arr[0];
  //  double prev_pitch_cam = pitch_arr[0];
  //  double prev_yaw_cam = yaw_arr[0];
  double vel_mag = 0;
  //  double prev_camNoiseTransl = 0.0001;
  //  double prev_camNoiseRot = 0.00001;

  double xunit_camRobust = xunit_arr[0];
  double yunit_camRobust = yunit_arr[0];
  double zunit_camRobust = zunit_arr[0];
  double roll_camRobust = roll_arr[0];
  double pitch_camRobust = pitch_arr[0];
  double yaw_camRobust = yaw_arr[0];
  //  double prev_vel_magRobust = 0;
  double camNoiseTranslRobust = 0.0001;
  double camNoiseRotRobust = 0.00001;

  //initialize counters:
  //  int iSonVel = 0;
  int iSon2 = 0;
  int iCam2 = 0;

  //for(i = 0; i<lengthSon; i++)    
  for(i = 0; i<max(lastSonNode,lastCamNode)+1; i++)    
    {
      //      cout << i+1 << "/" << max(lastSonNode,lastCamNode) << endl;
      
      /*********************** Add Sonar Nodes *************************/
      if(i == t2son_arr[iSon2]) //If there is a sonar node here:
	{
	  timeDiffSon = t2son_arr[iSon2]-t1son_arr[iSon2];
	  
	  //Get estimate for odometry using all nodes, even bad ones:
	  x_sum_son += x_son_arr[iSon2];
	  y_sum_son += y_son_arr[iSon2];
	  yaw_sum_son += yaw_son_arr[iSon2];
	  
	  //Get robust estimate for odometry using only good nodes and extrapolation:
	  if((numInliers_son_arr[iSon2] >= SON_INLIERS_THRESH)&&(estValid_son_arr[iSon2]==1)) //Good Sonar Node:
	    {
	      x_sum_sonRobust += x_son_arr[iSon2];
	      y_sum_sonRobust += y_son_arr[iSon2];
	      yaw_sum_sonRobust += yaw_son_arr[iSon2];
	      
	      x_sonRobust = x_son_arr[iSon2];
	      y_sonRobust = y_son_arr[iSon2];
	      yaw_sonRobust = yaw_son_arr[iSon2];
	      xvel_sonRobust = x_son_arr[iSon2]/timeDiffSon;
	      yvel_sonRobust = y_son_arr[iSon2]/timeDiffSon;
	      yawvel_sonRobust = yaw_son_arr[iSon2]/timeDiffSon;
	      //	      prev_vel_mag_sonRobust = sqrt(prev_xvel_sonRobust*prev_xvel_sonRobust + prev_yvel_sonRobust*prev_yvel_sonRobust);
	    }
	  else //Not a good sonar node - use previous good estimate
	    {
	      x_sum_sonRobust += x_sonRobust;
	      y_sum_sonRobust += y_sonRobust;
	      yaw_sum_sonRobust += yaw_sonRobust;
	    }

	  double sonNoiseMult;
	  if(VERBOSE)
	    cout << "son corners,matches,inliers=" << numCorners_son_arr[iSon2] << "," << numMatches_son_arr[iSon2] << "," << numInliers_son_arr[iSon2] << endl;
	  //      sonNoiseMult = 1-(numInliers_son_arr[i]/MAX_SON_CORNERS);
	  if(estValid_son_arr[iSon2] != 1) //non-valid data
	    sonNoiseMult = 1;
	  else if(numInliers_son_arr[iSon2] < SON_INLIERS_THRESH) //if not enough matches...
	    sonNoiseMult = 1-(numInliers_son_arr[iSon2]/SON_INLIERS_THRESH); //weight the confidence measure
	  else //Good enough matches to assume accurate
	    sonNoiseMult = 0.01;
	  
	  if(sonNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
	    sonNoiseMult = 0.01;

	  if(VERBOSE)
	    cout << "Son Noise multipler=" << sonNoiseMult << endl;
	  sonNoiseTransl = 0.01*sonNoiseMult; //allInliers=>0.0001m, noInliers=>0.01m
	  //double sonNoiseTransl = 10*sonNoiseMult; //allInliers=>0.1m, noInliers=>10m
	  sonNoiseRot = 0.001*sonNoiseMult; //allInliers=>0.00001rad, noInliers=>0.001 deg
	  //double sonNoiseRot = 10*sonNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
	  //	  double sonFuseNoiseTransl = sonNoiseMult*100*sonNoiseTransl; //*sonNoiseMult*100 to reduce effect 
	  //double sonFuseNoiseRot = sonNoiseMult*100*sonNoiseRot; //*sonNoiseMult*100 to reduce effect 
	  
	  //sonNoiseTranslArr[iSon2] = sonNoiseTransl;
	  
	  if(VERBOSE)
	    cout << "Son Noise (transl,rot): " << sonNoiseTransl << "," << sonNoiseRot << endl;

	  if((numInliers_son_arr[iSon2] >= SON_INLIERS_THRESH)&&(estValid_son_arr[iSon2]==1)) //Good Sonar Node then update noise nodes:
	    {
	      sonNoiseTranslRobust = sonNoiseTransl;
	      sonNoiseRotRobust = sonNoiseRot;
	    }

	  if(iSon2==0) //Save first input as robust regardless
	    {
	      sonNoiseTranslRobust = sonNoiseTransl;
	      sonNoiseRotRobust = sonNoiseRot;
	    }

	  //Update prev_ vbls
	  //prev_x_son = x_son_arr[iSon2];
	  //prev_y_son = y_son_arr[iSon2];
	  //prev_yaw_son = yaw_son_arr[iSon2];
	  //timeDiffSon = t2son_arr[iSon2]-t1son_arr[iSon2];
	  
	  //Calculate Velocities:
	  xvel_son = x_son_arr[iSon2]/timeDiffSon;
	  yvel_son = y_son_arr[iSon2]/timeDiffSon;
	  yawvel_son = yaw_son_arr[iSon2]/timeDiffSon;

	  //	  prev_vel_mag_son = sqrt(prev_xvel_son*prev_xvel_son + prev_yvel_son*prev_yvel_son);
	  //	  prev_sonNoiseTransl = sonNoiseTransl;
	  //	  prev_sonNoiseRot = sonNoiseRot;


	  //NOTE: If make sonar noise 0.1, causes indeterminate solution! 
	  //...Problem with yaw growing unbounded. For now, set to ZERO_NOISE!!! 
	  noiseModel::Diagonal::shared_ptr constSonarNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.01, 0.01, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	  noiseModel::Diagonal::shared_ptr zeroSonarNoise = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	  noiseModel::Diagonal::shared_ptr constSonarNoise1 = noiseModel::Diagonal::Variances((Vector(6) << 1,1,1,1,1,1));
	  noiseModel::Diagonal::shared_ptr constSonarNoise10 = noiseModel::Diagonal::Variances((Vector(6) << 10,10,10,10,10,10));
	  noiseModel::Diagonal::shared_ptr sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << sonNoiseTransl, sonNoiseTransl, 0.01, 0.001, 0.001, sonNoiseRot));
	  //	  noiseModel::Diagonal::shared_ptr sonarFuseNoise = noiseModel::Diagonal::Variances((Vector(6) << sonFuseNoiseTransl, sonFuseNoiseTransl, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, sonNoiseRot));
	  	  
	  ///graph.add(BetweenFactor<Pose3>(t1son_arr[iSon2], t2son_arr[iSon2], Pose3(Rot3::ypr(yaw_son_arr[iSon2],0,0), Point3(x_son_arr[iSon2],y_son_arr[iSon2],0)), sonarNoise));//sonarFuseNoise));
	  graphSonOnly.add(BetweenFactor<Pose3>(t1son_arr[iSon2], t2son_arr[iSon2], Pose3(Rot3::ypr(yaw_son_arr[iSon2],0,0), Point3(x_son_arr[iSon2],y_son_arr[iSon2],0)), sonarNoise));//zeroSonarNoise));
	  
	  ///cout << "Graph Node " << t1son_arr[iSon2] << "-" << t2son_arr[iSon2]  << " - Son (x,y,yaw):     " << x_son_arr[iSon2] << "," << y_son_arr[iSon2] << "," << yaw_son_arr[iSon2] << "       - Noise (Transl, Rot): " << sonNoiseTransl << "," << sonNoiseRot << endl;
	  //string tmp;
	  //sonarNoise->print(tmp);
	  //graph.at(graph.size()-1)->print();
	  
	  initialSon.insert(t2son_arr[iSon2], Pose3(Rot3::ypr(yaw_sum_son+addedErr,0,0), Point3(x_sum_son+addedErr,y_sum_son+addedErr,0)));
	  initialSonRobust.insert(t2son_arr[iSon2], Pose3(Rot3::ypr(yaw_sum_sonRobust+addedErr,0,0), Point3(x_sum_sonRobust+addedErr,y_sum_sonRobust+addedErr,0)));
	  
	  if(VERBOSE)
	    {
	      cout << "sonar sum: " << x_sum_son << "," << y_sum_son << "," << yaw_sum_son << endl;
	    }

	  // if(i != t2cam_arr[iCam2]) //If no corresponding camera node, use past data:
	  bool sonarEnd = false;
	  while(t2cam_arr[iCam2] <= t2son_arr[iSon2]) //Back calculate the sonar and camera nodes
	    {
	      double timeDiffCam = t2cam_arr[iCam2]-t1cam_arr[iCam2];

	      x_sum_cam += xunit_arr[iCam2];
	      y_sum_cam += yunit_arr[iCam2];
	      z_sum_cam += zunit_arr[iCam2];
	      roll_sum_cam += roll_arr[iCam2];
	      pitch_sum_cam += pitch_arr[iCam2];
	      yaw_sum_cam += yaw_arr[iCam2];


	      if((numInliers_arr[iCam2] >= CAM_INLIERS_THRESH)&&(estValid_arr[iCam2]==1)) //Good cam node, update noise vbls:
		{
		  xunit_camRobust = xunit_arr[iCam2]; //get the closest cam unit vector to the sonar vector
		  yunit_camRobust = yunit_arr[iCam2];
		  zunit_camRobust = zunit_arr[iCam2];
		  roll_camRobust = roll_arr[iCam2];
		  pitch_camRobust = pitch_arr[iCam2];
		  yaw_camRobust = yaw_arr[iCam2];

		  x_sum_camRobust += xunit_arr[iCam2];
		  y_sum_camRobust += yunit_arr[iCam2];
		  z_sum_camRobust += zunit_arr[iCam2];
		  roll_sum_camRobust += roll_arr[iCam2];
		  pitch_sum_camRobust += pitch_arr[iCam2];
		  yaw_sum_camRobust += yaw_arr[iCam2];

		  //		  prev_camNoiseTranslRobust = camSonNoiseTransl;
		  //prev_camNoiseRotRobust = camNoiseRot;
		}
	      else
		{
		  x_sum_camRobust += xunit_camRobust;
		  y_sum_camRobust += yunit_camRobust;
		  z_sum_camRobust += zunit_camRobust;
		  roll_sum_camRobust += roll_camRobust;
		  pitch_sum_camRobust += pitch_camRobust;
		  yaw_sum_camRobust += yaw_camRobust;
		}
	      
	      //Update prev vbls:
	      //prev_unitx = xunit_arr[iCam2];
	      //prev_unity = yunit_arr[iCam2];
	      //prev_unitz = zunit_arr[iCam2];
	      //prev_roll_cam = roll_arr[iCam2];
	      //prev_pitch_cam = pitch_arr[iCam2];
	      //prev_yaw_cam = yaw_arr[iCam2];
	      //prev_camNoiseTransl = camSonNoiseTransl;
	      //prev_camNoiseRot = camNoiseRot;
	      
	      
	      //Get previous velocities from sonar data and the corresponding cam unit vector:
	      /*while(t2son_arr[iSonVel+1] <= t2cam_arr[iCam2]) //want sonar t2 that is closest but less than or equal to camera t2
		{*/
	      /* if(numInliers_arr[iCam2] >= CAM_INLIERS_THRESH)
		 {
		 prev_unitxRobust = xunit_arr[iCam2]; //get the closest cam unit vector to the sonar vector
		 prev_unityRobust = yunit_arr[iCam2];
		 prev_unitzRobust = zunit_arr[iCam2];
		 prev_roll_camRobust = roll_arr[iCam2];
		 prev_pitch_camRobust = pitch_arr[iCam2];
		 prev_yaw_camRobust = yaw_arr[iCam2];
		 }*/
	      
	      /*  if(iSonVel >= lengthSon-1)
		  break;
		  else
		  iSonVel++;
		  }*/
	      
	      /*if(VERBOSE)
		cout << "Sonar Node: " << t2son_arr[iSonVel] << " -- Using Velocity from Cam Node: " << t2cam_arr[iCam2] << endl;
		
		if(numInliers_son_arr[iSonVel] >= SON_INLIERS_THRESH) //only update prev vel if sufficient inliers
		{
		prev_velx = x_son_arr[iSonVel]/(t2son_arr[iSonVel]-t1son_arr[iSonVel]);
		prev_vely = y_son_arr[iSonVel]/(t2son_arr[iSonVel]-t1son_arr[iSonVel]);
		}*/
	      
	      //Use previous sonar and camera data to get estimate of previous velocity magnitude
	      double MIN_UNIT = 0.1; //min value for xunit,yunit to be considered for calculation of vel_mag
	      if((abs(xunit_camRobust) < MIN_UNIT)&&(abs(yunit_camRobust) < MIN_UNIT)) //avoid div by zero
		{
		  vel_mag = (double)1/timeDiffSon; //if unit vector only z (0,0,1) then give it a guess of 1 velocity in z direction
		  if(VERBOSE)
		    cout << "vel_mag: can't use either x,y - setting at " << vel_mag << endl;
		}
	      else if(abs(xunit_camRobust) < MIN_UNIT)
		{	
		  vel_mag = abs(yvel_sonRobust/yunit_camRobust); //get extimate of VEL magnitude from x estimate. 
		  if(VERBOSE)
		    cout << "vel_mag: using y" << endl;
		}
	      else if(abs(yunit_camRobust) < MIN_UNIT)
		{
		  vel_mag = abs(xvel_sonRobust/xunit_camRobust); //get extimate of VEL magnitude from y estimate. 
		  if(VERBOSE)
		    cout << "vel_mag: using x" << endl;
		}
	      else
		{
		  vel_mag = (abs(xvel_sonRobust/xunit_camRobust)+abs(yvel_sonRobust/yunit_camRobust))/2; //get extimate of VEL magnitude by averaging x and y estimates.
		  if(VERBOSE)
		    cout << "vel_mag: using x AND y" << endl;
		}
	      zvel_sonRobust = vel_mag*zunit_camRobust; //...then multiply by unitz to get estimated z component corresponding to that unit vector.
	      //prevSonNoiseTransl = sonNoiseTranslArr[iSonVel];
	      
	      if(VERBOSE)
		{
		  cout << "vel_mag componenents (x,y)= " << abs(xvel_sonRobust/xunit_camRobust) << "," << abs(yvel_sonRobust/yunit_camRobust) << endl;
		  cout << "cam robust unit (x,y,z)=" << xunit_camRobust << "," << yunit_camRobust << "," << zunit_camRobust << endl;
		  cout << "son robust vel (x,y,z,mag)=" << xvel_sonRobust << "," << yvel_sonRobust << "," << zvel_sonRobust << "," << vel_mag << endl;
		}

	      if(VERBOSE)
		cout << "sonNoiseTranslRobust=" << sonNoiseTranslRobust << endl;
	      
	      double camNoiseMult;
	      if(VERBOSE)
		cout << "cam corners,matches,inliers=" << numCorners_arr[iCam2] << "," << numMatches_arr[iCam2] << "," << numInliers_arr[iCam2] << endl;
	      //DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:
	      /*if(numInliers_arr[iCam2] != 0)
		camNoiseMult = (CAM_MATCHES_WEIGHT*(numCorners_arr[iCam2]/numMatches_arr[iCam2]))+(CAM_INLIERS_WEIGHT*(numMatches_arr[iCam2]/numInliers_arr[iCam2]))+(CAM_CORNERS_WEIGHT*(MAX_CAM_CORNERS/numCorners_arr[iCam2])); //Inversly related to matches/corners, inliers/matches, and corners. This reduces to just 1/numInliers?. 
		else
		camNoiseMult = 1;
	      */
	      //END DEBUG: USE MATCHES, CORNERS, and INLIERS for Noise calculation:
	      
	      //Just use numInliers for noise calculation 
	      //camNoiseMult = 1-(numInliers_arr[iCam2]/MAX_CAM_CORNERS);
	      if(estValid_arr[iCam2] != 1) //non-valid data
		camNoiseMult = 1;
	      else if(numInliers_arr[iCam2] < CAM_INLIERS_THRESH) //If not enough good matches...
		camNoiseMult = 1-(numInliers_arr[iCam2]/CAM_INLIERS_THRESH); //Weight the confidence by inliers
	      else //Sufficiently good matches to assume high accuracy
		camNoiseMult = 0.01;
	      
	      if(camNoiseMult < 0.01) //Avoid zeros, if 1000 maxCorners this is 990 inliers.
		camNoiseMult = 0.01;
	      
	      //double camNoiseTransl = camNoiseMult; //allInliers=>0.01 , noInliers=>1. transl is a unit vector
	      //      double camNoiseTransl = 10*camNoiseMult; //allInliers=>0.1 , noInliers=>10. transl is a unit vector
	      camNoiseTransl = 0.01*camNoiseMult; //allInliers=>0.0001 , noInliers=>0.01. transl is a unit vector
	      if((numInliers_arr[iCam2] < CAM_INLIERS_THRESH)||(estValid_arr[iCam2]!=1))
		camNoiseTransl*=100; //1000
	      camNoiseRot = 0.001*camNoiseMult; //allInliers=>0.00001deg, noInliers=>0.001 deg
	      //if(numInliers_arr[iCam2] < CAM_INLIERS_THRESH)
		//	camNoiseRot*=1; //100
	      //double camNoiseRot = 10*camNoiseMult; //allInliers=>0.1deg, noInliers=>10deg
	      camSonNoiseTransl = /*camNoiseMult*100**/sqrt(sonNoiseTranslRobust*camNoiseTransl); //Geometric Mean * camNoiseMult*100 to reduce effect 
	      //	  if(numInliers_arr[iCam2] < CAM_INLIERS_THRESH)
	      //  camNoiseMult *= 10; //AMS ADDED 6/17 - otherwise get bad result because bad cam data bleeding through. maybe need this for sonar too. Maybe change magnitude of both errors by 100 - maybe 10^camNoiseMult? Right now sonar is behaving, but maybe because only 0,0 for erronious nodes and not different direction.
	      
	      if(VERBOSE)
		{
		  cout << "Cam Noise multipler=" << camNoiseMult << endl;
		  cout << "Cam Noise(transl,rot,camsontransl)=" << camNoiseTransl << "," << camNoiseRot << "," << camSonNoiseTransl << endl;
		}



	      noiseModel::Diagonal::shared_ptr cameraNoise = noiseModel::Diagonal::Variances((Vector(5) << camNoiseTransl, camNoiseTransl, ZERO_NOISE/*camNoiseTransl*/, ZERO_NOISE/*camNoiseRot*/, camNoiseRot)); //4th (2nd from last) was ZERO_NOISE but changed.??
	      
	      noiseModel::Diagonal::shared_ptr constCameraNoise = noiseModel::Diagonal::Variances((Vector(5) << 0.1, 0.1, 0.1, 0.1, 0.1));//ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	      
	      noiseModel::Diagonal::shared_ptr cameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camNoiseTransl, camNoiseTransl, camNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));

	      //noiseModel::Diagonal::shared_ptr cameraSonarNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camSonNoiseTransl, camSonNoiseTransl, camSonNoiseTransl, camNoiseRot, camNoiseRot, camNoiseRot));
	      noiseModel::Diagonal::shared_ptr cameraSonarNoise6 = noiseModel::Diagonal::Variances((Vector(6) << camSonNoiseTransl, camSonNoiseTransl, 0.01, 0.001, 0.001, camNoiseRot));
	      
	      noiseModel::Diagonal::shared_ptr constCameraNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	      noiseModel::Diagonal::shared_ptr constCameraNoise6_1 = noiseModel::Diagonal::Variances((Vector(6) << 1,1,1,1,1,1)); //ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	      
	      noiseModel::Diagonal::shared_ptr zeroNoise6 = noiseModel::Diagonal::Variances((Vector(6) << ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE, ZERO_NOISE));
	      
	      noiseModel::Diagonal::shared_ptr bigNoise6 = noiseModel::Diagonal::Variances((Vector(6) << 10000,10000,10000,10000,10000,10000));
	      
	      /*NOTE: The 4th variance (Roll?) in the output is constant at the maximum, which seems to be the estimated input value for the first guess. The 5 values in the noise input vector seem to correspond to: 1-1, 2-2, 3-3, 4-5, 5-6 to the output covariance 6 item vector. This was determined by changing one input noise value at a time and looking at the resultant noise vector at the output. Seems to be a problem with the GTSAM code? So, for now, since we aren't using roll at all, just ignore it. BUT make sure to remember the correspondences - so set first three in the Noise(5) vector to translNoise and the last two to rotNoise. */
	      
	      //	      double timeDiffCam = t2cam_arr[iCam2]-t1cam_arr[iCam2];
	      double totalEstShiftMag = vel_mag*timeDiffCam;
	      if(VERBOSE)
		{
		  cout << "vel_mag=" << vel_mag << endl;
		  cout << "timeDiffCam=" << timeDiffCam << endl;
		  cout << "totalEstShiftMag=" << totalEstShiftMag << endl;
		}
	      //prev_x_cam = xunit_arr[iCam2]*totalEstShiftMag;  
	      //prev_y_cam = yunit_arr[iCam2]*totalEstShiftMag;  
	      //prev_z_cam = zunit_arr[iCam2]*totalEstShiftMag;  
	      
	      //Update Initial Guesses for Fused Graph
	      if((numInliers_arr[iCam2] >= CAM_INLIERS_THRESH)&&(estValid_arr[iCam2]==1)) //Good cam node
		{
		  x_sum_camSon += xunit_arr[iCam2]*totalEstShiftMag;
		  y_sum_camSon += yunit_arr[iCam2]*totalEstShiftMag;
		  z_sum_camSon += zunit_arr[iCam2]*totalEstShiftMag;
		}
	      else //Use previous good sonar data
		{
		  x_sum_camSon += xunit_camRobust*totalEstShiftMag;
		  y_sum_camSon += yunit_camRobust*totalEstShiftMag;
		  z_sum_camSon += zunit_camRobust*totalEstShiftMag;//0.0001;
		}

	      if((numInliers_arr[iCam2] >= CAM_INLIERS_THRESH)&&(estValid_arr[iCam2]==1)) //Good cam node, update noise vbls:
		{
		  camNoiseTranslRobust = camSonNoiseTransl;
		  camNoiseRotRobust = camNoiseRot;
		}	      
	      //	      prev_camNoiseTransl = camSonNoiseTransl;
	      //	      prev_camNoiseRot = camNoiseRot;	      

	      //      graph.add(EssentialMatrixConstraint(t1cam_arr[iCam2], t2cam_arr[iCam2], EssentialMatrix(Rot3::ypr(yaw_arr[iCam2],pitch_arr[iCam2],roll_arr[iCam2]), Unit3(xunit_arr[iCam2],yunit_arr[iCam2],zunit_arr[iCam2])), cameraNoise));
	      //            graph.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(0,0,0), Point3(1,1,1)), bigNoise6)); //Have to add this or else underconstrained if no sonar. Doesn't seem to have much effect - GOOD!. Here, just add random vector because the large noise will essential weight it to zero.
	      graph.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_arr[iCam2],pitch_arr[iCam2],roll_arr[iCam2]), Point3(xunit_arr[iCam2]*totalEstShiftMag,yunit_arr[iCam2]*totalEstShiftMag,zunit_arr[iCam2]*totalEstShiftMag)), cameraSonarNoise6)); //Have to add this or else underconstrained     
	  //      if(VERBOSE)
	      cout << "Graph Node " << t1cam_arr[iCam2] << "-" << t2cam_arr[iCam2] << " - Cam (x,y,z,r,p,y): " << xunit_arr[iCam2]*totalEstShiftMag << "," << yunit_arr[iCam2]*totalEstShiftMag << "," << zunit_arr[iCam2]*totalEstShiftMag << "," << roll_arr[iCam2] << "," << pitch_arr[iCam2] << "," << yaw_arr[iCam2] << " - Noise (Transl, Rot): " << camSonNoiseTransl << "," << camNoiseRot << endl;
	      outfileNoise << camSonNoiseTransl << ";" << camNoiseRot << ";";	      
	      //	      graphCamOnly.add(EssentialMatrixConstraint(t1cam_arr[iCam2], t2cam_arr[iCam2], EssentialMatrix(Rot3::ypr(yaw_arr[iCam2],pitch_arr[iCam2],roll_arr[iCam2]), Unit3(xunit_arr[iCam2],yunit_arr[iCam2],zunit_arr[iCam2])), cameraNoise));
	      graphCamOnly.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_arr[iCam2],pitch_arr[iCam2],roll_arr[iCam2]), Point3(xunit_arr[iCam2],yunit_arr[iCam2],zunit_arr[iCam2])), cameraSonarNoise6));//zeroNoise6));//cameraNoise6)); //Have to add this or else underconstrained
	      
	      initialCam.insert(t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_sum_cam+addedErr,pitch_sum_cam+addedErr,roll_sum_cam+addedErr), Point3(x_sum_cam+addedErr,y_sum_cam+addedErr,z_sum_cam+addedErr)));
	      initialCamRobust.insert(t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_sum_camRobust+addedErr,pitch_sum_camRobust+addedErr,roll_sum_camRobust+addedErr), Point3(x_sum_camRobust+addedErr,y_sum_camRobust+addedErr,z_sum_camRobust+addedErr)));
	      
	      //Below is for the fused estimates with sonar magnitudes
	      //initialCamSon.insert(t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_sum_cam+addedErr,pitch_sum_cam+addedErr,roll_sum_cam+addedErr), Point3(x_sum_camSon+addedErr,y_sum_camSon+addedErr,z_sum_camSon+addedErr)));
	      if(AVERAGE_INITIAL_VALUES) //Average sonar and camera estimates
		initial.insert(t2cam_arr[iCam2], Pose3(Rot3::ypr((yaw_sum_camRobust+yaw_sum_sonRobust)/2+addedErr,pitch_sum_camRobust+addedErr,roll_sum_camRobust+addedErr), Point3((x_sum_camSon+x_sum_sonRobust)/2+addedErr,(y_sum_camSon+y_sum_sonRobust)/2+addedErr,z_sum_camSon+addedErr)));
	      else //Else use camSon estimates only
		initial.insert(t2cam_arr[iCam2], Pose3(Rot3::ypr(yaw_sum_camRobust+addedErr,pitch_sum_camRobust+addedErr,roll_sum_camRobust+addedErr), Point3(x_sum_camSon+addedErr,y_sum_camSon+addedErr,z_sum_camSon+addedErr)));
	      
	      if(VERBOSE)
		{
		  cout << "camera sum (x,y,yaw): " << x_sum_cam << "," << y_sum_cam << "," << yaw_sum_cam << endl;
		}
	      
	      /*if(i != t2son_arr[iSon2]) //If no corresponding sonar node, use past data:
		{
		  noiseModel::Diagonal::shared_ptr prev_sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << prev_sonNoiseTransl, prev_sonNoiseTransl, 0.01, 0.001, 0.001, prev_sonNoiseRot)); //Tried changing yaw error to ZERO, but didn't change yaw estimate at all
		  //	      noiseModel::Diagonal::shared_ptr prev_sonarNoise = noiseModel::Diagonal::Variances((Vector(6) << prev_sonNoiseTransl*10, prev_sonNoiseTransl*10, 0.1, 0.01, 0.01, prev_sonNoiseRot*10)); //Tried changing yaw error to ZERO, but didn't change yaw estimate at all
		  graph.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(prev_yawvel_son*timeDiffCam,0,0), Point3(prev_xvel_son*timeDiffCam,prev_yvel_son*timeDiffCam,0)), prev_sonarNoise));
		  cout << "Graph Node " << t1cam_arr[iCam2] << "-" << t2cam_arr[iCam2]  << " - Son Gap (x,y,yaw): " << prev_xvel_son*timeDiffCam << "," << prev_yvel_son*timeDiffCam << "," << prev_yawvel_son*timeDiffCam << "       - Noise (Transl, Rot): " << prev_sonNoiseTransl*10 << "," << prev_sonNoiseRot*10 << endl;
		  }	  

	      noiseModel::Diagonal::shared_ptr prev_cameraSonarNoise6 = noiseModel::Diagonal::Variances((Vector(6) << prev_camNoiseTransl*10, prev_camNoiseTransl*10, prev_camNoiseTransl*10, prev_camNoiseRot*10, prev_camNoiseRot*10, prev_camNoiseRot*10)); //10 times worse noise because it is previous data.
	      graph.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(prev_yaw_cam,prev_pitch_cam,prev_roll_cam), Point3(prev_x_cam,prev_y_cam,prev_z_cam)), prev_cameraSonarNoise6)); //Have to add this or else underconstrained   
	      initialCamSon.insert(t2son_arr[iSon2], initial.at<Pose3>(t2cam_arr[iCam2-1]));//Pose3(Rot3::ypr(yaw_sum_cam+addedErr,pitch_sum_cam+addedErr,roll_sum_cam+addedErr), Point3(x_sum_camSon+addedErr,y_sum_camSon+addedErr,z_sum_camSon+addedErr)));  //Have to insert for case of no camera node
	      */

	      //Add Sonar Node
	      graph.add(BetweenFactor<Pose3>(t1cam_arr[iCam2], t2cam_arr[iCam2], Pose3(Rot3::ypr(yawvel_son*timeDiffCam,0,0), Point3(xvel_son*timeDiffCam,yvel_son*timeDiffCam,0)), sonarNoise));
	      cout << "Graph Node " << t1cam_arr[iCam2] << "-" << t2cam_arr[iCam2]  << " - Son (x,y,yaw):     " << xvel_son*timeDiffCam << "," << yvel_son*timeDiffCam << "," << yawvel_son*timeDiffCam << "       - Noise (Transl, Rot): " << sonNoiseTransl << "," << sonNoiseRot << endl;
	      outfileNoise << sonNoiseTransl << ";" << sonNoiseRot << ";" << endl;

	      //cout << iSon2 << "---" <<  iCam2 << endl;

	      if(t2cam_arr[iCam2]==lastCamNode)
		{
		  sonarEnd = true;
		  cout << "Hit Last Camera Node!!" << endl;
		  break;
		}
	      else
		iCam2++;
	    }

	  //Break if hit either last sonar OR camera nodes. Stop at first end.
	  if((t2son_arr[iSon2]==lastSonNode)||sonarEnd)
	    {
	      cout << "Hit Last Sonar (or Camera) Node!!" << endl;
	      break; 
	    }
	  
	} //END IF SONAR NODE HERE

      //Increment counters if needed:
      if(i == t2son_arr[iSon2]) 
	iSon2++;
      //if(i == t2cam_arr[iCam2]) 
      //	iCam2++;


    }
  
  //  graphSonOnly.print();
  //  initialSon.print();

  cout << "graphCamOnly=" << endl;
  graphCamOnly.print();
  cout << "initialCam=" << endl;
  initialCam.print();

  /********************Create Fused Initial Guess*******************/
  /*  int iSon1 = 0;
  int iCam1 = 0;
  int offset = 1; //Used as a multiplier to avoid identical nodes
  //  int fusedSum_x = 0;
  //  int fusedSum_y = 0;
  //  int fusedSum_yaw = 0;


  for(int iFuse=1; iFuse<max(lastSonNode,lastCamNode)+1; iFuse++)
    {
      if(
	 ((t2son_arr[iSon1] == iFuse)&&(numInliers_son_arr[iSon1]>=SON_INLIERS_THRESH)) //If there is a GOOD sonar node here
	 || ((t2son_arr[iSon1] == iFuse)&&(t2cam_arr[iCam1] == iFuse)&&(numInliers_arr[iCam1]<CAM_INLIERS_THRESH)) //If bad sonar node AND bad cam node, use sonar node
	 || ((t2son_arr[iSon1] == iFuse)&&(t2cam_arr[iCam1] != iFuse))) //If sonar node, but no cam node
	{
	  //	  fusedSum_x += x_son_arr[iFuse];
	  //	  fusedSum_y += y_son_arr[iFuse];
	  //	  fusedSum_yaw += yaw_son_arr[iFuse];

	  cout << iFuse << "(sonar node)" << endl;
	  //initial.insert(iFuse, Pose3(Rot3::ypr(initialSonFuse.at<Pose3>(iFuse).rotation().yaw(),initialSonFuse.at<Pose3>(iFuse).rotation().pitch(),initialSonFuse.at<Pose3>(iFuse).rotation().roll()), Point3(initialSonFuse.at<Pose3>(iFuse).x(),initialSonFuse.at<Pose3>(iFuse).y(),initialSonFuse.at<Pose3>(iFuse).z())));
	  initial.insert(iFuse, Pose3(Rot3::ypr(initialCamSon.at<Pose3>(iFuse).rotation().yaw(),initialCamSon.at<Pose3>(iFuse).rotation().pitch(),initialCamSon.at<Pose3>(iFuse).rotation().roll()), Point3(initialCamSon.at<Pose3>(iFuse).x(),initialCamSon.at<Pose3>(iFuse).y(),initialCamSon.at<Pose3>(iFuse).z())));	  //DEBUG!! DELETE!!!
	  //initial.insert(iFuse, initialSon.at<Pose3>(iFuse));
	  iSon1++;
	  if(t2cam_arr[iCam1] == iFuse) //If there is also a camera node here:
	    iCam1++;
	  offset = 1; //reset offset
	}
      else if(t2cam_arr[iCam1] == iFuse) //If there is a good camera node, but no good sonar node here, NEW: use camera estimate //OLD:so use last sonar estimate again:
	{ 
  */
	  /***************OLD WAY: ****************************
	  //NOTE!!! MUST NOT BE EQUAL TO PREVIOUS NODE. CAUSES NAN TO BE OUTPUT FOR COVARIANCES. So, here I add a small amount to x. 
	  cout << iFuse << " (camonly)"<< endl;
	  if(iSon1>0) //Make sure not trying to access zero
	    initial.insert(iFuse, Pose3(Rot3::ypr(initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().yaw(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().pitch(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).rotation().roll()), Point3(initialSon.at<Pose3>(t2son_arr[iSon1-1]).x()+0.001*offset++,initialSon.at<Pose3>(t2son_arr[iSon1-1]).y(),initialSon.at<Pose3>(t2son_arr[iSon1-1]).z())));
	  	  
	  else
	    initial.insert(iFuse, Pose3(Rot3::ypr(0,0,0), Point3(0.001*offset++,0,0))); //Must add small amount in for x or else optimize returns NAN for covariances. Also, must be different each time - hence the *(iFuse-iSon1)
	  //initial.insert(iFuse, initialSon.at<Pose3>(t2son_arr[iSon1-1]));
	  ****************************************************/
	  //  fusedSum_x += initialCamSon.at<Pose3>(iFuse).x();
	  //fusedSum_y += initialCamSon.at<Pose3>(iFuse).y();
	  //fusedSum_yaw += initialCamSon.at<Pose3>(iFuse).rotation().yaw();
  /*
	  cout << iFuse << " (camonly)"<< endl;
	  initial.insert(iFuse, Pose3(Rot3::ypr(initialCamSon.at<Pose3>(iFuse).rotation().yaw(),initialCamSon.at<Pose3>(iFuse).rotation().pitch(),initialCamSon.at<Pose3>(iFuse).rotation().roll()), Point3(initialCamSon.at<Pose3>(iFuse).x(),initialCamSon.at<Pose3>(iFuse).y(),initialCamSon.at<Pose3>(iFuse).z())));	  

	  //Increment:
	  iCam1++;
	  if(t2son_arr[iSon1] == iFuse) //If there is also a camera node here:
	    iSon1++;
	}
      else
	continue; //Neither cam nor son here. skip node.
      //cout << "Fused Sum (x,y,yaw): " << fusedSum_x << "," << fusedSum_y << "," << fusedSum_yaw << endl;
      }*/
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
     
    initial.print("\nInitial Estimate:\n"); // print

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

  //AMS Custom Print Graph:
  /*  cout << "Fused Size = " << graph.size() << endl;

   for(i=0;i<graph.size();i++) 
    {
      if(graph.exists(i))
	{
	  cout << i << endl;
	  //graph.at(i)->print();//< endl;
	  cout << graph.at(i)->x() << endl;
	  for(;;);
	  double xprint = graph.at<Pose3>(i).x();
	  double yprint = graph.at<Pose3>(i).y();
	  double yawprint = graph.at<Pose3>(i).rotation().yaw();
	  
	  if(abs(xprint) < 0.001)
	    xprint = 0;
	  if(abs(yprint) < 0.001)
	    yprint = 0;
	  if(abs(yawprint) < 0.001)
	    yawprint = 0;
	  cout << i << " Fused Graph: x,y,yaw = " << xprint << "," << yprint << "," << yawprint << endl;

	}
      else
	continue;
    }
	  */

  //NOTE: graph.print() prints sigmas which are the sqrt of variance inputs
  cout << "graph=" << endl;
  graph.print();
  // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  cout << "Optimized Fusion Graph" << endl;
  Values resultSonOnly = LevenbergMarquardtOptimizer(graphSonOnly, initialSonRobust).optimize();
  cout << "Optimized Sonar Graph" << endl;
  Values resultCamOnly = LevenbergMarquardtOptimizer(graphCamOnly, initialCamRobust).optimize();
  cout << "Optimized Camera Graph" << endl;

  // result.print("Final Result:\n");

  //resultSonOnly.print();

  Marginals marginals(graph, result);
  cout << "Calculated Fusion Marginals" << endl;
  Marginals marginalsSonOnly(graphSonOnly, resultSonOnly);
  cout << "Calculated Sonar Marginals" << endl;
  Marginals marginalsCamOnly(graphCamOnly, resultCamOnly);
  cout << "Calculated Camera Marginals" << endl;

  double xprint;
  double yprint;
  double zprint;
  double rollprint;
  double pitchprint;
  double yawprint;
  double xinitprint;
  double yinitprint;
  double zinitprint;
  double rollinitprint;
  double pitchinitprint;
  double yawinitprint;

  cout << 0 << " Result: x,y,yaw = " << result.at<Pose3>(0).x() << "," << result.at<Pose3>(0).y() << "," << result.at<Pose3>(0).rotation().yaw() << endl;

  if(PRINT_UNIX_TIMES)
    {
      outfile << SONAR_TIME0 << ";"; 
      outfilexls << SONAR_TIME0 << "\t";
    }
  else
    {
      outfile << 0 << ";"; 
      outfilexls << 0 << "\t";
    }

  xprint = result.at<Pose3>(0).x();
  yprint = result.at<Pose3>(0).y();
  zprint = result.at<Pose3>(0).z();
  rollprint = result.at<Pose3>(0).rotation().roll()*180/PI;
  pitchprint = result.at<Pose3>(0).rotation().pitch()*180/PI;
  yawprint = result.at<Pose3>(0).rotation().yaw()*180/PI;

  xinitprint = initial.at<Pose3>(0).x();
  yinitprint = initial.at<Pose3>(0).y();
  zinitprint = initial.at<Pose3>(0).z();
  rollinitprint = initial.at<Pose3>(0).rotation().roll()*180/PI;
  pitchinitprint = initial.at<Pose3>(0).rotation().pitch()*180/PI;
  yawinitprint = initial.at<Pose3>(0).rotation().yaw()*180/PI;

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
  
  outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
  outfile << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";

  outfilexls << xprint << "\t" << yprint << "\t" << zprint << "\t" << rollprint << "\t" << pitchprint << "\t" << yawprint << "\t";
  outfilexls << xinitprint << "\t" << yinitprint << "\t" << zinitprint << "\t" << rollinitprint << "\t" << pitchinitprint << "\t" << yawinitprint << "\t";

  //------------------------------------------------------------

  xprint = resultSonOnly.at<Pose3>(0).x();
  yprint = resultSonOnly.at<Pose3>(0).y();
  zprint = resultSonOnly.at<Pose3>(0).z();
  rollprint = resultSonOnly.at<Pose3>(0).rotation().roll()*180/PI;
  pitchprint = resultSonOnly.at<Pose3>(0).rotation().pitch()*180/PI;
  yawprint = resultSonOnly.at<Pose3>(0).rotation().yaw()*180/PI;

  xinitprint = initialSon.at<Pose3>(0).x();
  yinitprint = initialSon.at<Pose3>(0).y();
  zinitprint = initialSon.at<Pose3>(0).z();
  rollinitprint = initialSon.at<Pose3>(0).rotation().roll()*180/PI;
  pitchinitprint = initialSon.at<Pose3>(0).rotation().pitch()*180/PI;
  yawinitprint = initialSon.at<Pose3>(0).rotation().yaw()*180/PI;

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
  
  outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
  outfile << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";

  outfilexls << xprint << "\t" << yprint << "\t" << zprint << "\t" << rollprint << "\t" << pitchprint << "\t" << yawprint << "\t";
  outfilexls << xinitprint << "\t" << yinitprint << "\t" << zinitprint << "\t" << rollinitprint << "\t" << pitchinitprint << "\t" << yawinitprint << "\t";

  //----------------------------------------------------------

  xprint = resultCamOnly.at<Pose3>(0).x();
  yprint = resultCamOnly.at<Pose3>(0).y();
  zprint = resultCamOnly.at<Pose3>(0).z();
  rollprint = resultCamOnly.at<Pose3>(0).rotation().roll()*180/PI;
  pitchprint = resultCamOnly.at<Pose3>(0).rotation().pitch()*180/PI;
  yawprint = resultCamOnly.at<Pose3>(0).rotation().yaw()*180/PI;

  xinitprint = initialCam.at<Pose3>(0).x();
  yinitprint = initialCam.at<Pose3>(0).y();
  zinitprint = initialCam.at<Pose3>(0).z();
  rollinitprint = initialCam.at<Pose3>(0).rotation().roll()*180/PI;
  pitchinitprint = initialCam.at<Pose3>(0).rotation().pitch()*180/PI;
  yawinitprint = initialCam.at<Pose3>(0).rotation().yaw()*180/PI;

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
  
  outfile << xprint << ";" << yprint << ";" << zprint << ";" << rollprint << ";" << pitchprint << ";" << yawprint << ";";
  outfile << xinitprint << ";" << yinitprint << ";" << zinitprint << ";" << rollinitprint << ";" << pitchinitprint << ";" << yawinitprint << ";";

  outfilexls << xprint << "\t" << yprint << "\t" << zprint << "\t" << rollprint << "\t" << pitchprint << "\t" << yawprint << "\t";
  outfilexls << xinitprint << "\t" << yinitprint << "\t" << zinitprint << "\t" << rollinitprint << "\t" << pitchinitprint << "\t" << yawinitprint << "\t";

  //-----------------------------------------------------------------

  outfile << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";" << ZERO_NOISE << ";";
  if(argc > 3)
    outfile << xtruth_arr[0] << ";" << ytruth_arr[0] << ";" << yawtruth_arr[0] << ";";
  outfile << endl;

  outfilexls << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t" << ZERO_NOISE << "\t";
  if(argc > 3)
    outfilexls << xtruth_arr[0] << "\t" << ytruth_arr[0] << "\t" << yawtruth_arr[0] << "\t";
  outfilexls << endl;

  int iSon = 0;
  int iCam = 0;
  int iTruth = 0;
  bool doneSon = false;
  bool doneCam = false; //Flag to tell when done with Cam data
  bool doneTruth = false;
  double prev_cam_yaw = 0;
  double prev_son_yaw = 0;
  double prev_fuse_yaw = 0;

  double xprintFuse;
  double yprintFuse;
  double zprintFuse;
  double rollprintFuse;
  double pitchprintFuse;
  double yawprintFuse;
  double xinitprintFuse;
  double yinitprintFuse;
  double zinitprintFuse;
  double rollinitprintFuse;
  double pitchinitprintFuse;
  double yawinitprintFuse;

  double xprintSon;
  double yprintSon;
  double zprintSon;
  double rollprintSon;
  double pitchprintSon;
  double yawprintSon;
  double xinitprintSon;
  double yinitprintSon;
  double zinitprintSon;
  double rollinitprintSon;
  double pitchinitprintSon;
  double yawinitprintSon;

  double xprintCam;
  double yprintCam;
  double zprintCam;
  double rollprintCam;
  double pitchprintCam;
  double yawprintCam;
  double xinitprintCam;
  double yinitprintCam;
  double zinitprintCam;
  double rollinitprintCam;
  double pitchinitprintCam;
  double yawinitprintCam;

  double xprintFuseNoise;
  double yprintFuseNoise;
  double zprintFuseNoise;
  double rollprintFuseNoise;
  double pitchprintFuseNoise;
  double yawprintFuseNoise;

  double xprintSonNoise;
  double yprintSonNoise;
  double zprintSonNoise;
  double rollprintSonNoise;
  double pitchprintSonNoise;
  double yawprintSonNoise;

  double xprintCamNoise;
  double yprintCamNoise;
  double zprintCamNoise;
  double rollprintCamNoise;
  double pitchprintCamNoise;
  double yawprintCamNoise;

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

      //-------------Fused----------------//
      if(t2cam_arr[iCam]==nodeNum)
	{
	  xprintFuse = result.at<Pose3>(nodeNum).x();
	  yprintFuse = result.at<Pose3>(nodeNum).y();
	  zprintFuse = result.at<Pose3>(nodeNum).z();
	  rollprintFuse = result.at<Pose3>(nodeNum).rotation().roll();
	  pitchprintFuse = result.at<Pose3>(nodeNum).rotation().pitch();
	  yawprintFuse = result.at<Pose3>(nodeNum).rotation().yaw();
	  
	  //Initial Guess: 
	  xinitprintFuse = initial.at<Pose3>(nodeNum).x();
	  yinitprintFuse = initial.at<Pose3>(nodeNum).y();
	  zinitprintFuse = initial.at<Pose3>(nodeNum).z();
	  rollinitprintFuse = initial.at<Pose3>(nodeNum).rotation().roll();
	  pitchinitprintFuse = initial.at<Pose3>(nodeNum).rotation().pitch();
	  yawinitprintFuse = initial.at<Pose3>(nodeNum).rotation().yaw();
	  
	  if(abs(xprintFuse) < 0.001)
	    xprintFuse = 0;
	  if(abs(yprintFuse) < 0.001)
	    yprintFuse = 0;
	  if(abs(zprintFuse) < 0.001)
	    zprintFuse = 0;
	  if(abs(rollprintFuse) < 0.001)
	    rollprintFuse = 0;
	  if(abs(pitchprintFuse) < 0.001)
	    pitchprintFuse = 0;
	  if(abs(yawprintFuse) < 0.001)
	    yawprintFuse = 0;
	  
	  if(abs(xinitprintFuse) < 0.001)
	    xinitprintFuse = 0;
	  if(abs(yinitprintFuse) < 0.001)
	    yinitprintFuse = 0;
	  if(abs(zinitprintFuse) < 0.001)
	    zinitprintFuse = 0;
	  if(abs(rollinitprintFuse) < 0.001)
	    rollinitprintFuse = 0;
	  if(abs(pitchinitprintFuse) < 0.001)
	    pitchinitprintFuse = 0;
	  if(abs(yawinitprintFuse) < 0.001)
	    yawinitprintFuse = 0;

	  if((yawprintFuse - prev_fuse_yaw) > 3*PI/4)
	    yawprintFuse -= 2*PI;
	  else if((yawprintFuse - prev_fuse_yaw) < -3*PI/4)
	    yawprintFuse += 2*PI;
 
	  prev_fuse_yaw = yawprintFuse;	  

	  cout << nodeNum << " Result: x,y,yaw = " << xprintFuse << "," << yprintFuse << "," << yawprintFuse << endl;
	  
	  if(PRINT_UNIX_TIMES)
	    {
	      outfile << ((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0 << ";";
	      outfilexls << ((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0 << "\t"; 
	    }
	  else
	    {
	      outfile << nodeNum << ";"; 
	      outfilexls << nodeNum << "\t"; 
	    }
	  outfile << xprintFuse << ";" << yprintFuse << ";" << zprintFuse << ";" << rollprintFuse*180/PI << ";" << pitchprintFuse*180/PI << ";" << yawprintFuse*180/PI << ";";
	  outfile << xinitprintFuse << ";" << yinitprintFuse << ";" << zinitprintFuse << ";" << rollinitprintFuse*180/PI << ";" << pitchinitprintFuse*180/PI << ";" << yawinitprintFuse*180/PI << ";";

	  outfilexls << xprintFuse << "\t" << yprintFuse << "\t" << zprintFuse << "\t" << rollprintFuse*180/PI << "\t" << pitchprintFuse*180/PI << "\t" << yawprintFuse*180/PI << "\t";
	  outfilexls << xinitprintFuse << "\t" << yinitprintFuse << "\t" << zinitprintFuse << "\t" << rollinitprintFuse*180/PI << "\t" << pitchinitprintFuse*180/PI << "\t" << yawinitprintFuse*180/PI << "\t";
	}
      else
	{
	  if(PRINT_UNIX_TIMES)
	    {
	      outfile << ((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0 << ";"; 
	      outfilexls << ((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0 << "\t"; 
	    }
	  else
	    {
	      outfile << nodeNum << ";"; 
	      outfilexls << nodeNum << "\t";
	    }

	  //outfile << ";;;;;;";
	  //outfile << ";;;;;;";
	  //outfilexls << "\t\t\t\t\t\t";
	  //outfilexls << "\t\t\t\t\t\t";

	  outfile << xprintFuse << ";" << yprintFuse << ";" << zprintFuse << ";" << rollprintFuse*180/PI << ";" << pitchprintFuse*180/PI << ";" << yawprintFuse*180/PI << ";";
	  outfile << xinitprintFuse << ";" << yinitprintFuse << ";" << zinitprintFuse << ";" << rollinitprintFuse*180/PI << ";" << pitchinitprintFuse*180/PI << ";" << yawinitprintFuse*180/PI << ";";

	  outfilexls << xprintFuse << "\t" << yprintFuse << "\t" << zprintFuse << "\t" << rollprintFuse*180/PI << "\t" << pitchprintFuse*180/PI << "\t" << yawprintFuse*180/PI << "\t";
	  outfilexls << xinitprintFuse << "\t" << yinitprintFuse << "\t" << zinitprintFuse << "\t" << rollinitprintFuse*180/PI << "\t" << pitchinitprintFuse*180/PI << "\t" << yawinitprintFuse*180/PI << "\t";
	}

      //-------------SonOnly-------------//
      if(t2son_arr[iSon]==nodeNum)
	{
	  xprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).x();
	  yprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).y();
	  zprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).z();
	  rollprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().roll();
	  pitchprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().pitch();
	  yawprintSon = resultSonOnly.at<Pose3>(t2son_arr[iSon]).rotation().yaw();

	  //Initial Guess: 
	  xinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).x();
	  yinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).y();
	  zinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).z();
	  rollinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().roll();
	  pitchinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().pitch();
	  yawinitprintSon = initialSon.at<Pose3>(t2son_arr[iSon]).rotation().yaw();
	  
	  if(abs(xprintSon) < 0.001)
	    xprintSon = 0;
	  if(abs(yprintSon) < 0.001)
	    yprintSon = 0;
	  if(abs(zprintSon) < 0.001)
	    zprintSon = 0;
	  if(abs(rollprintSon) < 0.001)
	    rollprintSon = 0;
	  if(abs(pitchprintSon) < 0.001)
	    pitchprintSon = 0;
	  if(abs(yawprintSon) < 0.001)
	    yawprintSon = 0;

	  if(abs(xinitprintSon) < 0.001)
	    xinitprintSon = 0;
	  if(abs(yinitprintSon) < 0.001)
	    yinitprintSon = 0;
	  if(abs(zinitprintSon) < 0.001)
	    zinitprintSon = 0;
	  if(abs(rollinitprintSon) < 0.001)
	    rollinitprintSon = 0;
	  if(abs(pitchinitprintSon) < 0.001)
	    pitchinitprintSon = 0;
	  if(abs(yawinitprintSon) < 0.001)
	    yawinitprintSon = 0;

	  if((yawprintSon - prev_son_yaw) > 3*PI/4)
	    yawprintSon -= 2*PI;
	  else if((yawprintSon - prev_son_yaw) < -3*PI/4)
	    yawprintSon += 2*PI;

	  prev_son_yaw = yawprintSon;	  

	  cout << t2son_arr[iSon] << " Sonar Only Result: x,y,yaw = " << xprintSon << "," << yprintSon << "," << yawprintSon << endl;
	  
	  outfile << xprintSon << ";" << yprintSon << ";" << zprintSon << ";" << rollprintSon*180/PI << ";" << pitchprintSon*180/PI << ";" << yawprintSon*180/PI << ";";
	  outfile << xinitprintSon << ";" << yinitprintSon << ";" << zinitprintSon << ";" << rollinitprintSon*180/PI << ";" << pitchinitprintSon*180/PI << ";" << yawinitprintSon*180/PI << ";";

	  outfilexls << xprintSon << "\t" << yprintSon << "\t" << zprintSon << "\t" << rollprintSon*180/PI << "\t" << pitchprintSon*180/PI << "\t" << yawprintSon*180/PI << "\t";
	  outfilexls << xinitprintSon << "\t" << yinitprintSon << "\t" << zinitprintSon << "\t" << rollinitprintSon*180/PI << "\t" << pitchinitprintSon*180/PI << "\t" << yawinitprintSon*180/PI << "\t";
	  
	}
      else
	{
	  //outfile << ";;;;;;;;;;;;";
	  //outfilexls << "\t\t\t\t\t\t\t\t\t\t\t\t";
	  outfile << xprintSon << ";" << yprintSon << ";" << zprintSon << ";" << rollprintSon*180/PI << ";" << pitchprintSon*180/PI << ";" << yawprintSon*180/PI << ";";
	  outfile << xinitprintSon << ";" << yinitprintSon << ";" << zinitprintSon << ";" << rollinitprintSon*180/PI << ";" << pitchinitprintSon*180/PI << ";" << yawinitprintSon*180/PI << ";";

	  outfilexls << xprintSon << "\t" << yprintSon << "\t" << zprintSon << "\t" << rollprintSon*180/PI << "\t" << pitchprintSon*180/PI << "\t" << yawprintSon*180/PI << "\t";
	  outfilexls << xinitprintSon << "\t" << yinitprintSon << "\t" << zinitprintSon << "\t" << rollinitprintSon*180/PI << "\t" << pitchinitprintSon*180/PI << "\t" << yawinitprintSon*180/PI << "\t";
	}
      //-------------CamOnly----------------//
      if(t2cam_arr[iCam]==nodeNum)
	{
	  xprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).x();
	  yprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).y();
	  zprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).z();
	  rollprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().roll();
	  pitchprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().pitch();
	  yawprintCam = resultCamOnly.at<Pose3>(t2cam_arr[iCam]).rotation().yaw();

	  //Initial Guess: 
	  xinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).x();
	  yinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).y();
	  zinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).z();
	  rollinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().roll();
	  pitchinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().pitch();
	  yawinitprintCam = initialCam.at<Pose3>(t2cam_arr[iCam]).rotation().yaw();
	  
	  if(abs(xprintCam) < 0.001)
	    xprintCam = 0;
	  if(abs(yprintCam) < 0.001)
	    yprintCam = 0;
	  if(abs(zprintCam) < 0.001)
	    zprintCam = 0;
	  if(abs(rollprintCam) < 0.001)
	    rollprintCam = 0;
	  if(abs(pitchprintCam) < 0.001)
	    pitchprintCam = 0;
	  if(abs(yawprintCam) < 0.001)
	    yawprintCam = 0;
	  
	  if(abs(xinitprintCam) < 0.001)
	    xinitprintCam = 0;
	  if(abs(yinitprintCam) < 0.001)
	    yinitprintCam = 0;
	  if(abs(zinitprintCam) < 0.001)
	    zinitprintCam = 0;
	  if(abs(rollinitprintCam) < 0.001)
	    rollinitprintCam = 0;
	  if(abs(pitchinitprintCam) < 0.001)
	    pitchinitprintCam = 0;
	  if(abs(yawinitprintCam) < 0.001)
	    yawinitprintCam = 0;

	  if((yawprintCam - prev_cam_yaw) > 3*PI/4)
	    yawprintCam -= 2*PI;
	  else if((yawprintCam - prev_cam_yaw) < -3*PI/4)
	    yawprintCam += 2*PI;

	  prev_cam_yaw = yawprintCam;

	  cout << t2cam_arr[iCam] << " Cam Only Result: x,y,yaw = " << xprintCam << "," << yprintCam << "," << yawprintCam*180/PI << endl;
	  cout << t2cam_arr[iCam] << " Cam Only Initial: x,y,yaw = " << xinitprintCam << "," << yinitprintCam << "," << yawinitprintCam*180/PI << endl;
	  
	  outfile << xprintCam << ";" << yprintCam << ";" << zprintCam << ";" << rollprintCam*180/PI << ";" << pitchprintCam*180/PI << ";" << yawprintCam*180/PI << ";";
	  outfile << xinitprintCam << ";" << yinitprintCam << ";" << zinitprintCam << ";" << rollinitprintCam*180/PI << ";" << pitchinitprintCam*180/PI << ";" << yawinitprintCam*180/PI << ";";
	  outfilexls << xprintCam << "\t" << yprintCam << "\t" << zprintCam << "\t" << rollprintCam*180/PI << "\t" << pitchprintCam*180/PI << "\t" << yawprintCam*180/PI << "\t";
	  outfilexls << xinitprintCam << "\t" << yinitprintCam << "\t" << zinitprintCam << "\t" << rollinitprintCam*180/PI << "\t" << pitchinitprintCam*180/PI << "\t" << yawinitprintCam*180/PI << "\t";

	}

      else
	{
	  //outfile << ";;;;;;;;;;;;";
	  //outfilexls << "\t\t\t\t\t\t\t\t\t\t\t\t";
	  outfile << xprintCam << ";" << yprintCam << ";" << zprintCam << ";" << rollprintCam*180/PI << ";" << pitchprintCam*180/PI << ";" << yawprintCam*180/PI << ";";
	  outfile << xinitprintCam << ";" << yinitprintCam << ";" << zinitprintCam << ";" << rollinitprintCam*180/PI << ";" << pitchinitprintCam*180/PI << ";" << yawinitprintCam*180/PI << ";";
	  outfilexls << xprintCam << "\t" << yprintCam << "\t" << zprintCam << "\t" << rollprintCam*180/PI << "\t" << pitchprintCam*180/PI << "\t" << yawprintCam*180/PI << "\t";
	  outfilexls << xinitprintCam << "\t" << yinitprintCam << "\t" << zinitprintCam << "\t" << rollinitprintCam*180/PI << "\t" << pitchinitprintCam*180/PI << "\t" << yawinitprintCam*180/PI << "\t";
	}
      //------------------------------------------------------------------------
      // Calculate and print marginal covariances for all variables
      cout.precision(2);
      cout << fixed;
      //cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << endl;

      if(t2cam_arr[iCam]==nodeNum)
	{
	  cout << "X" << nodeNum << " covariance: ";
	  xprintFuseNoise = marginals.marginalCovariance(nodeNum)(0,0);
	  yprintFuseNoise = marginals.marginalCovariance(nodeNum)(1,1);
	  zprintFuseNoise = marginals.marginalCovariance(nodeNum)(2,2);
	  rollprintFuseNoise = marginals.marginalCovariance(nodeNum)(3,3);
	  pitchprintFuseNoise = marginals.marginalCovariance(nodeNum)(4,4);
	  yawprintFuseNoise = marginals.marginalCovariance(nodeNum)(5,5);

	  cout << xprintFuseNoise << "," << yprintFuseNoise << "," << zprintFuseNoise << "," << rollprintFuseNoise << "," << pitchprintFuseNoise << "," << yawprintFuseNoise << ",";
	  outfile << xprintFuseNoise << ";" << yprintFuseNoise << ";" << zprintFuseNoise << ";" << rollprintFuseNoise << ";" << pitchprintFuseNoise << ";" << yawprintFuseNoise << ";";
	  outfilexls << xprintFuseNoise << "\t" << yprintFuseNoise << "\t" << zprintFuseNoise << "\t" << rollprintFuseNoise << "\t" << pitchprintFuseNoise << "\t" << yawprintFuseNoise << "\t";
	}
      else
	{
	  //outfile << ";;;;;;";
	  //outfilexls << "\t\t\t\t\t\t";
	  outfile << xprintFuseNoise << ";" << yprintFuseNoise << ";" << zprintFuseNoise << ";" << rollprintFuseNoise << ";" << pitchprintFuseNoise << ";" << yawprintFuseNoise << ";";
	  outfilexls << xprintFuseNoise << "\t" << yprintFuseNoise << "\t" << zprintFuseNoise << "\t" << rollprintFuseNoise << "\t" << pitchprintFuseNoise << "\t" << yawprintFuseNoise << "\t";
	}
      //------------SonOnly--------------//
      //      if(t2son_arr[iSon]==nodeNum)
      if((!doneSon)&&(t2son_arr[iSon]==nodeNum))
	{
	  cout << "X" << nodeNum << " Son covariance: ";
	  xprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(0,0);
	  yprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(1,1);
	  zprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(2,2);
	  rollprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(3,3);
	  pitchprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(4,4);
	  yawprintSonNoise = marginalsSonOnly.marginalCovariance(nodeNum)(5,5);

	  cout << xprintSonNoise << "," << yprintSonNoise << "," << zprintSonNoise << "," << rollprintSonNoise << "," << pitchprintSonNoise << "," << yawprintSonNoise << ",";
	  outfile << xprintSonNoise << ";" << yprintSonNoise << ";" << zprintSonNoise << ";" << rollprintSonNoise << ";" << pitchprintSonNoise << ";" << yawprintSonNoise << ";";
	  outfilexls << xprintSonNoise << "\t" << yprintSonNoise << "\t" << zprintSonNoise << "\t" << rollprintSonNoise << "\t" << pitchprintSonNoise << "\t" << yawprintSonNoise << "\t";
	}
      else
	{
	  //outfile << ";;;;;;";
	  //outfilexls << "\t\t\t\t\t\t";
	  outfile << xprintSonNoise << ";" << yprintSonNoise << ";" << zprintSonNoise << ";" << rollprintSonNoise << ";" << pitchprintSonNoise << ";" << yawprintSonNoise << ";";
	  outfilexls << xprintSonNoise << "\t" << yprintSonNoise << "\t" << zprintSonNoise << "\t" << rollprintSonNoise << "\t" << pitchprintSonNoise << "\t" << yawprintSonNoise << "\t";
	}
      //------------VidOnly------------//
      //      if(t2cam_arr[iCam]==nodeNum)
      if((!doneCam)&&(t2cam_arr[iCam]==nodeNum))
	{	  
	  cout << "X" << nodeNum << " Cam covariance: ";
	  xprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(0,0);
	  yprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(1,1);
	  zprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(2,2);
	  rollprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(3,3);
	  pitchprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(4,4);
	  yawprintCamNoise = marginalsCamOnly.marginalCovariance(nodeNum)(5,5);

	  cout << xprintCamNoise << "," << yprintCamNoise << "," << zprintCamNoise << "," << rollprintCamNoise << "," << pitchprintCamNoise << "," << yawprintCamNoise << ",";
	  outfile << xprintCamNoise << ";" << yprintCamNoise << ";" << zprintCamNoise << ";" << rollprintCamNoise << ";" << pitchprintCamNoise << ";" << yawprintCamNoise << ";";
	  outfilexls << xprintCamNoise << "\t" << yprintCamNoise << "\t" << zprintCamNoise << "\t" << rollprintCamNoise << "\t" << pitchprintCamNoise << "\t" << yawprintCamNoise << "\t";
	}
      else
	{
	  //outfile << ";;;;;;";
	  //outfilexls << "\t\t\t\t\t\t";
	  outfile << xprintCamNoise << ";" << yprintCamNoise << ";" << zprintCamNoise << ";" << rollprintCamNoise << ";" << pitchprintCamNoise << ";" << yawprintCamNoise << ";";
	  outfilexls << xprintCamNoise << "\t" << yprintCamNoise << "\t" << zprintCamNoise << "\t" << rollprintCamNoise << "\t" << pitchprintCamNoise << "\t" << yawprintCamNoise << "\t";
	}

      //-------------TRUTH output -------------------//
      if(argc > 3)
	{
	  if((t2son_arr[iSon] == nodeNum)||(t2cam_arr[iCam] == nodeNum))
	    {
	      //cout << "abs(" << nodeNum << " + " << SONAR_TIME0*TIME_NODE_MULT << "-" << t2truth_arr[iTruth] << ") = ";
	      //while(abs(((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0-t2truth_arr[iTruth]) > abs(((double)nodeNum/TIME_NODE_MULT)+SONAR_TIME0-t2truth_arr[iTruth+1]))
	      while(abs((double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 - t2truth_arr[iTruth]) > abs((double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 - t2truth_arr[iTruth+1]))
		{
		  if(iTruth < lengthTruth-2)//only increment if not at end
		    iTruth++; //increment until find closest time
		}
		if(iTruth > 0)
		  cout << "Last node: " << iTruth-1 << " = " << abs((double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 - t2truth_arr[iTruth-1]) << endl;
		cout << "This node: " << iTruth << " = " << abs((double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 - t2truth_arr[iTruth]) << endl;
		cout << "Next node: " << iTruth+1 << " = " << abs((double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 - t2truth_arr[iTruth+1]) << endl;
		
		cout << iTruth << ":" << (double)nodeNum/TIME_NODE_MULT + SONAR_TIME0 << " -- " << t2truth_arr[iTruth] << endl;
		outfile << xtruth_arr[iTruth] << ";" << ytruth_arr[iTruth] << ";" << yawtruth_arr[iTruth] << ";";
		outfilexls << xtruth_arr[iTruth] << "\t" << ytruth_arr[iTruth] << "\t" << yawtruth_arr[iTruth] << "\t";
	    }
	}

      outfile << endl;
      outfilexls << endl;

      //Increment counters if needed:
      if((t2son_arr[iSon] == nodeNum)&&(nodeNum!=lastSonNode))
	iSon++;
      if((t2cam_arr[iCam] == nodeNum)&&(nodeNum!=lastCamNode))
	iCam++;
      
      //set done flags if needed:
      if(nodeNum==lastSonNode)
	{
	  doneSon = true;
	  if(VERBOSE)
	    cout << "Hit Last Sonar Node!!" << endl;
	}
      if(nodeNum==lastCamNode)
	{
	  doneCam = true;
	  if(VERBOSE)
	    cout << "Hit Last Camera Node!!" << endl;
	}
    }
  // while((!doneSon)||(!doneCam)); //do while not finished with BOTH files
  while((!doneSon)&&(!doneCam)); //do while not finished with files. Stop at first end.
    
  cout << "Initial Estimate Error (Fused): " <<  graph.error(initial) << endl;
  cout << "Result Error (Fused): " << graph.error(result) << endl;

  //--------------------- Son Only -----------------------//
  cout << "Initial Estimate Error (Sonar): " <<  graphSonOnly.error(initialSon) << endl;
  cout << "Result Error (Sonar): " << graphSonOnly.error(resultSonOnly) << endl;

  //--------------------- Cam Only -----------------------//
  cout << "Initial Estimate Error (Camera): " <<  graphCamOnly.error(initialCam) << endl;
  cout << "Result Error (Camera): " << graphCamOnly.error(resultCamOnly) << endl;

  outfile.close();
  outfilexls.close();
  outfileNoise.close();
  return 0;
}
