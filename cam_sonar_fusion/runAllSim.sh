rm outputsSim/consoleout_DownAllIce.txt
rm outputsSim/consoleout_frontBrashIceForward.txt
rm outputsSim/consoleout_frontLakeIceForward.txt
rm outputsSim/consoleout_frontPlateletIceForward.txt
rm outputsSim/consoleout_frontVoronoiIceForward.txt
rm outputsSim/consoleout_frontBrashIceCircle.txt
rm outputsSim/consoleout_frontLakeIceCircle.txt
rm outputsSim/consoleout_frontPlateletIceCircle.txt
rm outputsSim/consoleout_frontVoronoiIceCircle.txt
rm outputsSim/consoleout_downBrashIceCircle.txt
rm outputsSim/consoleout_downLakeIceCircle.txt
rm outputsSim/consoleout_downPlateletIceCircle.txt
rm outputsSim/consoleout_downVoronoiIceCircle.txt
rm outputsSim/consoleout_frontSCurve.txt
rm outputsSim/consoleout_upSCurve.txt
rm outputsSim/consoleout_frontAllIceForward.txt
rm outputsSim/consoleout_upAllIceForward.txt

#####################################################

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simRect200x100_1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownAllIceFinalGTSAM.csv ../../Data/Simulated2014/simulatedDownAllIce.csv >> outputsSim/consoleout_DownAllIce.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_DownAllIce.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_DownAllIce.xls

#./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simRect200x100_10x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownAllIceFinalGTSAM.csv >> outputsSim/consoleout_DownAllIce10x.csv
#cp outputGTSAM.csv outputsSim/outputGTSAM_DownAllIce10x.csv

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward200frames1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontBrashIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward200.csv >> outputsSim/consoleout_frontBrashIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontBrashIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontBrashIceForward.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward200frames1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontLakeIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward200.csv >> outputsSim/consoleout_frontLakeIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontLakeIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontLakeIceForward.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward200frames1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontPlateletIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward200.csv >> outputsSim/consoleout_frontPlateletIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontPlateletIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontPlateletIceForward.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward200frames1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontVoronoiIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward200.csv >> outputsSim/consoleout_frontVoronoiIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontVoronoiIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontVoronoiIceForward.xls

########################

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontBrashIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_frontBrashIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontBrashIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontBrashIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontLakeIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_frontLakeIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontLakeIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontLakeIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontPlateletIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_frontPlateletIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontPlateletIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontPlateletIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontVoronoiIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_frontVoronoiIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontVoronoiIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontVoronoiIceCircle.xls

########################

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownBrashIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_downBrashIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_downBrashIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_downBrashIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownLakeIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_downLakeIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_downLakeIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_downLakeIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownPlateletIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_downPlateletIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_downPlateletIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_downPlateletIceCircle.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simCircle360.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraDownVoronoiIceCircleGTSAM.csv ../../Data/Simulated2014/simulatedCircle360.csv >> outputsSim/consoleout_downVoronoiIceCircle.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_downVoronoiIceCircle.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_downVoronoiIceCircle.xls

##############################

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simScurve_1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontSCurveGTSAM.csv ../../Data/Simulated2014/simulatedSCurve.csv >> outputsSim/consoleout_frontSCurve.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontSCurve.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontSCurve.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simScurve_1x.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraUpSCurveGTSAM.csv ../../Data/Simulated2014/simulatedSCurve.csv >> outputsSim/consoleout_upSCurve.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_upSCurve.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_upSCurve.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward600frames.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraFrontAllIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward600.csv >> outputsSim/consoleout_frontAllIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_frontAllIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_frontAllIceForward.xls

./CamSonarFusion ../../opencv-as/shiftEstSonarOnly/outputsSim/outputSonarGTSAM_simForward600frames.csv ../../opencv-as/shiftEstVideoOnly/outputsSim/outputCameraUpAllIceForwardGTSAM.csv ../../Data/Simulated2014/simulatedForward600.csv >> outputsSim/consoleout_upAllIceForward.txt
cp outputGTSAM.csv outputsSim/outputGTSAM_upAllIceForward.csv
cp outputGTSAM.xls outputsSim/outputGTSAM_upAllIceForward.xls
