K4Wv2OpenCVModule
=================

Kinect for Windows v2 OpenCV Module

0. Launch KinectService.exe
1. Call InitializeKinectDevice() to create connection.
2. Call UpdateData() to retrieve next frame data.
2-a. Call calculateMappedFrame() to get color frame filtered by valid depth value.