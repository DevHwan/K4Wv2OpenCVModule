#pragma once
// Default IO
#include <iostream>
// OpenCV
#include <opencv2\opencv.hpp>
// Kinect for Windows SDK 2.0
#include <Kinect.h>

/*
#ifdef _DEBUG
#pragma comment( lib, "opencv_core248d.lib" )
#pragma comment( lib, "opencv_highgui248d.lib" )
#pragma comment( lib, "opencv_imgproc248d.lib" )
#else
#pragma comment( lib, "opencv_core248.lib" )
#pragma comment( lib, "opencv_highgui248.lib" )
#pragma comment( lib, "opencv_imgproc248.lib" )
#endif
*/

#pragma comment( lib, "Kinect20.lib" )

// Namespaces
using namespace std;
using namespace cv;


class CK4Wv2OpenCVModule
{
	// Sensor frame data values
	// Color frame resolution
	static const int COLOR_FRAME_WIDTH = 1920;
	static const int COLOR_FRAME_HEIGHT = 1080;
	// Depth frame resolution
	static const int DEPTH_FRAME_WIDTH = 512;
	static const int DEPTH_FRAME_HEIGHT = 424;
	// Infrared frame resolution
	static const int INFRARED_FRAME_WIDTH = 512;
	static const int INFRARED_FRAME_HEIGHT = 424;



public:
	CK4Wv2OpenCVModule();
	~CK4Wv2OpenCVModule();

	HRESULT InitializeKinectDevice();


	// Image frame Mat
	Mat colorRAWFrameMat;
	Mat depthRAWFrameMat;
	Mat infraRAWFrameMat;
	Mat colorMappedFrameMat;
	//Mat depthMappedFrameMat;

	// Process frame
	void UpdateData();

	// Calculate Mapped Frame
	HRESULT calculateMappedFrame();


private:
	// Device
	IKinectSensor* pSensor;
	ICoordinateMapper* pCoordinateMapper;

	// Frame reader
	IMultiSourceFrameReader* pMultiSourceFrameReader;

	// Frame data buffers
	RGBQUAD* pColorRAWBuffer;
	ushort* pDepthRAWBuffer;
	ushort* pInfraRAWBuffer;

	// Coordinate
	ColorSpacePoint* pColorCoodinate;
	DepthSpacePoint* pDepthCoordinate;

	// Release function
	template< class T > void SafeRelease( T** ppT );
};

