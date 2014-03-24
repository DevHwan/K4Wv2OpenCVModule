#include "K4Wv2OpenCVModule.h"

// Log tag
static const string TAG = "[K4WOCV]";

CK4Wv2OpenCVModule::CK4Wv2OpenCVModule()
{
	cout << TAG << "Initializing Kinect for Windows OpenCV Module." << endl;

	pSensor = nullptr;
	pCoordinateMapper = nullptr;
	pMultiSourceFrameReader = nullptr;

	// Allocate buffers
	pColorRAWBuffer = new RGBQUAD[COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT];
	pDepthRAWBuffer = new ushort[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];
	pInfraRAWBuffer = new ushort[INFRARED_FRAME_WIDTH * INFRARED_FRAME_HEIGHT];
	pColorCoodinate = new ColorSpacePoint[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];
	pDepthCoordinate = new DepthSpacePoint[COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT];

	// Set 0
	memset( pColorRAWBuffer, 0, COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * sizeof( RGBQUAD ) );
	memset( pColorRAWBuffer, 0, DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT * sizeof( ushort ) );
	memset( pColorRAWBuffer, 0, INFRARED_FRAME_WIDTH * INFRARED_FRAME_HEIGHT * sizeof( ushort ) );

	// Set Mat
	colorRAWFrameMat = Mat( Size( COLOR_FRAME_WIDTH, COLOR_FRAME_HEIGHT ), CV_8UC4, (void*)pColorRAWBuffer );
	depthRAWFrameMat = Mat( Size( DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT ), CV_16UC1, (void*)pDepthRAWBuffer );
	infraRAWFrameMat = Mat( Size( INFRARED_FRAME_WIDTH, INFRARED_FRAME_HEIGHT ), CV_16UC1, (void*)pInfraRAWBuffer );
}


CK4Wv2OpenCVModule::~CK4Wv2OpenCVModule()
{
	cout << TAG << "Releasing Kinect for Windows OpenCV Module." << endl;
	// Release buffers
	if ( pColorRAWBuffer )
	{
		delete pColorRAWBuffer;
		pColorRAWBuffer = nullptr;
	}
	if ( pDepthRAWBuffer )
	{
		delete pDepthRAWBuffer;
		pDepthRAWBuffer = nullptr;
	}
	if ( pInfraRAWBuffer )
	{
		delete pInfraRAWBuffer;
		pInfraRAWBuffer = nullptr;
	}
	if ( pColorCoodinate )
	{
		delete pColorCoodinate;
		pColorCoodinate = nullptr;
	}
	if ( pDepthCoordinate )
	{
		delete pDepthCoordinate;
		pDepthCoordinate = nullptr;
	}

	if ( pSensor )
	{
		pSensor->Close();
		SafeRelease( &pSensor );
	}
}

HRESULT CK4Wv2OpenCVModule::InitializeKinectDevice()
{
	HRESULT hr;

	// Get default sensor
	hr = GetDefaultKinectSensor( &pSensor );
	if ( FAILED( hr ) )
	{
		cerr << TAG << "Sensor initialization error at - " << __FUNCTIONW__ << endl;
		return hr;
	}

	if ( pSensor )
	{
		// Get coordinate mapper
		pSensor->get_CoordinateMapper( &pCoordinateMapper );
		// Open sensor
		hr = pSensor->Open();
		if ( SUCCEEDED( hr ) )
		{
			hr = pSensor->OpenMultiSourceFrameReader( FrameSourceTypes_Infrared | FrameSourceTypes_Color | FrameSourceTypes_Depth, &pMultiSourceFrameReader );
		}
	}

	if ( !pSensor || FAILED( hr ) )
	{
		cerr << TAG << "No devices ready." << endl;
		return E_FAIL;
	}

	return hr;
}

void CK4Wv2OpenCVModule::UpdateData()
{
	if ( !pMultiSourceFrameReader )
		return;

	IMultiSourceFrame* pMultiSourceFrame = nullptr;
	IDepthFrame* pDepthFrame = nullptr;
	IColorFrame* pColorFrame = nullptr;
	IInfraredFrame* pInfraFrame = nullptr;

	HRESULT hr = pMultiSourceFrameReader->AcquireLatestFrame( &pMultiSourceFrame );

	if ( SUCCEEDED( hr ) )
	{
		// Receive color
		IColorFrameReference* pColorFrameReference = nullptr;

		HRESULT hrColor = pMultiSourceFrame->get_ColorFrameReference( &pColorFrameReference );
		if ( SUCCEEDED( hrColor ) )
		{
			hrColor = pColorFrameReference->AcquireFrame( &pColorFrame );
		}
		SafeRelease( &pColorFrameReference );

		// Receive depth
		IDepthFrameReference* pDepthFrameReference = nullptr;

		HRESULT hrDepth = pMultiSourceFrame->get_DepthFrameReference( &pDepthFrameReference );
		if ( SUCCEEDED( hrDepth ) )
		{
			hrDepth = pDepthFrameReference->AcquireFrame( &pDepthFrame );
		}
		SafeRelease( &pDepthFrameReference );

		IInfraredFrameReference* pInfraFrameReference = nullptr;
		HRESULT hrInfra = pMultiSourceFrame->get_InfraredFrameReference( &pInfraFrameReference );
		if ( SUCCEEDED( hrInfra ) )
		{
			hrInfra = pInfraFrameReference->AcquireFrame( &pInfraFrame );
		}
		SafeRelease( &pInfraFrameReference );

		// Check color & depth
		if ( SUCCEEDED( hrColor ) && SUCCEEDED( hrDepth ) )
		{
			IFrameDescription* pColorFrameDescription = nullptr;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			int colorFrameWidth = 0;
			int colorFrameHeight = 0;
			unsigned int colorBufferSize = 0;
			RGBQUAD* pTmpColorBuffer = nullptr;

			IFrameDescription* pDepthFrameDescription = nullptr;
			int64 depthTime = 0;
			int depthFrameWidth = 0;
			int depthFrameHeight = 0;
			unsigned int depthBufferSize = 0;
			ushort* pTmpDepthBuffer = nullptr;


			// Color Data
			hr = pColorFrame->get_FrameDescription( &pColorFrameDescription );
			if ( SUCCEEDED( hr ) )
			{
				pColorFrameDescription->get_Width( &colorFrameWidth );
				pColorFrameDescription->get_Height( &colorFrameHeight );
				hr = pColorFrame->get_RawColorImageFormat( &imageFormat );
			}

			if ( SUCCEEDED( hr ) )
			{
				if ( imageFormat == ColorImageFormat_Bgra )
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer( &colorBufferSize, reinterpret_cast< BYTE** >( &pTmpColorBuffer ) );
				}
				else if ( pColorRAWBuffer )
				{
					pTmpColorBuffer = pColorRAWBuffer;
					colorBufferSize = COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * sizeof( RGBQUAD );
					hr = pColorFrame->CopyConvertedFrameDataToArray( colorBufferSize, reinterpret_cast< BYTE* >( pTmpColorBuffer ), ColorImageFormat_Bgra );
				}
				else
				{
					hr = E_FAIL;
				}
			}

			if ( SUCCEEDED( hr ) )
			{
				hr = pDepthFrame->get_FrameDescription( &pDepthFrameDescription );
			}

			if ( SUCCEEDED( hr ) )
			{
				pDepthFrameDescription->get_Width( &depthFrameWidth );
				pDepthFrameDescription->get_Height( &depthFrameHeight );
				hr = pDepthFrame->AccessUnderlyingBuffer( &depthBufferSize, &pTmpDepthBuffer );
			}


			if ( SUCCEEDED( hr ) )
			{
				// Check color buffer size
				if ( colorBufferSize == COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * sizeof( RGBQUAD ) )
				{
					// Copy color data
					memcpy( pColorRAWBuffer, pTmpColorBuffer, colorBufferSize );
				}

				// Check depth buffer size
				if ( depthBufferSize == DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT )
				{
					// Copy depth data
					memcpy( pDepthRAWBuffer, pTmpDepthBuffer, depthBufferSize * sizeof( ushort ) );
				}

			}
			SafeRelease( &pColorFrameDescription );
			SafeRelease( &pDepthFrameDescription );
		}

		if ( SUCCEEDED( hrInfra ) )
		{
			IFrameDescription* pInfraFrameDescription = nullptr;
			unsigned int infraBufferSize = 0;
			int infraFrameWidth = 0;
			int infraFrameHeight = 0;
			UINT16* pTmpInfraBuffer = nullptr;

			hr = pInfraFrame->get_FrameDescription( &pInfraFrameDescription );

			if ( SUCCEEDED( hr ) )
			{
				hr = pInfraFrameDescription->get_Width( &infraFrameWidth );
				hr = pInfraFrameDescription->get_Height( &infraFrameHeight );
				hr = pInfraFrame->AccessUnderlyingBuffer( &infraBufferSize, &pTmpInfraBuffer );
			}

			if ( SUCCEEDED( hr ) )
			{
				if ( infraBufferSize == INFRARED_FRAME_WIDTH * INFRARED_FRAME_HEIGHT )
				{
					memcpy( pInfraRAWBuffer, pTmpInfraBuffer, infraBufferSize * sizeof( ushort ) );
				}
			}
		}

		SafeRelease( &pColorFrame );
		SafeRelease( &pDepthFrame );
		SafeRelease( &pInfraFrame );
		SafeRelease( &pMultiSourceFrame );
	}
}

HRESULT CK4Wv2OpenCVModule::calculateMappedFrame()
{
	HRESULT hr = E_FAIL;

	// Depth coordinate mapping
	if ( pCoordinateMapper != nullptr )
	{
		/*
		// Color 2 Depth Mapping
		hr = pCoordinateMapper->MapDepthFrameToColorSpace( DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT, (UINT16*)pTmpDepthBuffer, DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT, pColorCoodinate );
		if ( SUCCEEDED( hr ) )
		{
		Mat t = Mat( Size( DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT ), CV_8UC4 );
		Vec4b* p = t.ptr< Vec4b >( 0 );
		for ( int idx = 0 ; idx < DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT ; idx++ )
		{
		ColorSpacePoint csp = pColorCoodinate[idx];
		int colorX = (int)floor( csp.X + 0.5 );
		int colorY = (int)floor( csp.Y + 0.5 );
		if ( colorX >= 0 && colorX < COLOR_FRAME_WIDTH && colorY >= 0 && colorY < COLOR_FRAME_HEIGHT )
		{
		p[idx] = colorRAWFrameMat.at< Vec4b >( colorY, colorX );
		}
		}
		imshow( "C2D", t );
		}
		*/
		// Depth 2 Color Mapping
		hr = pCoordinateMapper->MapColorFrameToDepthSpace( DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT, (UINT16*)pDepthRAWBuffer, COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT, pDepthCoordinate );
		if ( SUCCEEDED( hr ) )
		{
			colorMappedFrameMat = Mat::zeros( Size( COLOR_FRAME_WIDTH, COLOR_FRAME_HEIGHT ), CV_8UC4 );
			Vec4b* pMappedFrame = colorMappedFrameMat.ptr< Vec4b >( 0 );
			#pragma omp parallel for
			for ( int idx = 0 ; idx < COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT ; idx++ )
			{
				DepthSpacePoint dsp = pDepthCoordinate[idx];
				int depthX = (int)floor( dsp.X + 0.5 );
				int depthY = (int)floor( dsp.Y + 0.5 );
				if ( depthX >= 0 && depthX < DEPTH_FRAME_WIDTH && depthY >= 0 && depthY < DEPTH_FRAME_HEIGHT )
				{
					ushort val = pDepthRAWBuffer[depthY * DEPTH_FRAME_WIDTH + depthX];
					if ( val )
						pMappedFrame[idx] = ( (Vec4b*)pColorRAWBuffer )[idx];
					else
						pMappedFrame[idx] = Vec4b( 0, 0, 0, 0 );
				}
			}
		}
	}

	return hr;
}

template< class T > void CK4Wv2OpenCVModule::SafeRelease( T** ppT )
{
	if ( *ppT )
	{
		( *ppT )->Release();
		*ppT = nullptr;
	}
}