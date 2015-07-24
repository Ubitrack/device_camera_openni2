/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Synchronouse capture of camera images using Point Grey's FlyCapture2 library.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 *
 */

#include "OpenNI2FrameGrabber.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <utDataflow/ComponentFactory.h>
#include <utUtil/OS.h>
#include <boost/array.hpp>

#include <log4cpp/Category.hh>

namespace Ubitrack { namespace Drivers {
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.OpenNI2FrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Drivers;
using namespace FlyCapture2;


OpenNI2Module::OpenNI2Module( const Dataflow::SingleModuleKey& moduleKey, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
        : Module< Dataflow::SingleModuleKey, OpenNI2ComponentKey, OpenNI2Module, OpenNI2Component >( moduleKey, pFactory )
        , m_bStop(false)
{

	// start thread immediately - it will not send images if the module is not running ..
	m_Thread.reset( new boost::thread( boost::bind( &OpenNI2Module::ThreadProc, this ) ) );
}

OpenNI2Module::~OpenNI2Module()
{
	// may need a lock here ...
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}
}


void OpenNI2Module::ThreadProc()
{
	
	LOG4CPP_DEBUG( logger, "Thread started" );

	// initialize FlyCapture
	BusManager busMgr;
	unsigned nCameras;
	if ( busMgr.GetNumOfCameras( &nCameras ) != PGRERROR_OK || nCameras == 0 )
	{
		LOG4CPP_ERROR( logger, "No PointGrey cameras found!" );
		return;
	}
	
	
    ComponentList allComponents( getAllComponents() );
	
	if (nCameras < allComponents.size()) {
		LOG4CPP_ERROR( logger, "Not enough PointGrey cameras found for current config!" );
		return;	
	}

	Camera** ppCameras = new Camera*[allComponents.size()];
    int index = 0;

    for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {

		ppCameras[index] = new Camera();

		unsigned int serial = (*it)->getKey().getCameraSerialNumber();
		PGRGuid guid;

		if (serial >= 0) {
			if ( busMgr.GetCameraFromSerialNumber((unsigned int) serial, &guid) != PGRERROR_OK )
			{
				LOG4CPP_ERROR( logger, "Error in FlyCapture2::BusManager::GetCameraFromSerialNumber" );
				return;
			}
		} else {
			LOG4CPP_ERROR( logger, "Invalid Serial specified for Camera!" );
			return;
		}
		
		if ( ppCameras[index]->Connect( &guid ) != PGRERROR_OK )
		{
			LOG4CPP_ERROR( logger, "Error in FlyCapture2::Camera::Connect" );
			return;
		}

		// delegate setup to component
		(*it)->setupCamera(index, ppCameras[index]);

    }	
	
	if( Camera::StartSyncCapture( allComponents.size(), (const Camera**)ppCameras ) != PGRERROR_OK)
    {
		LOG4CPP_ERROR( logger, "Error in FlyCapture2::Camera::StartSyncCapture" );
        return;
    }
	
	while ( !m_bStop )
	{

		Measurement::Timestamp ts;

	    for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
			int index = (*it)->getCameraIndex();
			FlyCapture2::Image image;

			if ( ppCameras[index]->RetrieveBuffer( &image ) != PGRERROR_OK )
			{
				LOG4CPP_ERROR( logger, "Could not retrieve buffer" );
				return;
			}
			// eventually use the timestamp from the image ??
			// but for now, set timestamp when captured image from first cam in the list
			if (it == allComponents.begin()) {
				ts = Measurement::now();
			}

			if ( !m_running )
				continue;

			(*it)->processImage(ts, image);
		}
	}

    for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
			int index = (*it)->getCameraIndex();
			ppCameras[index]->StopCapture();
			ppCameras[index]->Disconnect();
			delete ppCameras[index];

	}

	delete [] ppCameras;
	LOG4CPP_DEBUG( logger, "Thread stopped" );
}

boost::shared_ptr< OpenNI2Module::ComponentClass > OpenNI2Module::createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph, const ComponentKey& key, ModuleClass* pModule ) {
	return boost::shared_ptr< ComponentClass >( new OpenNI2Component( name, subgraph, key, pModule ) );
}


OpenNI2Component::OpenNI2Component( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const OpenNI2ComponentKey& componentKey, OpenNI2Module* pModule )
	: OpenNI2Module::Component( name, componentKey, pModule )
	, m_videoMode( FlyCapture2::NUM_VIDEOMODES )
	, m_frameRate( FlyCapture2::NUM_FRAMERATES )
	, m_pixelFormat( FlyCapture2::NUM_PIXEL_FORMATS )
	, m_index( -1 )
	, m_gainDB( -1 ) // -1 is auto
	, m_shutterMS( -1 ) // -1 is auto
	, m_triggerFlash( false )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
{
	subgraph->m_DataflowAttributes.getAttributeData( "gainDB", m_gainDB );
	subgraph->m_DataflowAttributes.getAttributeData( "shutterMS", m_shutterMS );

	if ( subgraph->m_DataflowAttributes.getAttributeString( "triggerFlash" ) == "true")
	{
		m_triggerFlash = true;
	}


	if ( subgraph->m_DataflowAttributes.hasAttribute( "videoMode" ) )
	{
		std::string sVideoMode = subgraph->m_DataflowAttributes.getAttributeString( "videoMode" );
		if ( flyCaptureModeMap.find( sVideoMode ) == flyCaptureModeMap.end() )
			UBITRACK_THROW( "unknown video mode: \"" + sVideoMode + "\"" );
		m_videoMode = flyCaptureModeMap[ sVideoMode ];
	}

	//if ( subgraph->m_DataflowAttributes.hasAttribute( "pixelFormat" ) )
	//{
	//	std::string sPixelFormat = subgraph->m_DataflowAttributes.getAttributeString( "pixelFormat" );
	//	if ( flyCapturePixelFormatMap.find( sPixelFormat ) == flyCapturePixelFormatMap.end() )
	//		UBITRACK_THROW( "unknown pixel format: \"" + sPixelFormat + "\"" );
	//	m_pixelFormat = flyCapturePixelFormatMap[ sPixelFormat ];
	//}

	if ( subgraph->m_DataflowAttributes.hasAttribute( "frameRate" ) )
	{
		std::string sFrameRate = subgraph->m_DataflowAttributes.getAttributeString( "frameRate" );
		if ( flyCaptureFrameRateMap.find( sFrameRate ) == flyCaptureFrameRateMap.end() )
			UBITRACK_THROW( "unknown frame rate: \"" + sFrameRate + "\"" );
		m_frameRate = flyCaptureFrameRateMap[ sFrameRate ];
	}
	
	if ( ( m_frameRate != NUM_FRAMERATES ) != ( m_videoMode != NUM_VIDEOMODES ) )
		LOG4CPP_WARN( logger, "Both videoMode and frameRate must be set for any value to have an effect!" );

}


void OpenNI2Component::setupCamera( int index, FlyCapture2::Camera* cam ) {
	// store the index locally
	m_index = index;

	if ( m_frameRate != NUM_FRAMERATES && m_videoMode != NUM_VIDEOMODES )
	{
		LOG4CPP_INFO( logger, "Setting framerate and videomode" );
		if ( cam->SetVideoModeAndFrameRate( m_videoMode, m_frameRate ) != PGRERROR_OK )
			LOG4CPP_WARN( logger, "Error in FlyCapture2::Camera::SetVideoModeAndFrameRate" );
	}

	if(m_triggerFlash) {
		// set GPIO to output
		cam->WriteRegister(0x11f8, 0xe0000000);
		// set GPIO signal to delay 0 and duration 2 (last 6 bytes)
		cam->WriteRegister(0x1500, 0x83000003);
	} else {
		// how to turn it off?!
		//cam.WriteRegister(0x1500, 0x83000000);	
	}

	// set gain
	if(m_gainDB < 0) {
		Property prop;
		prop.type = GAIN;
		prop.onePush = true;
		prop.onOff = true;
		prop.autoManualMode = true;
		prop.valueA = 0;
		prop.valueB = 0;
		if ( cam->SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting auto Gain." );
	} else {
		Property prop;
		prop.type = GAIN;
		prop.onePush = false;
		prop.onOff = true;
		prop.autoManualMode = false;
		prop.absControl = true;
		prop.absValue = (float)m_gainDB;
		if ( cam->SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting manual Gain." );
	}

	// set shutter
	if(m_shutterMS < 0) {
		Property prop;
		prop.type = SHUTTER;
		prop.onePush = true;
		prop.onOff = true;
		prop.autoManualMode = true;
		prop.valueA = 0;
		prop.valueB = 0;
		if ( cam->SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting auto Gain." );
	} else {
		Property prop;
		prop.type = SHUTTER;
		prop.onePush = false;
		prop.onOff = true;
		prop.autoManualMode = false;
		prop.absControl = true;
		prop.absValue = (float)m_shutterMS;
		if ( cam->SetProperty(&prop) != PGRERROR_OK )
			LOG4CPP_ERROR( logger, "Error setting manual Gain." );
	}

}




void OpenNI2Component::processImage( Measurement::Timestamp ts, const FlyCapture2::Image& image) {
	boost::shared_ptr< Vision::Image > pColorImage;
	boost::shared_ptr< Vision::Image > pGreyImage;

	if ( image.GetPixelFormat() == PIXEL_FORMAT_MONO8 )
	{
		pGreyImage.reset(new Vision::Image( image.GetCols(), image.GetRows(), 1, image.GetData() ) );
		pGreyImage->widthStep = image.GetStride();

		m_outPort.send( Measurement::ImageMeasurement( ts, pGreyImage ) );
			
		if ( m_colorOutPort.isConnected() )
			m_colorOutPort.send( Measurement::ImageMeasurement( ts, pGreyImage->CvtColor( CV_GRAY2RGB, 3 ) ) );
	} 
	else if ( image.GetPixelFormat() == PIXEL_FORMAT_RGB8 )
	{
		pColorImage.reset(new Vision::Image( image.GetCols(), image.GetRows(), 3, image.GetData() ) );
		pColorImage->widthStep = image.GetStride();

		if ( m_colorOutPort.isConnected() )
			m_colorOutPort.send( Measurement::ImageMeasurement( ts, pColorImage ) );
		if ( m_outPort.isConnected() )
			m_outPort.send( Measurement::ImageMeasurement( ts, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );
	} 
	else if ( image.GetPixelFormat() == PIXEL_FORMAT_RAW8 )
	{
		// convert RAW image to RGB8
		FlyCapture2::Image convertedImage;
		image.Convert(PIXEL_FORMAT_RGB, &convertedImage);

		pColorImage.reset(new Vision::Image( convertedImage.GetCols(), convertedImage.GetRows(), 3, convertedImage.GetData() ) );
		pColorImage->widthStep = convertedImage.GetStride();

		if ( m_colorOutPort.isConnected() )
			m_colorOutPort.send( Measurement::ImageMeasurement( ts, pColorImage ) );
		if ( m_outPort.isConnected() )
			m_outPort.send( Measurement::ImageMeasurement( ts, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );
	} 
	else {
		LOG4CPP_DEBUG( logger, "UNKOWN PIXEL FORMAT: " << image.GetPixelFormat() );	
	}
}

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerModule< OpenNI2Module > ( "OpenNI2FrameGrabber" );
}

} } // namespace Ubitrack::Drivers
