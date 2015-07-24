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
 * @ingroup driver_components
 * @file
 * OpenNI2 driver
 * This file contains the driver component to
 * synchronously capture camera images using flycapture.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 */

#ifndef __OpenNI2_h_INCLUDED__
#define __OpenNI2_h_INCLUDED__


#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>

#include <utVision/Image.h>
#include <opencv/cv.h>

#include <FlyCapture2.h>





namespace {
	class OpenNI2ModeMap 
		: public std::map< std::string, FlyCapture2::VideoMode >
	{
	public:
		OpenNI2ModeMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "640x480RGB" ] = VIDEOMODE_640x480RGB;
			(*this)[ "640x480Y8" ] = VIDEOMODE_640x480Y8;
			(*this)[ "800x600RGB" ] = VIDEOMODE_800x600RGB;
			(*this)[ "800x600Y8" ] = VIDEOMODE_800x600Y8;
			(*this)[ "1024x768RGB" ] = VIDEOMODE_1024x768RGB;
			(*this)[ "1024x768Y8" ] = VIDEOMODE_1024x768Y8;
			(*this)[ "1280x960RGB" ] = VIDEOMODE_1280x960RGB;
			(*this)[ "1280x960Y8" ] = VIDEOMODE_1280x960Y8;
			(*this)[ "1600x1200RGB" ] = VIDEOMODE_1600x1200RGB;
			(*this)[ "1600x1200Y8" ] = VIDEOMODE_1600x1200Y8;
		}
	};
	static OpenNI2ModeMap flyCaptureModeMap;

	class OpenNI2PixelFormatMap 
		: public std::map< std::string, FlyCapture2::PixelFormat >
	{
	public:
		OpenNI2PixelFormatMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "MONO8" ] = PIXEL_FORMAT_MONO8;
			(*this)[ "411YUV8" ] = PIXEL_FORMAT_411YUV8;
			(*this)[ "422YUV8" ] = PIXEL_FORMAT_422YUV8;
			(*this)[ "444YUV8" ] = PIXEL_FORMAT_444YUV8;
			(*this)[ "RGB8" ] = PIXEL_FORMAT_RGB8;
			(*this)[ "MONO16" ] = PIXEL_FORMAT_MONO16;
			(*this)[ "RGB16" ] = PIXEL_FORMAT_RGB16;
			(*this)[ "S_MONO16" ] = PIXEL_FORMAT_S_MONO16;
			(*this)[ "S_RGB16" ] = PIXEL_FORMAT_S_RGB16;
			(*this)[ "RAW8" ] = PIXEL_FORMAT_RAW8;
			(*this)[ "RAW16" ] = PIXEL_FORMAT_RAW16;
			(*this)[ "MONO12" ] = PIXEL_FORMAT_MONO12;
			(*this)[ "RAW12" ] = PIXEL_FORMAT_RAW12;
			(*this)[ "BGR" ] = PIXEL_FORMAT_BGR;
			(*this)[ "BGRU" ] = PIXEL_FORMAT_BGRU;
			(*this)[ "RGB" ] = PIXEL_FORMAT_RGB;
			(*this)[ "RGBU" ] = PIXEL_FORMAT_RGBU;
		}
	};
	static OpenNI2PixelFormatMap flyCapturePixelFormatMap;

	class OpenNI2FrameRateMap 
		: public std::map< std::string, FlyCapture2::FrameRate >
	{
	public:
		OpenNI2FrameRateMap()
		{
			using namespace FlyCapture2;
			
			(*this)[ "1.875" ] = FRAMERATE_1_875;
			(*this)[ "3.75" ] = FRAMERATE_3_75;
			(*this)[ "7.5" ] = FRAMERATE_7_5;
			(*this)[ "15" ] = FRAMERATE_15;
			(*this)[ "30" ] = FRAMERATE_30;
			(*this)[ "60" ] = FRAMERATE_60;
			(*this)[ "120" ] = FRAMERATE_120;
			(*this)[ "240" ] = FRAMERATE_240;
		}
	};
	static OpenNI2FrameRateMap flyCaptureFrameRateMap;

} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

// forward declaration
class OpenNI2Component;

/**
 * Component key for flycapture.
 * Represents the camera
 */
class OpenNI2ComponentKey
{
public:

	OpenNI2ComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: m_cameraSerialNumber( -1 )
	{
	  subgraph->m_DataflowAttributes.getAttributeData( "cameraSerialNumber", m_cameraSerialNumber );
	  if (( m_cameraSerialNumber <= 0 ))
            UBITRACK_THROW( "Invalid camera serial number" );

	}

	// construct from body number
	OpenNI2ComponentKey( int a )
		: m_cameraSerialNumber( a )
 	{}
	
	unsigned int  getCameraSerialNumber() const {
		return m_cameraSerialNumber;
	}

	// less than operator for map
	bool operator<( const OpenNI2ComponentKey& b ) const
    {
		return m_cameraSerialNumber < b.m_cameraSerialNumber;
    }

protected:
	unsigned int m_cameraSerialNumber;
};



/**
 * Module for FlyCapture2 tracker.
 * Does all the work
 */
class OpenNI2Module
	: public Module< SingleModuleKey, OpenNI2ComponentKey, OpenNI2Module, OpenNI2Component >
{
public:
	/** UTQL constructor */
	OpenNI2Module( const SingleModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* pFactory );

	/** destructor */
	~OpenNI2Module();

protected:

	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	/** create the components **/
	boost::shared_ptr< ComponentClass > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
														 const ComponentKey& key, ModuleClass* pModule );

};

std::ostream& operator<<( std::ostream& s, const OpenNI2ComponentKey& k );

/**
 * Component for FlyCapture2 tracker.
 */
class OpenNI2Component : public OpenNI2Module::Component {
public:
	/** constructor */
	OpenNI2Component( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const OpenNI2ComponentKey& componentKey, OpenNI2Module* pModule );

	void setupCamera( int index, FlyCapture2::Camera* cam );

	inline int getCameraIndex() {
		return m_index;
	}

	void processImage( Measurement::Timestamp ts, const FlyCapture2::Image& image);

	/** destructor */
	~OpenNI2Component() {};

protected:

	// fly capture stuff
	FlyCapture2::VideoMode m_videoMode;
	FlyCapture2::FrameRate m_frameRate;
	FlyCapture2::PixelFormat m_pixelFormat;

	// index in the camera struct
	int m_index;


	// the serial number
	int m_cameraSerialNumber;

	// trigger flash
	bool m_triggerFlash;

	// gain
	double m_gainDB;

	// shutter
	double m_shutterMS;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;

};

} } // namespace Ubitrack::Drivers

#endif
