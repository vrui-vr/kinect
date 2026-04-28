/***********************************************************************
CameraOrbbec - Class to represent the color and depth camera interface
aspects of an Orbbec 3D camera supported by the Orbbec SDK.
Copyright (c) 2025-2026 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef KINECT_CAMERAORBBEC_INCLUDED
#define KINECT_CAMERAORBBEC_INCLUDED

#include <memory>
#include <Misc/SizedTypes.h>
#include <Misc/Autopointer.h>
#include <Kinect/Config.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
namespace Video {
class ImageExtractor;
}
namespace ob {
class Device;
class Sensor;
class StreamProfile;
class VideoStreamProfile;\
class Frame;
class DepthFrame;
}
namespace Kinect {
class OrbbecSDKContext;
typedef Misc::Autopointer<OrbbecSDKContext> OrbbecSDKContextPtr;
}

namespace Kinect {

class CameraOrbbec:public DirectFrameSource
	{
	/* Embedded classes: */
	private:
	typedef std::shared_ptr<ob::Device> DevicePtr;
	typedef std::shared_ptr<ob::Sensor> SensorPtr;
	typedef std::shared_ptr<ob::StreamProfile> StreamProfilePtr;
	typedef std::shared_ptr<ob::VideoStreamProfile> VideoStreamProfilePtr;
	typedef std::shared_ptr<ob::DepthFrame> DepthFramePtr;
	public:
	typedef Misc::UInt16 ObDepthPixel; // Type for raw depth values received from an Orbbec depth sensor
	
	/* Elements: */
	OrbbecSDKContextPtr context; // Pointer to the Orbbec SDK context shared by all Orbbec cameras connected to the host
	DevicePtr device; // The device from which to stream data
	ColorStreamFormat colorStreamFormat; // The requested color stream format
	DepthStreamFormat depthStreamFormat; // The requested depth stream format
	ZRange zRange; // The requested absolute z value range
	SensorPtr colorSensor; // The color sensor
	VideoStreamProfilePtr colorProfile; // Profile of the color video stream
	SensorPtr depthSensor; // The depth sensor
	VideoStreamProfilePtr depthProfile; // Profile of the depth video stream
	bool sensorsAcquired; // Flag if the selected camera's color and depth sensors have already been acquired
	float zQuant[2]; // Parameters for the depth quantization formula
	Video::ImageExtractor* colorFrameExtractor; // Helper object to convert a raw color frame to RGB
	
	/* Private methods: */
	void acquireSensors(void); // Acquires the selected Orbbec camera's depth and color sensors
	static IntrinsicParameters::LensDistortion getLensDistortion(ob::VideoStreamProfile& profile,bool flipX,bool flipY); // Returns the lens distortion correction parameters of the given video stream profile
	void colorFrameCallback(std::shared_ptr<ob::Frame> frame); // Callback called when the color sensor delivers a new frame
	void depthFrameCallback(std::shared_ptr<ob::Frame> frame); // Callback called when the depth sensor delivers a new frame
	void initialize(void); // Initializes the object after an Orbbec device has been selected
	
	/* Constructors and destructors: */
	public:
	static size_t getNumDevices(void); // Returns the number of Orbbec cameras connected to the host
	CameraOrbbec(size_t index =0); // Opens the index-th Orbbec camera connected to the host
	CameraOrbbec(const char* serialNumber); // Opens the Orbbec camera with the given serial number
	virtual ~CameraOrbbec(void);
	
	/* Methods from class FrameSource: */
	virtual ColorStreamFormat getColorStreamFormat(void) const;
	virtual DepthStreamFormat getDepthStreamFormat(void) const;
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual const Size& getActualFrameSize(int sensor) const;
	virtual void startStreaming(void);
	virtual void stopStreaming(void);
	
	/* Methods from class DirectFrameSource: */
	virtual std::string getSerialNumber(void);
	virtual void requestColorStreamFormat(const ColorStreamFormat& format);
	virtual void requestDepthStreamFormat(const DepthStreamFormat& format);
	virtual void requestZRange(const ZRange& zRange);
	virtual void configure(Misc::ConfigurationFileSection& configFileSection);
	virtual void fixFormats(void);
	virtual void buildSettingsDialog(GLMotif::RowColumn* settingsDialog);
	};

}

#endif
