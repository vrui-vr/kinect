/***********************************************************************
CameraOrbbec - Class to represent the color and depth camera interface
aspects of an Orbbec 3D camera supported by the Orbbec SDK.
Copyright (c) 2025 Oliver Kreylos

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

#include <libobsensor/ObSensor.hpp>
#include <Misc/SizedTypes.h>
#include <Misc/Autopointer.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
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
	Size depthSize; // Requested size for streamed depth frames
	Size colorSize; // Requested size for streamed color frames
	unsigned int fps; // Requested frame rate for depth and color frames
	bool sensorsAcquired; // Flag if the selected camera's depth and color sensors have already been acquired
	SensorPtr depthSensor; // The depth sensor
	VideoStreamProfilePtr depthProfile; // Profile of the depth video stream
	DepthPixel dMax; // Maximum valid depth pixel reported at FrameSource interface
	float zRange[2]; // Range of valid depth values reported at the FrameSource interface in cm
	float zQuant[2]; // Parameters for the depth quantization formula
	SensorPtr colorSensor; // The color sensor
	VideoStreamProfilePtr colorProfile; // Profile of the color video stream
	
	/* Private methods: */
	void acquireSensors(void); // Acquires the selected Orbbec camera's depth and color sensors
	static IntrinsicParameters::LensDistortion getLensDistortion(ob::VideoStreamProfile& profile); // Returns the lens distortion correction parameters of the given video stream profile
	
	/* Constructors and destructors: */
	public:
	CameraOrbbec(size_t index =0); // Opens the index-th Orbbec camera connected to the host
	CameraOrbbec(const char* serialNumber); // Opens the Orbbec camera with the given serial number
	virtual ~CameraOrbbec(void);
	
	/* Methods from class FrameSource: */
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual const Size& getActualFrameSize(int sensor) const;
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
	virtual void stopStreaming(void);
	
	/* Methods from class DirectFrameSource: */
	virtual std::string getSerialNumber(void);
	virtual void configure(Misc::ConfigurationFileSection& configFileSection);
	virtual void buildSettingsDialog(GLMotif::RowColumn* settingsDialog);
	
	/* New methods: */
	void setDepthFrameSize(const Size& newDepthFrameSize); // Sets the depth frame size to be requested when streaming starts
	void setColorFrameSize(const Size& newColorFrameSize); // Sets the color frame size to be requested when streaming starts
	unsigned int getFps(void) const // Returns the requested frame rate
		{
		return fps;
		}
	const float* getZRange(void) const // Returns the requested depth range in cm as an array of {zMin, zMax}
		{
		return zRange;
		}
	void setFps(unsigned int newFps); // Sets the frame rate to be requested when streaming starts
	void setZRange(float zMin,float zMax); // Sets the limits of the reported quantized depth range in cm
	};

}

#endif
