/***********************************************************************
Camera - Wrapper class to represent the color and depth camera interface
aspects of the Kinect sensor.
Copyright (c) 2010-2026 Oliver Kreylos

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

#ifndef KINECT_CAMERA_INCLUDED
#define KINECT_CAMERA_INCLUDED

/* Set to 1 for frame stream analysis: */
#define KINECT_CAMERA_DUMP_HEADERS 0

#include <string>
#include <Misc/SizedTypes.h>
#include <Misc/Timer.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <USB/Device.h>
#if KINECT_CAMERA_DUMP_HEADERS
#include <IO/File.h>
#endif
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
struct libusb_device;
struct libusb_transfer;
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace USB {
class DeviceList;
}
namespace IO {
class File;
}

namespace Kinect {

class Camera:public DirectFrameSource
	{
	/* Embedded classes: */
	public:
	enum FrameSize // Enumerated type to select color and depth frame sizes
		{
		FS_640_480=0, // 640x480 frames
		FS_1280_1024 // 1280x1024 frames, only valid for color camera
		};
	enum FrameRate // Enumerated type to select frame rates for the color and depth cameras
		{
		FR_15_HZ=0, // 15 Hz
		FR_30_HZ // 30 Hz, only possible for 640x480 pixel frames
		};
	
	struct CalibrationParameters // Structure to hold factory calibration parameters read from Kinect camera's non-volatile RAM
		{
		/* Elements: */
		public:
		
		/*********************************************************************
		For whatever reason, the calibration parameters are queried in four
		subsections at the USB interface. We'll have to deal with that.
		*********************************************************************/
		
		/* First parameter subsection: registration parameters */
		Misc::SInt32 dxCenter;
		Misc::SInt32 ax,bx,cx,dx;
		Misc::SInt32 dxStart;
		Misc::SInt32 ay,by,cy,dy;
		Misc::SInt32 dyStart;
		Misc::SInt32 dxBetaStart,dyBetaStart;
		Misc::SInt32 rolloutBlank,rolloutSize;
		Misc::SInt32 dxBetaInc,dyBetaInc;
		Misc::SInt32 dxdxStart,dxdyStart,dydxStart,dydyStart;
		Misc::SInt32 dxdxdxStart,dydxdxStart,dxdxdyStart,dydxdyStart;
		Misc::SInt32 backComp1;
		Misc::SInt32 dydydxStart;
		Misc::SInt32 backComp2;
		Misc::SInt32 dydydyStart;
		
		/* Second parameter subsection: padding parameters */
		Misc::UInt16 startLines;
		Misc::UInt16 endLines;
		Misc::UInt16 croppingLines;
		
		/* Third parameter subsection: constant shift */
		Misc::UInt16 constantShift;
		
		/* Fourth parameter subsection: zero-plane parameters */
		Misc::Float32 dcmosEmitterDist;
		Misc::Float32 dcmosRcmosDist;
		Misc::Float32 referenceDistance;
		Misc::Float32 referencePixelSize;
		
		/* Methods: */
		void read(int subsection,IO::File& file); // Reads the given calibration parameter subsection from file
		void read(IO::File& file); // Reads all calibration parameters from file
		void write(IO::File& file) const; // Writes all calibration parameters to file
		};
	
	private:
	struct CameraParameters; // Structure containing the imaging parameters of the color camera
	typedef Misc::UInt16 USBWord; // Type for words of data exchanged at the USB library API
	struct StreamingState; // Structure containing necessary state to stream color or depth frames from the respective camera
	friend class StreamingState;
	
	/* Elements: */
	private:
	static const Size actualFrameSizes[2]; // Actual frame sizes in pixels for the two selectable resolutions
	static const Rational actualFrameRates[2]; // Actual frame rates in Hz for the two selectable frame rates
	USB::Device device; // The USB device representing this Kinect camera
	std::string serialNumber; // This Kinect camera's serial number
	size_t calibrationParameterReplySizes[4]; // USB reply sizes when querying the four subsets of factory calibration parameters
	bool needAltInterface; // Flag whether the camera device needs to be switched to an alternate interface
	bool hasNearMode; // Flag whether the camera device has a "near mode" (for Kinect-for-Windows)
	FrameSize frameSizes[2]; // Selected frame sizes for the color and depth cameras
	FrameRate frameRates[2]; // Selected frame rates for the color and depth cameras
	USBWord messageSequenceNumber; // Incrementing sequence number for command messages to the camera
	bool compressDepthFrames; // Flag whether to request RLE/differential compressed depth frames from the depth camera
	bool smoothDepthFrames; // Flag whether to smooth depth frames inside the Kinect's processor, whatever that means
	unsigned short irIntensity; // Intensity of IR projector
	bool nearMode; // Flag if "near mode" is enabled on supporting camera devices
	unsigned int exposure; // Color camera exposure value
	unsigned int sharpening; // Color camera sharpening value for next streaming operation
	StreamingState* streamers[2]; // Streaming states for color and depth frames
	
	#if KINECT_CAMERA_DUMP_HEADERS
	IO::FilePtr headerFile;
	#endif
	
	/* Private methods: */
	size_t sendMessage(USBWord messageType,const USBWord* messageData,size_t messageSize,void* replyBuffer,size_t replyBufferSize); // Sends a general message to the camera device; returns reply size in bytes
	bool sendCommand(USBWord command,USBWord value); // Sends a command message to the camera device; returns true if command was processed properly
	USBWord readRegister(USBWord address); // Reads a camera register and returns its value
	void writeRegister(USBWord address,USBWord value); // Sets a camera register to the given value
	void* colorDecodingThreadMethod(void); // The color decoding thread method
	void* depthDecodingThreadMethod(void); // The depth decoding thread method
	void* compressedDepthDecodingThreadMethod(void); // The depth decoding thread method for RLE/differential-compressed frames
	void initialize(USB::DeviceList* deviceList =0); // Initializes the Kinect camera; called from constructors
	void resetCameras(void); // Resets the cameras to non-streaming mode after streaming is stopped
	void nearModeToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData); // Called when user toggles "near mode" button
	void irIntensitySliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData); // Called when user moves IR intensity slider
	void colorExposureSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData); // Called when user moves color exposure slider
	void colorSharpeningSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData); // Called when user moves color sharpening slider
	
	/* Constructors and destructors: */
	public:
	static size_t getNumDevices(void); // Returns the number of Kinect cameras on the local host's bus
	Camera(libusb_device* sDevice); // Creates a Kinect camera wrapper around the given USB device, which is assumed to be a Kinect camera
	Camera(size_t index =0); // Opens the index-th Kinect camera device
	Camera(const char* serialNumber); // Opens the Kinect camera with the given serial number
	virtual ~Camera(void); // Destroys the camera
	
	/* Methods from class FrameSource: */
	virtual ColorStreamFormat getColorStreamFormat(void) const;
	virtual DepthStreamFormat getDepthStreamFormat(void) const;
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual const Size& getActualFrameSize(int sensor) const;
	virtual DepthRange getDepthRange(void) const;
	virtual void startStreaming(void);
	virtual void stopStreaming(void);
	
	/* Methods from class DirectFrameSource: */
	virtual std::string getSerialNumber(void);
	virtual void requestColorStreamFormat(const ColorStreamFormat& format);
	virtual void requestDepthStreamFormat(const DepthStreamFormat& format);
	virtual void requestZRange(const ZRange& zRange);
	virtual void configure(Misc::ConfigurationFileSection& configFileSection);
	virtual void buildSettingsDialog(GLMotif::RowColumn* settingsDialog);
	
	/* New methods: */
	void getCalibrationParameters(CalibrationParameters& calib); // Queries factory calibration parameters from Kinect's non-volatile RAM
	void setFrameSize(int camera,FrameSize newFrameSize); // Sets the frame size of the color or depth camera for the next streaming operation
	FrameSize getFrameSize(int camera) // Returns the selected frame size of the color or depth camera
		{
		return frameSizes[camera];
		}
	void setFrameRate(int camera,FrameRate newFrameRate); // Sets the frame rate of the color or depth camera for the next streaming operation
	FrameRate getFrameRate(int camera) const // Returns the selected frame rate of the color or depth camera
		{
		return frameRates[camera];
		}
	unsigned int getActualFrameRate(int camera) const; // Returns the selected frame rate of the color or depth camera in Hz
	void setCompressDepthFrames(bool newCompressDepthFrames); // Enables or disables depth frame compression for the next streaming operation
	void setSmoothDepthFrames(bool newSmoothDepthFrames); // Enables or disables depth frame smoothing for the next streaming operation
	unsigned short getIrIntensity(void) // Returns the currently selected IR projector intensity
		{
		return irIntensity;
		}
	void setIrIntensity(unsigned short newIrIntensity); // Sets the IR projector's intensity from 1 (low) to 50 (high)
	bool supportsNearMode(void) const // Returns true if the camera device supports "near mode"
		{
		return hasNearMode;
		}
	bool getNearMode(void) const // Returns true if camera device is in "near mode"
		{
		return nearMode;
		}
	void setNearMode(bool newNearMode); // Enables or disables "near mode" for camera devices supporting it
	
	/* Control methods for the color camera: */
	unsigned int getExposure(void); // Returns the color camera's exposure value
	void setExposure(unsigned int newExposure); // Sets the color camera's exposure value
	unsigned int getSharpening(void); // Returns the color camera's sharpening value
	void setSharpening(unsigned int newSharpening); // Sets the color camera's sharpening value
	};

}

#endif
