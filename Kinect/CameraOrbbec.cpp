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

#include <Kinect/CameraOrbbec.h>

#include <Misc/StdError.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ArrayValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Threads/FunctionCalls.h>
#include <Math/Math.h>
#include <Math/MathValueCoders.h>
#include <Video/VideoDataFormat.h>
#include <Video/FrameBuffer.h>
#include <Video/ImageExtractor.h>
#include <libobsensor/ObSensor.hpp>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Internal/OrbbecSDKContext.h>

namespace Kinect {

namespace {

static const char* obPixelFormats[OB_FORMAT_Y12C4+1-OB_FORMAT_YUYV]= // List of video stream pixel formats defined by the Orbbec SDK, to automatically create color frame converters
	{
	"YUYV","YUY2","UYVY","NV12","NV21","MJPG","H264","H265","Y16","Y8","Y10","Y11","Y12","GRAY","HEVC","I420",
	"ACCL","GYRO","INVD","PNT ","RGBP","RLE","RGB8","BGR8","Y14","BGRA","COMP","RVL","Z16","YV12","BA81",
	"RGBA","BYR2","RW16","Y12C"
	};

}

/*****************************
Methods of class CameraOrbbec:
*****************************/

void CameraOrbbec::acquireSensors(void)
	{
	/* Retrieve the list of sensors on the selected device: */
	std::shared_ptr<ob::SensorList> sensorList=device->getSensorList();
	
	/* Find the device's color sensor: */
	for(unsigned int sensorIndex=0;sensorIndex<sensorList->count();++sensorIndex)
		{
		/* Get the i-th sensor: */
		SensorPtr sensor=sensorList->getSensor(sensorIndex);
		if(sensor->type()==OB_SENSOR_COLOR)
			{
			colorSensor=sensor;
			break;
			}
		}
	if(colorSensor==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Selected device does not have color sensor");
	
	/* Find the stream profile best matching the requested color stream format: */
	{
	double requestedSize=double(colorStreamFormat.frameSize.volume());
	double requestedFrameRate=double(colorStreamFormat.frameRate);
	double bestSizeRatio=Math::Constants<double>::max;
	double bestFrameRateRatio=Math::Constants<double>::max;
	bool bestJpeg=false;
	const std::shared_ptr<ob::StreamProfileList> cspList=colorSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<cspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a color video stream profile: */
			VideoStreamProfilePtr vsp=cspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			if(vsp->type()==OB_STREAM_COLOR)
				{
				/* Calculate the ratios between the stream profile's frame size and frame rate and the requested frame size and frame rate, respectively: */
				double size=double(vsp->width())*double(vsp->height());
				double sizeRatio=size>=requestedSize?size/requestedSize:requestedSize/size;
				double frameRateRatio=double(vsp->fps())>=requestedFrameRate?double(vsp->fps())/requestedFrameRate:requestedFrameRate/double(vsp->fps());
				if(sizeRatio<bestSizeRatio)
					{
					/* Take the stream profile: */
					colorProfile=vsp;
					bestSizeRatio=sizeRatio;
					bestFrameRateRatio=frameRateRatio;
					bestJpeg=vsp->format()==OB_FORMAT_MJPG;
					}
				else if(sizeRatio==bestSizeRatio)
					{
					if(frameRateRatio<bestFrameRateRatio)
						{
						/* Take the stream profile: */
						colorProfile=vsp;
						bestFrameRateRatio=frameRateRatio;
						bestJpeg=vsp->format()==OB_FORMAT_MJPG;
						}
					else if(frameRateRatio==bestFrameRateRatio&&!bestJpeg&&vsp->format()==OB_FORMAT_MJPG)
						{
						/* Take the stream profile: */
						colorProfile=vsp;
						bestJpeg=true;
						}
					}
				}
			}
		catch(const std::runtime_error&)
			{
			/* Ignore the error and carry on... */
			}
		}
	}
	if(colorProfile==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No color stream profile matching %ux%u@%fHz found",colorStreamFormat.frameSize[0],colorStreamFormat.frameSize[1],double(colorStreamFormat.frameRate));
	
	/* Update the requested color stream format: */
	colorStreamFormat.frameSize=Size(colorProfile->width(),colorProfile->height());
	colorStreamFormat.frameRate=Rational(colorProfile->fps(),1);
	
	/* Set the requested color space: */
	colorSpace=colorStreamFormat.colorSpace;
	
	/* Find the device's depth sensor: */
	for(unsigned int sensorIndex=0;sensorIndex<sensorList->count();++sensorIndex)
		{
		/* Get the i-th sensor: */
		SensorPtr sensor=sensorList->getSensor(sensorIndex);
		if(sensor->type()==OB_SENSOR_DEPTH)
			{
			depthSensor=sensor;
			break;
			}
		}
	if(depthSensor==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Selected device does not have depth sensor");
	
	/* Find the stream profile best matching the requested depth stream format: */
	{
	double requestedSize=double(depthStreamFormat.frameSize.volume());
	double requestedFrameRate=double(depthStreamFormat.frameRate);
	double bestSizeRatio=Math::Constants<double>::max;
	double bestFrameRateRatio=Math::Constants<double>::max;
	const std::shared_ptr<ob::StreamProfileList> dspList=depthSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<dspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a video stream profile: */
			VideoStreamProfilePtr vsp=dspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			if(vsp->type()==OB_STREAM_DEPTH)
				{
				/* Calculate the ratios between the stream profile's frame size and frame rate and the requested frame size and frame rate, respectively: */
				double size=double(vsp->width())*double(vsp->height());
				double sizeRatio=size>=requestedSize?size/requestedSize:requestedSize/size;
				double frameRateRatio=double(vsp->fps())>=requestedFrameRate?double(vsp->fps())/requestedFrameRate:requestedFrameRate/double(vsp->fps());
				if(sizeRatio<bestSizeRatio)
					{
					/* Take the stream profile: */
					depthProfile=vsp;
					bestSizeRatio=sizeRatio;
					bestFrameRateRatio=frameRateRatio;
					}
				else if(sizeRatio==bestSizeRatio&&frameRateRatio<bestFrameRateRatio)
					{
					/* Take the stream profile: */
					depthProfile=vsp;
					bestFrameRateRatio=frameRateRatio;
					}
				}
			}
		catch(const std::runtime_error&)
			{
			/* Ignore the error and carry on... */
			}
		}
	}
	if(depthProfile==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No depth stream profile matching %ux%u@%fHz found",depthStreamFormat.frameSize[0],depthStreamFormat.frameSize[1],double(depthStreamFormat.frameRate));
	
	/* Update the requested depth stream format: */
	depthStreamFormat.frameSize=Size(depthProfile->width(),depthProfile->height());
	depthStreamFormat.frameRate=Rational(depthProfile->fps(),1);
	
	/* Update the raw depth value quantization coefficients: */
	float dMax=float(depthStreamFormat.depthRange.getMax());
	zQuant[0]=dMax*zRange.getMax()*zRange.getMin()/zRange.getSize();
	zQuant[1]=dMax+dMax*zRange.getMin()/zRange.getSize();
	
	/* Mark the sensors as acquired: */
	sensorsAcquired=true;
	}

FrameSource::IntrinsicParameters::LensDistortion CameraOrbbec::getLensDistortion(ob::VideoStreamProfile& profile,bool flipX,bool flipY)
	{
	/* Retrieve the profile's lens distortion correction parameters: */
	OBCameraDistortion distortion=profile.getDistortion();
	
	/* Return the parameters as a LensDistortion object: */
	IntrinsicParameters::LensDistortion result;
	result.setKappa(0,distortion.k1);
	result.setKappa(1,distortion.k2);
	result.setKappa(2,distortion.k3);
	result.setKappa(3,distortion.k4);
	result.setKappa(4,distortion.k5);
	result.setKappa(5,distortion.k6);
	result.setRho(0,flipY?-distortion.p1:distortion.p1);
	result.setRho(1,flipX?-distortion.p2:distortion.p2);
	
	return result;
	}

void CameraOrbbec::colorFrameCallback(std::shared_ptr<ob::Frame> frame)
	{
	/* Sample the timer: */
	Time now;
	
	/*********************************************************************
	This is where we would synchronize clocks to account for random OS
	delays, subtract expected hardware latency, etc. pp.
	*********************************************************************/
	
	/* Allocate a frame buffer and extract an RGB image from the color frame: */
	FrameBuffer colorFrame(colorStreamFormat.frameSize,colorStreamFormat.frameSize.volume()*sizeof(FrameSource::ColorPixel));
	colorFrame.timeStamp=double(now-timeBase);
	Video::FrameBuffer frameBuffer;
	frameBuffer.start=static_cast<unsigned char*>(frame->data());
	frameBuffer.used=frameBuffer.size=frame->dataSize();
	if(colorSpace==YPCBCR)
		colorFrameExtractor->extractYpCbCr(&frameBuffer,colorFrame.getData<FrameSource::ColorPixel>());
	else
		colorFrameExtractor->extractRGB(&frameBuffer,colorFrame.getData<FrameSource::ColorPixel>());
	
	/* Call the color streaming callback with the extracted frame: */
	(*colorStreamingCallback)(colorFrame);
	}

void CameraOrbbec::depthFrameCallback(std::shared_ptr<ob::Frame> frame)
	{
	/* Sample the timer: */
	Time now;
	
	/*********************************************************************
	This is where we would synchronize clocks to account for random OS
	delays, subtract expected hardware latency, etc. pp.
	*********************************************************************/
	
	/* Calculate depth quantization coefficients based on the selected Z value range in cm and the frame's raw depth value scale: */
	DepthFramePtr dFrame=frame->as<ob::DepthFrame>();
	float depthScale=dFrame->getValueScale(); // Scale factor from raw integer depth values to Z values in mm
	float b=float(depthStreamFormat.depthRange.getMax())*zRange.getMax()/(zRange.getMax()-zRange.getMin());
	float a=b*zRange.getMin()*10.0f/depthScale;
	
	/* Calculate the valid range of raw depth values: */
	ObDepthPixel min(Math::ceil(zRange.getMin()*10.0f/depthScale));
	ObDepthPixel max(Math::floor(zRange.getMax()*10.0f/depthScale));
	
	/* Allocate a frame buffer and quantize and flip the depth frame: */
	FrameBuffer depthFrame(depthStreamFormat.frameSize,depthStreamFormat.frameSize.volume()*sizeof(FrameSource::DepthPixel));
	depthFrame.timeStamp=double(now-timeBase);
	const ObDepthPixel* sRowPtr=static_cast<const ObDepthPixel*>(dFrame->data())+(depthStreamFormat.frameSize[1]-1)*depthStreamFormat.frameSize[0];
	FrameSource::DepthPixel* dPtr=depthFrame.getData<FrameSource::DepthPixel>();
	for(unsigned int y=0;y<depthStreamFormat.frameSize[1];++y,sRowPtr-=depthStreamFormat.frameSize[0])
		{
		const ObDepthPixel* sPtr=sRowPtr;
		for(unsigned int x=0;x<depthStreamFormat.frameSize[0];++x,++sPtr,++dPtr)
			*dPtr=*sPtr>=min&&*sPtr<=max?FrameSource::DepthPixel(b-a/float(*sPtr)+0.5f):FrameSource::invalidDepth;
		}
	
	/* Handle background capture and removal: */
	processDepthFrameBackground(depthFrame);
	
	/* Call the depth streaming callback with the quantized frame: */
	(*depthStreamingCallback)(depthFrame);
	}

void CameraOrbbec::initialize(void)
	{
	/* Set the default color and depth streaming formats: */
	colorStreamFormat.frameSize=Size(1920,1080);
	colorStreamFormat.frameRate=Rational(30);
	colorStreamFormat.colorSpace=YPCBCR;
	
	depthStreamFormat.frameSize=Size(640,576);
	depthStreamFormat.frameRate=Rational(30);
	depthStreamFormat.depthRange=DepthRange(0,FrameSource::invalidDepth-1);
	
	zRange=ZRange(50.0f,386.0f); // Values from Orbbec Femto Bolt datasheet
	}

size_t CameraOrbbec::getNumDevices(void)
	{
	/* Acquire an Orbbec SDK context: */
	OrbbecSDKContextPtr context(OrbbecSDKContext::acquireContext());
	
	/* Retrieve the list of Orbbec devices and return the number of devices: */
	return context->queryDeviceList()->deviceCount();
	}

CameraOrbbec::CameraOrbbec(size_t index)
	:context(OrbbecSDKContext::acquireContext()),
	 sensorsAcquired(false),
	 colorFrameExtractor(0)
	{
	/* Request the list of all connected Orbbec cameras: */
	std::shared_ptr<ob::DeviceList> devList=context->queryDeviceList();
	if(index>=devList->deviceCount())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Fewer than %u Orbbec devices connected to host",(unsigned int)(index+1));
	
	/* Acquire the requested device: */
	device=devList->getDevice(index);
	
	/* Initialize the requested device: */
	initialize();
	}

CameraOrbbec::CameraOrbbec(const char* serialNumber)
	:context(OrbbecSDKContext::acquireContext()),
	 sensorsAcquired(false),
	 colorFrameExtractor(0)
	{
	/* Request the list of all connected Orbbec cameras: */
	std::shared_ptr<ob::DeviceList> devList=context->queryDeviceList();
	
	/* Acquire the camera with the requested serial number: */
	device=devList->getDeviceBySN(serialNumber);
	
	/* Initialize the requested device: */
	initialize();
	}

CameraOrbbec::~CameraOrbbec(void)
	{
	/* Stop streaming, just in case: */
	stopStreaming();
	
	/* Release the sensors if they have been acquired: */
	if(sensorsAcquired)
		{
		depthProfile=0;
		depthSensor=0;
		colorProfile=0;
		colorSensor=0;
		}
	
	/* Release the acquired device: */
	device=0;
	}

FrameSource::ColorStreamFormat CameraOrbbec::getColorStreamFormat(void) const
	{
	/* Return the current stream format: */
	return colorStreamFormat;
	}

FrameSource::DepthStreamFormat CameraOrbbec::getDepthStreamFormat(void) const
	{
	/* Return the current stream format: */
	return depthStreamFormat;
	}

FrameSource::DepthCorrection* CameraOrbbec::getDepthCorrectionParameters(void)
	{
	/* Don't have 'em, don't need 'em: */
	return 0;
	}

FrameSource::IntrinsicParameters CameraOrbbec::getIntrinsicParameters(void)
	{
	IntrinsicParameters result;
	
	/* Acquire the camera's sensors to query intrinsic parameters: */
	if(!sensorsAcquired)
		acquireSensors();
	
	/* Retrieve the depth sensor's lens distortion correction coefficients: */
	result.depthLensDistortion=getLensDistortion(*depthProfile,false,true);
	
	/* Create the transformation from depth image space to tangent space: */
	OBCameraIntrinsic depthIntrinsics=depthProfile->getIntrinsic();
	IntrinsicParameters::ATransform::Matrix& di2tMat=result.di2t.getMatrix();
	di2tMat(0,0)=1.0/depthIntrinsics.fx;
	di2tMat(0,1)=0.0;
	di2tMat(0,2)=-(depthIntrinsics.cx+0.5)/depthIntrinsics.fx; // Add 0.5 because Orbbec SDK assumes pixels at integer positions
	di2tMat(1,0)=0.0;
	di2tMat(1,1)=1.0/depthIntrinsics.fy;
	di2tMat(1,2)=-(double(depthStreamFormat.frameSize[1])-(depthIntrinsics.cy+0.5))/depthIntrinsics.fy; // Invert because we flip the depth frame, and add 0.5 because see above
	
	/* Calculate the inverse: */
	result.dt2i=Geometry::invert(result.di2t);
	
	/* Create the projection from depth image space into 3D camera space: */
	IntrinsicParameters::PTransform::Matrix& dMat=result.depthProjection.getMatrix();
	dMat=IntrinsicParameters::PTransform::Matrix::zero;
	dMat(0,0)=di2tMat(0,0);
	dMat(0,3)=di2tMat(0,2);
	dMat(1,1)=di2tMat(1,1);
	dMat(1,3)=di2tMat(1,2);
	dMat(2,3)=-1.0;
	double b=double(depthStreamFormat.depthRange.getMax())*double(zRange.getMax())/(double(zRange.getMax())-double(zRange.getMin()));
	double a=b*double(zRange.getMin());
	dMat(3,2)=-1.0/a;
	dMat(3,3)=b/a;
	
	/* Retrieve the color sensor's lens distortion correction coefficients: */
	result.colorLensDistortion=getLensDistortion(*colorProfile,true,true);
	
	/* Create the transformation from tangent space to color image space: */
	OBCameraIntrinsic colorIntrinsics=colorProfile->getIntrinsic();
	IntrinsicParameters::ATransform::Matrix& ct2iMat=result.ct2i.getMatrix();
	ct2iMat(0,0)=-colorIntrinsics.fx/double(colorStreamFormat.frameSize[0]);
	ct2iMat(0,1)=0.0;
	ct2iMat(0,2)=1.0-(colorIntrinsics.cx+0.5)/double(colorStreamFormat.frameSize[0]); // Add 0.5 because Orbbec SDK assumes pixels at integer positions
	ct2iMat(1,0)=0.0;
	ct2iMat(1,1)=-colorIntrinsics.fy/double(colorStreamFormat.frameSize[1]);
	ct2iMat(1,2)=1.0-(colorIntrinsics.cy+0.5)/double(colorStreamFormat.frameSize[1]); // Invert because we flip the color frame, and add 0.5 because see above
	
	/* Calculate the inverse: */
	result.ci2t=Geometry::invert(result.ct2i);
	
	/* Create the projection from 3D camera space into color image space: */
	IntrinsicParameters::PTransform::Matrix& cMat=result.colorProjection.getMatrix();
	cMat=IntrinsicParameters::PTransform::Matrix::zero;
	cMat(0,0)=ct2iMat(0,0);
	cMat(0,2)=ct2iMat(0,2);
	cMat(1,1)=ct2iMat(1,1);
	cMat(1,2)=ct2iMat(1,2);
	cMat(2,3)=-1.0;
	cMat(3,2)=1.0;
	
	/* Retrieve the transformation from depth sensor space to color sensor space: */
	OBExtrinsic ext=depthProfile->getExtrinsicTo(colorProfile);
	IntrinsicParameters::PTransform depthToColor=IntrinsicParameters::PTransform::identity;
	IntrinsicParameters::PTransform::Matrix& dtcMat=depthToColor.getMatrix();
	for(int i=0;i<3;++i)
		{
		for(int j=0;j<3;++j)
			dtcMat(i,j)=ext.rot[i*3+j];
		dtcMat(i,3)=ext.trans[i]/10.0;
		}
	
	/* Empirical correction for sub-optimal calibration on my camera, OB-CL8K14100BB: */
	depthToColor*=IntrinsicParameters::PTransform::rotate(IntrinsicParameters::PTransform::Rotation::rotateZ(0.01));
	
	result.colorProjection*=depthToColor;
	
	/* Concatenate the depth un-projection matrix to transform directly from depth image space to color image space: */
	result.colorProjection*=result.depthProjection;
	
	return result;
	}

const Size& CameraOrbbec::getActualFrameSize(int sensor) const
	{
	/* Return the requested frame size for the given sensor: */
	switch(sensor)
		{
		case COLOR:
			return colorStreamFormat.frameSize;
		
		case DEPTH:
			return depthStreamFormat.frameSize;
		
		default:
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid sensor");
		}
	}

void CameraOrbbec::startStreaming(void)
	{
	/* Throw an exception if already streaming: */
	if(streaming)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Already streaming");
	
	/* Acquire the color and depth sensors if that hasn't happened yet: */
	if(!sensorsAcquired)
		acquireSensors();
	
	/* If a color streaming callback has been registered, create a color frame extractor: */
	if(colorStreamingCallback!=0)
		{
		/* Create a video data format descriptor for the color sensor's selected profile: */
		Video::VideoDataFormat videoDataFormat;
		videoDataFormat.setPixelFormat(obPixelFormats[colorProfile->format()-OB_FORMAT_YUYV]);
		videoDataFormat.size=colorStreamFormat.frameSize;
		videoDataFormat.frameInterval=colorStreamFormat.frameRate.inverse();
		
		/* Create a color frame extractor: */
		colorFrameExtractor=Video::ImageExtractor::createExtractor(videoDataFormat);
		}
	
	/* Start streaming on the sensor(s) for which callbacks were registered: */
	if(colorStreamingCallback!=0)
		colorSensor->start(colorProfile,std::bind(&CameraOrbbec::colorFrameCallback,this,std::placeholders::_1));
	if(depthStreamingCallback!=0)
		depthSensor->start(depthProfile,std::bind(&CameraOrbbec::depthFrameCallback,this,std::placeholders::_1));
	
	/* Call the base class method: */
	DirectFrameSource::startStreaming();
	}

void CameraOrbbec::stopStreaming(void)
	{
	/* Bail out if not actually streaming: */
	if(!streaming)
		return;
	
	/* Call the base class method: */
	DirectFrameSource::stopStreaming();
	
	/* Stop streaming on the sensor(s) for which callbacks were registered: */
	if(colorStreamingCallback!=0)
		colorSensor->stop();
	if(depthStreamingCallback!=0)
		depthSensor->stop();
	
	/* Delete a potential color frame extractor: */
	delete colorFrameExtractor;
	colorFrameExtractor=0;
	}

std::string CameraOrbbec::getSerialNumber(void)
	{
	/* Combine the Orbbec prefix and the device's serial number: */
	std::string result="OB-";
	result.append(device->getDeviceInfo()->serialNumber());
	
	return result;
	}

void CameraOrbbec::requestColorStreamFormat(const FrameSource::ColorStreamFormat& format)
	{
	/* Ignore the request if the sensors have already been acquired: */
	if(sensorsAcquired)
		return;
	
	/* Store the requested format: */
	colorStreamFormat=format;
	}

void CameraOrbbec::requestDepthStreamFormat(const FrameSource::DepthStreamFormat& format)
	{
	/* Ignore the request if the sensors have already been acquired: */
	if(sensorsAcquired)
		return;
	
	/* Store the requested format: */
	depthStreamFormat=format;
	}

void CameraOrbbec::requestZRange(const DirectFrameSource::ZRange& newZRange)
	{
	/* Ignore the request if the sensors have already been acquired: */
	if(sensorsAcquired)
		return;
	
	/* Store the requested Z range: */
	zRange=newZRange;
	}

void CameraOrbbec::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Throw an exception if the sensors have already been acquired, because that means the caller already queried something that depends on the depth and/or color frame sizes: */
	if(sensorsAcquired)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Sensors already acquired");
	
	/* Call the base class method: */
	DirectFrameSource::configure(configFileSection);
	
	/* Configure the streaming frame sizes and frame rate: */
	configFileSection.updateValue("./colorFrameSize",colorStreamFormat.frameSize);
	configFileSection.updateValue("./colorFrameRate",colorStreamFormat.frameRate);
	configFileSection.updateValue("./depthFrameSize",depthStreamFormat.frameSize);
	configFileSection.updateValue("./depthFrameRate",depthStreamFormat.frameRate);
	
	/* Configure the Z value range for custom quantization: */
	if(configFileSection.hasTag("./depthValueRange"))
		{
		Misc::FixedArray<float,2> depthValueRange=configFileSection.retrieveValue<Misc::FixedArray<float,2> >("./depthValueRange");
		zRange=ZRange(depthValueRange[0],depthValueRange[1]);
		}
	}

void CameraOrbbec::fixFormats(void)
	{
	/* Acquire the color and depth sensors if that hasn't happened yet: */
	if(!sensorsAcquired)
		acquireSensors();
	}

void CameraOrbbec::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Create the base class settings dialog: */
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	
	// const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	}

}
