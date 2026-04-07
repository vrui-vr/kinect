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
#include <Video/VideoDataFormat.h>
#include <Video/FrameBuffer.h>
#include <Video/ImageExtractor.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Internal/OrbbecSDKContext.h>

namespace Kinect {

/*************************************
Static elements of class CameraOrbbec:
*************************************/

const char* CameraOrbbec::pixelFormats[OB_FORMAT_UNKNOWN+1-OB_FORMAT_YUYV]=
	{
	"YUYV","YUY2","UYVY","NV12","NV21","MJPG","H264","H265","Y16","Y8","Y10","Y12","GRAY","HEVC","I420",
	"ACCL","GYRO","PNT ","RGBP","RLE","RGB","BGR","Y14","BGRA","COMP","RVL","Z16","YV12","BA81",
	"RGBA","BYR2","RW16","DS16","UNKNOWN"
	};

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
	
	/* Find a matching stream profile: */
	const std::shared_ptr<ob::StreamProfileList> cspList=colorSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<cspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a video stream profile: */
			VideoStreamProfilePtr vsp=cspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			
			/* Check if the profile matches: */
			if(vsp->type()==OB_STREAM_COLOR&&vsp->width()==frameSizes[0][0]&&vsp->height()==frameSizes[0][1]&&vsp->fps()==fps)
				{
				colorProfile=vsp;
				break;
				}
			}
		catch(const std::runtime_error&)
			{
			/* Ignore the error and carry on... */
			}
		}
	if(colorProfile==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No color stream profile matching %ux%u@%uHz found",frameSizes[0][0],frameSizes[0][1],fps);
	
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
	
	/* Find a matching stream profile: */
	const std::shared_ptr<ob::StreamProfileList> dspList=depthSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<dspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a video stream profile: */
			VideoStreamProfilePtr vsp=dspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			
			/* Check if the profile matches: */
			if(vsp->type()==OB_STREAM_DEPTH&&vsp->width()==frameSizes[1][0]&&vsp->height()==frameSizes[1][1]&&vsp->fps()==fps)
				{
				depthProfile=vsp;
				break;
				}
			}
		catch(const std::runtime_error&)
			{
			/* Ignore the error and carry on... */
			}
		}
	if(depthProfile==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No depth stream profile matching %ux%u@%uHz found",frameSizes[1][0],frameSizes[1][1],fps);
	
	/* Mark the sensors as acquired: */
	sensorsAcquired=true;
	}

FrameSource::IntrinsicParameters::LensDistortion CameraOrbbec::getLensDistortion(ob::VideoStreamProfile& profile)
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
	result.setRho(0,-distortion.p1); // Negate this because we flip depth and color frames vertically upon decoding
	result.setRho(1,distortion.p2);
	
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
	FrameBuffer colorFrame(frameSizes[0],frameSizes[0].volume()*sizeof(FrameSource::ColorPixel));
	colorFrame.timeStamp=double(now-timeBase);
	Video::FrameBuffer frameBuffer;
	frameBuffer.start=static_cast<unsigned char*>(frame->data());
	frameBuffer.used=frameBuffer.size=frame->dataSize();
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
	float b=float(dMax)*zRange[1]/(zRange[1]-zRange[0]);
	float a=b*zRange[0]*10.0f/depthScale;
	
	/* Calculate the valid range of raw depth values: */
	ObDepthPixel min(Math::ceil(zRange[0]*10.0f/depthScale));
	ObDepthPixel max(Math::floor(zRange[1]*10.0f/depthScale));
	
	/* Allocate a frame buffer and quantize and flip the depth frame: */
	FrameBuffer depthFrame(frameSizes[1],frameSizes[1].volume()*sizeof(FrameSource::DepthPixel));
	depthFrame.timeStamp=double(now-timeBase);
	const ObDepthPixel* sRowPtr=static_cast<const ObDepthPixel*>(dFrame->data())+(frameSizes[1][1]-1)*frameSizes[1][0];
	FrameSource::DepthPixel* dPtr=depthFrame.getData<FrameSource::DepthPixel>();
	for(unsigned int y=0;y<frameSizes[1][1];++y,sRowPtr-=frameSizes[1][0])
		{
		const ObDepthPixel* sPtr=sRowPtr;
		for(unsigned int x=0;x<frameSizes[1][0];++x,++sPtr,++dPtr)
			*dPtr=*sPtr>=min&&*sPtr<=max?FrameSource::DepthPixel(b-a/float(*sPtr)+0.5f):FrameSource::invalidDepth;
		}
	
	/* Call the depth streaming callback with the quantized frame: */
	(*depthStreamingCallback)(depthFrame);
	}

void CameraOrbbec::initialize(void)
	{
	/* Set the default color and depth streaming formats: */
	// frameSizes[0]=Size(1920,1080);
	frameSizes[0]=Size(3840,2160);
	frameSizes[1]=Size(640,576);
	fps=30;
	
	/* Set the maximum valid depth pixel value: */
	dMax=FrameSource::invalidDepth-1;
	
	/* Set a default Z range: */
	setZRange(50.0f,386.0f); // Values from Orbbec Femto Bolt datasheet
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
	result.depthLensDistortion=getLensDistortion(*depthProfile);
	
	/* Create the transformation from depth image space to tangent space: */
	OBCameraIntrinsic depthIntrinsics=depthProfile->getIntrinsic();
	IntrinsicParameters::ATransform::Matrix& di2tMat=result.di2t.getMatrix();
	di2tMat(0,0)=1.0/depthIntrinsics.fx;
	di2tMat(0,1)=0.0;
	di2tMat(0,2)=-(depthIntrinsics.cx+0.5)/depthIntrinsics.fx; // Add 0.5 because Orbbec SDK assumes pixels at integer positions
	di2tMat(1,0)=0.0;
	di2tMat(1,1)=1.0/depthIntrinsics.fy;
	di2tMat(1,2)=-(double(frameSizes[1][1])-(depthIntrinsics.cy+0.5))/depthIntrinsics.fy; // Invert because we flip the depth frame, and add 0.5 because see above
	
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
	double b=double(dMax)*double(zRange[1])/(double(zRange[1])-double(zRange[0]));
	double a=b*double(zRange[0]);
	dMat(3,2)=-1.0/a;
	dMat(3,3)=b/a;
	
	/* Retrieve the color sensor's lens distortion correction coefficients: */
	result.colorLensDistortion=getLensDistortion(*colorProfile);
	
	/* Create the transformation from tangent space to color image space: */
	OBCameraIntrinsic colorIntrinsics=colorProfile->getIntrinsic();
	IntrinsicParameters::ATransform::Matrix& ct2iMat=result.ct2i.getMatrix();
	ct2iMat(0,0)=-colorIntrinsics.fx;
	ct2iMat(0,1)=0.0;
	ct2iMat(0,2)=colorIntrinsics.cx+0.5; // Add 0.5 because Orbbec SDK assumes pixels at integer positions
	ct2iMat(1,0)=0.0;
	ct2iMat(1,1)=-colorIntrinsics.fy;
	ct2iMat(1,2)=double(frameSizes[0][1])-(colorIntrinsics.cy+0.5); // Invert because we flip the color frame, and add 0.5 because see above
	
	/* Calculate the inverse: */
	result.ci2t=Geometry::invert(result.ct2i);
	
	/* Create the projection from 3D camera space into color image space: */
	IntrinsicParameters::PTransform::Matrix& cMat=result.colorProjection.getMatrix();
	cMat=IntrinsicParameters::PTransform::Matrix::zero;
	
	cMat(0,0)=ct2iMat(0,0)/double(frameSizes[0][0]);
	cMat(0,2)=ct2iMat(0,2)/double(frameSizes[0][0]);
	cMat(1,1)=ct2iMat(1,1)/double(frameSizes[0][1]);
	cMat(1,2)=ct2iMat(1,2)/double(frameSizes[0][1]);
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
	result.colorProjection*=depthToColor;
	
	/* Concatenate the depth un-projection matrix to transform directly from depth image space to color image space: */
	result.colorProjection*=result.depthProjection;
	
	return result;
	}

const Size& CameraOrbbec::getActualFrameSize(int sensor) const
	{
	/* Return the requested frame size for the given sensor: */
	return frameSizes[sensor];
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
		videoDataFormat.setPixelFormat(pixelFormats[colorProfile->format()-OB_FORMAT_YUYV]);
		videoDataFormat.size=frameSizes[0];
		videoDataFormat.frameInterval=Math::Rational(1,fps);
		
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

void CameraOrbbec::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Throw an exception if the sensors have already been acquired, because that means the caller already queried something that depends on the depth and/or color frame sizes: */
	if(sensorsAcquired)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Sensors already acquired");
	
	/* Call the base class method: */
	DirectFrameSource::configure(configFileSection);
	
	/* Configure the streaming frame sizes and frame rate: */
	configFileSection.updateValue("./colorFrameSize",frameSizes[0]);
	configFileSection.updateValue("./depthFrameSize",frameSizes[1]);
	configFileSection.updateValue("./frameRate",fps);
	
	/* Configure the Z value range for custom quantization: */
	if(configFileSection.hasTag("./depthValueRange"))
		{
		Misc::FixedArray<float,2> depthValueRange=configFileSection.retrieveValue<Misc::FixedArray<float,2> >("./depthValueRange");
		setZRange(depthValueRange[0],depthValueRange[1]);
		}
	}

void CameraOrbbec::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Create the base class settings dialog: */
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	
	// const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	}

void CameraOrbbec::setColorFrameSize(const Size& newColorFrameSize)
	{
	/* Throw an exception if the sensors have already been acquired, because that means the caller already queried something that depends on the color frame size: */
	if(sensorsAcquired)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Sensors already acquired");
	
	/* Update the requested color frame size: */
	frameSizes[0]=newColorFrameSize;
	}

void CameraOrbbec::setDepthFrameSize(const Size& newDepthFrameSize)
	{
	/* Throw an exception if the sensors have already been acquired, because that means the caller already queried something that depends on the depth frame size: */
	if(sensorsAcquired)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Sensors already acquired");
	
	/* Update the requested depth frame size: */
	frameSizes[1]=newDepthFrameSize;
	}

void CameraOrbbec::setFps(unsigned int newFps)
	{
	/* Throw an exception if already streaming: */
	if(streaming)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Already streaming");
	
	/* Update the requested streaming frame rate for both the depth and color sensors: */
	fps=newFps;
	}

void CameraOrbbec::setZRange(float zMin,float zMax)
	{
	/* Check the z value range: */
	if(zMin>=zMax)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid Z value range [%f, %f]",zMin,zMax);
	
	/* Update the z value range: */
	zRange[0]=zMin;
	zRange[1]=zMax;
	
	/* Update the raw depth value quantization coefficients: */
	zQuant[0]=float(dMax)*zRange[1]*zRange[0]/(zRange[1]-zRange[0]);
	zQuant[1]=float(dMax)+float(dMax)*zRange[0]/(zRange[1]-zRange[0]);
	}

}
