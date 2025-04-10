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

#include <Kinect/CameraOrbbec.h>

#include <Misc/StdError.h>
#include <Kinect/Internal/OrbbecSDKContext.h>

namespace Kinect {

/*****************************
Methods of class CameraOrbbec:
*****************************/

void CameraOrbbec::acquireSensors(void)
	{
	/* Retrieve the list of sensors on the selected device: */
	std::shared_ptr<ob::SensorList> sensorList=device->getSensorList();
	
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
	
	/* Find a matching stream profiles: */
	const std::shared_ptr<ob::StreamProfileList> dspList=depthSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<dspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a video stream profile: */
			VideoStreamProfilePtr vsp=dspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			
			/* Check if the profile matches: */
			if(vsp->type()==OB_STREAM_DEPTH&&vsp->width()==depthSize[0]&&vsp->height()==depthSize[1]&&vsp->fps()==fps)
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
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No depth stream profile matching %ux%u@%uHz found",depthSize[0],depthSize[1],fps);
	
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
	
	/* Find a matching stream profiles: */
	const std::shared_ptr<ob::StreamProfileList> cspList=colorSensor->getStreamProfileList();
	for(unsigned int streamProfileIndex=0;streamProfileIndex<cspList->count();++streamProfileIndex)
		{
		try
			{
			/* Get the i-th stream profile and check whether it's a video stream profile: */
			VideoStreamProfilePtr vsp=cspList->getProfile(streamProfileIndex)->as<ob::VideoStreamProfile>();
			
			/* Check if the profile matches: */
			if(vsp->type()==OB_STREAM_COLOR&&vsp->width()==colorSize[0]&&vsp->height()==colorSize[1]&&vsp->fps()==fps)
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
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No color stream profile matching %ux%u@%uHz found",colorSize[0],colorSize[1],fps);
	
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
	result.setRho(0,distortion.p1);
	result.setRho(1,distortion.p2);
	
	return result;
	}

CameraOrbbec::CameraOrbbec(size_t index)
	:context(OrbbecSDKContext::acquireContext()),
	 depthSize(640,576),colorSize(1920,1080),fps(30),
	 sensorsAcquired(false),
	 dMax(FrameSource::invalidDepth-1)
	{
	/* Request the list of all connected Orbbec cameras: */
	std::shared_ptr<ob::DeviceList> devList=context->queryDeviceList();
	if(index>=devList->deviceCount())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Fewer than %u Orbbec devices connected to host",(unsigned int)(index+1));
	
	/* Acquire the requested device: */
	device=devList->getDevice(index);
	
	/* Initialize the depth quantization formulas: */
	setZRange(25.0f,400.0f);
	}

CameraOrbbec::CameraOrbbec(const char* serialNumber)
	:context(OrbbecSDKContext::acquireContext()),
	 depthSize(640,576),colorSize(1920,1080),fps(30),
	 sensorsAcquired(false),
	 dMax(FrameSource::invalidDepth-1)
	{
	/* Request the list of all connected Orbbec cameras: */
	std::shared_ptr<ob::DeviceList> devList=context->queryDeviceList();
	
	/* Acquire the camera with the requested serial number: */
	device=devList->getDeviceBySN(serialNumber);
	
	/* Initialize the depth quantization formulas: */
	setZRange(25.0f,400.0f);
	}

CameraOrbbec::~CameraOrbbec(void)
	{
	}

FrameSource::DepthCorrection* CameraOrbbec::getDepthCorrectionParameters(void)
	{
	/* Don't have 'em, don't need 'em: */
	return 0;
	}

FrameSource::IntrinsicParameters CameraOrbbec::getIntrinsicParameters(void)
	{
	FrameSource::IntrinsicParameters result;
	
	/* Acquire the camera's sensors to query intrinsic parameters: */
	if(!sensorsAcquired)
		acquireSensors();
	
	/* Retrieve the depth sensor's lens distortion correction coefficients: */
	result.depthLensDistortion=getLensDistortion(*depthProfile);
	
	/* Create the projection from depth image space into 3D camera space: */
	IntrinsicParameters::PTransform::Matrix& dMat=result.depthProjection.getMatrix();
	dMat=IntrinsicParameters::PTransform::Matrix::zero;
	OBCameraIntrinsic depthIntrinsics=depthProfile->getIntrinsic();
	dMat(0,0)=-1.0/depthIntrinsics.fx;
	dMat(0,3)=depthIntrinsics.cx/depthIntrinsics.fx;
	dMat(1,1)=-1.0/depthIntrinsics.fy;
	dMat(1,3)=depthIntrinsics.cy/depthIntrinsics.fy;
	dMat(2,3)=-1.0;
	dMat(3,2)=-1.0/double(dQuant[0]);
	dMat(3,3)=double(dQuant[1])/double(dQuant[0]);
	
	
	
	
	
	
	
	
	
	
	
	
	/* Retrieve the color sensor's intrinsic parameters and distortion correction coefficients: */
	OBCameraIntrinsic colorIntrinsic=colorProfile->getIntrinsic();
	colorIntrinsic[0]=-intrinsic.fx;
	colorIntrinsic[1]=float(intrinsic.width)-intrinsic.cx;
	colorIntrinsic[2]=-intrinsic.fy;
	colorIntrinsic[3]=float(intrinsic.height)-intrinsic.cy;
	
	/* Retrieve the color sensor's lens distortion correction coefficients: */
	result.colorLensDistortion=getLensDistortion(*colorProfile);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/* Query the camera's intrinsic parameters: */
	rs_error* error=0;
	rs_intrinsics colorIntrinsics,depthIntrinsics;
	if(error==0)
		rs_get_stream_intrinsics(device,RS_STREAM_COLOR,&colorIntrinsics,&error);
	if(error==0)
		rs_get_stream_intrinsics(device,RS_STREAM_DEPTH,&depthIntrinsics,&error);
	rs_extrinsics extrinsics; // Still an intrinsic parameter
	if(error==0)
		rs_get_device_extrinsics(device,RS_STREAM_DEPTH,RS_STREAM_COLOR,&extrinsics,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::makeStdErrMsg(__PRETTY_FUNCTION__,"Error %s while querying camera intrinsics",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	
	IntrinsicParameters result;
	
	typedef IntrinsicParameters::PTransform PTransform;
	
	/* Calculate the un-projection matrix from 3D depth image space into 3D camera space: */
	PTransform::Matrix& dum=result.depthProjection.getMatrix();
	dum=PTransform::Matrix::zero;
	dum(0,0)=-1.0/double(depthIntrinsics.fx);
	dum(0,3)=double(depthIntrinsics.ppx)/double(depthIntrinsics.fx);
	dum(1,1)=-1.0/double(depthIntrinsics.fy);
	dum(1,3)=(double(depthIntrinsics.height)-double(depthIntrinsics.ppy))/double(depthIntrinsics.fy);
	dum(2,3)=-1.0;
	dum(3,2)=-1.0/double(a);
	dum(3,3)=double(b)/double(a);
	
	/* Scale the depth unprojection matrix to cm: */
	result.depthProjection.leftMultiply(PTransform::scale(0.1));
	
	/* Calculate the texture projection matrix from 3D camera space into 2D color camera texture space: */
	PTransform::Matrix& cpm=result.colorProjection.getMatrix();
	cpm=PTransform::Matrix::zero;
	cpm(0,0)=double(colorIntrinsics.fx)/double(colorIntrinsics.width);
	cpm(0,2)=double(colorIntrinsics.ppx)/double(colorIntrinsics.width);
	cpm(1,1)=-double(colorIntrinsics.fy)/double(colorIntrinsics.height);
	cpm(1,2)=(double(colorIntrinsics.height)-double(colorIntrinsics.ppy))/double(colorIntrinsics.height);
	cpm(2,3)=1.0;
	cpm(3,2)=1.0;
	
	/* Calculate an affine transformation from 3D depth camera space into 3D color camera space: */
	PTransform extrinsicTransform=PTransform::identity;
	PTransform::Matrix& etm=extrinsicTransform.getMatrix();
	for(int i=0;i<3;++i)
		{
		for(int j=0;j<3;++j)
			etm(i,j)=double(extrinsics.rotation[i+j*3]);
		etm(i,3)=double(extrinsics.translation[i]);
		}
	result.colorProjection*=extrinsicTransform;
	
	/* Scale 3D depth camera space from cm to m: */
	result.colorProjection*=PTransform::scale(PTransform::Scale(-0.01,0.01,-0.01));
	
	/* Concatenate the depth un-projection matrix to transform directly from depth image space to color image space: */
	result.colorProjection*=result.depthProjection;
	
	return result;
	}

const Size& CameraRealSense::getActualFrameSize(int sensor) const
	{
	return frameSizes[sensor];
	}

void CameraRealSense::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	/* Throw an exception if already streaming: */
	if(runStreamingThread)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"RealSense device is already streaming");
	
	/* Remember the provided callback functions: */
	colorStreamingCallback=newColorStreamingCallback;
	depthStreamingCallback=newDepthStreamingCallback;
	
	try
		{
		/* Enable the requested camera streams: */
		setColorStreamState(colorStreamingCallback!=0);
		setDepthStreamState(depthStreamingCallback!=0);
		
		/* Start streaming: */
		rs_error* error=0;
		rs_start_device(device,&error);
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::makeStdErrMsg(__PRETTY_FUNCTION__,"Error %s while starting device",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		
		/* Start the background streaming thread: */
		runStreamingThread=true;
		streamingThread.start(this,&CameraRealSense::streamingThreadMethod);
		}
	catch(...)
		{
		/* Clean up: */
		delete colorStreamingCallback;
		colorStreamingCallback=0;
		delete depthStreamingCallback;
		depthStreamingCallback=0;
		
		/* Re-throw the exception: */
		throw;
		}
	}

void CameraRealSense::stopStreaming(void)
	{
	/* Bail out if not actually streaming: */
	if(!runStreamingThread)
		return;
	
	/* Stop the background streaming thread: */
	runStreamingThread=false;
	streamingThread.join();
	
	/* Delete the callback functions: */
	delete colorStreamingCallback;
	colorStreamingCallback=0;
	delete depthStreamingCallback;
	depthStreamingCallback=0;
	
	/* Stop streaming: */
	rs_error* error=0;
	rs_stop_device(device,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::makeStdErrMsg(__PRETTY_FUNCTION__,"Error %s while stopping device",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	}

std::string CameraRealSense::getSerialNumber(void)
	{
	/* Combine the RealSense prefix and the device's serial number: */
	std::string result="RS-";
	rs_error* error=0;
	const char* serialNumber=rs_get_device_serial(device,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::makeStdErrMsg(__PRETTY_FUNCTION__,"Error %s while querying device's serial number",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	result.append(serialNumber);
	
	return result;
	}

void CameraRealSense::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Call the base class method: */
	DirectFrameSource::configure(configFileSection);
	
	/* Select the color frame size and frame rate: */
	if(configFileSection.hasTag("./colorFrameRate"))
		setFrameRate(COLOR,configFileSection.retrieveValue<int>("./colorFrameRate"));
	if(configFileSection.hasTag("./colorFrameSize"))
		setFrameSize(COLOR,configFileSection.retrieveValue<Size>("./colorFrameSize"));
	
	/* Select the depth frame size and frame rate: */
	if(configFileSection.hasTag("./depthFrameRate"))
		setFrameRate(DEPTH,configFileSection.retrieveValue<int>("./depthFrameRate"));
	if(configFileSection.hasTag("./depthFrameSize"))
		setFrameSize(DEPTH,configFileSection.retrieveValue<Size>("./depthFrameSize"));
	
	/* Configure the Z value range for custom quantization: */
	if(configFileSection.hasTag("./depthValueRange"))
		{
		Misc::FixedArray<RSDepthPixel,2> depthValueRange=configFileSection.retrieveValue<Misc::FixedArray<RSDepthPixel,2> >("./depthValueRange");
		setZRange(depthValueRange[0],depthValueRange[1]);
		}
	
	/* Configure the IR emitter and cameras: */
	if(configFileSection.hasTag("./irEmitterEnabled"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_EMITTER_ENABLED,configFileSection.retrieveValue<bool>("./irEmitterEnabled")?1.0:0.0,0);
		}
	if(configFileSection.hasTag("./irGain"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_GAIN,configFileSection.retrieveValue<double>("./irGain"),0);
		}
	if(configFileSection.hasTag("./irExposureAuto"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,configFileSection.retrieveValue<bool>("./irExposureAuto")?1.0:0.0,0);
		}
	if(configFileSection.hasTag("./irExposure"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_EXPOSURE,configFileSection.retrieveValue<double>("./irExposure"),0);
		}
	if(configFileSection.hasTag("./depthControlPreset"))
		{
		setDepthStreamState(true);
		static const char* presetNames[]=
			{
			"Default",
			"Off",
			"Low",
			"Medium",
			"Optimized",
			"High",
			0
			};
		std::string depthControlPreset=configFileSection.retrieveString("./depthControlPreset");
		int i;
		for(i=0;presetNames[i]!=0;++i)
			if(strcasecmp(depthControlPreset.c_str(),presetNames[i])==0)
				{
				rs_apply_depth_control_preset(device,i);
				break;
				}
		if(presetNames[i]==0)
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid depth control preset \"%s\"",depthControlPreset.c_str());
		}
	}

void CameraRealSense::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Create the base class settings dialog: */
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	
	const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	
	double optionMin,optionMax,optionStep;
	
	/* Create widgets to turn the IR emitter on/off and set IR cameras' gain: */
	GLMotif::RowColumn* irEmitterGainBox=new GLMotif::RowColumn("IREmitterGainBox",settingsDialog,false);
	irEmitterGainBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	irEmitterGainBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	irEmitterGainBox->setNumMinorWidgets(1);
	
	GLMotif::ToggleButton* irEmitterEnabledToggle=new GLMotif::ToggleButton("IREmitterEnabledToggle",irEmitterGainBox,"IR Emitter");
	irEmitterEnabledToggle->setBorderWidth(0.0f);
	irEmitterEnabledToggle->setBorderType(GLMotif::Widget::PLAIN);
	irEmitterEnabledToggle->setToggle(rs_get_device_option(device,RS_OPTION_R200_EMITTER_ENABLED,0)!=0.0);
	irEmitterEnabledToggle->getValueChangedCallbacks().add(this,&CameraRealSense::irEmitterEnabledToggleCallback);
	
	new GLMotif::Label("IRGainLabel",irEmitterGainBox,"IR Gain");
	
	GLMotif::TextFieldSlider* irGainSlider=new GLMotif::TextFieldSlider("IRGainSlider",irEmitterGainBox,5,ss.fontHeight*5.0f);
	irGainSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	irGainSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	rs_get_device_option_range(device,RS_OPTION_R200_LR_GAIN,&optionMin,&optionMax,&optionStep,0);
	irGainSlider->setValueRange(optionMin,optionMax,optionStep);
	irGainSlider->setValue(rs_get_device_option(device,RS_OPTION_R200_LR_GAIN,0));
	irGainSlider->getValueChangedCallbacks().add(this,&CameraRealSense::irGainSliderCallback);
	
	irEmitterGainBox->manageChild();
	
	/* Create widgets to set the IR cameras' exposure: */
	GLMotif::RowColumn* irExposureBox=new GLMotif::RowColumn("IRExposureBox",settingsDialog,false);
	irExposureBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	irExposureBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	irExposureBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("IRExposureLabel",irExposureBox,"IR Exposure");
	
	GLMotif::ToggleButton* irExposureAutoToggle=new GLMotif::ToggleButton("IRExposureAutoToggle",irExposureBox,"Auto");
	irExposureAutoToggle->setBorderWidth(0.0f);
	irExposureAutoToggle->setBorderType(GLMotif::Widget::PLAIN);
	irExposureAutoToggle->setToggle(rs_get_device_option(device,RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,0)!=0.0);
	irExposureAutoToggle->getValueChangedCallbacks().add(this,&CameraRealSense::irExposureAutoToggleCallback);
	
	GLMotif::TextFieldSlider* irExposureSlider=new GLMotif::TextFieldSlider("IRExposureSlider",irExposureBox,5,ss.fontHeight*5.0f);
	irExposureSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	irExposureSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	rs_get_device_option_range(device,RS_OPTION_R200_LR_EXPOSURE,&optionMin,&optionMax,&optionStep,0);
	irExposureSlider->setValueRange(optionMin,optionMax,optionStep);
	irExposureSlider->setValue(rs_get_device_option(device,RS_OPTION_R200_LR_EXPOSURE,0));
	irExposureSlider->getValueChangedCallbacks().add(this,&CameraRealSense::irExposureSliderCallback);
	
	/* Disable the exposure slider if auto exposure is enabled: */
	if(irExposureAutoToggle->getToggle())
		irExposureSlider->setEnabled(false);
	
	irExposureBox->manageChild();
	
	/* Create a drop-down box to select 3D reconstruction quality presets: */
	GLMotif::Margin* qualityMargin=new GLMotif::Margin("QualityMargin",settingsDialog,false);
	qualityMargin->setAlignment(GLMotif::Alignment::LEFT);
	
	GLMotif::RowColumn* qualityBox=new GLMotif::RowColumn("QualityBox",qualityMargin,false);
	qualityBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	qualityBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	qualityBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("QualityLabel",qualityBox,"3D Outlier Removal");
	
	GLMotif::DropdownBox* qualityMenu=new GLMotif::DropdownBox("QualityMenu",qualityBox,false);
	qualityMenu->addItem("Default");
	qualityMenu->addItem("Off");
	qualityMenu->addItem("Low");
	qualityMenu->addItem("Medium");
	qualityMenu->addItem("Optimized");
	qualityMenu->addItem("High");
	qualityMenu->setSelectedItem(0);
	qualityMenu->getValueChangedCallbacks().add(this,&CameraRealSense::qualityMenuValueChangedCallback);
	qualityMenu->manageChild();
	
	qualityBox->manageChild();
	
	qualityMargin->manageChild();
	}

void CameraOrbbec::setDepthFrameSize(const Size& newDepthFrameSize)
	{
	/* Update the requested depth frame size: */
	depthFrameSize=newDepthFrameSize;
	}

void CameraOrbbec::setColorFrameSize(const Size& newColorFrameSize)
	{
	/* Update the requested color frame size: */
	colorFrameSize=newColorFrameSize;
	}

void CameraOrbbec::setFps(unsigned int newFps)
	{
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
