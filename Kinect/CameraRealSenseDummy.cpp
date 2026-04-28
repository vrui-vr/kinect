/***********************************************************************
CameraRealSenseDummy - Class to dummy out support for Intel RealSense
cameras.
Copyright (c) 2017-2026 Oliver Kreylos

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

#include <Kinect/CameraRealSense.h>

#include <string>
#include <Misc/StdError.h>

namespace Kinect {

/*******************************
Dummy LibRealSenseContext class:
*******************************/

class LibRealSenseContext
	{
	friend class Misc::Autopointer<LibRealSenseContext>;
	
	/* Private methods: */
	void ref(void) // Adds a reference to the context
		{
		}
	void unref(void) // Removes a reference from the context
		{
		}
	};

/********************************
Methods of class CameraRealSense:
********************************/

void CameraRealSense::initialize(void)
	{
	/* Never called... */
	}

void CameraRealSense::setColorStreamState(bool enable)
	{
	/* Never called... */
	}

void CameraRealSense::setDepthStreamState(bool enable)
	{
	/* Never called... */
	}

void* CameraRealSense::streamingThreadMethod(void)
	{
	/* Never called... */
	return 0;
	}

void CameraRealSense::irEmitterEnabledToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Never called... */
	}

void CameraRealSense::irGainSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Never called... */
	}

void CameraRealSense::irExposureAutoToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Never called... */
	}

void CameraRealSense::irExposureSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Never called... */
	}

void CameraRealSense::qualityMenuValueChangedCallback(GLMotif::DropdownBox::ValueChangedCallbackData* cbData)
	{
	/* Never called... */
	}

size_t CameraRealSense::getNumDevices(void)
	{
	/* There are no RealSense cameras: */
	return 0;
	}

CameraRealSense::CameraRealSense(size_t index)
	{
	throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Intel RealSense cameras not supported by Kinect library");
	}

CameraRealSense::CameraRealSense(const char* serialNumber)
	{
	throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Intel RealSense cameras not supported by Kinect library");
	}

CameraRealSense::~CameraRealSense(void)
	{
	/* Never called... */
	}

FrameSource::ColorStreamFormat CameraRealSense::getColorStreamFormat(void) const
	{
	/* Never called... */
	return ColorStreamFormat();
	}

FrameSource::DepthStreamFormat CameraRealSense::getDepthStreamFormat(void) const
	{
	/* Never called... */
	return DepthStreamFormat();
	}

FrameSource::DepthCorrection* CameraRealSense::getDepthCorrectionParameters(void)
	{
	/* Never called... */
	return 0;
	}

FrameSource::IntrinsicParameters CameraRealSense::getIntrinsicParameters(void)
	{
	/* Never called... */
	return IntrinsicParameters();
	}

const Size& CameraRealSense::getActualFrameSize(int sensor) const
	{
	/* Never called... */
	return frameSizes[sensor];
	}

void CameraRealSense::startStreaming(void)
	{
	/* Never called... */
	}

void CameraRealSense::stopStreaming(void)
	{
	/* Never called... */
	}

std::string CameraRealSense::getSerialNumber(void)
	{
	/* Never called... */
	return std::string();
	}

void CameraRealSense::requestColorStreamFormat(const FrameSource::ColorStreamFormat& format)
	{
	/* Never called... */
	}

void CameraRealSense::requestDepthStreamFormat(const FrameSource::DepthStreamFormat& format)
	{
	/* Never called... */
	}

void CameraRealSense::requestZRange(const DirectFrameSource::ZRange& zRange)
	{
	/* Never called... */
	}

void CameraRealSense::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Never called... */
	}

void CameraRealSense::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Never called... */
	}

}
