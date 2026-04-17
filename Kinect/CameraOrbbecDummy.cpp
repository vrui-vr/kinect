/***********************************************************************
CameraOrbbecDummy - Class to dummy out support for Orbbec cameras.
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

#include <string>
#include <Misc/StdError.h>

namespace Kinect {

/*****************************
Methods of class CameraOrbbec:
*****************************/

void CameraOrbbec::acquireSensors(void)
	{
	/* Never called */
	}

FrameSource::IntrinsicParameters::LensDistortion CameraOrbbec::getLensDistortion(ob::VideoStreamProfile& profile,bool flipX,bool flipY)
	{
	/* Never called */
	return FrameSource::IntrinsicParameters::LensDistortion();
	}

void CameraOrbbec::colorFrameCallback(std::shared_ptr<ob::Frame> frame)
	{
	/* Never called */
	}

void CameraOrbbec::depthFrameCallback(std::shared_ptr<ob::Frame> frame)
	{
	/* Never called */
	}

void CameraOrbbec::initialize(void)
	{
	/* Never called */
	}

size_t CameraOrbbec::getNumDevices(void)
	{
	return 0;
	}

CameraOrbbec::CameraOrbbec(size_t index)
	{
	throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Orbbec cameras using Orbbec SDK not supported by Kinect library");
	}

CameraOrbbec::CameraOrbbec(const char* serialNumber)
	{
	throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Orbbec cameras using Orbbec SDK not supported by Kinect library");
	}

CameraOrbbec::~CameraOrbbec(void)
	{
	}

FrameSource::DepthCorrection* CameraOrbbec::getDepthCorrectionParameters(void)
	{
	/* Never called */
	return 0;
	}

FrameSource::IntrinsicParameters CameraOrbbec::getIntrinsicParameters(void)
	{
	/* Never called */
	return IntrinsicParameters();
	}

const Size& CameraOrbbec::getActualFrameSize(int sensor) const
	{
	/* Never called */
	return frameSizes[sensor];
	}

void CameraOrbbec::startStreaming(void)
	{
	/* Never called */
	}

void CameraOrbbec::stopStreaming(void)
	{
	/* Never called */
	}

std::string CameraOrbbec::getSerialNumber(void)
	{
	/* Never called */
	return std::string();
	}

void CameraOrbbec::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Never called */
	}

void CameraOrbbec::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Never called */
	}

void CameraOrbbec::setColorFrameSize(const Size& newColorFrameSize)
	{
	/* Never called */
	}

void CameraOrbbec::setDepthFrameSize(const Size& newDepthFrameSize)
	{
	/* Never called */
	}

void CameraOrbbec::setFps(unsigned int newFps)
	{
	/* Never called */
	}

void CameraOrbbec::setZRange(float zMin,float zMax)
	{
	/* Never called */
	}

}
