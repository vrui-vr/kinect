/***********************************************************************
CameraOrbbecDummy - Class to dummy out support for Orbbec cameras.
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

#include <string>
#include <stdexcept>
#include <Misc/FunctionCalls.h>

namespace Kinect {

/*****************************
Methods of class CameraOrbbec:
*****************************/

void CameraOrbbec::initialize(void)
	{
	/* Never called */
	}

size_t CameraOrbbec::getNumDevices(void)
	{
	return 0;
	}

CameraOrbbec::CameraOrbbec(libusb_device* sDevice)
	{
	throw std::runtime_error("Kinect::CameraOrbbec: Orbbec cameras using Orbbec SDK not supported by Kinect library");
	}

CameraOrbbec::CameraOrbbec(size_t index)
	{
	throw std::runtime_error("Kinect::CameraOrbbec: Orbbec cameras using Orbbec SDK not supported by Kinect library");
	}

CameraOrbbec::CameraOrbbec(const char* serialNumber)
	{
	throw std::runtime_error("Kinect::CameraOrbbec: Orbbec cameras using Orbbec SDK not supported by Kinect library");
	}

CameraOrbbec::~CameraOrbbec(void)
	{
	}

FrameSource::DepthCorrection* CameraOrbbec::getDepthCorrectionParameters(void)
	{
	return 0;
	}

FrameSource::IntrinsicParameters CameraOrbbec::getIntrinsicParameters(void)
	{
	return IntrinsicParameters();
	}

const Size& CameraOrbbec::getActualFrameSize(int sensor) const
	{
	/* Return the appropriate frame size: */
	return frameSizes[sensor];
	}

void CameraOrbbec::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	delete newColorStreamingCallback;
	delete newDepthStreamingCallback;
	}

void CameraOrbbec::stopStreaming(void)
	{
	}

std::string CameraOrbbec::getSerialNumber(void)
	{
	return std::string();
	}

}
