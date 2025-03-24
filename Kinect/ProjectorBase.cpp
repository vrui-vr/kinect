/***********************************************************************
ProjectorBase - Base class for different methods to project a depth
frame captured from a Kinect camera back into calibrated 3D camera
space, and texture-map it with a matching color frame.
Copyright (c) 2010-2025 Oliver Kreylos

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

#include <Kinect/ProjectorBase.h>

#include <Math/Math.h>

namespace Kinect {

/******************************
Methods of class ProjectorBase:
******************************/

ProjectorBase::ProjectorBase(void)
	:depthSize(0,0),
	 depthCorrection(0),colorSpace(FrameSource::RGB),
	 triangleDepthRange(5)
	{
	}

ProjectorBase::ProjectorBase(FrameSource& frameSource)
	:depthSize(frameSource.getActualFrameSize(FrameSource::DEPTH)),
	 depthCorrection(0),colorSpace(frameSource.getColorSpace()),
	 triangleDepthRange(5)
	{
	/* Query the source's depth correction parameters and calculate the depth correction buffer: */
	FrameSource::DepthCorrection* dc=frameSource.getDepthCorrectionParameters();
	setDepthCorrection(dc);
	delete dc;
	
	/* Query the source's intrinsic and extrinsic parameters: */
	intrinsicParameters=frameSource.getIntrinsicParameters();
	extrinsicParameters=frameSource.getExtrinsicParameters();
	worldDepthProjection=extrinsicParameters;
	worldDepthProjection*=intrinsicParameters.depthProjection;
	}

ProjectorBase::~ProjectorBase(void)
	{
	/* Release the depth correction buffer: */
	delete[] depthCorrection;
	}

void ProjectorBase::setDepthFrameSize(const Size& newDepthFrameSize)
	{
	/* Copy the depth frame size: */
	depthSize=newDepthFrameSize;
	}

void ProjectorBase::setDepthCorrection(const FrameSource::DepthCorrection* dc)
	{
	/* Delete the current per-pixel depth correction buffer: */
	delete depthCorrection;
	depthCorrection=0;
	
	if(dc!=0)
		{
		/* Evaluate the depth correction parameters to create a per-pixel depth correction buffer: */
		depthCorrection=dc->getPixelCorrection(depthSize);
		}
	}

void ProjectorBase::setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips)
	{
	/* Replace the stored intrinsic parameters: */
	intrinsicParameters=ips;
	
	/* Calculate the combined world-space depth projection matrix: */
	worldDepthProjection=extrinsicParameters;
	worldDepthProjection*=intrinsicParameters.depthProjection;
	}

void ProjectorBase::setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps)
	{
	/* Replace the stored extrinsic parameters: */
	extrinsicParameters=eps;
	
	/* Calculate the combined world-space depth projection matrix: */
	worldDepthProjection=extrinsicParameters;
	worldDepthProjection*=intrinsicParameters.depthProjection;
	}

void ProjectorBase::setColorSpace(const FrameSource::ColorSpace newColorSpace)
	{
	/* Replace the stored color space: */
	colorSpace=newColorSpace;
	}

void ProjectorBase::setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange)
	{
	/* Set the triangle depth range immediately; it won't kill the depth frame processing thread if changed in mid-process: */
	triangleDepthRange=newTriangleDepthRange;
	}

ProjectorBase::Point ProjectorBase::projectPoint(const ProjectorBase::Point& p) const
	{
	/* Transform the point from world space to depth image space: */
	Point dip=worldDepthProjection.inverseTransform(p);
	
	/* Apply inverse lens distortion correction if necessary: */
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Calculate the undistorted pixel position in pixel space: */
		typedef IntrinsicParameters::Scalar Scalar;
		typedef IntrinsicParameters::Point2 Point2;
		IntrinsicParameters::Point2 up=intrinsicParameters.undistortDepthPixel(Point2(Scalar(dip[0]),Scalar(dip[1])));
		dip[0]=up[0];
		dip[1]=up[1];
		}
	
	if(depthCorrection!=0)
		{
		/* Apply per-pixel depth correction to the depth-image point: */
		int dipx=int(Math::floor(dip[0]));
		int dipy=int(Math::floor(dip[1]));
		if(dipx>=0&&(unsigned int)dipx<depthSize[0]&&dipy>=0&&(unsigned int)dipy<depthSize[1])
			{
			const PixelCorrection* dcPtr=depthCorrection+(dipy*depthSize[0]+dipx);
			dip[2]=(dip[2]-dcPtr->offset)/dcPtr->scale;
			}
		}
	
	return dip;
	}

}
