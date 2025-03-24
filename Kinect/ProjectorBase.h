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

#ifndef KINECT_PROJECTORBASE_INCLUDED
#define KINECT_PROJECTORBASE_INCLUDED

#include <Kinect/Types.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

class ProjectorBase
	{
	/* Embedded classes: */
	protected:
	typedef FrameSource::DepthCorrection::PixelCorrection PixelCorrection; // Type for per-pixel depth correction factors
	typedef FrameSource::IntrinsicParameters IntrinsicParameters; // Type for combined depth and color camera intrinsic parameters
	typedef IntrinsicParameters::PTransform PTransform; // Type for projective transformations
	typedef FrameSource::ExtrinsicParameters ExtrinsicParameters; // Type for transformations from 3D camera space to 3D world space
	public:
	typedef PTransform::Point Point; // Type for points in depth image or world space
	
	/* Elements: */
	Size depthSize; // Width and height of all incoming depth frames
	PixelCorrection* depthCorrection; // Buffer of per-pixel depth correction parameters
	IntrinsicParameters intrinsicParameters; // Intrinsic parameters for the color and depth cameras
	ExtrinsicParameters extrinsicParameters; // Transformation from 3D camera space into 3D world space
	PTransform worldDepthProjection; // Projection transformation from depth image space into 3D world space
	FrameSource::ColorSpace colorSpace; // Color space of frame source's color stream
	FrameSource::DepthPixel triangleDepthRange; // Maximum depth distance between a triangle's vertices
	
	/* Constructors and destructors: */
	public:
	ProjectorBase(void); // Creates a base facade projector with uninitialized camera parameters
	ProjectorBase(FrameSource& frameSource); // Creates a base facade projector for the given frame source
	~ProjectorBase(void);
	
	/* Methods: */
	const Size& getDepthFrameSize(void) const // Returns the current depth frame size
		{
		return depthSize;
		}
	unsigned int getDepthFrameSize(int index) const // Ditto
		{
		return depthSize[index];
		}
	const PixelCorrection* getDepthCorrection(void) const // Returns the array of per-pixel depth correction factors
		{
		return depthCorrection;
		}
	const IntrinsicParameters& getIntrinsicParameters(void) const // Returns the intrinsic parameters for the color and depth cameras
		{
		return intrinsicParameters;
		}
	const ExtrinsicParameters& getExtrinsicParameters(void) const // Returns the transformation from 3D camera space into 3D world space
		{
		return extrinsicParameters;
		}
	FrameSource::ColorSpace getColorSpace(void) const // Returns the color stream's color space
		{
		return colorSpace;
		}
	FrameSource::DepthPixel getTriangleDepthRange(void) const // Returns the maximum depth range for generated triangles
		{
		return triangleDepthRange;
		}
	void setDepthFrameSize(const Size& newDepthFrameSize); // Sets the size of all future incoming depth frames
	void setDepthCorrection(const FrameSource::DepthCorrection* dc); // Enables per-pixel depth correction using the given depth correction parameters
	void setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips); // Sets the projector's intrinsic camera parameters
	void setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps); // Sets the projector's extrinsic camera parameters
	void setColorSpace(const FrameSource::ColorSpace newColorSpace); // Sets the color stream's color space
	void setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange); // Sets the maximum depth range for valid triangles
	Point projectPoint(const Point& p) const; // Projects a point from world space into depth image space
	};

}

#endif
