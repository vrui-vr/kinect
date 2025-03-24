/***********************************************************************
FrameSource - Base class for objects that create streams of depth and
color frames.
Copyright (c) 2011-2025 Oliver Kreylos

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

#include <Kinect/FrameSource.h>

#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Kinect/FrameBuffer.h>

namespace Kinect {

/*********************************************
Methods of class FrameSource::DepthCorrection:
*********************************************/

FrameSource::DepthCorrection::DepthCorrection(unsigned int sDegree,const Size& sNumSegments)
	:degree(sDegree),numSegments(sNumSegments),
	 controlPoints(0)
	{
	/* Allocate and initialize the control point array: */
	unsigned int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(unsigned int i=0;i<numControlPoints;++i)
		{
		controlPoints[i].scale=1.0f;
		controlPoints[i].offset=0.0f;
		}
	}

FrameSource::DepthCorrection::DepthCorrection(IO::File& file)
	:controlPoints(0)
	{
	/* Read the B-spline degree and number of segments: */
	degree=file.read<Misc::UInt32>();
	for(int i=0;i<2;++i)
		numSegments[i]=file.read<Misc::UInt32>();
	
	/* Allocate and read the control point array: */
	unsigned int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(unsigned int i=0;i<numControlPoints;++i)
		{
		controlPoints[i].scale=file.read<Misc::Float32>();
		controlPoints[i].offset=file.read<Misc::Float32>();
		}
	}

FrameSource::DepthCorrection::DepthCorrection(const FrameSource::DepthCorrection& source)
	:degree(source.degree),numSegments(source.numSegments),
	 controlPoints(0)
	{
	/* Allocate and copy the control point array: */
	unsigned int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(unsigned int i=0;i<numControlPoints;++i)
		controlPoints[i]=source.controlPoints[i];
	}

FrameSource::DepthCorrection::~DepthCorrection(void)
	{
	/* Delete the control point array: */
	delete[] controlPoints;
	}

void FrameSource::DepthCorrection::write(IO::File& file) const
	{
	/* Write the B-spline degree and number of segments: */
	file.write<Misc::UInt32>(degree);
	for(int i=0;i<2;++i)
		file.write<Misc::UInt32>(numSegments[i]);
	
	/* Write the control point array: */
	unsigned int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	for(unsigned int i=0;i<numControlPoints;++i)
		{
		file.write<Misc::Float32>(controlPoints[i].scale);
		file.write<Misc::Float32>(controlPoints[i].offset);
		}
	}

namespace {

/****************
Helper functions:
****************/

/* Calculate the value of a univariate uniform non-rational B-spline: */

inline float bs(int i,int n,float x)
	{
	/* Check whether x is inside the B-spline's support [i, i+n+1): */
	if(x<float(i)||x>=float(i+n+1))
		return 0.0f;
	
	/* Calculate the B-spline using Cox-deBoor recursion: */
	float bsTemp[21]; // Maximum degree is 20
	for(int j=0;j<=n;++j)
		bsTemp[j]=x>=float(i+j)&&x<float(i+j+1)?1.0:0.0;
	
	for(int ni=1;ni<=n;++ni)
		for(int j=0;j<=n-ni;++j)
			bsTemp[j]=((x-float(i+j))*bsTemp[j]+(float(i+j+ni+1)-x)*bsTemp[j+1])/float(ni);
	
	return bsTemp[0];
	}

}

FrameSource::DepthCorrection::PixelCorrection FrameSource::DepthCorrection::getPixelCorrection(const Offset& pixel,const Size& frameSize) const
	{
	/* Convert the pixel position to B-spline space: */
	float d[2];
	for(int i=0;i<2;++i)
		d[i]=((float(pixel[i])+0.5f)*float(numSegments[i]))/float(frameSize[i]);
	
	/* Evaluate the B-spline: */
	PixelCorrection result;
	result.scale=0.0f;
	result.offset=0.0f;
	int id(degree);
	int maxi(numSegments[1]+degree);
	int maxj(numSegments[0]+degree);
	const PixelCorrection* cpPtr=controlPoints;
	for(int i=0;i<maxi;++i)
		{
		float bsi=bs(i-id,id,d[1]);
		for(int j=0;j<maxj;++j,++cpPtr)
			{
			float bsj=bs(j-id,id,d[0]);
			result.scale+=cpPtr->scale*bsi*bsj;
			result.offset+=cpPtr->offset*bsi*bsj;
			}
		}
	
	return result;
	}

namespace {

/****************
Helper functions:
****************/

/* Evaluate a bivariate uniform non-rational B-spline: */

inline FrameSource::DepthCorrection::PixelCorrection bspline(unsigned int degree,const Size& numSegments,const FrameSource::DepthCorrection::PixelCorrection controlPoints[],float x,float y)
	{
	/* Find the segment index containing the evaluation point: */
	unsigned int i0=Math::floor(y);
	unsigned int j0=Math::floor(x);
	
	/* Run deBoor's algorithm to evaluate the B-spline: */
	FrameSource::DepthCorrection::PixelCorrection bsTemp[16][16]; // Maximum degree is 15
	for(unsigned int i=0;i<=degree;++i)
		for(unsigned int j=0;j<=degree;++j)
			bsTemp[i][j]=controlPoints[(i0+i)*(numSegments[0]+degree)+(j0+j)];
	for(unsigned int ni=0;ni<degree;++ni)
		{
		unsigned int sd=degree-ni;
		for(unsigned int j=0;j<sd;++j)
			{
			float w0=(float(j0+j+1)-x)/float(sd);
			float w1=1.0f-w0;
			for(unsigned int i=0;i<=sd;++i)
				{
				bsTemp[i][j].scale=w1*bsTemp[i][j+1].scale+w0*bsTemp[i][j].scale;
				bsTemp[i][j].offset=w1*bsTemp[i][j+1].offset+w0*bsTemp[i][j].offset;
				}
			}
		
		for(unsigned int i=0;i<sd;++i)
			{
			float w0=(float(i0+i+1)-y)/float(sd);
			float w1=1.0f-w0;
			for(unsigned int j=0;j<=sd;++j)
				{
				bsTemp[i][j].scale=w1*bsTemp[i+1][j].scale+w0*bsTemp[i][j].scale;
				bsTemp[i][j].offset=w1*bsTemp[i+1][j].offset+w0*bsTemp[i][j].offset;
				}
			}
		}
	
	return bsTemp[0][0];
	}

}

FrameSource::DepthCorrection::PixelCorrection* FrameSource::DepthCorrection::getPixelCorrection(const Size& frameSize) const
	{
	/* Allocate the result array: */
	PixelCorrection* result=new PixelCorrection[frameSize.volume()];
	
	PixelCorrection* rPtr=result;
	for(unsigned int y=0;y<frameSize[1];++y)
		{
		float dy=((float(y)+0.5f)*float(numSegments[1]))/float(frameSize[1]);
		for(unsigned x=0;x<frameSize[0];++x,++rPtr)
			{
			float dx=((float(x)+0.5f)*float(numSegments[0]))/float(frameSize[0]);
			*rPtr=bspline(degree,numSegments,controlPoints,dx,dy);
			}
		}
	
	return result;
	}

/*************************************************
Methods of class FrameSource::IntrinsicParameters:
*************************************************/

FrameSource::IntrinsicParameters::LensDistortion FrameSource::IntrinsicParameters::readLensDistortion(IO::File& file,bool newFormat)
	{
	/* Read into a parameter vector to avoid partial initializations: */
	IntrinsicParameters::LensDistortion::ParameterVector pv;
	
	/* Read the distortion center point: */
	for(int i=0;i<2;++i)
		pv[i]=IntrinsicParameters::LensDistortion::Scalar(file.read<Misc::Float64>());
	
	/* Read the first three radial distortion coefficients: */
	for(int i=0;i<3;++i)
		pv[2+i]=IntrinsicParameters::LensDistortion::Scalar(file.read<Misc::Float64>());
	
	if(newFormat)
		{
		/* Read the remaining three radial distortion coefficients: */
		for(int i=3;i<6;++i)
			pv[2+i]=IntrinsicParameters::LensDistortion::Scalar(file.read<Misc::Float64>());
		}
	else
		{
		/* Reset the remaining three radial distortion coefficients: */
		for(int i=3;i<6;++i)
			pv[2+i]=IntrinsicParameters::LensDistortion::Scalar(0);
		}
	
	/* Read the tangential distortion coefficients: */
	for(int i=0;i<2;++i)
		pv[2+6+i]=IntrinsicParameters::LensDistortion::Scalar(file.read<Misc::Float64>());
	
	IntrinsicParameters::LensDistortion result;
	result.setParameterVector(pv);
	return result;
	}

void FrameSource::IntrinsicParameters::writeLensDistortion(const FrameSource::IntrinsicParameters::LensDistortion& ld,IO::File& file)
	{
	/* Retrieve the distortion correction formula's parameter vector: */
	IntrinsicParameters::LensDistortion::ParameterVector pv=ld.getParameterVector();
	
	/* Write all parameters to the file: */
	for(int i=0;i<2+6+2;++i)
		file.write(Misc::Float64(pv[i]));
	}

FrameSource::IntrinsicParameters::Point2 FrameSource::IntrinsicParameters::undistortDepthPixel(const FrameSource::IntrinsicParameters::Point2& distortedPixel) const
	{
	/* Transform the depth-image point to depth tangent space: */
	LensDistortion::Point dtp;
	dtp[1]=(distortedPixel[1]-cy)/fy;
	dtp[0]=(distortedPixel[0]-sk*dtp[1]-cx)/fx;
	
	/* Calculate the undistorted point in depth tangent space: */
	LensDistortion::Point utp=depthLensDistortion.undistort(dtp);
	
	/* Return the undistorted point transformed back to depth-image space: */
	return LensDistortion::Point(utp[0]*fx+utp[1]*sk+cx,utp[1]*fy+cy);
	}

/****************************
Methods of class FrameSource:
****************************/

FrameSource::FrameSource(void)
	:colorSpace(RGB)
	{
	}

FrameSource::~FrameSource(void)
	{
	}

void FrameSource::setTimeBase(const FrameSource::Time& newTimeBase)
	{
	/* Just override the old time base: */
	timeBase=newTimeBase;
	}

FrameSource::DepthCorrection* FrameSource::getDepthCorrectionParameters(void)
	{
	/* Create and return a dummy depth correction object: */
	return new DepthCorrection(0,Size(1,1));
	}

FrameSource::DepthRange FrameSource::getDepthRange(void) const
	{
	/* Return the full range of theoretically valid depth values: */
	return DepthRange(0,invalidDepth-1);
	}

}
