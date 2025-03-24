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

#ifndef KINECT_FRAMESOURCE_INCLUDED
#define KINECT_FRAMESOURCE_INCLUDED

#include <Misc/SizedTypes.h>
#include <Realtime/Time.h>
#include <Math/Interval.h>
#if !KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
#include <Geometry/OrthogonalTransformation.h>
#endif
#include <Geometry/AffineTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Video/LensDistortion.h>
#include <Kinect/Config.h>
#include <Kinect/Types.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace IO {
class File;
}
namespace Kinect {
class FrameBuffer;
}

namespace Kinect {

class FrameSource
	{
	/* Embedded classes: */
	public:
	enum Sensor // Enumerated type to select one of the source's streams
		{
		COLOR=0,DEPTH
		};
	
	enum ColorSpace // Color space used by the source's color stream
		{
		RGB=0, // RGB color space
		YPCBCR // Y'CbCr color space compatible with JPEG, MPEG, and Theora codecs
		};
	
	typedef Realtime::TimePointMonotonic Time; // Type for timestamp base points
	typedef Misc::UInt16 DepthPixel; // Type for raw depth pixels
	typedef Math::Interval<DepthPixel> DepthRange; // Type for ranges of depth pixel values
	typedef Misc::UInt8 ColorComponent; // Type for color pixel components
	
	struct ColorPixel // Type for color pixels
		{
		/* Embedded classes: */
		public:
		typedef ColorComponent Component; // Type for color components
		
		/* Elements: */
		public:
		Component components[3]; // RGB or Y'CbCr color components
		
		/* Methods: */
		Component operator[](int index) const // Returns one color component
			{
			return components[index];
			}
		Component& operator[](int index) // Ditto
			{
			return components[index];
			}
		};
	
	class DepthCorrection // Class defining the depth correction parameters of a depth frame source
		{
		/* Embedded classes: */
		public:
		struct PixelCorrection // Structure describing a per-pixel depth correction factor
			{
			/* Elements: */
			public:
			float scale,offset; // Scale and offset, parameters of the formula depth' = depth*scale + offset
			
			/* Methods: */
			float correct(float depth) const // Corrects the given raw depth value
				{
				return float(depth)*scale+offset;
				}
			};
		
		/* Elements: */
		private:
		unsigned int degree; // Degree of bivariate B-spline approximating the per-pixel depth correction offsets
		Size numSegments; // Number of B-spline segments horizontally and vertically
		PixelCorrection* controlPoints; // Array of control points defining the depth correction B-spline
		
		/* Constructors and destructors: */
		public:
		DepthCorrection(unsigned int sDegree,const Size& sNumSegments); // Creates a depth correction object with the given frame size, degree, and number of segments
		DepthCorrection(IO::File& file); // Reads a depth correction object from a binary file or pipe
		DepthCorrection(const DepthCorrection& source); // Copy constructor
		private:
		DepthCorrection& operator=(const DepthCorrection& source); // Prohibit assignment operator
		public:
		~DepthCorrection(void); // Destroys a depth correction object
		
		/* Methods: */
		bool isValid(void) const // Returns true if the depth correction parameters are valid
			{
			return degree>0;
			}
		void write(IO::File& file) const; // Writes a depth correction object to a binary file or pipe
		PixelCorrection getPixelCorrection(const Offset& pixel,const Size& frameSize) const; // Returns the depth correction factor for the depth image pixel at the given position
		PixelCorrection* getPixelCorrection(const Size& frameSize) const; // Returns pointer to a new-allocated array containing per-pixel depth correction parameters
		};
	
	struct IntrinsicParameters // Structure defining the intrinsic parameters of a depth and color frame source
		{
		/* Embedded classes: */
		public:
		typedef Video::LensDistortion LensDistortion; // Type for non-linear radial and tangential lens distortion correction formulas
		typedef LensDistortion::Scalar Scalar; // Scalar type for tangent or image space
		typedef LensDistortion::Point Point2; // Type for 2D points in tangent or image space
		typedef Geometry::ProjectiveTransformation<Scalar,3> PTransform; // Type for projective transformations
		typedef Geometry::AffineTransformation<Scalar,2> ATransform; // Type for 2D affine transformations in image space
		
		/* Elements: */
		LensDistortion depthLensDistortion; // Lens distortion correction parameters for the depth camera
		PTransform depthProjection; // The projection transformation from depth image space into 3D camera space
		ATransform di2t,dt2i; // Depth image space to depth camera tangent space transformation and its inverse
		LensDistortion colorLensDistortion; // Lens distortion correction parameters for the color camera
		PTransform colorProjection; // The projection transformation from 3D camera space into color image space
		ATransform ci2t,ct2i; // Color image space to color camera tangent space transformation and its inverse
		
		/* Methods: */
		static LensDistortion readLensDistortion(IO::File& file,bool newFormat); // Reads lens distortion correction parameters in old (Kinect V2) or new format from a file stream
		static void writeLensDistortion(const LensDistortion& ld,IO::File& file); // Writes lens distortion correction parameters in new format to a file stream
		void updateTransforms(void); // Re-calculates the image space transformations after depth and/or color projection transformations have been changed
		Point2 distortDepthPixel(const Point2& undistortedPixel) const; // Calculates forward lens distortion correction formula for the given point in depth image space
		Scalar depthDistortScalePixel(const Point2& distortedPixel) const; // Calculates the scaling factor of the forward lens distortion correction formula for the given point in depth image space
		Point2 undistortDepthPixel(const Point2& distortedPixel) const; // Calculates inverse lens distortion correction formula for the given point in depth image space
		Point2 undistortDepthPixel(unsigned int x,unsigned int y) const // Ditto, for the center of the pixel with the given indices
			{
			/* Convert the pixel index to the pixel center position and call the other method: */
			return undistortDepthPixel(Point2(Scalar(x)+Scalar(0.5),Scalar(y)+Scalar(0.5)));
			}
		Point2 undistortColorPixel(const Point2& distortedPixel) const; // Calculates inverse lens distortion correction formula for the given point in color image space
		Point2 undistortColorPixel(unsigned int x,unsigned int y) const // Ditto, for the center of the pixel with the given indices
			{
			/* Convert the pixel index to the pixel center position and call the other method: */
			return undistortColorPixel(Point2(Scalar(x)+Scalar(0.5),Scalar(y)+Scalar(0.5)));
			}
		};
	
	#if KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
	typedef Geometry::ProjectiveTransformation<double,3> ExtrinsicParameters; // Type for extrinsic camera parameters
	#else
	typedef Geometry::OrthogonalTransformation<double,3> ExtrinsicParameters; // Type for extrinsic camera parameters
	#endif
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	static const DepthPixel invalidDepth=0x07ffU; // The depth value indicating an invalid (or removed) pixel
	
	protected:
	ColorSpace colorSpace; // Color space used by the source's color stream
	Time timeBase; // Time base point for timestamp calculation
	
	/* Constructors and destructors: */
	public:
	FrameSource(void);
	virtual ~FrameSource(void);
	
	/* Methods: */
	ColorSpace getColorSpace(void) const // Returns the color stream's color space
		{
		return colorSpace;
		}
	virtual void setTimeBase(const Time& newTimeBase); // Sets the frame source's timestamp base point
	virtual DepthCorrection* getDepthCorrectionParameters(void); // Returns the camera depth correction object, i.e., per-pixel depth value offsets
	virtual IntrinsicParameters getIntrinsicParameters(void) =0; // Returns the intrinsic camera parameters, i.e., the virtual cameras' lens distortion correction formulas and projection matrices in camera space
	virtual ExtrinsicParameters getExtrinsicParameters(void) =0; // Returns the extrinsic camera parameters, i.e., the virtual cameras' position and orientation in 3D world space
	virtual const Size& getActualFrameSize(int sensor) const =0; // Returns the selected frame size of the color or depth stream as an array of (width, height) in pixels
	virtual DepthRange getDepthRange(void) const; // Returns the range of valid depth pixel values delivered by this frame source
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback) =0; // Installs the given streaming callback and starts receiving color and depth frames
	virtual void stopStreaming(void) =0; // Stops streaming; blocks until all pending frame transfers have either completed or been cancelled
	};

}

#endif
