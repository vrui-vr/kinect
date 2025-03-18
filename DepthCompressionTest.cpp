/***********************************************************************
DepthCompressionTest - Utility to check the results of compressing a
depth frame file.
Copyright (c) 2010-2022 Oliver Kreylos

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

#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/Marshaller.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Geometry/GeometryMarshallers.h>
#include <Kinect/Types.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>
#include <Kinect/DepthFrameReader.h>

int main(int argc,char* argv[])
	{
	/* Open a compressed depth stream file: */
	IO::FilePtr depthFrameFile(IO::openFile(argv[1]));
	depthFrameFile->setEndianness(Misc::LittleEndian);
	
	/* Read the file's format version number: */
	unsigned int fileFormatVersion=depthFrameFile->read<Misc::UInt32>();
	
	/* Check if there are per-pixel depth correction coefficients: */
	if(fileFormatVersion>=4)
		{
		/* Read new B-spline based depth correction parameters: */
		Kinect::FrameSource::DepthCorrection dc(*depthFrameFile);
		}
	else
		{
		if(fileFormatVersion>=2&&depthFrameFile->read<Misc::UInt8>()!=0)
			{
			/* Skip the depth correction buffer: */
			Kinect::Size size;
			depthFrameFile->read<Misc::UInt32,unsigned int>(size.getComponents(),2);
			depthFrameFile->skip<Misc::Float32>(size.volume()*2);
			}
		}
	
	/* Check if the depth stream uses lossy compression: */
	bool depthIsLossy=fileFormatVersion>=3&&depthFrameFile->read<Misc::UInt8>()!=0;
	
	/* Check if the depth camera has lens distortion correction parameters: */
	Kinect::FrameSource::IntrinsicParameters intrinsicParameters;
	if(fileFormatVersion>=5)
		{
		/* Read the depth camera's lens distortion correction parameters: */
		intrinsicParameters.depthLensDistortion.read(*depthFrameFile);
		}
	
	/* Read the depth projection from the file: */
	intrinsicParameters.depthProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*depthFrameFile);
	
	/* Read the camera transformation from the depth file: */
	Kinect::FrameSource::ExtrinsicParameters extrinsicParameters;
	extrinsicParameters=Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::read(*depthFrameFile);
	
	/* Create a depth frame reader: */
	Kinect::DepthFrameReader depthFrameReader(*depthFrameFile);
	size_t numFrames=0;
	while(!depthFrameFile->eof())
		{
		/* Read the next depth frame: */
		Kinect::FrameBuffer frame=depthFrameReader.readNextFrame();
		++numFrames;
		}
	
	/* Calculate uncompressed depth stream size: */
	size_t frameSize=depthFrameReader.getSize().volume()*12;
	size_t uncompressedSize=(numFrames*frameSize+7)/8;
	std::cout<<numFrames<<" frames, "<<uncompressedSize<<" bytes uncompressed"<<std::endl;
	
	return 0;
	}
