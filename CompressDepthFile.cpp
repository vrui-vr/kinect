/***********************************************************************
CompressDepthFile - Utility to compress a Kinect depth frame file using
the DepthFrameWriter class.
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/Timer.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Kinect/Types.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>
#include <Kinect/DepthFrameWriter.h>

int main(int argc,char* argv[])
	{
	/* Open the depth stream file: */
	IO::FilePtr depthFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/DepthFrames.dat"));
	depthFrameFile->setEndianness(Misc::LittleEndian);
	
	/* Read the file header: */
	Kinect::Size size;
	depthFrameFile->read<Misc::UInt32,unsigned int>(size.getComponents(),2);
	
	/* Create a background frame: */
	unsigned short* backgroundFrame=new unsigned short[size.volume()];
	unsigned short* bfPtr=backgroundFrame;
	unsigned short* bfEnd=bfPtr+size.volume();
	for(;bfPtr!=bfEnd;++bfPtr)
		*bfPtr=Kinect::FrameSource::invalidDepth;
	unsigned int numCaptureFrames=150;
	
	/* Create the depth frame writer: */
	IO::FilePtr compressedDepthFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/CompressedDepthFrames.dat",IO::File::WriteOnly));
	Kinect::DepthFrameWriter depthFrameWriter(*compressedDepthFrameFile,size);
	
	/* Process all frames from the depth frame file: */
	size_t totalSize=0;
	double totalTime=0.0;
	double maxTime=0.0;
	unsigned int numFrames=0;
	while(!depthFrameFile->eof())
		{
		/* Read the next frame's time stamp: */
		double timeStamp=depthFrameFile->read<double>();
		
		/* Read the next depth frame: */
		Kinect::FrameBuffer frame(size,size.volume()*sizeof(unsigned short));
		unsigned short* frameBuffer=frame.getData<unsigned short>();
		depthFrameFile->read(frameBuffer,size.volume());
		
		if(numCaptureFrames>0)
			{
			/* Add the depth frame to the background frame: */
			unsigned short* bfPtr=backgroundFrame;
			unsigned short* bfEnd=bfPtr+size.volume();
			const unsigned short* dfPtr=frameBuffer;
			for(;bfPtr!=bfEnd;++bfPtr,++dfPtr)
				if(*bfPtr>*dfPtr-2)
					*bfPtr=*dfPtr-2;
			--numCaptureFrames;
			}
		else
			{
			/* Remove background from the depth frame: */
			unsigned short* dfPtr=frameBuffer;
			unsigned short* dfEnd=dfPtr+size.volume();
			const unsigned short* bfPtr=backgroundFrame;
			for(;dfPtr!=dfEnd;++dfPtr,++bfPtr)
				if(*dfPtr>=*bfPtr)
					*dfPtr=Kinect::FrameSource::invalidDepth;
			}
		
		/* Compress and save the depth frame: */
		Misc::Timer compressTime;
		compressedDepthFrameFile->write<double>(timeStamp);
		size_t compressedSize=depthFrameWriter.writeFrame(frame);
		compressTime.elapse();
		double time=compressTime.getTime();
		totalSize+=compressedSize;
		totalTime+=time;
		if(maxTime<time)
			maxTime=time;
		++numFrames;
		}
	std::cout<<"Total compression time: "<<totalTime*1000.0<<" ms, total file size: "<<totalSize<<", "<<numFrames<<" frames"<<std::endl;
	std::cout<<"Maximum compression time: "<<maxTime*1000.0<<" ms"<<std::endl;
	std::cout<<"Compression frame rate: "<<double(numFrames)/totalTime<<" Hz"<<std::endl;
	std::cout<<"Bandwidth: "<<double(totalSize)*30.0/double(numFrames)/(1024.0*1024.0)<<"MB/s"<<std::endl;
	
	return 0;
	}
