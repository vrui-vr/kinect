/***********************************************************************
CalibrateCameras - Simple utility to read calibration tie points between
a depth camera and a color camera, and calculate the optimal projective
transformation mapping color to depth.
Copyright (c) 2010-2024 Oliver Kreylos

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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <string>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/StdError.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <IO/CSVSource.h>
#include <Math/Math.h>
#include <Math/Matrix.h>

int main(int argc,char* argv[])
	{
	/* Parse the command line: */
	int imgSize[2]={640,480};
	const char* tiePointFileName="CalibrationData.csv";
	const char* matrixFileName="CameraCalibrationMatrices.dat";
	int nameState=0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"size")==0)
				{
				for(int j=0;j<2;++j)
					{
					++i;
					imgSize[j]=atoi(argv[i]);
					}
				}
			}
		else if(nameState==0)
			{
			tiePointFileName=argv[i];
			++nameState;
			}
		else if(nameState==1)
			{
			matrixFileName=argv[i];
			++nameState;
			}
		}
	
	/* Create the linear system: */
	Math::Matrix a(12,12,0.0);
	
	{
	/* Open the calibration data file: */
	IO::CSVSource data(IO::openFile(tiePointFileName));
	
	unsigned int numEntries=0;
	while(!data.eof())
		{
		/* Read a calibration entry from the data file: */
		double x=data.readField<double>();
		double y=data.readField<double>();
		double z=data.readField<double>();
		double s=data.readField<double>()/double(imgSize[0]);
		double t=data.readField<double>()/double(imgSize[1]);
		
		// s=1.0-s;
		// t=1.0-t;
		
		/* Insert the entry's two linear equations into the linear system: */
		double eq[2][12];
		eq[0][0]=x;
		eq[0][1]=y;
		eq[0][2]=z;
		eq[0][3]=1.0;
		eq[0][4]=0.0;
		eq[0][5]=0.0;
		eq[0][6]=0.0;
		eq[0][7]=0.0;
		eq[0][8]=-s*x;
		eq[0][9]=-s*y;
		eq[0][10]=-s*z;
		eq[0][11]=-s;
		
		eq[1][0]=0.0;
		eq[1][1]=0.0;
		eq[1][2]=0.0;
		eq[1][3]=0.0;
		eq[1][4]=x;
		eq[1][5]=y;
		eq[1][6]=z;
		eq[1][7]=1.0;
		eq[1][8]=-t*x;
		eq[1][9]=-t*y;
		eq[1][10]=-t*z;
		eq[1][11]=-t;
		
		for(int row=0;row<2;++row)
			{
			for(unsigned int i=0;i<12;++i)
				for(unsigned int j=0;j<12;++j)
					a.set(i,j,a(i,j)+eq[row][i]*eq[row][j]);
			}
		
		++numEntries;
		}
	std::cout<<numEntries<<" calibration data entries read from file"<<std::endl;
	}
	
	/* Find the linear system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> qe=a.jacobiIteration();
	unsigned int minEIndex=0;
	double minE=Math::abs(qe.second(0,0));
	for(unsigned int i=1;i<12;++i)
		{
		if(minE>Math::abs(qe.second(i,0)))
			{
			minEIndex=i;
			minE=Math::abs(qe.second(i,0));
			}
		}
	
	/* Create the normalized homography: */
	Math::Matrix hom(3,4);
	double scale=qe.first(11,minEIndex);
	for(int i=0;i<3;++i)
		for(int j=0;j<4;++j)
			hom.set(i,j,qe.first(i*4+j,minEIndex)/scale);
	
	{
	/* Open the calibration data file again: */
	IO::CSVSource data(IO::openFile(tiePointFileName));
	
	/* Test the homography on all calibration data entries: */
	double rms=0.0;
	size_t numTiePoints=0;
	while(!data.eof())
		{
		/* Read a calibration entry from the data file: */
		Math::Matrix world(4,1);
		for(unsigned int i=0;i<3;++i)
			world.set(i,data.readField<double>());
		world.set(3,1.0);
		
		/* Read s and t: */
		double s=data.readField<double>();
		double t=data.readField<double>();
		
		/* Apply the homography: */
		Math::Matrix str=hom*world;
		double sp=str(0)/str(2);
		double tp=str(1)/str(2);
		
		// sp=1.0-sp;
		// tp=1.0-tp;
		
		// std::cout<<"Result: s = "<<sp*double(imgSize[0])<<", t = "<<tp*double(imgSize[1])<<std::endl;
		// std::cout<<world(0)<<", "<<world(1)<<", "<<world(2)<<", "<<sp*double(imgSize[0])<<", "<<tp*double(imgSize[1])<<std::endl;
		
		rms+=Math::sqr(s-sp*double(imgSize[0]))+Math::sqr(t-tp*double(imgSize[1]));
		++numTiePoints;
		}
	
	std::cout<<"Reprojection residual: "<<Math::sqrt(rms/double(numTiePoints))<<" pixel RMS"<<std::endl;
	}
	
	/* Read the intrinsic parameter file: */
	Misc::Float64 lensDistortionParameters[5];
	Misc::Float64 depthMatrix[4*4];
	Misc::Float64 colorMatrix[4*4];
	{
	/* Open the intrinsic parameter file: */
	IO::FilePtr matrixFile(IO::openFile(matrixFileName));
	matrixFile->setEndianness(Misc::LittleEndian);
	
	/* Read the file's contents: */
	matrixFile->read(lensDistortionParameters,5);
	matrixFile->read(depthMatrix,4*4);
	matrixFile->read(colorMatrix,4*4);
	}
	
	/* Back up the original intrinsic parameter file: */
	std::string backupMatrixFileName=matrixFileName;
	backupMatrixFileName.append(".backup");
	if(rename(matrixFileName,backupMatrixFileName.c_str())!=0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Cannot back up original intrinsic parameter file %s due to error %d (%s)",matrixFileName,errno,strerror(errno));
	
	{
	/* Open the intrinsic calibration file for writing: */
	IO::FilePtr matrixFile(IO::openFile(matrixFileName,IO::File::WriteOnly));
	matrixFile->setEndianness(Misc::LittleEndian);
	
	Math::Matrix depthProjection(4,4,0.0);
	for(unsigned int i=0;i<4;++i)
		for(unsigned int j=0;j<4;++j)
			depthProjection(i,j)=double(depthMatrix[i*4+j]);
	
	/* Write the original lens distortion parameters and depth projection matrix: */
	matrixFile->write(lensDistortionParameters,5);
	matrixFile->write(depthMatrix,4*4);
	
	/* Create the color projection matrix by extending the homography: */
	Math::Matrix colorProjection(4,4);
	for(unsigned int i=0;i<2;++i)
		for(unsigned int j=0;j<4;++j)
			colorProjection(i,j)=hom(i,j);
	for(unsigned int j=0;j<4;++j)
		colorProjection(2,j)=j==2?1.0:0.0;
	for(unsigned int j=0;j<4;++j)
		colorProjection(3,j)=hom(2,j);
	
	/* Modify the color projection matrix by the depth projection matrix: */
	colorProjection*=depthProjection;
	
	/* Print the color projection matrix: */
	std::cout<<std::endl;
	for(unsigned int i=0;i<4;++i)
		{
		std::cout<<colorProjection(i,0);
		for(unsigned int j=1;j<4;++j)
			std::cout<<", "<<colorProjection(i,j);
		std::cout<<std::endl;
		}
	
	/* Save the color projection matrix: */
	for(unsigned int i=0;i<4;++i)
		for(unsigned int j=0;j<4;++j)
			matrixFile->write(Misc::Float64(colorProjection(i,j)));
	}
	
	return 0;
	}
