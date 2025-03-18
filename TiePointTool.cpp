/***********************************************************************
TiePointTool - Calibration tool for RawKinectViewer.
Copyright (c) 2010-2023 Oliver Kreylos

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

#include "TiePointTool.h"

#include <stdio.h>
#include <vector>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/FunctionCalls.h>
#include <Misc/MessageLogger.h>
#include <Misc/FileTests.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <Math/Matrix.h>
#include <Geometry/ArrayKdTree.h>
#include <Geometry/OutputOperators.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Kinect/Internal/Config.h>

namespace {

/**************
Helper classes:
**************/

struct LinkCorner:public Kinect::CornerExtractor::Point // Structure to represent links between extracted corners
	{
	/* Embedded classes: */
	public:
	typedef Kinect::CornerExtractor::Scalar Scalar;
	typedef Kinect::CornerExtractor::Point Point;
	typedef Kinect::CornerExtractor::Vector Vector;
	
	/* Elements: */
	public:
	Vector bw,wb; // Separation line directions from black to white and white to black, respectively, forming a right-handed frame
	LinkCorner* links[4]; // Array of four pointers to neighboring corners along the grid directions bw+, wb+, bw-, wb-, respectively
	
	/* Methods: */
	void unlink(int dir)
		{
		if(links[dir]!=0)
			{
			/* Find and remove the linked node's link back to this node: */
			LinkCorner* other=links[dir];
			int otherDir=(dir+1)%2;
			if(other->links[otherDir]==this)
				other->links[otherDir]=0;
			otherDir+=2;
			if(other->links[otherDir]==this)
				other->links[otherDir]=0;
			
			links[dir]=0;
			}
		}
	};

typedef Geometry::ArrayKdTree<LinkCorner> LinkCornerTree; // Tree to enumerate corners in distance order

struct CornerLinker // Functor to link corners based on their separator directions
	{
	/* Embedded classes: */
	public:
	typedef LinkCorner::Scalar Scalar;
	
	/* Elements: */
	LinkCorner* treeBase;
	LinkCorner* corner;
	Scalar maxAngleCos;
	Scalar maxSearchDist;
	Scalar linkedDists[4];
	
	/* Methods: */
	const LinkCornerTree::Point& getQueryPosition(void) const
		{
		return *corner;
		}
	bool operator()(const LinkCornerTree::StoredPoint& node,int splitDimension)
		{
		/* Get a non-const pointer to the other corner: */
		LinkCorner* other=treeBase+(&node-treeBase);
		
		/* Get the direction and distance to the other node: */
		LinkCorner::Vector d=*other-*corner;
		Scalar dist=d.mag();
		
		/* Find the separator line most closely aligned with the direction to the other node: */
		int dir=-1;
		Scalar mac=maxAngleCos;
		for(int i=0;i<4;++i)
			{
			/* Check whether the link is unused or the currently linked node is farther away than the one being tested: */
			if(corner->links[i]==0||linkedDists[i]>dist)
				{
				/* Calculate the angle of the other node w.r.t. the current separator line: */
				Scalar angleCos=i%2==0?(corner->bw*d)/dist:(corner->wb*d)/dist; // Dirs 0 and 2 are bw, dirs 1 and 3 are wb
				if(i>=2) // Dirs 0 and 1 are +bw and +wb, dirs 2 and 3 are -bw and -wb
					angleCos=-angleCos;
				
				if(mac<angleCos)
					{
					dir=i;
					mac=angleCos;
					}
				}
			}
		
		if(dir>=0)
			{
			/* Check if the tested node's orientation is compatible with the corner: */
			int otherDir=(dir+1)%2; // wb can only link with bw, and vice versa
			Scalar otherAngleCos=otherDir==0?-(other->bw*d)/dist:-(other->wb*d)/dist;
			if(otherAngleCos<Scalar(0)) // If the node's separator points the other way, go to the opposite separator
				{
				otherDir+=2;
				otherAngleCos=-otherAngleCos;
				}
			
			/* Check whether the link is possible: */
			if(otherAngleCos>maxAngleCos&&(other->links[otherDir]==0||Geometry::sqrDist(*other,*other->links[otherDir])>Math::sqr(dist)))
				{
				/* Unlink any previously existing link: */
				corner->unlink(dir);
				other->unlink(otherDir);
				
				/* Link the two corners: */
				corner->links[dir]=other;
				other->links[otherDir]=corner;
				linkedDists[dir]=dist;
				
				/* Check if all the corner's links are occupied: */
				float maxLinkedDist=Scalar(0);
				int i;
				for(i=0;i<4&&corner->links[i]!=0;++i)
					{
					if(maxLinkedDist<linkedDists[i])
						maxLinkedDist=linkedDists[i];
					}
				if(i==4)
					{
					/* Reduce the maximum search distance: */
					maxSearchDist=maxLinkedDist;
					}
				}
			}
		
		return Math::abs(node[splitDimension]-(*corner)[splitDimension])<maxSearchDist;
		}
	};

}

/*************************************
Static elements of class TiePointTool:
*************************************/

TiePointToolFactory* TiePointTool::factory=0;

/*****************************
Methods of class TiePointTool:
*****************************/

void TiePointTool::cornerExtractionCallback(const TiePointTool::CornerList& corners)
	{
	/* Enter the new corner list into the triple buffer: */
	CornerList& newValue=cornerBuffer.startNewValue();
	newValue.clear();
	
	/* Create a new kd-tree from all root corner candidate points to assemble a grid: */
	LinkCornerTree cornerTree(corners.size());
	LinkCorner* cPtr=cornerTree.accessPoints();
	for(CornerList::const_iterator cIt=corners.begin();cIt!=corners.end();++cIt,++cPtr)
		{
		cPtr->Corner::Point::operator=(*cIt);
		cPtr->bw=cIt->bw;
		cPtr->wb=cIt->wb;
		for(int i=0;i<4;++i)
			cPtr->links[i]=0;
		}
	cornerTree.releasePoints();
	
	/* Create links between any pair of corners that roughly lie along their separating directions: */
	CornerLinker cl;
	cl.treeBase=cornerTree.accessPoints();
	cl.maxAngleCos=Math::cos(Math::rad(CornerLinker::Scalar(30)));
	LinkCorner* ctEnd=cornerTree.accessPoints()+cornerTree.getNumNodes();
	for(LinkCorner* cPtr=cornerTree.accessPoints();cPtr!=ctEnd;++cPtr)
		{
		/* Prepare to look for links from the current corner: */
		cl.corner=cPtr;
		cl.maxSearchDist=Math::Constants<Corner::Scalar>::max;
		for(int i=0;i<4;++i)
			cl.linkedDists[i]=Corner::Scalar(0); // Not actually necessary
		
		/* Traverse the tree to find all links: */
		cornerTree.traverseTreeDirected(cl);
		}
	
	/* Look for a corner with four outgoing links whose intersections are close to the corner itself: */
	for(LinkCorner* cPtr=cornerTree.accessPoints();cPtr!=ctEnd;++cPtr)
		{
		if(cPtr->links[0]!=0&&cPtr->links[1]!=0&&cPtr->links[2]!=0&&cPtr->links[3]!=0)
			{
			/* Find the intersection between lines through opposing neighbors: */
			const Corner::Point& p0=*cPtr->links[0];
			const Corner::Point& p1=*cPtr->links[1];
			const Corner::Point& p2=*cPtr->links[2];
			const Corner::Point& p3=*cPtr->links[3];
			
			Corner::Scalar det=(p2[0]-p0[0])*(p1[1]-p3[1])-(p1[0]-p3[0])*(p2[1]-p0[1]);
			Corner::Scalar alpha=((p1[1]-p3[1])*(p1[0]-p0[0])+(p3[0]-p1[0])*(p1[1]-p0[1]))/det;
			Corner::Scalar beta=((p0[1]-p2[1])*(p1[0]-p0[0])+(p2[0]-p0[0])*(p1[1]-p0[1]))/det;
			Corner::Point intersect=Geometry::mid(p0+(p2-p0)*alpha,p1+(p3-p1)*beta);
			
			/* Check if the intersection is close enough to the central point: */
			if(Geometry::sqrDist(intersect,*cPtr)<Math::sqr(2.0))
				{
				/* Store the center point: */
				Corner newCorner;
				newCorner.Corner::Point::operator=(Geometry::mid(*cPtr,intersect));
				newCorner.bw=cPtr->bw;
				newCorner.wb=cPtr->wb;
				newValue.push_back(newCorner);
				}
			}
		}
	
	cornerBuffer.postNewValue();
	Vrui::requestUpdate();
	}

void TiePointTool::diskExtractionCallback(const TiePointTool::DiskList& disks)
	{
	/* Enter the new disk list into the triple buffer: */
	DiskList& newValue=diskBuffer.startNewValue();
	newValue=disks;
	diskBuffer.postNewValue();
	Vrui::requestUpdate();
	}

void TiePointTool::calibrateCameras(void)
	{
	/* Enter all collected tie point pairs into a linear system to calculate the color transformation: */
	Math::Matrix a(12,12,0.0);
	for(std::vector<TiePointPair>::iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Normalize the colorspace point: */
		double s=tpIt->colorPoint[0]/double(application->colorFrameSize[0]);
		double t=tpIt->colorPoint[1]/double(application->colorFrameSize[1]);
		
		/* Insert the entry's two linear equations into the linear system: */
		double eq[2][12];
		eq[0][0]=tpIt->cameraPoint[0];
		eq[0][1]=tpIt->cameraPoint[1];
		eq[0][2]=tpIt->cameraPoint[2];
		eq[0][3]=1.0;
		eq[0][4]=0.0;
		eq[0][5]=0.0;
		eq[0][6]=0.0;
		eq[0][7]=0.0;
		eq[0][8]=-s*tpIt->cameraPoint[0];
		eq[0][9]=-s*tpIt->cameraPoint[1];
		eq[0][10]=-s*tpIt->cameraPoint[2];
		eq[0][11]=-s;
		
		eq[1][0]=0.0;
		eq[1][1]=0.0;
		eq[1][2]=0.0;
		eq[1][3]=0.0;
		eq[1][4]=tpIt->cameraPoint[0];
		eq[1][5]=tpIt->cameraPoint[1];
		eq[1][6]=tpIt->cameraPoint[2];
		eq[1][7]=1.0;
		eq[1][8]=-t*tpIt->cameraPoint[0];
		eq[1][9]=-t*tpIt->cameraPoint[1];
		eq[1][10]=-t*tpIt->cameraPoint[2];
		eq[1][11]=-t;
		
		for(int row=0;row<2;++row)
			for(unsigned int i=0;i<12;++i)
				for(unsigned int j=0;j<12;++j)
					a(i,j)+=eq[row][i]*eq[row][j];
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
			hom(i,j)=qe.first(i*4+j,minEIndex)/scale;
	
	/* Calculate the reprojection error: */
	double rms=0.0;
	double max=0.0;
	for(std::vector<TiePointPair>::iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Create a homogeneous vector representing the camera-space point: */
		Math::Matrix camP(4,1);
		for(int i=0;i<3;++i)
			camP(i)=tpIt->cameraPoint[i];
		camP(3)=1.0;
		
		/* Project the camera-space point to image space: */
		Math::Matrix colP=hom*camP;
		
		/* Extract the affine image-space point in pixel coordinates: */
		double colX=colP(0)*double(application->colorFrameSize[0])/colP(2);
		double colY=colP(1)*double(application->colorFrameSize[1])/colP(2);
		
		/* Calculate the squared approximation error: */
		double err2=Math::sqr(tpIt->colorPoint[0]-colX)+Math::sqr(tpIt->colorPoint[1]-colY);
		rms+=err2;
		if(max<err2)
			max=err2;
		}
	rms=Math::sqrt(rms/double(tiePoints.size()));
	Misc::formattedUserNote("TiePointTool: Camera calibration reprojection error: %f pixels RMS, %f pixels max",rms,Math::sqrt(max));
	
	/* Assemble the name of the intrinsic parameter file: */
	std::string intrinsicParameterFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	intrinsicParameterFileName.push_back('/');
	intrinsicParameterFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX);
	intrinsicParameterFileName.push_back('-');
	intrinsicParameterFileName.append(application->camera->getSerialNumber());
	intrinsicParameterFileName.append(".dat");
	
	/* Back up the original intrinsic parameter file if it exists: */
	if(Misc::doesPathExist(intrinsicParameterFileName.c_str()))
		{
		std::string backupFileName=intrinsicParameterFileName;
		backupFileName.append(".backup");
		rename(intrinsicParameterFileName.c_str(),backupFileName.c_str());
		}
	
	/* Write the new intrinsic parameter file: */
	IO::FilePtr intrinsicParameterFile(IO::openFile(intrinsicParameterFileName.c_str(),IO::File::WriteOnly));
	intrinsicParameterFile->setEndianness(Misc::LittleEndian);
	
	Kinect::FrameSource::IntrinsicParameters& ip=application->intrinsicParameters;
	
	/* Write depth lens distortion correction parameters: */
	for(int i=0;i<3;++i)
		intrinsicParameterFile->write(Misc::Float64(ip.depthLensDistortion.getKappa(i)));
	for(int i=0;i<2;++i)
		intrinsicParameterFile->write(Misc::Float64(ip.depthLensDistortion.getRho(i)));
	
	/* Write the depth unprojection matrix: */
	Math::Matrix depthProjection(4,4);
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			{
			depthProjection(i,j)=ip.depthProjection.getMatrix()(i,j);
			intrinsicParameterFile->write(Misc::Float64(depthProjection(i,j)));
			}
	
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
	
	/* Write the color projection matrix: */
	for(unsigned int i=0;i<4;++i)
		for(unsigned int j=0;j<4;++j)
			intrinsicParameterFile->write(Misc::Float64(colorProjection(i,j)));
	}

TiePointToolFactory* TiePointTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new TiePointToolFactory("TiePointTool","Tie Points",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Save Point Pair");
	factory->setButtonFunction(1,"Calibrate Cameras");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

TiePointTool::TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 colorFrameCallback(0),depthFrameCallback(0),
	 cornerExtractor(0),diskExtractor(0),
	 accumulate(false)
	{
	}

TiePointTool::~TiePointTool(void)
	{
	}

void TiePointTool::initialize(void)
	{
	/* Set up the color frame processing pipeline: */
	cornerExtractor=new Kinect::CornerExtractor(application->colorFrameSize,7,3);
	cornerExtractor->setInputGamma(2.2f);
	cornerExtractor->setNormalizationWindowSize(48);
	cornerExtractor->setRegionThreshold(64U);
	colorFrameCallback=Misc::createFunctionCall(cornerExtractor,&Kinect::CornerExtractor::submitFrame);
	
	/* Set up the depth frame processing pipeline: */
	diskExtractor=new Kinect::DiskExtractor(application->depthFrameSize,application->depthCorrection,application->intrinsicParameters);
	diskExtractor->setMaxBlobMergeDist(5);
	diskExtractor->setMinNumPixels(300);
	diskExtractor->setDiskRadius(6.0);
	diskExtractor->setDiskRadiusMargin(1.1);
	diskExtractor->setDiskFlatness(25.0);
	depthFrameCallback=Misc::createFunctionCall(diskExtractor,&Kinect::DiskExtractor::submitFrame);
	
	/* Start processing on both pipelines: */
	cornerExtractor->startStreaming(Misc::createFunctionCall(this,&TiePointTool::cornerExtractionCallback));
	diskExtractor->startStreaming(Misc::createFunctionCall(this,&TiePointTool::diskExtractionCallback));
	
	/* Register streaming callbacks with the RawKinectViewer application: */
	application->registerColorCallback(colorFrameCallback);
	application->registerDepthCallback(depthFrameCallback);
	}

void TiePointTool::deinitialize(void)
	{
	/* Unregister the color and depth streaming callbacks: */
	application->unregisterColorCallback(colorFrameCallback);
	delete colorFrameCallback;
	colorFrameCallback=0;
	application->unregisterDepthCallback(depthFrameCallback);
	delete depthFrameCallback;
	depthFrameCallback=0;
	
	/* Shut down the color and depth frame processing pipelines: */
	cornerExtractor->stopStreaming();
	diskExtractor->stopStreaming();
	
	delete cornerExtractor;
	cornerExtractor=0;
	delete diskExtractor;
	diskExtractor=0;
	}

const Vrui::ToolFactory* TiePointTool::getFactory(void) const
	{
	return factory;
	}

void TiePointTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(buttonSlotIndex==0) // Save point pair
		{
		if(cbData->newButtonState)
			{
			/* Start accumulating: */
			accumulate=true;
			}
		else
			{
			/* Stop accumulating: */
			accumulate=false;
			}
		}
	else // Calibrate cameras
		{
		if(cbData->newButtonState)
			{
			/* Check if there are enough tie points: */
			if(tiePoints.size()>=5)
				{
				/* Calculate the camera calibration: */
				calibrateCameras();
				}
			else
				Misc::userError("TiePointTool: Not enough tie points for camera calibration; please collect at least 5");
			}
		}
	}

void TiePointTool::frame(void)
	{
	/* Lock the most recent extraction results: */
	cornerBuffer.lockNewValue();
	diskBuffer.lockNewValue();
	
	/* Check if the current results are valid and need to be accumulated: */
	if(accumulate&&cornerBuffer.getLockedValue().size()==1U&&diskBuffer.getLockedValue().size()==1U)
		{
		/* Append a tie point pair to the list: */
		tiePoints.push_back(TiePointPair(diskBuffer.getLockedValue().front().center,cornerBuffer.getLockedValue().front()));
		}
	}

void TiePointTool::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(3.0f);
	glPointSize(3.0f);
	
	/* Go to navigational space: */
	Vrui::goToNavigationalSpace(contextData);
	
	if(!cornerBuffer.getLockedValue().empty())
		{
		/* Draw all current grid corners: */
		Corner::Scalar cis=application->colorImageScale;
		Corner::Scalar radius=Corner::Scalar(cornerExtractor->getTestRadius())*Corner::Scalar(5);
		glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,1.0f);
		for(CornerList::const_iterator cIt=cornerBuffer.getLockedValue().begin();cIt!=cornerBuffer.getLockedValue().end();++cIt)
			{
			glVertex3f(((*cIt)[0]-cIt->bw[0]*radius)*cis,((*cIt)[1]-cIt->bw[1]*radius)*cis,0.02f);
			glVertex3f(((*cIt)[0]+cIt->bw[0]*radius)*cis,((*cIt)[1]+cIt->bw[1]*radius)*cis,0.02f);
			glVertex3f(((*cIt)[0]-cIt->wb[0]*radius)*cis,((*cIt)[1]-cIt->wb[1]*radius)*cis,0.02f);
			glVertex3f(((*cIt)[0]+cIt->wb[0]*radius)*cis,((*cIt)[1]+cIt->wb[1]*radius)*cis,0.02f);
			}
		glEnd();
		}
	
	/* Draw all current disk centroids: */
	if(!diskBuffer.getLockedValue().empty())
		{
		glColor3f(1.0f,1.0f,1.0f);
		for(DiskList::const_iterator dIt=diskBuffer.getLockedValue().begin();dIt!=diskBuffer.getLockedValue().end();++dIt)
			{
			/* Project the extracted 3D disk into depth image space: */
			Vector x=Geometry::normal(dIt->normal);
			x.normalize();
			Vector y=dIt->normal^x;
			y.normalize();
			glBegin(GL_LINE_LOOP);
			for(int i=0;i<32;++i)
				{
				Scalar angle=Scalar(2)*Math::Constants<Scalar>::pi*Scalar(i)/Scalar(32);
				Point ip=application->intrinsicParameters.depthProjection.inverseTransform(dIt->center+x*(Math::cos(angle)*dIt->radius)+y*(Math::sin(angle)*dIt->radius));
				glVertex3d(ip[0]-application->depthImageOffset,ip[1],0.02);
				}
			glEnd();
			
			glBegin(GL_POINTS);
			Point ip=application->intrinsicParameters.depthProjection.inverseTransform(dIt->center);
			glVertex3d(ip[0]-application->depthImageOffset,ip[1],0.02);
			glEnd();
			}
		}
	
	/* Return to physical space: */
	glPopMatrix();
	
	glPopAttrib();
	}
