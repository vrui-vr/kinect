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

#ifndef TIEPOINTTOOL_INCLUDED
#define TIEPOINTTOOL_INCLUDED

#include <vector>
#include <Threads/TripleBuffer.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <Geometry/Point.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>
#include <Kinect/FrameSource.h>
#include <Kinect/CornerExtractor.h>
#include <Kinect/DiskExtractor.h>

#include "RawKinectViewer.h"

class TiePointTool;
typedef Vrui::GenericToolFactory<TiePointTool> TiePointToolFactory;

class TiePointTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<TiePointTool>;
	
	/* Embedded classes: */
	private:
	typedef Kinect::CornerExtractor::Corner Corner;
	typedef Kinect::CornerExtractor::CornerList CornerList;
	typedef Kinect::DiskExtractor::Scalar Scalar;
	typedef Kinect::DiskExtractor::Point Point;
	typedef Kinect::DiskExtractor::Vector Vector;
	typedef Kinect::DiskExtractor::DiskList DiskList;
	
	struct TiePointPair // Structure to hold a pair of 3D camera space / color image space tie points
		{
		/* Elements: */
		public:
		Point cameraPoint; // 3D point in camera space
		Corner::Point colorPoint; // 2D point in color image space
		
		/* Constructors and destructors: */
		TiePointPair(const Point& sCameraPoint,const Corner::Point& sColorPoint) // Element-wise constructor
			:cameraPoint(sCameraPoint),colorPoint(sColorPoint)
			{
			}
		};
	
	/* Elements: */
	private:
	static TiePointToolFactory* factory; // Pointer to the factory object for this class
	RawKinectViewer::FrameStreamingCallback* colorFrameCallback; // Color streaming callback registered with RawKinectViewer application
	RawKinectViewer::FrameStreamingCallback* depthFrameCallback; // Depth streaming callback registered with RawKinectViewer application
	Kinect::CornerExtractor* cornerExtractor; // Helper object to extract grid corners from color frames
	Kinect::DiskExtractor* diskExtractor; // Helper object to extract disks from depth frames
	Threads::TripleBuffer<CornerList> cornerBuffer; // Triple buffer of corner extraction results
	Threads::TripleBuffer<DiskList> diskBuffer; // Triple buffer of disk extraction results
	bool accumulate;
	std::vector<TiePointPair> tiePoints; // List of collected tie points
	
	/* Private methods: */
	void cornerExtractionCallback(const CornerList& corners);
	void diskExtractionCallback(const DiskList& disks);
	void calibrateCameras(void);
	
	/* Constructors and destructors: */
	public:
	static TiePointToolFactory* initClass(Vrui::ToolManager& toolManager);
	TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~TiePointTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual void initialize(void);
	virtual void deinitialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
