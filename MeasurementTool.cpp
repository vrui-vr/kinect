/***********************************************************************
MeasurementTool - Tool to measure 3D positions from the depth image
stream.
Copyright (c) 2014-2023 Oliver Kreylos

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

#include "MeasurementTool.h"

#include <string>
#include <iostream>
#include <iomanip>
#include <Geometry/OutputOperators.h>
#include <Vrui/ToolManager.h>

#include "RawKinectViewer.h"

/****************************************
Static elements of class MeasurementTool:
****************************************/

MeasurementToolFactory* MeasurementTool::factory=0;

/********************************
Methods of class MeasurementTool:
********************************/

MeasurementToolFactory* MeasurementTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new MeasurementToolFactory("DepthMeasurementTool","Measure 3D Positions",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(1);
	factory->setButtonFunction(0,"Measure Position");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

MeasurementTool::MeasurementTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

MeasurementTool::~MeasurementTool(void)
	{
	}

const Vrui::ToolFactory* MeasurementTool::getFactory(void) const
	{
	return factory;
	}

void MeasurementTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		/* Calculate the selected depth image point in corrected depth camera space: */
		RawKinectViewer::CPoint depthImagePoint=application->getDepthImagePoint(application->calcImagePoint(getButtonDeviceRay(0)));
		
		/* Check if the depth image point is valid: */
		if(depthImagePoint[2]>=RawKinectViewer::CPoint::Scalar(0))
			{
			/* Transform the image point to world space and print it: */
			RawKinectViewer::CPoint worldPoint=application->intrinsicParameters.depthProjection.transform(depthImagePoint);
			std::cout<<std::setw(20)<<worldPoint<<std::endl;
			}
		}
	}
