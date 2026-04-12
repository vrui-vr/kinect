/***********************************************************************
PointPlaneTool - Calibration tool for RawKinectViewer.
Copyright (c) 2013-2026 Oliver Kreylos

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

#include "PointPlaneTool.h"

#include <iostream>
#include <iomanip>
#include <Math/Math.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <Vrui/Vrui.h>

#include "RawKinectViewer.h"

/***************************************
Static elements of class PointPlaneTool:
***************************************/

PointPlaneToolFactory* PointPlaneTool::factory=0;

/*******************************
Methods of class PointPlaneTool:
*******************************/

PointPlaneToolFactory* PointPlaneTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new PointPlaneToolFactory("PointPlaneTool","Define Depth Planes",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Pick Point");
	factory->setButtonFunction(1,"Define Plane");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

PointPlaneTool::PointPlaneTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

PointPlaneTool::~PointPlaneTool(void)
	{
	}

const Vrui::ToolFactory* PointPlaneTool::getFactory(void) const
	{
	return factory;
	}

void PointPlaneTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
			{
			/* Select another depth image point: */
			RawKinectViewer::CPoint imagePoint=application->getDepthImagePoint(application->calcImagePoint(getButtonDeviceRay(0)));
			if(imagePoint[2]>=RawKinectViewer::CPoint::Scalar(0))
				{
				/* Add the point to the list: */
				points.push_back(imagePoint);
				}
			}
		else
			{
			if(points.size()>=3)
				{
				/* Calculate the camera-space plane defined by the selected points: */
				Geometry::PCACalculator<3> pca;
				for(std::vector<Point>::iterator pIt=points.begin();pIt!=points.end();++pIt)
					pca.accumulatePoint(*pIt);
				
				Geometry::PCACalculator<3>::Point centroid=pca.calcCentroid();
				pca.calcCovariance();
				double evs[3];
				pca.calcEigenvalues(evs);
				Geometry::PCACalculator<3>::Vector normal=pca.calcEigenvector(evs[2]);
				
				/* Check for any nans or infs: */
				bool allFinite=true;
				for(int i=0;i<3;++i)
					{
					allFinite=allFinite&&Math::isFinite(normal[i]);
					allFinite=allFinite&&Math::isFinite(centroid[i]);
					}
				
				if(allFinite)
					{
					/* Print the approximation residual: */
					std::cout<<"Depth-space approximation residual: "<<evs[2]<<std::endl;
					
					/* Flip the plane's normal vector if it points the wrong way: */
					if(centroid*normal<0.0)
						normal=-normal;
					
					/* Print the plane equation in depth image space: */
					std::cout<<"Depth-space plane equation: x * "<<normal<<" = "<<centroid*normal<<std::endl;
					
					/* Set the application's depth plane in camera and world space: */
					application->depthPlaneValid=true;
					application->camDepthPlane=RawKinectViewer::Plane(normal,centroid);
					application->worldDepthPlane=application->camDepthPlane; 
					application->worldDepthPlane.transform(application->intrinsicParameters.depthProjection);
					if(application->worldDepthPlane.getOffset()>0.0)
						application->worldDepthPlane.flip();
					application->worldDepthPlane.normalize();
					
					/* Print the plane equation in camera space: */
					std::cout<<"Camera-space plane equation: x * "<<application->worldDepthPlane.getNormal()<<" = "<<application->worldDepthPlane.getOffset()<<std::endl;
					
					/* Clear the selected point set: */
					points.clear();
					}
				else
					{
					/* Show an error message: */
					Vrui::showErrorMessage("Define Plane","Depth plane equation is undefined");
					}
				}
			else
				{
				/* Show an error message: */
				Vrui::showErrorMessage("Define Plane","Need at least three depth points to define depth plane");
				}
			}
		}
	}

void PointPlaneTool::display(GLContextData& contextData) const
	{
	if(!points.empty())
		{
		glPushAttrib(GL_ENABLE_BIT|GL_POINT_BIT);
		glDisable(GL_LIGHTING);
		glPointSize(3.0f);

		/* Go to navigational space: */
		Vrui::goToNavigationalSpace(contextData);
		
		glBegin(GL_POINTS);
		glColor3f(1.0f,1.0f,1.0f);
		for(std::vector<Point>::const_iterator pIt=points.begin();pIt!=points.end();++pIt)
			glVertex3d((*pIt)[0]-application->depthImageOffset,(*pIt)[1],0.01);
		glEnd();
		
		/* Return to physical space: */
		glPopMatrix();
		
		glPopAttrib();
		}
	}
