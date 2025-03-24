/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
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

#include "RawKinectViewer.h"

#include <ctype.h>
#include <stdexcept>
#include <iostream>
#include <Misc/SelfDestructPointer.h>
#include <Misc/StringPrintf.h>
#include <Misc/FunctionCalls.h>
#include <IO/File.h>
#include <IO/Directory.h>
#include <Geometry/Point.h>
#include <Geometry/Ray.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/Extensions/GLARBTextureNonPowerOfTwo.h>
#include <GL/GLContextData.h>
#include <Images/RGBImage.h>
#include <Images/WriteImageFile.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Kinect/Config.h>
#include <Kinect/FrameSource.h>
#include <Kinect/Camera.h>
#include <Kinect/OpenDirectFrameSource.h>
#include "PauseTool.h"
#include "MeasurementTool.h"
#include "TiePointTool.h"
#include "LineTool.h"
#include "DepthCorrectionTool.h"
#include "GridTool.h"
#include "PlaneTool.h"
#include "PointPlaneTool.h"
#include "CalibrationCheckTool.h"

/******************************************
Methods of class RawKinectViewer::DataItem:
******************************************/

RawKinectViewer::DataItem::DataItem(void)
	:colorTextureId(0),colorFrameVersion(0),
	 depthTextureId(0),depthFrameVersion(0)
	{
	/* Allocate texture objects: */
	glGenTextures(1,&depthTextureId);
	glGenTextures(1,&colorTextureId);
	}

RawKinectViewer::DataItem::~DataItem(void)
	{
	/* Destroy texture objects: */
	glDeleteTextures(1,&depthTextureId);
	glDeleteTextures(1,&colorTextureId);
	}

/********************************
Methods of class RawKinectViewer:
********************************/

void RawKinectViewer::mapDepth(const Offset& pixel,float depth,GLubyte* result) const
	{
	if(depthPlaneValid)
		{
		/* Color depth pixels by distance to the depth plane: */
		float dist=camDepthPlane.calcDistance(Plane::Point(float(pixel[0])+0.5f,float(pixel[1])+0.5f,depth));
		if(dist>=0.0f)
			{
			GLubyte col=dist<depthPlaneDistMax?255U-GLubyte((dist*255.0f)/depthPlaneDistMax+0.5f):0U;
			result[0]=col;
			result[1]=col;
			result[2]=255U;
			}
		else
			{
			GLubyte col=-dist<depthPlaneDistMax?255U-GLubyte((-dist*255.0f)/depthPlaneDistMax+0.5f):0U;
			result[0]=255U;
			result[1]=col;
			result[2]=col;
			}
		}
	else
		{
		/* Color depth pixels by depth value: */
		static const GLubyte mapColors[6][3]=
			{
			{255,0,0},
			{255,255,0},
			{0,255,0},
			{0,255,255},
			{0,0,255},
			{255,0,255}
			};
		float d=(depth-depthValueRange[0])*5.0f/(depthValueRange[1]-depthValueRange[0]);
		if(d<=0.0f)
			{
			for(int i=0;i<3;++i)
				result[i]=GLubyte(mapColors[0][i]*0.2f);
			}
		else if(d>=5.0f)
			{
			for(int i=0;i<3;++i)
				result[i]=mapColors[5][i];
			}
		else
			{
			int i0=int(d);
			d-=float(i0);
			for(int i=0;i<3;++i)
				result[i]=GLubyte((mapColors[i0][i]*(1.0f-d)+mapColors[i0+1][i]*d)*(d*0.8f+0.2f));
			}
		}
	}

Vrui::Point RawKinectViewer::calcImagePoint(const Vrui::Ray& physicalRay) const
	{
	/* Transform the ray to navigational space: */
	Vrui::Ray navRay=physicalRay;
	navRay.transform(Vrui::getInverseNavigationTransformation());
	
	/* Intersect the ray with the z=0 plane in navigational space: */
	if(navRay.getDirection()[2]!=Vrui::Scalar(0))
		{
		Vrui::Scalar lambda=-navRay.getOrigin()[2]/navRay.getDirection()[2];
		return navRay(lambda);
		}
	else
		return Vrui::Point::origin;
	}

RawKinectViewer::CPoint RawKinectViewer::calcDepthImagePoint(const Vrui::Point& imagePoint) const
	{
	/* Offset the image point to depth image space: */
	CPoint dip=CPoint(CPoint::Scalar(imagePoint[0])+depthImageOffset,CPoint::Scalar(imagePoint[1]),CPoint::Scalar(0));
	
	/* Apply lens distortion correction if necessary: */
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Distort the depth image point: */
		IntrinsicParameters::Point2 ddip=intrinsicParameters.distortDepthPixel(IntrinsicParameters::Point2(IntrinsicParameters::Scalar(dip[0]),IntrinsicParameters::Scalar(dip[1])));
		dip[0]=CPoint::Scalar(ddip[0]);
		dip[1]=CPoint::Scalar(ddip[1]);
		}
	
	return dip;
	}

float RawKinectViewer::getDepthImagePixel(const Offset& pixel) const
	{
	/* Calculate the pixel's linear index: */
	unsigned int index=pixel[1]*depthFrameSize[0]+pixel[0];
	
	/* Get the pixel's uncorrected depth value: */
	float result=-1.0f;
	if(averageFrameValid)
		{
		/* Get the average depth value of the selected pixel if it is valid: */
		if(averageFrameForeground[index]>=float(averageNumFrames)*0.5f)
			result=averageFrameDepth[index]/averageFrameForeground[index];
		}
	else
		{
		/* Get the currently locked depth value of the selected pixel if it is valid: */
		const Kinect::FrameBuffer& fb=depthFrames.getLockedValue();
		const DepthPixel* depthImage=fb.getData<DepthPixel>();
		if(depthImage[index]!=Kinect::FrameSource::invalidDepth)
			result=float(depthImage[index]);
		}
	
	/* If the result is valid and there is depth correction, apply it: */
	if(result!=-1.0f&&depthCorrection!=0)
		result=depthCorrection[index].correct(result);
	
	return result;
	}

RawKinectViewer::CPoint RawKinectViewer::getDepthImagePoint(const RawKinectViewer::Offset& pixel) const
	{
	/* Convert pixel index to depth image space with the given pixel's depth value: */
	CPoint dip((CPoint::Scalar)(pixel[0])+CPoint::Scalar(0.5),(CPoint::Scalar)(pixel[1])+CPoint::Scalar(0.5),CPoint::Scalar(getDepthImagePixel(pixel)));
	
	/* Apply lens distortion correction if necessary: */
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Undistort the depth image point: */
		IntrinsicParameters::Point2 udip=intrinsicParameters.undistortDepthPixel(IntrinsicParameters::Point2(IntrinsicParameters::Scalar(dip[0]),IntrinsicParameters::Scalar(dip[1])));
		dip[0]=CPoint::Scalar(udip[0]);
		dip[1]=CPoint::Scalar(udip[1]);
		}
	
	/* Transform the point from depth image space to image-plane space: */
	dip[0]-=depthImageOffset;
	
	return dip;
	}

RawKinectViewer::CPoint RawKinectViewer::getDepthImagePoint(const Vrui::Point& imagePoint) const
	{
	/* Transform the image-plane point to depth image space: */
	CPoint dip=CPoint(CPoint::Scalar(imagePoint[0])+depthImageOffset,CPoint::Scalar(imagePoint[1]),CPoint::Scalar(0));
	
	/* Apply lens distortion correction if necessary: */
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Distort the depth image point: */
		IntrinsicParameters::Point2 ddip=intrinsicParameters.distortDepthPixel(IntrinsicParameters::Point2(IntrinsicParameters::Scalar(dip[0]),IntrinsicParameters::Scalar(dip[1])));
		dip[0]=CPoint::Scalar(ddip[0]);
		dip[1]=CPoint::Scalar(ddip[1]);
		}
	
	/* Check if the depth image point is inside the depth image: */
	if(dip[0]>=CPoint::Scalar(0)&&dip[0]<CPoint::Scalar(depthFrameSize[0])&&
	   dip[1]>=CPoint::Scalar(0)&&dip[1]<CPoint::Scalar(depthFrameSize[1]))
		{
		/* Calculate the depth image point's pixel coordinates: */
		Offset diPixel((int)(dip[0]),(int)(dip[1]));
		
		/* Create the result point in undistorted depth image space: */
		CPoint result;
		result[0]=CPoint::Scalar(imagePoint[0])+depthImageOffset;
		result[1]=CPoint::Scalar(imagePoint[1]);
		result[2]=CPoint::Scalar(getDepthImagePixel(diPixel));
		
		return result;
		}
	else
		{
		/* Return some invalid point: */
		return CPoint(0,0,-1);
		}
	}

void RawKinectViewer::registerColorCallback(RawKinectViewer::FrameStreamingCallback* newCallback)
	{
	Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
	colorFrameCallbacks.push_back(newCallback);
	}

void RawKinectViewer::unregisterColorCallback(RawKinectViewer::FrameStreamingCallback* callback)
	{
	Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
	
	/* Find the callback in the list: */
	for(std::vector<FrameStreamingCallback*>::iterator cbIt=colorFrameCallbacks.begin();cbIt!=colorFrameCallbacks.end();++cbIt)
		if(*cbIt==callback)
			{
			*cbIt=colorFrameCallbacks.back();
			colorFrameCallbacks.pop_back();
			break;
			}
	}

void RawKinectViewer::registerDepthCallback(RawKinectViewer::FrameStreamingCallback* newCallback)
	{
	Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
	depthFrameCallbacks.push_back(newCallback);
	}

void RawKinectViewer::unregisterDepthCallback(RawKinectViewer::FrameStreamingCallback* callback)
	{
	Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
	
	/* Find the callback in the list: */
	for(std::vector<FrameStreamingCallback*>::iterator cbIt=depthFrameCallbacks.begin();cbIt!=depthFrameCallbacks.end();++cbIt)
		if(*cbIt==callback)
			{
			*cbIt=depthFrameCallbacks.back();
			depthFrameCallbacks.pop_back();
			break;
			}
	}

void RawKinectViewer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		#if 0
		
		/* Normalize the color frame: */
		Kinect::FrameBuffer normalizedFrame(frameBuffer.getSize(),frameBuffer.getSize().volume()*sizeof(ColorPixel));
		ColorPixel* nfPtr=normalizedFrame.getData<ColorPixel>();
		ColorPixel* nfEnd=nfPtr+frameBuffer.getSize().volume();
		const ColorPixel* fPtr=frameBuffer.getData<ColorPixel>();
		for(;nfPtr!=nfEnd;++nfPtr,++fPtr)
			{
			#if 1
			/* Divide all color components by the largest component: */
			unsigned int max=fPtr->rgb[0];
			for(int i=1;i<3;++i)
				if(max<fPtr->rgb[i])
					max=fPtr->rgb[i];
			for(int i=0;i<3;++i)
				nfPtr->rgb[i]=ColorComponent(((unsigned int)fPtr->rgb[i]*256U)/(max+1));
			#endif
			
			#if 0
			
			/* Classify the unprocessed color: */
			if(fPtr->rgb[0]<32U&&fPtr->rgb[1]<32U&&fPtr->rgb[2]<32U)
				nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=255U;
			else
				nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=0U;
			
			#elif 0
			
			/* Classify the normalized color: */
			if(nfPtr->rgb[0]>=240U&&nfPtr->rgb[1]<80U&&nfPtr->rgb[2]>=80U)
				nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=255U;
			else
				nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=0U;
			
			#endif
			}
		colorFrames.postNewValue(normalizedFrame);
		
		#else
		
		if(backgroundCaptureNumFrames>0)
			{
			/* Enter the new color frame into the background removal buffer: */
			Kinect::FrameSource::ColorPixel* bPtr=colorBackground;
			Kinect::FrameSource::ColorPixel* bEnd=bPtr+colorFrameSize.volume()*2;
			const Kinect::FrameSource::ColorPixel* fPtr=frameBuffer.getData<ColorPixel>();
			for(;bPtr!=bEnd;bPtr+=2,++fPtr)
				for(int i=0;i<3;++i)
					{
					if(bPtr[0][i]>(*fPtr)[i])
						bPtr[0][i]=(*fPtr)[i];
					if(bPtr[1][i]<(*fPtr)[i])
						bPtr[1][i]=(*fPtr)[i];
					}
			
			--backgroundCaptureNumFrames;
			}
		
		if(colorBackground!=0)
			{
			/* Remove background pixels from the new color frame: */
			Kinect::FrameBuffer backgroundRemovedFrame(frameBuffer.getSize(),frameBuffer.getSize().volume()*sizeof(ColorPixel));
			Kinect::FrameSource::ColorPixel* bfPtr=backgroundRemovedFrame.getData<ColorPixel>();
			Kinect::FrameSource::ColorPixel* bfEnd=bfPtr+frameBuffer.getSize().volume();
			Kinect::FrameSource::ColorPixel* bPtr=colorBackground;
			const Kinect::FrameSource::ColorPixel* fPtr=frameBuffer.getData<ColorPixel>();
			for(;bfPtr!=bfEnd;++bfPtr,bPtr+=2,++fPtr)
				{
				bool isBackground=true;
				for(int i=0;i<3&&isBackground;++i)
					isBackground=bPtr[0][i]<=(*fPtr)[i]&&(*fPtr)[i]<=bPtr[1][i];
				
				if(isBackground)
					(*bfPtr)[2]=(*bfPtr)[1]=(*bfPtr)[0]=0;
				else
					{
					for(int i=0;i<3;++i)
						(*bfPtr)[i]=(*fPtr)[i];
					}
				}
			
			/* Post the new frame into the color frame triple buffer: */
			colorFrames.postNewValue(backgroundRemovedFrame);
			}
		else
			{
			/* Post the new frame into the color frame triple buffer: */
			colorFrames.postNewValue(frameBuffer);
			}
		
		
		#endif
		
		/* Call all color streaming callbacks: */
		{
		Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
		for(std::vector<FrameStreamingCallback*>::iterator cbIt=colorFrameCallbacks.begin();cbIt!=colorFrameCallbacks.end();++cbIt)
			(**cbIt)(frameBuffer);
		}
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		/* Post the new frame into the depth frame triple buffer: */
		depthFrames.postNewValue(frameBuffer);
		
		/* Call all depth streaming callbacks: */
		{
		Threads::Spinlock::Lock frameCallbacksLock(frameCallbacksMutex);
		for(std::vector<FrameStreamingCallback*>::iterator cbIt=depthFrameCallbacks.begin();cbIt!=depthFrameCallbacks.end();++cbIt)
			(**cbIt)(frameBuffer);
		}
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::requestAverageFrame(RawKinectViewer::AverageFrameReadyCallback* callback)
	{
	/* Check if there already is an average frame: */
	if(averageFrameValid)
		{
		/* Just call the callback immediately and forget about it: */
		if(callback!=0)
			{
			(*callback)(0);
			delete callback;
			}
		}
	else
		{
		/* Check if there is already an average frame capture underway: */
		if(averageFrameCounter==0)
			{
			/* Start averaging frames: */
			float* afdPtr=averageFrameDepth;
			float* afdEnd=afdPtr+depthFrameSize.volume();
			float* affPtr=averageFrameForeground;
			for(;afdPtr!=afdEnd;++afdPtr,++affPtr)
				{
				*afdPtr=0.0f;
				*affPtr=0.0f;
				}
			averageFrameCounter=averageNumFrames;
			
			/* Show a progress dialog: */
			Vrui::popupPrimaryWidget(averageDepthFrameDialog);
			}
		
		/* Add the callback to the callback list: */
		if(callback!=0)
			averageFrameReadyCallbacks.push_back(callback);
		}
	}

void RawKinectViewer::locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData)
	{
	/* Transform the tools' origin to distorted depth image space: */
	CPoint dip=calcDepthImagePoint(cbData->currentTransformation.getOrigin());
	
	/* Check if the depth image point is inside the depth image: */
	if(dip[0]>=CPoint::Scalar(0)&&dip[0]<CPoint::Scalar(depthFrameSize[0])&&
	   dip[1]>=CPoint::Scalar(0)&&dip[1]<CPoint::Scalar(depthFrameSize[1]))
		{
		/* Select the pixel under the locator for tracking: */
		selectedPixel=Offset((unsigned int)(dip[0]),(unsigned int)(dip[1]));
		
		/* Start the selected pixel's EKG: */
		selectedPixelCurrentIndex=0;
		const DepthPixel* dfPtr=depthFrames.getLockedValue().getData<DepthPixel>();
		selectedPixelPulse[0]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
		for(int i=1;i<128;++i)
			selectedPixelPulse[i]=0;
		}
	else
		{
		/* Select an invalid pixel: */
		selectedPixel=Offset(-1,-1);
		}
	}

void RawKinectViewer::minDepthSliderValueChangedCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set the minimum depth value: */
	float newMin=float(cbData->value);
	if(newMin<depthValueRange[1])
		depthValueRange[0]=newMin;
	else
		{
		depthValueRange[0]=depthValueRange[1]-1.0f;
		cbData->slider->setValue(depthValueRange[0]);
		}
	}

void RawKinectViewer::maxDepthSliderValueChangedCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set the maximum depth value: */
	float newMax=float(cbData->value);
	if(newMax>depthValueRange[0])
		depthValueRange[1]=newMax;
	else
		{
		depthValueRange[1]=depthValueRange[0]+1.0f;
		cbData->slider->setValue(depthValueRange[1]);
		}
	}

GLMotif::PopupWindow* RawKinectViewer::createDepthRangeDialog(void)
	{
	/* Create the depth range dialog window: */
	const GLMotif::StyleSheet& ss=*Vrui::getUiStyleSheet();
	GLMotif::PopupWindow* depthRangeDialogPopup=new GLMotif::PopupWindow("DepthRangeDialogPopup",Vrui::getWidgetManager(),"Depth Range");
	
	GLMotif::RowColumn* depthRangeDialog=new GLMotif::RowColumn("DepthRangeDialog",depthRangeDialogPopup,false);
	depthRangeDialog->setOrientation(GLMotif::RowColumn::VERTICAL);
	depthRangeDialog->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	depthRangeDialog->setNumMinorWidgets(2);
	
	new GLMotif::Label("MinDepthLabel",depthRangeDialog,"Min Depth");
	
	GLMotif::TextFieldSlider* minDepthSlider=new GLMotif::TextFieldSlider("MinDepthSlider",depthRangeDialog,8,ss.fontHeight*10.0f);
	minDepthSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	minDepthSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	minDepthSlider->setValueRange(0,Kinect::FrameSource::invalidDepth-1,1);
	minDepthSlider->setValue(depthValueRange[0]);
	minDepthSlider->getValueChangedCallbacks().add(this,&RawKinectViewer::minDepthSliderValueChangedCallback);
	
	new GLMotif::Label("MaxDepthLabel",depthRangeDialog,"Max Depth");
	
	GLMotif::TextFieldSlider* maxDepthSlider=new GLMotif::TextFieldSlider("MaxDepthSlider",depthRangeDialog,8,ss.fontHeight*10.0f);
	maxDepthSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	maxDepthSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	maxDepthSlider->setValueRange(0,Kinect::FrameSource::invalidDepth-1,1);
	maxDepthSlider->setValue(depthValueRange[1]);
	maxDepthSlider->getValueChangedCallbacks().add(this,&RawKinectViewer::maxDepthSliderValueChangedCallback);
	
	depthRangeDialog->manageChild();
	
	return depthRangeDialogPopup;
	}

void RawKinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	/* Capture five seconds worth of background frames: */
	if(colorBackground==0)
		colorBackground=new Kinect::FrameSource::ColorPixel[colorFrameSize.volume()*2];
	Kinect::FrameSource::ColorPixel* bPtr=colorBackground;
	Kinect::FrameSource::ColorPixel* bEnd=bPtr+colorFrameSize.volume()*2;
	for(;bPtr!=bEnd;bPtr+=2)
		{
		/* Initialize the pixel's background interval to "empty": */
		bPtr[0][2]=bPtr[0][1]=bPtr[0][0]=255U;
		bPtr[1][2]=bPtr[1][1]=bPtr[1][0]=0U;
		}
	backgroundCaptureNumFrames=150;
	
	camera->captureBackground(150,true);
	}

void RawKinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	camera->setRemoveBackground(cbData->set);
	if(!cbData->set)
		{
		delete[] colorBackground;
		colorBackground=0;
		}
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(camera->getRemoveBackground());
	}

void RawKinectViewer::showDepthRangeDialogCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Show or hide the depth range dialog: */
	if(cbData->set)
		Vrui::popupPrimaryWidget(depthRangeDialog);
	else
		Vrui::popdownPrimaryWidget(depthRangeDialog);
	}

void RawKinectViewer::averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	showAverageFrame=cbData->set;
	if(cbData->set)
		{
		/* Request a new average frame: */
		requestAverageFrame(0);
		}
	else
		{
		/* Invalidate the current average frame: */
		averageFrameValid=false;
		depthPlaneValid=false;
		}
	}

void RawKinectViewer::saveAverageFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Open the average frame file: */
		IO::FilePtr frameFile(cbData->selectedDirectory->openFile(cbData->selectedFileName,IO::File::WriteOnly));
		
		/* Write the averaged frame: */
		frameFile->write<Misc::UInt32,unsigned int>(depthFrameSize.getComponents(),2);
		float cutoff=float(averageNumFrames)*0.5f;
		float* afdPtr=averageFrameDepth;
		float* afdEnd=afdPtr+depthFrameSize.volume();
		float* affPtr=averageFrameForeground;
		if(depthCorrection!=0)
			{
			const PixelCorrection* dcPtr=depthCorrection;
			for(;afdPtr!=afdEnd;++afdPtr,++affPtr,++dcPtr)
				frameFile->write<Misc::Float32>(*affPtr>=cutoff?dcPtr->correct((*afdPtr)/(*affPtr)):2047.0f);
			}
		else
			{
			for(;afdPtr!=afdEnd;++afdPtr,++affPtr)
				frameFile->write<Misc::Float32>(*affPtr>=cutoff?(*afdPtr)/(*affPtr):2047.0f);
			}
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...",Misc::stringPrintf("Could not write depth frame file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what()).c_str());
		}
	
	/* Destroy the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void RawKinectViewer::saveAverageFrameCallback(Misc::CallbackData* cbData)
	{
	if(!averageFrameValid)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...","No valid average depth frame to save");
		return;
		}
	
	try
		{
		/* Create a uniquely-named depth image file in the current directory: */
		std::string depthFrameFileName=IO::Directory::getCurrent()->createNumberedFileName("DepthFrame.dat",4);
		
		/* Create a file selection dialog to select an alternative depth frame file name: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveAverageFrameDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Average Depth Frame...",IO::Directory::getCurrent(),depthFrameFileName.c_str(),".dat"));
		saveAverageFrameDialog->getOKCallbacks().add(this,&RawKinectViewer::saveAverageFrameOKCallback);
		saveAverageFrameDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(saveAverageFrameDialog.releaseTarget());
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...",Misc::stringPrintf("Could not save average depth frame due to exception %s",err.what()).c_str());
		}
	}

void RawKinectViewer::saveColorFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		#if 1 // Quick shortcut
		
		/* Convert the current color frame into an RGB image: */
		Images::RGBImage colorImage(colorFrameSize);
		Images::RGBImage::Color* dPtr=colorImage.modifyPixels();
		Images::RGBImage::Color* dEnd=dPtr+colorFrameSize.volume();
		const ColorPixel* sPtr=colorFrames.getLockedValue().getData<ColorPixel>();
		for(;dPtr!=dEnd;++dPtr,++sPtr)
			for(int i=0;i<3;++i)
				(*dPtr)[i]=sPtr->components[i];
		
		/* Write the RGB image to the file: */
		Images::writeImageFile(colorImage,cbData->getSelectedPath().c_str());
		
		#else
		
		/* Write the current depth frame to a raw image: */
		IO::FilePtr depthFile=cbData->selectedDirectory->openFile(cbData->selectedFileName,IO::File::WriteOnly);
		depthFile->setEndianness(Misc::LittleEndian);
		depthFile->write(depthFrames.getLockedValue().getData<DepthPixel>(),depthFrameSize.volume());
		
		#endif
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Color Frame...",Misc::stringPrintf("Could not write color frame file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what()).c_str());
		}
	
	/* Destroy the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void RawKinectViewer::saveColorFrameCallback(Misc::CallbackData* cbData)
	{
	try
		{
		/* Create a uniquely-named color image file in the current directory: */
		std::string colorFrameFileName=IO::Directory::getCurrent()->createNumberedFileName("ColorFrame.png",4);
		
		/* Create a file selection dialog to select an alternative color frame file name: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveColorFrameDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Color Frame...",IO::Directory::getCurrent(),colorFrameFileName.c_str(),".png"));
		saveColorFrameDialog->getOKCallbacks().add(this,&RawKinectViewer::saveColorFrameOKCallback);
		saveColorFrameDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(saveColorFrameDialog.releaseTarget());
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Color Frame...",Misc::stringPrintf("Could not save color frame due to exception %s",err.what()).c_str());
		}
	}

GLMotif::PopupMenu* RawKinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Raw Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to capture a background frame: */
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",mainMenu,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&RawKinectViewer::captureBackgroundCallback);
	
	/* Create a toggle button to enable/disable background removal: */
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",mainMenu,"Remove Background");
	removeBackgroundToggle->setToggle(camera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&RawKinectViewer::removeBackgroundCallback);
	
	/* Create a toggle button to show the depth range dialog: */
	GLMotif::ToggleButton* showDepthRangeDialogButton=new GLMotif::ToggleButton("ShowDepthRangeDialogButton",mainMenu,"Set Depth Range...");
	showDepthRangeDialogButton->getValueChangedCallbacks().add(this,&RawKinectViewer::showDepthRangeDialogCallback);
	
	/* Create a toggle button to calculate and show an averaged depth frame: */
	GLMotif::ToggleButton* averageFramesButton=new GLMotif::ToggleButton("AverageFramesButton",mainMenu,"Average Frames");
	averageFramesButton->getValueChangedCallbacks().add(this,&RawKinectViewer::averageFramesCallback);
	
	/* Create a button to save the current averaged depth frame: */
	GLMotif::Button* saveAverageFrameButton=new GLMotif::Button("SaveAverageFrameButton",mainMenu,"Save Average Frame");
	saveAverageFrameButton->getSelectCallbacks().add(this,&RawKinectViewer::saveAverageFrameCallback);
	
	/* Create a button to save the current color frame: */
	GLMotif::Button* saveColorFrameButton=new GLMotif::Button("SaveColorFrameButton",mainMenu,"Save Color Frame");
	saveColorFrameButton->getSelectCallbacks().add(this,&RawKinectViewer::saveColorFrameCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

GLMotif::PopupWindow* RawKinectViewer::createAverageDepthFrameDialog(void)
	{
	/* Create the average depth frame dialog window: */
	GLMotif::PopupWindow* averageDepthFrameDialogPopup=new GLMotif::PopupWindow("AverageDepthFrameDialogPopup",Vrui::getWidgetManager(),"RawKinectViewer");
	
	new GLMotif::Label("AverageDepthFrameLabel",averageDepthFrameDialogPopup,"Capturing average depth frame...");
	
	return averageDepthFrameDialogPopup;
	}

namespace {

/****************
Helper functions:
****************/

inline bool isUInt(const char* string)
	{
	/* Empty string is not an unsigned integer: */
	if(*string=='\0')
		return false;
	
	bool result=true;
	while(*string!='\0'&&result)
		{
		result=isdigit(*string);
		++string;
		}
	
	return result;
	}

}

RawKinectViewer::RawKinectViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 camera(0),
	 backgroundCaptureNumFrames(0),colorBackground(0),
	 colorFrameVersion(0),
	 depthCorrection(0),depthPlaneDistMax(10.0),depthFrameVersion(0),
	 paused(false),
	 averageNumFrames(150),averageFrameCounter(0),
	 averageFrameDepth(0),averageFrameForeground(0),
	 averageFrameValid(false),showAverageFrame(false),
	 depthPlaneValid(false),
	 depthRangeDialog(0),mainMenu(0),averageDepthFrameDialog(0)
	{
	/*********************************************************************
	Register the custom tool classes with the Vrui tool manager:
	*********************************************************************/
	
	PauseTool::initClass(*Vrui::getToolManager());
	MeasurementTool::initClass(*Vrui::getToolManager());
	TiePointTool::initClass(*Vrui::getToolManager());
	LineTool::initClass(*Vrui::getToolManager());
	DepthCorrectionTool::initClass(*Vrui::getToolManager());
	GridTool::initClass(*Vrui::getToolManager());
	PlaneTool::initClass(*Vrui::getToolManager());
	PointPlaneTool::initClass(*Vrui::getToolManager());
	CalibrationCheckTool::initClass(*Vrui::getToolManager());
	
	/* Parse the command line: */
	bool printHelp=false;
	int cameraIndex=0; // Use first 3D camera device on USB bus
	Kinect::Camera::FrameSize selectedColorFrameSize=Kinect::Camera::FS_640_480;
	Kinect::Camera::FrameSize selectedDepthFrameSize=Kinect::Camera::FS_640_480;
	bool compressDepthFrames=false;
	bool depthValueRangeRequested=false;
	depthValueRange[0]=0.0f;
	depthValueRange[1]=float(Kinect::FrameSource::invalidDepth-1);
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"h")==0)
				printHelp=true;
			else if(strcasecmp(argv[i]+1,"high")==0)
				{
				/* Select high-resolution color image for Kinect v1: */
				selectedColorFrameSize=Kinect::Camera::FS_1280_1024;
				// selectedDepthFrameSize=Kinect::Camera::FS_1280_1024;
				}
			else if(strcasecmp(argv[i]+1,"compress")==0)
				{
				/* Select depth frame compression for Kinect v1: */
				compressDepthFrames=true;
				}
			else if(strcasecmp(argv[i]+1,"gridSize")==0)
				{
				GridTool::setGridSize(GridTool::Size(atoi(argv[i+1]),atoi(argv[i+2])));
				i+=2;
				}
			else if(strcasecmp(argv[i]+1,"tileSize")==0)
				{
				GridTool::setTileSize(GridTool::DSize(atof(argv[i+1]),atof(argv[i+2])));
				i+=2;
				}
			else if(strcasecmp(argv[i]+1,"depthRange")==0)
				{
				depthValueRangeRequested=true;
				for(int j=0;j<2;++j)
					depthValueRange[j]=float(atof(argv[i+1+j]));
				i+=2;
				}
			else
				{
				std::cerr<<"Ignoring unrecognized command line parameter "<<argv[i]<<std::endl;
				printHelp=true;
				}
			}
		else if(isUInt(argv[i]))
			cameraIndex=atoi(argv[i]);
		else
			{
			std::cerr<<"Ignoring unrecognized command line argument "<<argv[i]<<std::endl;
			printHelp=true;
			}
		}
	
	if(printHelp)
		{
		std::cout<<"Usage: RawKinectViewer [option 1] ... [option n] [<camera index>]"<<std::endl;
		std::cout<<"  <camera index>"<<std::endl;
		std::cout<<"     Selects the local 3D camera of the given index (0: first camera on USB bus)"<<std::endl;
		std::cout<<"     Default: 0"<<std::endl;
		std::cout<<"  Options:"<<std::endl;
		std::cout<<"  -h"<<std::endl;
		std::cout<<"     Prints this help message"<<std::endl;
		std::cout<<"  -high"<<std::endl;
		std::cout<<"    Sets color frame size for the selected first-generation Kinect camera to 1280x1024 @ 15Hz"<<std::endl;
		std::cout<<"  -compress"<<std::endl;
		std::cout<<"     Requests compressed depth frames from the selected first-generation Kinect camera"<<std::endl;
		std::cout<<"  -gridSize <grid width> <grid height>"<<std::endl;
		std::cout<<"     Sets the number of tiles of the semi-transparent calibration grid"<<std::endl;
		std::cout<<"     Default: 7 5"<<std::endl;
		std::cout<<"  -tileSize <tile width> <tile height>"<<std::endl;
		std::cout<<"     Sets the size of each tile of the semi-transparent calibration grid"<<std::endl;
		std::cout<<"     Default: 3.5 3.5 (assumed to be inches)"<<std::endl;
		std::cout<<"  -depthRange <min depth> <max depth>"<<std::endl;
		std::cout<<"     Sets the range of depth values mapped to the full color range"<<std::endl;
		std::cout<<"     Default: 300 1100"<<std::endl;
		}
	
	/* Connect to the 3D camera of the given index: */
	camera=Kinect::openDirectFrameSource(cameraIndex,true);
	std::cout<<"RawKinectViewer: Connected to 3D camera with serial number "<<camera->getSerialNumber()<<std::endl;
	
	/* Check if it's a first-generation Kinect to apply type-specific settings: */
	Kinect::Camera* kinectV1=dynamic_cast<Kinect::Camera*>(camera);
	if(kinectV1!=0)
		{
		/* Set the color camera's frame size: */
		kinectV1->setFrameSize(Kinect::FrameSource::COLOR,selectedColorFrameSize);
		kinectV1->setFrameSize(Kinect::FrameSource::DEPTH,selectedDepthFrameSize);
		
		/* Set depth frame compression: */
		kinectV1->setCompressDepthFrames(compressDepthFrames);
		}
	
	/* Get the cameras' actual frame sizes: */
	colorFrameSize=camera->getActualFrameSize(Kinect::FrameSource::COLOR);
	depthFrameSize=camera->getActualFrameSize(Kinect::FrameSource::DEPTH);
	
	if(!depthValueRangeRequested)
		{
		/* Get the camera's raw depth value range: */
		Kinect::FrameSource::DepthRange dr=camera->getDepthRange();
		depthValueRange[0]=float(dr.getMin());
		depthValueRange[1]=float(dr.getMax());
		}
	
	/* Get the camera's depth correction parameters: */
	Kinect::FrameSource::DepthCorrection* dc=camera->getDepthCorrectionParameters();
	
	if(dc!=0)
		{
		/* Evaluate the camera's depth correction parameters into a per-pixel offset array: */
		depthCorrection=dc->getPixelCorrection(depthFrameSize);
		delete dc;
		}
	else
		depthCorrection=0;
	
	/* Get the camera's intrinsic parameters: */
	intrinsicParameters=camera->getIntrinsicParameters();
	
	/* Calculate the depth image offset and color image scale: */
	double depthSize=Math::sqrt(Math::sqr(double(depthFrameSize[0]))+Math::sqr(double(depthFrameSize[1])));
	double colorSize=Math::sqrt(Math::sqr(double(colorFrameSize[0]))+Math::sqr(double(colorFrameSize[1])));
	if(intrinsicParameters.depthLensDistortion.isIdentity())
		{
		depthImageOffset=double(depthFrameSize[0]);
		colorImageScale=depthSize/colorSize;
		}
	else
		{
		depthImageOffset=double((depthFrameSize[0]*5U)/4U);
		colorImageScale=depthSize*1.25/colorSize;
		}
	
	/* Allocate the average depth frame buffer: */
	averageFrameDepth=new float[depthFrameSize.volume()];
	averageFrameForeground=new float[depthFrameSize.volume()];
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Create the dialog windows: */
	depthRangeDialog=createDepthRangeDialog();
	averageDepthFrameDialog=createAverageDepthFrameDialog();
	
	/* Start streaming: */
	camera->startStreaming(Misc::createFunctionCall(this,&RawKinectViewer::colorStreamingCallback),Misc::createFunctionCall(this,&RawKinectViewer::depthStreamingCallback));
	
	/* Select an invalid pixel: */
	selectedPixel[0]=selectedPixel[1]=~0x0U;
	}

RawKinectViewer::~RawKinectViewer(void)
	{
	delete mainMenu;
	delete depthRangeDialog;
	delete averageDepthFrameDialog;
	
	/* Stop streaming: */
	camera->stopStreaming();
	
	delete[] averageFrameDepth;
	delete[] averageFrameForeground;
	delete[] colorBackground;
	
	/* Disconnect from the Kinect camera device: */
	delete camera;
	}

void RawKinectViewer::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Call the base class method: */
	Vrui::Application::toolCreationCallback(cbData);
	
	/* Check if the new tool is a locator tool: */
	Vrui::LocatorTool* lt=dynamic_cast<Vrui::LocatorTool*>(cbData->tool);
	if(lt!=0)
		{
		/* Register callbacks with the locator tool: */
		lt->getButtonPressCallbacks().add(this,&RawKinectViewer::locatorButtonPressCallback);
		}
	}

void RawKinectViewer::frame(void)
	{
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		++depthFrameVersion;
		
		if(selectedPixel[0]>=0&&selectedPixel[1]>=0)
			{
			/* Update the selected pixel's EKG: */
			++selectedPixelCurrentIndex;
			if(selectedPixelCurrentIndex==128)
				selectedPixelCurrentIndex=0;
			const DepthPixel* dfPtr=depthFrames.getLockedValue().getData<DepthPixel>();
			selectedPixelPulse[selectedPixelCurrentIndex]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
			}
		
		if(averageFrameCounter>0)
			{
			/* Accumulate the new depth frame into the averaging buffer: */
			float* afdPtr=averageFrameDepth;
			float* afdEnd=afdPtr+depthFrameSize.volume();
			float* affPtr=averageFrameForeground;
			const DepthPixel* dfPtr=depthFrames.getLockedValue().getData<DepthPixel>();
			for(;afdPtr!=afdEnd;++afdPtr,++affPtr,++dfPtr)
				if(*dfPtr!=Kinect::FrameSource::invalidDepth)
					{
					*afdPtr+=float(*dfPtr);
					*affPtr+=1.0f;
					}
			--averageFrameCounter;
			if(averageFrameCounter==0)
				{
				/* Mark the average frame buffer as valid: */
				averageFrameValid=true;
				
				/* Call all registered callbacks: */
				for(std::vector<AverageFrameReadyCallback*>::iterator afrcIt=averageFrameReadyCallbacks.begin();afrcIt!=averageFrameReadyCallbacks.end();++afrcIt)
					{
					(**afrcIt)(0);
					delete *afrcIt;
					}
				averageFrameReadyCallbacks.clear();
				
				/* Hide the progress dialog: */
				Vrui::popdownPrimaryWidget(averageDepthFrameDialog);
				
				/* Invalidate the average depth frame immediately if it wasn't requested directly by the user: */
				averageFrameValid=showAverageFrame;
				}
			}
		}
	}

void RawKinectViewer::display(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_TEXTURE_BIT);
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	glColor3f(1.0f,1.0f,1.0f);
	
	/* Bind the depth texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(showAverageFrame&&averageFrameValid)
		{
		/* Convert the averaged depth image to RGB: */
		GLubyte* byteFrame=new GLubyte[depthFrameSize.volume()*3];
		GLubyte* bfPtr=byteFrame;
		const float* afdPtr=averageFrameDepth;
		const float* affPtr=averageFrameForeground;
		float foregroundCutoff=float(averageNumFrames)*0.5f;
		if(depthCorrection!=0)
			{
			const PixelCorrection* dcPtr=depthCorrection;
			for(unsigned int y=0;y<depthFrameSize[1];++y)
				for(unsigned int x=0;x<depthFrameSize[0];++x,bfPtr+=3,++afdPtr,++affPtr,++dcPtr)
					{
					if(*affPtr>=foregroundCutoff)
						{
						float d=dcPtr->correct((*afdPtr)/(*affPtr));
						mapDepth(Offset(x,y),d,bfPtr);
						}
					else
						bfPtr[2]=bfPtr[1]=bfPtr[0]=GLubyte(0);
					}
			}
		else
			{
			for(unsigned int y=0;y<depthFrameSize[1];++y)
				for(unsigned int x=0;x<depthFrameSize[0];++x,bfPtr+=3,++afdPtr,++affPtr)
					{
					if(*affPtr>=foregroundCutoff)
						{
						float d=(*afdPtr)/(*affPtr);
						mapDepth(Offset(x,y),d,bfPtr);
						}
					else
						bfPtr[2]=bfPtr[1]=bfPtr[0]=GLubyte(0);
					}
			}
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the depth texture image: */
		glTexSubImage2D(GL_TEXTURE_2D,0,0,0,depthFrameSize[0],depthFrameSize[1],GL_RGB,GL_UNSIGNED_BYTE,byteFrame);
		
		delete[] byteFrame;
		}
	else
		{
		if(dataItem->depthFrameVersion!=depthFrameVersion)
			{
			/* Upload the depth frame into the texture object: */
			const Kinect::FrameBuffer& depthFrame=depthFrames.getLockedValue();
			const GLushort* framePtr=depthFrame.getData<GLushort>();
			
			/* Convert the depth image to unsigned byte: */
			GLubyte* byteFrame=new GLubyte[depthFrameSize.volume()*3];
			const GLushort* fPtr=framePtr;
			GLubyte* bfPtr=byteFrame;
			if(depthCorrection!=0)
				{
				const PixelCorrection* dcPtr=depthCorrection;
				for(unsigned int y=0;y<depthFrameSize[1];++y)
					for(unsigned int x=0;x<depthFrameSize[0];++x,++fPtr,++dcPtr,bfPtr+=3)
						{
						if(*fPtr!=Kinect::FrameSource::invalidDepth)
							{
							float d=dcPtr->correct(*fPtr);
							mapDepth(Offset(x,y),d,bfPtr);
							}
						else
							bfPtr[2]=bfPtr[1]=bfPtr[0]=GLubyte(0);
						}
				}
			else
				{
				for(unsigned int y=0;y<depthFrameSize[1];++y)
					for(unsigned int x=0;x<depthFrameSize[0];++x,++fPtr,bfPtr+=3)
						{
						if(*fPtr!=Kinect::FrameSource::invalidDepth)
							mapDepth(Offset(x,y),*fPtr,bfPtr);
						else
							bfPtr[2]=bfPtr[1]=bfPtr[0]=GLubyte(0);
						}
				}
			
			/* Set up the texture parameters: */
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
			
			/* Upload the depth texture image: */
			glTexSubImage2D(GL_TEXTURE_2D,0,0,0,depthFrameSize[0],depthFrameSize[1],GL_RGB,GL_UNSIGNED_BYTE,byteFrame);
			
			delete[] byteFrame;
			
			/* Mark the cached depth frame as up-to-date: */
			dataItem->depthFrameVersion=depthFrameVersion;
			}
		}
	
	/* Draw the depth image: */
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Create a grid of undistorted pixel positions: */
		unsigned int gridSizeX=(depthFrameSize[0]+15U)/16U;
		unsigned int gridSizeY=(depthFrameSize[1]+15U)/16U;
		double scaleX=double(depthFrameSize[0])/double(gridSizeX);
		double scaleY=double(depthFrameSize[1])/double(gridSizeY);
		GLfloat texScaleX=GLfloat(depthFrameSize[0])/GLfloat(gridSizeX*dataItem->depthTextureSize[0]);
		GLfloat texScaleY=GLfloat(depthFrameSize[1])/GLfloat(gridSizeY*dataItem->depthTextureSize[1]);
		for(unsigned int y=1;y<=gridSizeY;++y)
			{
			glBegin(GL_QUAD_STRIP);
			for(unsigned int x=0;x<=gridSizeX;++x)
				{
				/* Calculate the quad vertex positions in distortion-corrected depth image space: */
				IntrinsicParameters::Point2 dp0(IntrinsicParameters::Scalar(double(x)*scaleX),IntrinsicParameters::Scalar(double(y-1)*scaleY));
				IntrinsicParameters::Point2 up0=intrinsicParameters.undistortDepthPixel(dp0);
				IntrinsicParameters::Point2 dp1(IntrinsicParameters::Scalar(double(x)*scaleX),IntrinsicParameters::Scalar(double(y)*scaleY));
				IntrinsicParameters::Point2 up1=intrinsicParameters.undistortDepthPixel(dp1);
				
				/* Draw the next quad: */
				glTexCoord2f(GLfloat(x)*texScaleX,GLfloat(y)*texScaleY);
				glVertex2d(up1[0]-depthImageOffset,up1[1]);
				glTexCoord2f(GLfloat(x)*texScaleX,GLfloat(y-1)*texScaleY);
				glVertex2d(up0[0]-depthImageOffset,up0[1]);
				}
			glEnd();
			}
		}
	else
		{
		/* Draw an undistorted depth image: */
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f,0.0f);
		glVertex2f(-GLfloat(depthFrameSize[0]),0.0f);
		glTexCoord2f(GLfloat(depthFrameSize[0])/GLfloat(dataItem->depthTextureSize[0]),0.0f);
		glVertex2f(0.0f,0.0f);
		glTexCoord2f(GLfloat(depthFrameSize[0])/GLfloat(dataItem->depthTextureSize[0]),GLfloat(depthFrameSize[1])/GLfloat(dataItem->depthTextureSize[1]));
		glVertex2f(0.0f,GLfloat(depthFrameSize[1]));
		glTexCoord2f(0.0f,GLfloat(depthFrameSize[1])/GLfloat(dataItem->depthTextureSize[1]));
		glVertex2f(-GLfloat(depthFrameSize[0]),GLfloat(depthFrameSize[1]));
		glEnd();
		}
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		const Kinect::FrameBuffer& colorFrame=colorFrames.getLockedValue();
		const GLubyte* framePtr=colorFrame.getData<GLubyte>();
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the color texture image: */
		glTexSubImage2D(GL_TEXTURE_2D,0,0,0,colorFrameSize[0],colorFrameSize[1],GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	
	if(!intrinsicParameters.depthLensDistortion.isIdentity())
		{
		/* Create a grid of undistorted pixel positions: */
		unsigned int gridSizeX=(colorFrameSize[0]+15U)/16U;
		unsigned int gridSizeY=(colorFrameSize[1]+15U)/16U;
		double scaleX=double(colorFrameSize[0])/double(gridSizeX);
		double scaleY=double(colorFrameSize[1])/double(gridSizeY);
		GLfloat texScaleX=GLfloat(colorFrameSize[0])/GLfloat(gridSizeX*dataItem->colorTextureSize[0]);
		GLfloat texScaleY=GLfloat(colorFrameSize[1])/GLfloat(gridSizeY*dataItem->colorTextureSize[1]);
		for(unsigned int y=1;y<=gridSizeY;++y)
			{
			glBegin(GL_QUAD_STRIP);
			for(unsigned int x=0;x<=gridSizeX;++x)
				{
				/* Calculate the quad vertex positions in distortion-corrected color image space: */
				IntrinsicParameters::Point2 dp0(IntrinsicParameters::Scalar(double(x)*scaleX),IntrinsicParameters::Scalar(double(y-1)*scaleY));
				IntrinsicParameters::Point2 up0=intrinsicParameters.undistortColorPixel(dp0);
				IntrinsicParameters::Point2 dp1(IntrinsicParameters::Scalar(double(x)*scaleX),IntrinsicParameters::Scalar(double(y)*scaleY));
				IntrinsicParameters::Point2 up1=intrinsicParameters.undistortColorPixel(dp1);
				
				/* Draw the next quad: */
				glTexCoord2f(GLfloat(x)*texScaleX,GLfloat(y)*texScaleY);
				glVertex2d(up1[0],up1[1]);
				glTexCoord2f(GLfloat(x)*texScaleX,GLfloat(y-1)*texScaleY);
				glVertex2d(up0[0],up0[1]);
				}
			glEnd();
			}
		}
	else
		{
		/* Draw an undistorted color image: */
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f,0.0f);
		glVertex2d(0.0,0.0);
		glTexCoord2f(GLfloat(colorFrameSize[0])/GLfloat(dataItem->colorTextureSize[0]),0.0f);
		glVertex2d(double(colorFrameSize[0])*colorImageScale,0.0);
		glTexCoord2f(GLfloat(colorFrameSize[0])/GLfloat(dataItem->colorTextureSize[0]),GLfloat(colorFrameSize[1])/GLfloat(dataItem->colorTextureSize[1]));
		glVertex2d(double(colorFrameSize[0])*colorImageScale,double(colorFrameSize[1])*colorImageScale);
		glTexCoord2f(0.0f,GLfloat(colorFrameSize[1])/GLfloat(dataItem->colorTextureSize[1]));
		glVertex2d(0.0,double(colorFrameSize[1])*colorImageScale);
		glEnd();
		}
	
	/* Protect the texture objects: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	if(selectedPixel[0]>=0&&selectedPixel[1]>=0)
		{
		/* Draw the selected pixel: */
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		
		CPoint dip=getDepthImagePoint(selectedPixel);
		glBegin(GL_LINES);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(dip[0]-5.0f,dip[1],0.1f);
		glVertex3f(dip[0]+5.0f,dip[1],0.1f);
		glVertex3f(dip[0],dip[1]-5.0f,0.1f);
		glVertex3f(dip[0],dip[1]+5.0f,0.1f);
		glEnd();
		
		/* Draw the selected pixel's EKG: */
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<128;++i)
			glVertex3f(GLfloat(i)*depthFrameSize[0]/128.0f-depthFrameSize[0],GLfloat(selectedPixelPulse[i])*0.25-512.0f,0.1f);
		glEnd();
		}
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void RawKinectViewer::resetNavigation(void)
	{
	/* Reset the navigation transformation: */
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(1024),Vrui::Vector(0,1,0));
	}

void RawKinectViewer::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Check for NPOTD texture support and initialize the padded texture sizes: */
	if(GLARBTextureNonPowerOfTwo::isSupported())
		{
		/* Initialize the extension: */
		GLARBTextureNonPowerOfTwo::initExtension();
		
		/* Use actual image sizes as texture sizes: */
		dataItem->colorTextureSize=colorFrameSize;
		dataItem->depthTextureSize=depthFrameSize;
		}
	else
		{
		/* Pad image sizes to the next-larger power of two: */
		for(int i=0;i<2;++i)
			{
			for(dataItem->colorTextureSize[i]=1;dataItem->colorTextureSize[i]<colorFrameSize[i];dataItem->colorTextureSize[i]<<=1)
				;
			for(dataItem->depthTextureSize[i]=1;dataItem->depthTextureSize[i]<depthFrameSize[i];dataItem->depthTextureSize[i]<<=1)
				;
			}
		}
	
	/* Prepare the depth texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,dataItem->depthTextureSize[0],dataItem->depthTextureSize[1],0,GL_RGB,GL_UNSIGNED_BYTE,0);
	
	/* Prepare the color texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,dataItem->colorTextureSize[0],dataItem->colorTextureSize[1],0,GL_RGB,GL_UNSIGNED_BYTE,0);
	
	/* Protect the texture images: */
	glBindTexture(GL_TEXTURE_2D,0);
	}

VRUI_APPLICATION_RUN(RawKinectViewer)
