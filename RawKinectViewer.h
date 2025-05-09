/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
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

#ifndef RAWKINECTVIEWER_INCLUDED
#define RAWKINECTVIEWER_INCLUDED

#include <Misc/FunctionCalls.h>
#include <Threads/Spinlock.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/FileSelectionDialog.h>
#include <Vrui/LocatorTool.h>
#include <Vrui/Application.h>
#include <Kinect/Types.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
namespace GLMotif {
class PopupMenu;
class PopupWindow;
}

class RawKinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	typedef Geometry::Plane<float,3> Plane; // Type for planes in depth camera or world space
	typedef Kinect::Size Size; // Type for image or frame sizes
	typedef Kinect::Offset Offset; // Type for image or frame offsets or pixel positions
	typedef Kinect::FrameSource::DepthPixel DepthPixel; // Type for depth frame pixels
	typedef Kinect::FrameSource::ColorComponent ColorComponent; // Type for color frame pixel components
	typedef Kinect::FrameSource::ColorPixel ColorPixel; // Type for color frame pixels
	typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelCorrection; // Type for per-pixel depth correction factors
	typedef Kinect::FrameSource::IntrinsicParameters IntrinsicParameters; // Type for camera intrinsic parameters
	typedef IntrinsicParameters::PTransform PTransform; // Type for depth camera unprojection and color camera projection matrices
	typedef PTransform::Point CPoint; // Type for camera-space points
	typedef PTransform::Vector CVector; // Type for camera-space vectors
	typedef Misc::FunctionCall<int> AverageFrameReadyCallback; // Type for callbacks when an average depth frame has been captured; int argument is a dummy
	typedef Misc::FunctionCall<const Kinect::FrameBuffer&> FrameStreamingCallback; // Type for callbacks when a color or depth frame arrives from the camera
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		Size colorTextureSize; // Padded size of the color texture
		GLuint colorTextureId; // ID of texture object holding color image
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		Size depthTextureSize; // Padded size of the depth texture
		GLuint depthTextureId; // ID of texture object holding depth image
		unsigned int depthFrameVersion; // Version number of frame currently texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	friend class PauseTool;
	friend class MeasurementTool;
	friend class TiePointTool;
	friend class LineTool;
	friend class DepthCorrectionTool;
	friend class GridTool;
	friend class PlaneTool;
	friend class PointPlaneTool;
	friend class CalibrationCheckTool;
	
	/* Elements: */
	Kinect::DirectFrameSource* camera; // Pointer to a directly-connected depth camera
	Threads::Spinlock frameCallbacksMutex; // Mutex serializing access to the frame streaming callbacks
	std::vector<FrameStreamingCallback*> colorFrameCallbacks; // List of callbacks to be called when a new color frame arrives
	std::vector<FrameStreamingCallback*> depthFrameCallbacks; // List of callbacks to be called when a new depth frame arrives
	Size colorFrameSize; // Size of color frames in pixels
	unsigned int backgroundCaptureNumFrames; // Number of color background frames left to capture
	Kinect::FrameSource::ColorPixel* colorBackground; // Frame buffer for color frame background removal
	Threads::TripleBuffer<Kinect::FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of current color frame
	Size depthFrameSize; // Size of depth frames in pixels
	PixelCorrection* depthCorrection; // Buffer containing per-pixel depth correction coefficients
	IntrinsicParameters intrinsicParameters; // Intrinsic parameters of the Kinect camera
	double depthImageOffset; // Offset to display depth image
	double colorImageScale; // Scale factor to display color image
	float depthValueRange[2]; // Range of depth values mapped to the depth color map
	float depthPlaneDistMax; // Range of depth plane color map around depth plane
	Threads::TripleBuffer<Kinect::FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of current depth frame
	bool paused; // Flag whether the video stream display is paused
	unsigned int averageNumFrames; // Number of depth frames to average
	unsigned int averageFrameCounter; // Number of depth frames still to accumulate
	std::vector<AverageFrameReadyCallback*> averageFrameReadyCallbacks; // Functions called when a new average depth frame has been captured
	float* averageFrameDepth; // Average depth values in depth frame
	float* averageFrameForeground; // Ratio of foreground vs background in depth frame
	bool averageFrameValid; // Flag whether the average depth frame buffer is currently valid
	bool showAverageFrame; // Flag whether to show the averaged frame
	bool depthPlaneValid; // Flag whether a depth plane has been defined
	Plane camDepthPlane; // Depth plane equation in depth camera image space
	Plane worldDepthPlane; // Depth plane equation in world space
	Offset selectedPixel; // Coordinates of the selected depth image pixel
	DepthPixel selectedPixelPulse[128]; // EKG of depth value of selected pixel
	int selectedPixelCurrentIndex; // Index of most recent value in selected pixel's EKG
	GLMotif::PopupWindow* depthRangeDialog; // A dialog window to adjust the depth frame's color mapping
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	GLMotif::PopupWindow* averageDepthFrameDialog; // A dialog window indicating that an average depth frame is being captured
	
	/* Private methods: */
	void mapDepth(const Offset& pixel,float depth,GLubyte* colorPtr) const; // Maps a depth value to a color
	Vrui::Point calcImagePoint(const Vrui::Ray& physicalRay) const; // Returns image-space point at which the given physical-space ray intersects the image plane
	CPoint calcDepthImagePoint(const Vrui::Point& imagePoint) const; // Returns the position of the given image-space point in distorted depth image pixel space
	float getDepthImagePixel(const Offset& pixel) const; // Returns the average or current depth value of the given depth image pixel
	CPoint getDepthImagePoint(const Offset& pixel) const; // Returns the distortion-corrected image-space position of the given depth image pixel
	CPoint getDepthImagePoint(const Vrui::Point& imagePoint) const; // Returns the image-space point at the given image-plane position
	void registerColorCallback(FrameStreamingCallback* newCallback); // Registers a callback to be called when a new color frame arrives; does not adopt callback object
	void unregisterColorCallback(FrameStreamingCallback* callback); // Unregisters a color streaming callback
	void registerDepthCallback(FrameStreamingCallback* newCallback); // Registers a callback to be called when a new depth frame arrives; does not adopt callback object
	void unregisterDepthCallback(FrameStreamingCallback* callback); // Unregisters a depth streaming callback
	void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void requestAverageFrame(AverageFrameReadyCallback* callback); // Requests collection of an average depth frame; given function will be called when it's ready
	void locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData); // Callback when a locator tool's button is pressed
	void minDepthSliderValueChangedCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void maxDepthSliderValueChangedCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	GLMotif::PopupWindow* createDepthRangeDialog(void); // Creates the depth range dialog
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void showDepthRangeDialogCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void saveAverageFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void saveAverageFrameCallback(Misc::CallbackData* cbData);
	void saveColorFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void saveColorFrameCallback(Misc::CallbackData* cbData);
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	GLMotif::PopupWindow* createAverageDepthFrameDialog(void); // Creates the depth frame averaging dialog
	
	/* Constructors and destructors: */
	public:
	RawKinectViewer(int& argc,char**& argv);
	virtual ~RawKinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

#endif
