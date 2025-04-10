/***********************************************************************
Projector - Class to project a depth frame captured from a Kinect camera
back into calibrated 3D camera space, and texture-map it with a matching
color frame.
Copyright (c) 2010-2025 Oliver Kreylos

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

#ifndef KINECT_PROJECTOR_INCLUDED
#define KINECT_PROJECTOR_INCLUDED

#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/MeshBuffer.h>
#include <Kinect/ProjectorBase.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

namespace Kinect {

class Projector:public ProjectorBase,public GLObject
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<const MeshBuffer&> StreamingCallback; // Function call type for streaming callbacks
	
	struct DataItem:public GLObject::DataItem // Structure containing per-context state
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of vertex buffer object holding the vertices of the current depth frame
		GLuint indexBufferId; // ID of index buffer object holding the triangles of the current depth frame
		unsigned int meshVersion; // Version number of mesh currently in vertex / index buffer
		GLuint textureId; // ID of texture object holding the current color frame
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	static const unsigned int quadCaseNumTriangles[16]; // Number of triangles to be generated for each quad corner validity case
	Threads::MutexCond inDepthFrameCond; // Condition variable to signal arrival of a new depth frame
	unsigned int inDepthFrameVersion; // Version number of most-recently arrived raw depth frame
	FrameBuffer inDepthFrame; // Most-recently arrived raw depth frame
	bool filterDepthFrames; // Flag if temporal depth frame filtering is enabled
	bool lowpassDepthFrames; // Flag it spatial depth frame filtering is enabled
	mutable GLfloat* filteredDepthFrame; // Temporally filtered depth frame, same version number as current depth frame
	mutable GLfloat* spatialFilterBuffer; // Intermediate buffer to filter depth frames spatially
	int quadCaseVertexOffsets[16][6]; // Offsets of triangle vertices to be used for each quad corner validity case
	Threads::Thread depthFrameProcessingThread; // Background thread to process incoming depth frames for rendering
	Threads::TripleBuffer<MeshBuffer> meshes; // Triple buffer of meshes ready for rendering
	unsigned int meshVersion; // Version number of current mesh
	StreamingCallback* streamingCallback; // Function to be called when a new mesh has been produced
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames ready for rendering
	unsigned int colorFrameVersion; // Version number of current color frame
	
	/* Private methods: */
	void* depthFrameProcessingThreadMethod(void); // Thread method for background depth frame processing
	
	/* Constructors and destructors: */
	public:
	Projector(void); // Creates a facade projector with uninitialized camera parameters
	Projector(FrameSource& frameSource); // Creates a facade projector for the given frame source
	~Projector(void);
	
	/* Overriden methods from class ProjectorBase: */
	void setDepthFrameSize(const Size& newDepthFrameSize);
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	bool getFilterDepthFrames(void) const // Returns true if depth frame filtering is enabled
		{
		return filterDepthFrames;
		}
	void setFilterDepthFrames(bool newFilterDepthFrames,bool newLowpassDepthFrames); // Enables or disables temporal and spatial depth frame filtering
	void processDepthFrame(const FrameBuffer& depthFrame,MeshBuffer& meshBuffer) const; // Processes the given depth frame into the given mesh buffer immediately and returns the resuling mesh
	void startStreaming(StreamingCallback* newStreamingCallback); // Starts processing depth frames in the background; calls the provided callback function every time a new mesh is produced
	void setDepthFrame(const FrameBuffer& newDepthFrame); // Updates the projector's current depth frame in streaming mode; can be called from any thread
	void setMesh(const MeshBuffer& newMesh); // Updates the projector's current mesh in streaming mode; can be called from any thread
	void setColorFrame(const FrameBuffer& newColorFrame); // Updates the projector's current color frame in streaming mode; can be called from any thread
	void stopStreaming(void); // Stops background processing of depth frames
	void updateFrames(void); // Selects the most recent depth and color frames for rendering; must be called from foreground thread
	double getColorTimeStamp(void) const // Returns the time stamp of the color frame currently locked for rendering
		{
		return colorFrames.getLockedValue().timeStamp;
		}
	double getMeshTimeStamp(void) const // Returns the time stamp of the triangle mesh currently locked for rendering
		{
		return meshes.getLockedValue().timeStamp;
		}
	void glRenderAction(GLContextData& contextData) const; // Draws the current depth and color frames in the current model coordinate system
	};

}

#endif
