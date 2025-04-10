/***********************************************************************
OrbbecSDKContext - Class to share Orbbec SDK context objects between
multiple Orbbec cameras (as required by the API).
Copyright (c) 2025 Oliver Kreylos

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

#ifndef KINECT_INTERNAL_ORBBECSDKCONTEXT_INCLUDED
#define KINECT_INTERNAL_ORBBECSDKCONTEXT_INCLUDED

#include <Misc/Autopointer.h>
#include <Threads/Mutex.h>
#include <libobsensor/h/ObTypes.h>

/* Forward declarations: */
namespace ob {
class Context;
}

namespace Kinect {

class OrbbecSDKContext;
typedef Misc::Autopointer<OrbbecSDKContext> OrbbecSDKContextPtr; // Type for pointers to the singleton Orbbec SDK context object

class OrbbecSDKContext
	{
	friend class Misc::Autopointer<OrbbecSDKContext>;
	
	/* Elements: */
	private:
	static OrbbecSDKContext theContext; // The singleton Orbbec SDK context
	Threads::Mutex refMutex; // Mutex protecting the ref() and unref() methods to guarantee that a call to acquireContext returns a valid context
	unsigned int refCount; // Reference counter for the singleton Orbbec SDK context
	class ob::Context* context; // Orbbec SDK ontext handle
	
	/* Private methods: */
	void ref(void); // Adds a reference to the context
	void unref(void); // Removes a reference from the context
	void logCallback(OBLogSeverity severity, const char *logMsg); // Callback called when the Orbbec SDK context wants to log a message
	
	/* Constructors and destructors: */
	OrbbecSDKContext(void); // Creates an uninitialized Orbbec SDK context
	OrbbecSDKContext(const OrbbecSDKContext& source); // Prohibit copy constructor
	OrbbecSDKContext& operator=(const OrbbecSDKContext& source); // Prohibit assignment operator
	
	/* Methods: */
	public:
	static OrbbecSDKContextPtr acquireContext(void); // Returns a pointer to the singleton Orbbec SDK context; throws exception if context can not be initialized
	};

}

#endif
