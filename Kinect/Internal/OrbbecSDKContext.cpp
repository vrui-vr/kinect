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

#include <Kinect/Internal/OrbbecSDKContext.h>

#include <functional>
#include <libobsensor/hpp/Version.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <Misc/MessageLogger.h>

namespace Kinect {

/*****************************************
Static elements of class OrbbecSDKContext:
*****************************************/

OrbbecSDKContext OrbbecSDKContext::theContext;

/*********************************
Methods of class OrbbecSDKContext:
*********************************/

void OrbbecSDKContext::logCallback(OBLogSeverity severity, const char *logMsg)
	{
	/* Forward the message to the central message handler: */
	switch(severity)
		{
		case OB_LOG_SEVERITY_DEBUG:
		case OB_LOG_SEVERITY_INFO:
			Misc::formattedLogNote("Kinect::OrbbecSDKContext: %s",logMsg);
			break;
		
		case OB_LOG_SEVERITY_WARN:
			Misc::formattedConsoleWarning("Kinect::OrbbecSDKContext: %s",logMsg);
			break;
		
		case OB_LOG_SEVERITY_ERROR:
		case OB_LOG_SEVERITY_FATAL:
			Misc::formattedUserError("Kinect::OrbbecSDKContext: %s",logMsg);
			break;
		
		default:
			; // Do nothing
		}
	}

void OrbbecSDKContext::ref(void)
	{
	/* Lock the reference mutex: */
	Threads::Mutex::Lock refLock(refMutex);
	
	/* Increment the reference counter and check if it was zero before: */
	if((refCount++)==0)
		{
		/* Print basic SDK info: */
		Misc::formattedLogNote("Kinect::OrbbecSDKContext:: SDK version %d.%d.%d",ob::Version::getMajor(),ob::Version::getMinor(),ob::Version::getPatch());
		Misc::formattedLogNote("Kinect::OrbbecSDKContext:: SDK stage version %d",ob::Version::getStageVersion());
		
		/* Initialize the Orbbec SDK context: */
		ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
		ob::Context::setLoggerToCallback(OB_LOG_SEVERITY_WARN,std::bind(&OrbbecSDKContext::logCallback,this,std::placeholders::_1,std::placeholders::_2));
		context=new ob::Context;
		}
	}

void OrbbecSDKContext::unref(void)
	{
	/* Lock the reference mutex: */
	Threads::Mutex::Lock refLock(refMutex);
	
	/* Decrement the reference counter and check if it reached zero: */
	if((--refCount)==0)
		{
		/* Destroy the Orbbec SDK context: */
		delete context;
		context=0;
		}
	}

OrbbecSDKContext::OrbbecSDKContext(void)
	:refCount(0),
	 context(0)
	{
	}

OrbbecSDKContextPtr OrbbecSDKContext::acquireContext(void)
	{
	/* Return an autopointer to the singleton Orbbec SDK context object; autopointer's call to ref() will set up context on first call: */
	return OrbbecSDKContextPtr(&theContext);
	}

}
