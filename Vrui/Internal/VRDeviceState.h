/***********************************************************************
VRDeviceState - Class to represent the current state of a single or
multiple VR devices.
Copyright (c) 2002-2023 Oliver Kreylos

This file is part of the Virtual Reality User Interface Library (Vrui).

The Virtual Reality User Interface Library is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Virtual Reality User Interface Library is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Virtual Reality User Interface Library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef VRUI_INTERNAL_VRDEVICESTATE_INCLUDED
#define VRUI_INTERNAL_VRDEVICESTATE_INCLUDED

#include <stddef.h>
#include <string.h>
#include <Misc/SizedTypes.h>
#include <Misc/ArrayMarshallers.h>
#include <Geometry/Vector.h>
#include <Geometry/OrthonormalTransformation.h>
#include <Geometry/GeometryMarshallers.h>

/* Forward declarations: */
namespace IO {
class File;
}

namespace Vrui {

class VRDeviceState
	{
	/* Embedded classes: */
	public:
	struct TrackerState // Type for tracker states
		{
		/* Embedded classes: */
		public:
		typedef Geometry::OrthonormalTransformation<float,3> PositionOrientation; // Type for tracker position/orientation
		typedef Geometry::Vector<float,3> LinearVelocity; // Type for linear velocity vectors
		typedef Geometry::Vector<float,3> AngularVelocity; // Type for angular velocity vectors
		
		/* Elements: */
		PositionOrientation positionOrientation; // Current tracker position/orientation
		LinearVelocity linearVelocity; // Current linear velocity in units/s in physical space
		AngularVelocity angularVelocity; // Current angular velocity in radians/s in physical space
		};
	
	typedef bool ButtonState; // Type for button states
	typedef float ValuatorState; // Type for valuator states
	typedef Misc::SInt32 TimeStamp; // Type for device state time stamps in microseconds
	typedef bool ValidFlag; // Type for device valid flags
	
	/* Elements: */
	private:
	int numTrackers; // Number of represented trackers
	int numButtons; // Number of represented buttons
	int numValuators; // Number of represented valuators
	size_t stateSize; // Total size of the memory block holding all device state components
	TrackerState* trackerStates; // Array of current tracker states, also base pointer of memory block holding all device state components
	TimeStamp* trackerTimeStamps; // Time stamps of current tracker states
	ValidFlag* trackerValids; // Array of flags if the current tracker states are valid, i.e., the devices are currently tracked
	ButtonState* buttonStates; // Array of current button states
	ValuatorState* valuatorStates; // Array of current valuator states
	
	/* Private methods: */
	void createState(void); // Creates new device state arrays
	void destroyState(void); // Destroys the current device state arrays
	
	/* Constructors and destructors: */
	public:
	VRDeviceState(void); // Creates empty device state
	VRDeviceState(int sNumTrackers,int sNumButtons,int sNumValuators); // Creates device state of given layout
	~VRDeviceState(void); // Destroys the device state and releases all allocated resources
	
	/* Methods: */
	void setLayout(int newNumTrackers,int newNumButtons,int newNumValuators); // Sets the number of represented trackers, buttons and valuators
	int getNumTrackers(void) const // Returns number of represented trackers
		{
		return numTrackers;
		}
	int getNumButtons(void) const // Returns number of represented buttons
		{
		return numButtons;
		}
	int getNumValuators(void) const // Returns number of represented valuators
		{
		return numValuators;
		}
	
	/* Methods to read and set device state components: */
	const TrackerState& getTrackerState(int trackerIndex) const // Returns state of single tracker
		{
		return trackerStates[trackerIndex];
		}
	void setTrackerState(int trackerIndex,const TrackerState& newTrackerState) // Updates state of single tracker
		{
		trackerStates[trackerIndex]=newTrackerState;
		}
	TimeStamp getTrackerTimeStamp(int trackerIndex) const // Returns time stamp of current state of the given tracker
		{
		return trackerTimeStamps[trackerIndex];
		}
	void setTrackerTimeStamp(int trackerIndex,TimeStamp newTrackerTimeStamp) const // Updates time stamp of current state of the given tracker
		{
		trackerTimeStamps[trackerIndex]=newTrackerTimeStamp;
		}
	ValidFlag getTrackerValid(int trackerIndex) const // Returns true if the tracker's current state is valid
		{
		return trackerValids[trackerIndex];
		}
	void setTrackerValid(int trackerIndex,ValidFlag newTrackerValid) // Updates valid flag of the given tracker
		{
		trackerValids[trackerIndex]=newTrackerValid;
		}
	ButtonState getButtonState(int buttonIndex) const // Returns state of single button
		{
		return buttonStates[buttonIndex];
		}
	void setButtonState(int buttonIndex,ButtonState newButtonState) // Updates state of single button
		{
		buttonStates[buttonIndex]=newButtonState;
		}
	ValuatorState getValuatorState(int valuatorIndex) const // Returns state of single valuator
		{
		return valuatorStates[valuatorIndex];
		}
	void setValuatorState(int valuatorIndex,ValuatorState newValuatorState) // Updates state of single valuator
		{
		valuatorStates[valuatorIndex]=newValuatorState;
		}
	const TrackerState* getTrackerStates(void) const // Returns array of tracker states
		{
		return trackerStates;
		}
	TrackerState* getTrackerStates(void) // Ditto
		{
		return trackerStates;
		}
	const TimeStamp* getTrackerTimeStamps(void) const // Returns array of tracker state time stamps
		{
		return trackerTimeStamps;
		}
	TimeStamp* getTrackerTimeStamps(void) // Ditto
		{
		return trackerTimeStamps;
		}
	const ValidFlag* getTrackerValids(void) const // Returns array of tracker valid flags
		{
		return trackerValids;
		}
	ValidFlag* getTrackerValids(void) // Ditto
		{
		return trackerValids;
		}
	const ButtonState* getButtonStates(void) const // Returns array of button states
		{
		return buttonStates;
		}
	ButtonState* getButtonStates(void) // Ditto
		{
		return buttonStates;
		}
	const ValuatorState* getValuatorStates(void) const // Returns array of valuator states
		{
		return valuatorStates;
		}
	ValuatorState* getValuatorStates(void) // Ditto
		{
		return valuatorStates;
		}
	
	/* Methods to access device states over shared memory: */
	size_t getStateSize(void) const // Returns the size of a memory segment to hold this device state
		{
		return stateSize;
		}
	void read(const void* sourceMemory) // Reads device state from a shared memory segment
		{
		/* Read the entire memory block containing the device state in one go: */
		memcpy(trackerStates,sourceMemory,stateSize);
		}
	void write(void* destMemory) const // Writes device state to a shared memory segment
		{
		/* Write the entire memory block containing the device state in one go: */
		memcpy(destMemory,trackerStates,stateSize);
		}
	
	/* Methods to stream device states over communication sockets: */
	void writeLayout(IO::File& sink) const; // Writes device state's layout to given data sink
	void readLayout(IO::File& source); // Reads device state's layout from given data source
	void write(IO::File& sink,bool writeTimeStamps,bool writeValids) const; // Writes device state to given data sink
	void read(IO::File& source,bool readTimeStamps,bool readValids) const; // Reads device state from given data source
	};

}

namespace Misc {

/***********************************
Marshaller class for tracker states:
***********************************/

template <>
class Marshaller<Vrui::VRDeviceState::TrackerState>
	{
	/* Embedded classes: */
	public:
	typedef Vrui::VRDeviceState::TrackerState Value;
	
	/* Methods: */
	static size_t getSize(const Value& value)
		{
		size_t result=Marshaller<Value::PositionOrientation>::getSize(value.positionOrientation);
		result+=Marshaller<Value::LinearVelocity>::getSize(value.linearVelocity);
		result+=Marshaller<Value::AngularVelocity>::getSize(value.angularVelocity);
		return result;
		}
	template <class DataSinkParam>
	static void write(const Value& value,DataSinkParam& sink)
		{
		Marshaller<Value::PositionOrientation>::write(value.positionOrientation,sink);
		Marshaller<Value::LinearVelocity>::write(value.linearVelocity,sink);
		Marshaller<Value::AngularVelocity>::write(value.angularVelocity,sink);
		}
	template <class DataSourceParam>
	static Value& read(DataSourceParam& source,Value& value)
		{
		Marshaller<Value::PositionOrientation>::read(source,value.positionOrientation);
		Marshaller<Value::LinearVelocity>::read(source,value.linearVelocity);
		Marshaller<Value::AngularVelocity>::read(source,value.angularVelocity);
		return value;
		}
	template <class DataSourceParam>
	static Value read(DataSourceParam& source)
		{
		Value result;
		Marshaller<Value::PositionOrientation>::read(source,result.positionOrientation);
		Marshaller<Value::LinearVelocity>::read(source,result.linearVelocity);
		Marshaller<Value::AngularVelocity>::read(source,result.angularVelocity);
		return result;
		}
	};

}

#endif
