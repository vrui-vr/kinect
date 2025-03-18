/***********************************************************************
VRDeviceClient - Class encapsulating the VR device protocol's client
side.
Copyright (c) 2002-2024 Oliver Kreylos

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

#define DEBUG_PROTOCOL 0
#define TRACK_LATENCY 0

#include <Vrui/Internal/VRDeviceClient.h>

#include <stddef.h>
#include <Misc/SizedTypes.h>
#include <Misc/StdError.h>
#include <Misc/Time.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Realtime/Time.h>
#include <Realtime/SharedMemory.h>
#include <Threads/Config.h>
#include <Comm/UNIXPipe.h>
#include <Comm/TCPPipe.h>
#include <Vrui/EnvironmentDefinition.h>
#include <Vrui/Internal/VRDeviceDescriptor.h>
#include <Vrui/Internal/HMDConfiguration.h>
#include <Vrui/Internal/VRBaseStation.h>

#if 1 || DEBUG_PROTOCOL || TRACK_LATENCY
#include <iostream>
#endif

namespace Vrui {

namespace {

/****************
Helper functions:
****************/

void adjustTrackerStateTimeStamps(VRDeviceState& state,VRDeviceState::TimeStamp timeStampDelta) // Sets tracker state time stamps to current monotonic time
	{
	/* Adjust all tracker state time stamps: */
	VRDeviceState::TimeStamp* tsPtr=state.getTrackerTimeStamps();
	for(int i=0;i<state.getNumTrackers();++i,++tsPtr)
		*tsPtr+=timeStampDelta;
	}

void setTrackerStateTimeStamps(VRDeviceState& state) // Sets tracker state time stamps to current monotonic time
	{
	/* Get the current time: */
	TimePoint now;
	
	/* Get the lower-order bits of the microsecond time: */
	VRDeviceState::TimeStamp ts=VRDeviceState::TimeStamp(now.tv_sec*1000000+(now.tv_nsec+500)/1000);
	
	/* Set all tracker state time stamps to the current time: */
	VRDeviceState::TimeStamp* tsPtr=state.getTrackerTimeStamps();
	for(int i=0;i<state.getNumTrackers();++i,++tsPtr)
		*tsPtr=ts;
	}

}

#if TRACK_LATENCY

/* State to calculate the latency of sending tracker updates over TCP: */
int trackerLatencyMin=1000000;
int trackerLatencyMax=-1000000;
size_t trackerLatencySum=0;
size_t trackerLatencyNumSamples=0;

#endif

/*******************************
Methods of class VRDeviceClient:
*******************************/

void VRDeviceClient::readConnectReply(void)
	{
	/* Check server protocol version number for compatibility: */
	serverProtocolVersionNumber=pipe->read<Misc::UInt32>();
	if(serverProtocolVersionNumber<1U||serverProtocolVersionNumber>protocolVersionNumber)
		throw std::runtime_error("Unsupported server protocol version");
	
	/* Read server's layout and initialize current state: */
	state.readLayout(*pipe);
	
	/* Check if the server will send virtual input device descriptors: */
	if(serverProtocolVersionNumber>=2U)
		{
		/* Read the list of virtual devices managed by the server: */
		unsigned int numVirtualDevices=pipe->read<Misc::UInt32>();
		for(unsigned int deviceIndex=0;deviceIndex<numVirtualDevices;++deviceIndex)
			{
			/* Create a new virtual input device and read its layout from the server: */
			VRDeviceDescriptor* newDevice=new VRDeviceDescriptor;
			newDevice->read(*pipe,serverProtocolVersionNumber);
			
			/* Store the virtual input device: */
			virtualDevices.push_back(newDevice);
			}
		}
	
	/* Check if the server will send tracker state time stamps: */
	serverHasTimeStamps=serverProtocolVersionNumber>=3U;
	
	/* Initialize the clock offset: */
	timeStampDelta=0;
	
	/* Create an array to cache virtual input devices' battery states: */
	if(!virtualDevices.empty())
		batteryStates=new BatteryState[virtualDevices.size()];
	
	/* Check if the server maintains battery states: */
	if(serverProtocolVersionNumber>=5U)
		{
		/* Read battery states for all virtual devices: */
		for(unsigned int i=0;i<virtualDevices.size();++i)
			batteryStates[i].read(*pipe);
		}
	
	/* Check if the server maintains HMD configurations: */
	if(serverProtocolVersionNumber>=4U)
		{
		/* Read the list of HMD configurations maintained by the server: */
		numHmdConfigurations=pipe->read<Misc::UInt32>();
		
		/* Read initial HMD configurations: */
		hmdConfigurations=new HMDConfiguration[numHmdConfigurations];
		for(unsigned int i=0;i<numHmdConfigurations;++i)
			{
			/* Read the update message ID (server will send it): */
			MessageIdType messageId=pipe->read<MessageIdType>();
			
			/* Read the HMD configuration's tracker index: */
			Misc::UInt16 trackerIndex=pipe->read<Misc::UInt16>();
			
			/* Read the HMD configuration: */
			hmdConfigurations[i].read(messageId,trackerIndex,*pipe);
			}
		
		/* Check if the server sends eye rotations: */
		if(serverProtocolVersionNumber>=10U)
			{
			for(unsigned int i=0;i<numHmdConfigurations;++i)
				{
				/* Skip the update message ID and tracker index (server will send 'em, we don't need 'em): */
				pipe->skip<MessageIdType>(1);
				pipe->skip<Misc::UInt16>(1);
				
				/* Read the HMD eye rotation: */
				hmdConfigurations[i].readEyeRotation(*pipe);
				}
			}
		
		/* Initialize HMD configuration update callback array: */
		hmdConfigurationUpdatedCallbacks=new HMDConfigurationUpdatedCallback*[numHmdConfigurations];
		for(unsigned int i=0;i<numHmdConfigurations;++i)
			hmdConfigurationUpdatedCallbacks[i]=0;
		}
	
	/* Check if the server will send tracker valid flags: */
	serverHasValidFlags=serverProtocolVersionNumber>=5U;
	
	/* Initialize all tracker states to "valid" if the server doesn't send valid flags: */
	if(!serverHasValidFlags)
		for(int i=0;i<state.getNumTrackers();++i)
			state.setTrackerValid(i,true);
	
	/* Check if the server maintains power and haptic features: */
	if(serverProtocolVersionNumber>=6U)
		{
		/* Read the number of power and haptic features maintained by the server: */
		numPowerFeatures=pipe->read<Misc::UInt32>();
		numHapticFeatures=pipe->read<Misc::UInt32>();
		}
	
	/* Check if the server supports shared-memory access to its device states: */
	Comm::UNIXPipe* unixPipe=dynamic_cast<Comm::UNIXPipe*>(pipe.getPointer());
	if(unixPipe!=0&&serverProtocolVersionNumber>=12U)
		{
		/* Read the file descriptor of the server's shared-memory block: */
		stateMemory=new Realtime::SharedMemory(unixPipe->readFd(),false);
		}
	}

bool VRDeviceClient::handlePipeMessage(void)
	{
	bool result=true;
	
	try
		{
		MessageIdType message=pipe->read<MessageIdType>();
		if(message==PACKET_REPLY)
			{
			#if DEBUG_PROTOCOL
			std::cout<<"Received PACKET_REPLY"<<std::endl;
			#endif
			
			/* Read server's state: */
			{
			Threads::Mutex::Lock stateLock(stateMutex);
			state.read(*pipe,serverHasTimeStamps,serverHasValidFlags);
			if(!serverHasTimeStamps)
				{
				/* Set all tracker time stamps to the current local time: */
				setTrackerStateTimeStamps(state);
				}
			else if(!local)
				{
				/* Adjust all received time stamps by the client/server clock difference: */
				adjustTrackerStateTimeStamps(state,timeStampDelta);
				}
			}
			
			/* Signal packet reception: */
			packetSignalCond.broadcast();
			
			/* Invoke packet notification callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(packetNotificationCallback!=0)
				(*packetNotificationCallback)(this);
			}
			}
		else if(message==TRACKER_UPDATE)
			{
			/* Read a tracker update packet: */
			{
			Threads::Mutex::Lock stateLock(stateMutex);
			
			unsigned int trackerIndex=pipe->read<Misc::UInt16>();
			VRDeviceState::TrackerState trackerState=Misc::Marshaller<VRDeviceState::TrackerState>::read(*pipe);
			state.setTrackerState(trackerIndex,trackerState);
			VRDeviceState::TimeStamp trackerTimeStamp=pipe->read<VRDeviceState::TimeStamp>();
			if(!local)
				trackerTimeStamp+=timeStampDelta;
			state.setTrackerTimeStamp(trackerIndex,trackerTimeStamp);
			#if TRACK_LATENCY
			{
			/* Get a timestamp for the current time: */
			TimePoint now;
			VRDeviceState::TimeStamp ts=VRDeviceState::TimeStamp(now.tv_sec*1000000+(now.tv_nsec+500)/1000);
			
			/* Calculate the tracker update latency in us: */
			int latency=int(ts-trackerTimeStamp);
			if(trackerLatencyMin>latency)
				trackerLatencyMin=latency;
			if(trackerLatencyMax<latency)
				trackerLatencyMax=latency;
			trackerLatencySum+=latency;
			++trackerLatencyNumSamples;
			}
			#endif
			bool trackerValid=pipe->read<Misc::UInt8>()!=0U;
			state.setTrackerValid(trackerIndex,trackerValid);
			
			#if DEBUG_PROTOCOL
			std::cout<<"Received TRACKER_UPDATE for tracker "<<trackerIndex<<", time "<<trackerTimeStamp<<", now "<<(trackerValid?"valid":"invalid")<<std::endl;
			#endif
			}
			
			/* Signal packet reception: */
			packetSignalCond.broadcast();
			
			/* Invoke packet notification callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(packetNotificationCallback!=0)
				(*packetNotificationCallback)(this);
			}
			}
		else if(message==BUTTON_UPDATE)
			{
			/* Read a button update packet: */
			{
			Threads::Mutex::Lock stateLock(stateMutex);
			
			unsigned int buttonIndex=pipe->read<Misc::UInt16>();
			VRDeviceState::ButtonState buttonState=pipe->read<Misc::UInt8>()!=0U;
			state.setButtonState(buttonIndex,buttonState);
			
			#if DEBUG_PROTOCOL
			std::cout<<"Received BUTTON_UPDATE for button "<<buttonIndex<<", state "<<(buttonState?"pressed":"released")<<std::endl;
			#endif
			}
			
			/* Signal packet reception: */
			packetSignalCond.broadcast();
			
			/* Invoke packet notification callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(packetNotificationCallback!=0)
				(*packetNotificationCallback)(this);
			}
			}
		else if(message==VALUATOR_UPDATE)
			{
			/* Read a valuator update packet: */
			{
			Threads::Mutex::Lock stateLock(stateMutex);
			
			unsigned int valuatorIndex=pipe->read<Misc::UInt16>();
			VRDeviceState::ValuatorState valuatorState=pipe->read<VRDeviceState::ValuatorState>();
			state.setValuatorState(valuatorIndex,valuatorState);
			
			#if DEBUG_PROTOCOL
			std::cout<<"Received VALUATOR_UPDATE for valuator "<<valuatorIndex<<", state "<<valuatorState<<std::endl;
			#endif
			}
			
			/* Signal packet reception: */
			packetSignalCond.broadcast();
			
			/* Invoke packet notification callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(packetNotificationCallback!=0)
				(*packetNotificationCallback)(this);
			}
			}
		else if(message==BATTERYSTATE_UPDATE)
			{
			Threads::Mutex::Lock batteryStatesLock(batteryStatesMutex);
			
			/* Read the index of the device whose battery state changed and the new battery state: */
			unsigned int deviceIndex=pipe->read<Misc::UInt16>();
			batteryStates[deviceIndex].read(*pipe);
			
			/* Call the battery state change callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(batteryStateUpdatedCallback!=0)
				(*batteryStateUpdatedCallback)(deviceIndex);
			}
			}
		else if((message&~0x7U)==HMDCONFIG_UPDATE)
			{
			/* Read the tracker index of the updated HMD configuration: */
			Misc::UInt16 updatedTrackerIndex=pipe->read<Misc::UInt16>();
			
			/* Find the to-be-updated HMD configuration in the list: */
			unsigned index;
			for(index=0;index<numHmdConfigurations&&updatedTrackerIndex!=hmdConfigurations[index].getTrackerIndex();++index)
				;
			if(index>=numHmdConfigurations)
				throw std::runtime_error("Invalid HMD tracker index");
			
			{
			Threads::Mutex::Lock hmdConfigurationLock(hmdConfigurationMutex);
			
			/* Read updated HMD configuration from server: */
			hmdConfigurations[index].read(message,updatedTrackerIndex,*pipe);
			
			/* Call the update callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(hmdConfigurationUpdatedCallbacks[index]!=0)
				(*hmdConfigurationUpdatedCallbacks[index])(hmdConfigurations[index]);
			}
			}
			}
		else if(message==HMDCONFIG_EYEROTATION_UPDATE)
			{
			/* Read the tracker index of the updated HMD configuration: */
			Misc::UInt16 updatedTrackerIndex=pipe->read<Misc::UInt16>();
			
			/* Find the to-be-updated HMD configuration in the list: */
			unsigned index;
			for(index=0;index<numHmdConfigurations&&updatedTrackerIndex!=hmdConfigurations[index].getTrackerIndex();++index)
				;
			if(index>=numHmdConfigurations)
				throw std::runtime_error("Invalid HMD tracker index");
			
			{
			Threads::Mutex::Lock hmdConfigurationLock(hmdConfigurationMutex);
			
			/* Read updated HMD configuration from server: */
			hmdConfigurations[index].readEyeRotation(*pipe);
			
			/* Call the update callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(hmdConfigurationUpdatedCallbacks[index]!=0)
				(*hmdConfigurationUpdatedCallbacks[index])(hmdConfigurations[index]);
			}
			}
			}
		else if(message==BASESTATIONS_REPLY)
			{
			Threads::MutexCond::Lock getBaseStationsLock(getBaseStationsCond);
			
			/* Check if there is a pending getBaseStations request: */
			if(getBaseStationsRequest==0)
				throw std::runtime_error("No pending getBaseStations request");
			
			/* Read the list of base stations: */
			unsigned int numBaseStations=pipe->read<Misc::UInt8>();
			getBaseStationsRequest->reserve(numBaseStations);
			for(unsigned int i=0;i<numBaseStations;++i)
				{
				VRBaseStation bs;
				bs.read(*pipe);
				getBaseStationsRequest->push_back(bs);
				}
			
			/* Signal the getBaseStations request as complete: */
			getBaseStationsCond.signal();
			}
		else if(message==ENVIRONMENTDEFINITION_REPLY)
			{
			Threads::MutexCond::Lock getEnvironmentDefinitionLock(getEnvironmentDefinitionCond);
			
			/* Check if there is a pending getEnvironmentDefinition request: */
			if(getEnvironmentDefinitionRequest==0)
				throw std::runtime_error("No pending getEnvironmentDefinition request");
			
			/* Read the environment definition: */
			getEnvironmentDefinitionRequest->read(*pipe);
			
			/* Signal the getEnvironmentDefinition request as complete: */
			getEnvironmentDefinitionCond.signal();
			}
		else if(message==ENVIRONMENTDEFINITION_UPDATE_NOTIFICATION)
			{
			/* Read the environment definition: */
			EnvironmentDefinition environmentDefinition;
			environmentDefinition.read(*pipe);
			
			/* Call the environment definition update callback: */
			{
			Threads::Mutex::Lock callbacksLock(callbacksMutex);
			if(environmentDefinitionUpdatedCallback!=0)
				(*environmentDefinitionUpdatedCallback)(environmentDefinition);
			}
			}
		else if(message==STOPSTREAM_REPLY)
			result=false;
		else
			throw std::runtime_error("Unexpected message");
		}
	catch(const std::runtime_error& err)
		{
		/* Signal an error and shut down: */
		{
		Threads::Mutex::Lock callbacksLock(callbacksMutex);
		if(errorCallback!=0)
			{
			std::string msg="VRDeviceClient: Caught exception ";
			msg.append(err.what());
			(*errorCallback)(ProtocolError(msg,this));
			}
		}
		connectionDead=true;
		packetSignalCond.broadcast();
		
		result=false;
		}
	
	return result;
	}

void VRDeviceClient::pipeCallback(Threads::EventDispatcher::IOEvent& event)
	{
	/* Forward the call to the message handler: */
	static_cast<VRDeviceClient*>(event.getUserData())->handlePipeMessage();
	}

void VRDeviceClient::initClient(void)
	{
	/* Determine whether client and server are running on the same host: */
	local=true;
	
	/* Only TCP pipes can be non-local: */
	Comm::TCPPipe* tcpPipe=dynamic_cast<Comm::TCPPipe*>(pipe.getPointer());
	if(tcpPipe!=0)
		local=tcpPipe->getAddress()==tcpPipe->getPeerAddress();
	
	/* Send the connection request message to the server: */
	pipe->write(MessageIdType(CONNECT_REQUEST));
	pipe->write(Misc::UInt32(protocolVersionNumber));
	pipe->flush();
	
	/* Wait for the server's connect reply: */
	if(!pipe->waitForData(Misc::Time(10,0)))
		throw ProtocolError("VRDeviceClient: Timeout while waiting for connection",this);
	if(pipe->read<MessageIdType>()!=CONNECT_REPLY)
		throw ProtocolError("VRDeviceClient: Wrong message type while waiting for connection",this);
	
	/* Read the connect reply message: */
	readConnectReply();
	}

VRDeviceClient::VRDeviceClient(Threads::EventDispatcher& sDispatcher,const char* deviceServerHostName,int deviceServerPort)
	:dispatcher(sDispatcher),
	 pipe(new Comm::TCPPipe(deviceServerHostName,deviceServerPort)),pipeEventKey(0),
	 serverProtocolVersionNumber(0),serverHasTimeStamps(false),serverHasValidFlags(false),
	 stateMemory(0),batteryStates(0),batteryStateUpdatedCallback(0),
	 numHmdConfigurations(0),hmdConfigurations(0),hmdConfigurationUpdatedCallbacks(0),
	 numPowerFeatures(0),numHapticFeatures(0),
	 getBaseStationsRequest(0),getEnvironmentDefinitionRequest(0),
	 environmentDefinitionUpdatedCallback(0),
	 active(false),streaming(false),connectionDead(false),
	 packetNotificationCallback(0),errorCallback(0)
	{
	initClient();
	}

VRDeviceClient::VRDeviceClient(Threads::EventDispatcher& sDispatcher,const char* deviceServerSocketName,bool deviceServerSocketAbstract)
	:dispatcher(sDispatcher),
	 pipe(new Comm::UNIXPipe(deviceServerSocketName,deviceServerSocketAbstract)),pipeEventKey(0),
	 serverProtocolVersionNumber(0),serverHasTimeStamps(false),serverHasValidFlags(false),
	 stateMemory(0),batteryStates(0),batteryStateUpdatedCallback(0),
	 numHmdConfigurations(0),hmdConfigurations(0),hmdConfigurationUpdatedCallbacks(0),
	 numPowerFeatures(0),numHapticFeatures(0),
	 getBaseStationsRequest(0),getEnvironmentDefinitionRequest(0),
	 environmentDefinitionUpdatedCallback(0),
	 active(false),streaming(false),connectionDead(false),
	 packetNotificationCallback(0),errorCallback(0)
	{
	initClient();
	}

namespace {

/******************************************************************************
Helper function to connect to a device server over a TCP or UNIX domain socket:
******************************************************************************/

Comm::Pipe* openServerPipe(const Misc::ConfigurationFileSection& configFileSection)
	{
	if(configFileSection.hasTag("./serverHostName")&&configFileSection.hasTag("./serverPort"))
		{
		/* Open a connection over a TCP socket: */
		return new Comm::TCPPipe(configFileSection.retrieveString("./serverHostName").c_str(),configFileSection.retrieveValue<int>("./serverPort"));
		}
	else if(configFileSection.hasTag("./serverSocketName")&&configFileSection.hasTag("./serverSocketAbstract"))
		{
		/* Open a connection over a UNIX domain socket: */
		return new Comm::UNIXPipe(configFileSection.retrieveString("./serverSocketName").c_str(),configFileSection.retrieveValue<bool>("./serverSocketAbstract"));
		}
	else
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Neither TCP nor UNIX domain server specification in configuration file section");
	}

}

VRDeviceClient::VRDeviceClient(Threads::EventDispatcher& sDispatcher,const Misc::ConfigurationFileSection& configFileSection)
	:dispatcher(sDispatcher),
	 pipe(openServerPipe(configFileSection)),pipeEventKey(0),
	 serverProtocolVersionNumber(0),serverHasTimeStamps(false),serverHasValidFlags(false),
	 stateMemory(0),batteryStates(0),batteryStateUpdatedCallback(0),
	 numHmdConfigurations(0),hmdConfigurations(0),hmdConfigurationUpdatedCallbacks(0),
	 numPowerFeatures(0),numHapticFeatures(0),
	 getBaseStationsRequest(0),getEnvironmentDefinitionRequest(0),
	 environmentDefinitionUpdatedCallback(0),
	 active(false),streaming(false),connectionDead(false),
	 packetNotificationCallback(0),errorCallback(0)
	{
	initClient();
	}

VRDeviceClient::~VRDeviceClient(void)
	{
	/* Leave streaming mode: */
	if(streaming)
		stopStream();
	
	/* Deactivate client: */
	if(active)
		deactivate();
	
	/* Disconnect from server: */
	pipe->write(MessageIdType(DISCONNECT_REQUEST));
	pipe->flush();
	
	/* Delete all callbacks: */
	{
	Threads::Mutex::Lock callbacksLock(callbacksMutex);
	delete batteryStateUpdatedCallback;
	for(unsigned int i=0;i<numHmdConfigurations;++i)
		delete hmdConfigurationUpdatedCallbacks[i];
	delete[] hmdConfigurationUpdatedCallbacks;
	delete environmentDefinitionUpdatedCallback;
	delete packetNotificationCallback;
	delete errorCallback;
	}
	
	/* Delete all virtual input devices: */
	for(std::vector<VRDeviceDescriptor*>::iterator vdIt=virtualDevices.begin();vdIt!=virtualDevices.end();++vdIt)
		delete *vdIt;
	
	/* Delete battery states and HMD configurations: */
	delete[] batteryStates;
	delete[] hmdConfigurations;
	
	/* Release the server's shared memory segment: */
	delete stateMemory;
	
	#if TRACK_LATENCY
	std::cout<<"Tracker update latency range: ["<<trackerLatencyMin<<", "<<trackerLatencyMax<<"]"<<std::endl;
	std::cout<<"Average tracker latency: "<<double(trackerLatencySum)/double(trackerLatencyNumSamples)<<std::endl;
	#endif
	}

const HMDConfiguration& VRDeviceClient::getHmdConfiguration(unsigned int index) const
	{
	return hmdConfigurations[index];
	}

void VRDeviceClient::activate(void)
	{
	/* Ignore a redundant request: */
	if(!active)
		{
		/* Send the activation request message: */
		pipe->write(MessageIdType(ACTIVATE_REQUEST));
		pipe->flush();
		
		/* Register server connection events with the event dispatcher: */
		pipeEventKey=dispatcher.addIOEventListener(pipe->getFd(),Threads::EventDispatcher::Read,&VRDeviceClient::pipeCallback,this);
		
		active=true;
		}
	}

void VRDeviceClient::deactivate(void)
	{
	/* Ignore a redundant request: */
	if(active)
		{
		active=false;
		
		/* Unregister server connection events with the event dispatcher: */
		dispatcher.removeIOEventListener(pipeEventKey);
		pipeEventKey=0;
		
		/* Send the deactivation request message: */
		pipe->write(MessageIdType(DEACTIVATE_REQUEST));
		pipe->flush();
		}
	}

void VRDeviceClient::getPacket(void)
	{
	if(!active)
		throw ProtocolError("VRDeviceClient: Client is not active",this);
	
	/* If the client is not streaming, send a packet request message to the server: */
	if(!streaming)
		{
		pipe->write(MessageIdType(PACKET_REQUEST));
		pipe->flush();
		}
	
	/* Wait for the arrival of the next packet reply message: */
	packetSignalCond.wait();
	}

void VRDeviceClient::powerOff(unsigned int powerFeatureIndex)
	{
	if(!active)
		throw ProtocolError("VRDeviceClient: Client is not active",this);
	
	/* Check if device server supports powering off devices: */
	if(serverProtocolVersionNumber>=6U)
		{
		/* Send power off request message: */
		pipe->write(MessageIdType(POWEROFF_REQUEST));
		pipe->write(Misc::UInt16(powerFeatureIndex));
		pipe->flush();
		}
	}

void VRDeviceClient::hapticTick(unsigned int hapticFeatureIndex,unsigned int duration,unsigned int frequency,unsigned int amplitude)
	{
	if(!active)
		throw ProtocolError("VRDeviceClient: Client is not active",this);
	
	/* Check if device server supports haptic feedback: */
	if(serverProtocolVersionNumber>=6U)
		{
		/* Send haptic tick request message: */
		pipe->write(MessageIdType(HAPTICTICK_REQUEST));
		pipe->write(Misc::UInt16(hapticFeatureIndex));
		pipe->write(Misc::UInt16(duration));
		if(serverProtocolVersionNumber>=8U)
			{
			pipe->write(Misc::UInt16(frequency));
			pipe->write(Misc::UInt8(amplitude));
			}
		pipe->flush();
		}
	}

void VRDeviceClient::setBatteryStateUpdatedCallback(VRDeviceClient::BatteryStateUpdatedCallback* newBatteryStateUpdatedCallback)
	{
	/* Replace the previous callback with the new one: */
	{
	Threads::Mutex::Lock callbacksLock(callbacksMutex);
	delete batteryStateUpdatedCallback;
	batteryStateUpdatedCallback=newBatteryStateUpdatedCallback;
	}
	}

void VRDeviceClient::setHmdConfigurationUpdatedCallback(unsigned int trackerIndex,VRDeviceClient::HMDConfigurationUpdatedCallback* newHmdConfigurationUpdatedCallback)
	{
	/* Find the HMD configuration associated with the given tracker index: */
	unsigned int index;
	for(index=0;index<numHmdConfigurations&&trackerIndex!=hmdConfigurations[index].getTrackerIndex();++index)
		;
	if(index<numHmdConfigurations)
		{
		/* Replace the previous callback for the given tracker index with the new one: */
		Threads::Mutex::Lock callbacksLock(callbacksMutex);
		delete hmdConfigurationUpdatedCallbacks[index];
		hmdConfigurationUpdatedCallbacks[index]=newHmdConfigurationUpdatedCallback;
		}
	else
		{
		/* Delete the new callback and just ignore it: */
		delete newHmdConfigurationUpdatedCallback;
		}
	}

std::vector<VRBaseStation> VRDeviceClient::getBaseStations(void)
	{
	std::vector<VRBaseStation> result;
	
	/* Check if the server knows about base stations: */
	if(serverProtocolVersionNumber>=11U)
		{
		if(active)
			{
			/* Let the message handler handle the request: */
			Threads::MutexCond::Lock getBaseStationsLock(getBaseStationsCond);
			
			/* Check if there is already a pending request: */
			if(getBaseStationsRequest!=0)
				throw ProtocolError("VRDeviceClient: Already pending getBaseStations request",this);
			
			/* Register the request: */
			getBaseStationsRequest=&result;
			
			/* Send a base station request message to the server: */
			pipe->write(MessageIdType(BASESTATIONS_REQUEST));
			pipe->flush();
			
			/* Wait for the server's reply: */
			getBaseStationsCond.wait(getBaseStationsLock);
			
			/* Unregister the request: */
			getBaseStationsRequest=0;
			}
		else
			{
			/* Send a base station request message to the server: */
			pipe->write(MessageIdType(BASESTATIONS_REQUEST));
			pipe->flush();
			
			/* Wait for the server's reply: */
			if(!pipe->waitForData(Misc::Time(10,0)))
				throw ProtocolError("VRDeviceClient: Timeout in getBaseStations",this);
			if(pipe->read<MessageIdType>()!=BASESTATIONS_REPLY)
				throw ProtocolError("VRDeviceClient: Wrong message type in getBaseStations",this);
			
			/* Read the list of base stations: */
			unsigned int numBaseStations=pipe->read<Misc::UInt8>();
			result.reserve(numBaseStations);
			for(unsigned int i=0;i<numBaseStations;++i)
				{
				VRBaseStation bs;
				bs.read(*pipe);
				result.push_back(bs);
				}
			}
		}
	
	return result;
	}

bool VRDeviceClient::getEnvironmentDefinition(EnvironmentDefinition& environmentDefinition)
	{
	/* Check if the server maintains environment definitions: */
	if(serverProtocolVersionNumber>=13U)
		{
		if(active)
			{
			/* Let the message handler handle the request: */
			Threads::MutexCond::Lock getEnvironmentDefinitionLock(getEnvironmentDefinitionCond);
			
			/* Check if there is already a pending request: */
			if(getEnvironmentDefinitionRequest!=0)
				throw ProtocolError("VRDeviceClient: Already pending getEnvironmentDefinition request",this);
			
			/* Register the request: */
			getEnvironmentDefinitionRequest=&environmentDefinition;
			
			/* Send an environment definition request message to the server: */
			pipe->write(MessageIdType(ENVIRONMENTDEFINITION_REQUEST));
			pipe->flush();
			
			/* Wait for the server's reply: */
			getEnvironmentDefinitionCond.wait(getEnvironmentDefinitionLock);
			
			/* Unregister the request: */
			getEnvironmentDefinitionRequest=0;
			}
		else
			{
			/* Send an environment definition request message to the server: */
			pipe->write(MessageIdType(ENVIRONMENTDEFINITION_REQUEST));
			pipe->flush();
			
			/* Wait for the server's reply: */
			if(!pipe->waitForData(Misc::Time(10,0)))
				throw ProtocolError("VRDeviceClient: Timeout in getEnvironmentDefinition",this);
			if(pipe->read<MessageIdType>()!=ENVIRONMENTDEFINITION_REPLY)
				throw ProtocolError("VRDeviceClient: Wrong message type in getEnvironmentDefinition",this);
			
			/* Read the environment definition: */
			environmentDefinition.read(*pipe);
			}
		
		return true;
		}
	else
		return false;
	}

bool VRDeviceClient::updateEnvironmentDefinition(const EnvironmentDefinition& environmentDefinition)
	{
	/* Check if the server maintains environment definitions: */
	if(serverProtocolVersionNumber>=13U)
		{
		try
			{
			/* Send an environment definition update request message: */
			pipe->write(MessageIdType(ENVIRONMENTDEFINITION_UPDATE_REQUEST));
			environmentDefinition.write(*pipe);
			pipe->flush();
			}
		catch(const std::runtime_error& err)
			{
			/* Mark the connection as dead and re-throw the original exception: */
			connectionDead=true;
			throw;
			}
		
		return true;
		}
	else
		return false;
	}

void VRDeviceClient::setEnvironmentDefinitionUpdatedCallback(VRDeviceClient::EnvironmentDefinitionUpdatedCallback* newEnvironmentDefinitionUpdatedCallback)
	{
	/* Replace the previous callback with the new one: */
	{
	Threads::Mutex::Lock callbacksLock(callbacksMutex);
	delete environmentDefinitionUpdatedCallback;
	environmentDefinitionUpdatedCallback=newEnvironmentDefinitionUpdatedCallback;
	}
	}

void VRDeviceClient::startStream(VRDeviceClient::Callback* newPacketNotificationCallback,VRDeviceClient::ErrorCallback* newErrorCallback)
	{
	if(active&&!streaming&&!connectionDead)
		{
		/* Install the new callback functions: */
		{
		Threads::Mutex::Lock callbacksLock(callbacksMutex);
		packetNotificationCallback=newPacketNotificationCallback;
		errorCallback=newErrorCallback;
		
		if(batteryStateUpdatedCallback!=0)
			{
			/* Send initial battery states for all devices: */
			Threads::Mutex::Lock batteryStatesLock(batteryStatesMutex);
			for(unsigned int i=0;i<virtualDevices.size();++i)
				(*batteryStateUpdatedCallback)(i);
			}
		}
		
		/* Send start streaming message and wait for first state packet to arrive: */
		{
		Threads::MutexCond::Lock packetSignalLock(packetSignalCond);
		pipe->write(MessageIdType(STARTSTREAM_REQUEST));
		pipe->flush();
		// packetSignalCond.wait(packetSignalLock);
		streaming=true;
		}
		}
	else
		{
		/* Just delete the new callback functions: */
		delete newPacketNotificationCallback;
		delete newErrorCallback;
		}
	}

void VRDeviceClient::stopStream(void)
	{
	if(streaming)
		{
		streaming=false;
		if(!connectionDead)
			{
			/* Send stop streaming message: */
			pipe->write(MessageIdType(STOPSTREAM_REQUEST));
			pipe->flush();
			}
		
		/* Delete the callback functions: */
		{
		Threads::Mutex::Lock callbacksLock(callbacksMutex);
		delete packetNotificationCallback;
		packetNotificationCallback=0;
		delete errorCallback;
		errorCallback=0;
		}
		}
	}

void VRDeviceClient::updateDeviceStates(void)
	{
	/* Keep reading from the shared memory segment until successful: */
	unsigned int* counter=stateMemory->getValue<unsigned int>(0);
	char* bufferHalves=stateMemory->getValue<char>(sizeof(ptrdiff_t));
	size_t stateSize=state.getStateSize();
	unsigned int counter0,counter1;
	do
		{
		/* Read the counter to determine which buffer half contains readable data: */
		#if THREADS_CONFIG_HAVE_BUILTIN_ATOMICS
		counter0=__atomic_load_n(counter,__ATOMIC_ACQUIRE);
		#endif
		
		/* Read device states from the readable buffer half: */
		{
		Threads::Mutex::Lock stateLock(stateMutex);
		state.read(bufferHalves+(counter0&0x1U)*stateSize);
		}
		
		/* Read the counter again to detect changes during the read: */
		#if THREADS_CONFIG_HAVE_BUILTIN_ATOMICS
		counter1=__atomic_load_n(counter,__ATOMIC_ACQUIRE);
		#endif
		}
	while(counter0!=counter1);
	}

}
