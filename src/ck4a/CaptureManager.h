/*
Copyright (c) 2020-23, Richard Eakin - All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided
that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and
the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "ck4a/CaptureTypes.h"
#include "ck4a/CaptureAzureKinect.h"
#include "mason/Info.h"

#include "cinder/app/KeyEvent.h"

#include <vector>
#include <set>
#include <map>

namespace cinder { namespace osc { 

class Message;
class ReceiverUdp;
class SenderUdp;

} } // namespace cinder::osc

namespace ck4a {

class CaptureManager {
public:
	CaptureManager();
	~CaptureManager();

	void init( const ma::Info& info );
	void save( ma::Info& info ) const;
	void uninit();
	void clearData();

	//! Starts all enabled devices
	void startAll();
	//! Stops all running devices
	void stopAll();

	bool isEnabled() const { return mEnabled; }
	//! Enables or disables CaptureManager. if disabled, will stop all running devices.
	void setEnabled( bool b );
	//!
	bool isPaused() const	{ return mPaused; }
	//!
	void setPaused( bool b );
	//!
	bool isAnyDeviceRunning() const;
	//! Returns true if Devices should run in synchronous mode
	bool isSyncDevicesEnabled() const	{ return mSyncDevicesEnabled; }
	//! When sync mode is enabled, returns true when master should start (it must be last in a k4a multi-device setup)
	bool canSyncMasterStart() const;

	double getMaxSecondsUntilBodyRemoved() const { return mMaxSecondsUntilBodyRemoved; }
	float  getMaxBodyDistance() const	{ return mMaxBodyDistance; }

	void update( double currentTime );

	//! if enabled, will draw this capture device's UI to a separate window
	void updateUI();
	//! elements within CaptureManager's UI
	void enabledUI( float nameOffset = 140 );

	CaptureAzureKinectRef getDevice( const std::string &deviceId ) const;

	const std::vector<CaptureAzureKinectRef>&	getDevices() const	{ return mCaptureDevices; }

	int	getNumDevices() const	{ return (int)mCaptureDevices.size(); }

	void keyDown( ci::app::KeyEvent &event );

	//! Methods called by Devices to inform CaptureManager of updated state
	void onLocalDeviceStatusChanged( CaptureDevice *device );
	//! Methods called when the status changes remotely
	void onRemoteDeviceStatusChanged( std::string id, CaptureDevice::Status status );

	const std::string&	getHostId() const	{ return mHostId; }

	double getCurrentTime() const;

	void sendBodyTracked( const CaptureDevice *device, Body body );

	//! Returns where the bodies from multiple devices will be merged each update.
	bool isMergeMultiDeviceEnabled() const { return mMergeMultiDevice; }
	//! Returns the merged bodies
	const std::vector<Body>& getMergedBodies() const	{ return mMergedBodies; }

private:
	// non-copyable
	CaptureManager( const CaptureManager& ) = delete;
	CaptureManager& operator=( const CaptureManager& ) = delete;

	void initCaptureDevices( const std::vector<ma::Info> &devInfos );
	void initNetworking();
	//! Separated out for ease of hot-reload
	void initOSCListeners();
	void updateNetworking();
	void sendTestMessage();
	void sendTestValue();
	void sendMessage( const ci::osc::Message &msg );
	void receiveBody( const ci::osc::Message &msg );
	void mergeBodies( double currentTime );

	struct Host {
		std::string mId;
		bool mMaster = false;
		std::string mIpAddress;
		int mReceivePort = -1;
		//int mSendPort = -1;
	};

	std::vector<Host>	mHosts; //! All hosts
	std::string			mHostId; //! This host (computer) id
	bool                mMasterHost = false;

	bool	mEnabled = false;
	bool	mAutoStart = true; //! If true, CaptureManager will automatically try to start devices, and if they have shut down will try to restart based on heartbeat
	bool    mNetworkingEnabled = true;
	bool    mPaused = false; //! If true, will leave devices open but data will not be updated (for development)
	bool	mUIEnabled = true;
	bool    mSyncDevicesEnabled = false;
	bool	mMergeMultiDevice = false; //! If true, merges bodies from multiple devices into one container, stored on CaptureManager
	float	mJointDistanceConsideredSame = 15;
	double	mMaxSecondsUntilBodyRemoved = 0.1f;
	float	mMaxBodyDistance = 250;
	double  mHeartbeatSeconds = 2;
	bool	mMergeBodySmoothingEnabled = true;
	bool	mBodyJointFiltersNeedInit = false;
	bool    mVerboseLogging = false;

	DepthMode	mDefaultDepthMode = DepthMode::WFovBinned;

	std::vector<Body>			mMergedBodies;
	std::set<JointType>			mMergeResolveJoints;

	std::vector<CaptureAzureKinectRef>		mCaptureDevices;
	std::vector<ma::Info>					mDeviceInfos;
	std::unique_ptr<ci::osc::ReceiverUdp>	mOSCReceiver;
	std::unique_ptr<ci::osc::SenderUdp>		mOSCSender;
	std::mutex								mMutexSender;
};

} // namespace ck4a
