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

#include "ck4a/CaptureDevice.h"
#include "ck4a/CaptureTypes.h"

#include "cinder/gl/Texture.h"

#include <thread>
#include <mutex>
#include <atomic>

namespace ck4a {

using CaptureAzureKinectRef = std::shared_ptr<class CaptureAzureKinect>;

class CaptureAzureKinect : public CaptureDevice {
public:
	CaptureAzureKinect( CaptureManager *manager );
	~CaptureAzureKinect();

	void init( const ma::Info& info );
	void save( ma::Info& info ) const;
	void uninit();
	void clearData();

	void start();
	void stop();

	bool isEnabled() const { return mEnabled; }
	void setEnabled( bool b );

	const std::string&	getSerialNumber() const		{ return mSerialNumber; }
	bool isSyncMaster() const	{ return mSyncMaster; }

	bool isLogVerboseEnabled() const { return mLogVerbose; }
	void setLogVerboseEnabled( bool b ) { mLogVerbose = b; }

	ci::Surface8u getColorSurfaceCloned() const;
	ci::Channel16u getDepthChannelCloned() const;
	const ci::Surface32f& getTableDepth2d3dSurface() const	{ return mTableDepth2d3dSurface; }

	ci::gl::TextureRef	getColorTexture() const				{ return mColorTexture; }
	ci::gl::TextureRef	getDepthTexture() const				{ return mDepthTexture; }
	ci::gl::TextureRef	getTableDepth2d3dTexture() const	{ return mTableDepth2d3dTexture; }

	//! Returns true if depth buffer is enabled
	bool		isDepthEnabled() const	{ return mDepthEnabled; }
	//! Returns the size of depth buffer, [0,0] if disabled
	ci::ivec2	getDepthSize() const;
	//! Returns the clip planes (in centimeters) for the depth buffer, [0,0] if disabled
	ci::vec2	getDepthClipPlanes() const;
	//! Returns vertical FOV of the d buffer, 0 if disabled
	float		getDepthFov() const;

	void openRecording( const ci::fs::path &filePath );

	const ci::Color& getDebugColor() const	{ return mDebugColor; }

	//! Returns the position of this CaptureDevice relative to the room's origin in centimeters
	const vec3& getPos() const			{ return mPos; }
	const quat& getOrientation() const	{ return mOrientation; }

	void update();

	// ------------------------------------
	// Thread-safe data access

	std::vector<Body> getBodies() const;
	void insertBody( const Body &body );
	// ------------------------------------


	//! if enabled, will draw this capture device's UI to a separate window
	void updateUI();
	//! elements within CaptureManager's UI
	void enabledUI( bool showId = true );

	void setPaused( bool b )	{ mPaused = b; }
	bool isPaused() const		{ return mPaused; }

	bool isUIEnabled() const { return mUIEnabled; }
	void setUIEnabled( bool b )	{ mUIEnabled = b; }

	static void managerUI();
	static int	getNumDevicesInstalled();
	static const char* getSDKVersionString();

private:
	void reinit();
	void threadEntry();

	//! where data processing takes place (async)
	void process();
	//! Returns true if this body should be considered valid
	bool fillBodyFromSkeleton( Body *body, double currentTime, const Body::SmoothParams &smoothParams );

	struct Data;
	std::unique_ptr<Data>	mData; //! contains azure-specific data

	int				mDeviceIndex = -1;
	std::string		mSerialNumber;
	std::string     mHostId; // TODO: consider keeping this on a separate CaptureNetwork subclass of CaptureDevice
	ci::fs::path	mRecordingFilePath;

	ci::Surface8u			mColorSurface;
	ci::Channel16u			mDepthChannel;
	ci::Channel8u			mBodyIndexMapChannel;
	ci::Surface32f			mTableDepth2d3dSurface;
	ci::gl::TextureRef		mColorTexture, mDepthTexture, mTableDepth2d3dTexture, mBodyIndexMapTexture;

	std::map<std::string,Body>	mBodies; //! key: body id

	bool			mEnabled = false;				//! if false, this device will not be opened or started by CaptureManager
	bool            mPaused = false;				//! if the device is running but we're not updating data
	bool			mUIEnabled = true;				//! if UI window will be drawn
	bool			mDepthEnabled = true;			//! if depth buffer is enabled
	DepthMode		mDepthMode = DepthMode::WFovBinned; //! Corresponds to k4a_depth_mode_t
	bool			mColorEnabled = false;			//! if color buffer is enabled
	bool			mBodyTrackingEnabled = true;	//! if body tracking is enabled
	bool			mBodyIndexMapEnabled = false;	//! if the body index map is extracted
	bool            mSyncMaster = false;
	vec3			mPos;
	quat			mOrientation;
	ci::Color		mDebugColor = ci::Color::white();


	bool	mCopyBuffersEnabled = true; // TODO: this needs to be on a per-buffer basis
	double	mTimeLastCapture = -1;
	double 	mTimeLastCaptureFailed = -1.0;
	bool 	mLogVerbose = true;

	mutable std::recursive_mutex	mMutexProcess, mMutexData;
	std::unique_ptr<std::thread>	mThread;
	std::atomic<bool>				mThreadShouldQuit = { false };
	std::atomic<double>				mTimeNeedsReinit = { -1 };
	double							mTimeStartedRunning = -1;
	int64_t							mCurrentBodyFrame = -1; // incremented whenever a new frame is processed on the capture thread
	size_t							mTotalBodiesTrackedLastFrame = 0;
	float							mTrackerTemporalSmoothingFactor = 0.6f;
	// profiling
	std::atomic<double>				mLastProcessDuration = { 0 };
	std::array<float, 180>			mProcessDurationsBuffer;
	size_t							mProcessDurationsBufferIndex = 0;		

	std::atomic<double>				mPlaybackLastCaptureTimestamp = { 0 };
	std::atomic<int64_t>			mSeekTimestep = { -1 };
	std::atomic<bool>				mLoopEnabled = { true };
	PlaybackStatus					mPlaybackStatus = PlaybackStatus::NotLoaded;
};

} // namespace ck4a
