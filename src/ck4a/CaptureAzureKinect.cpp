/*
Copyright (c) 2020, Richard Eakin - All rights reserved.

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

#if defined( DC_DEBUG_RELEASE )
#pragma optimize( "", off )
#endif

#include "ck4a/CaptureAzureKinect.h"
#include "ck4a/CaptureManager.h"

#include "mason/extra/ImGuiStuff.h"
#include "mason/extra/ImGuiTexture.h"
#include "cinder/Breakpoint.h"
#include "cinder/Log.h"
#include "cinder/Timer.h"
#include "cinder/Utilities.h"

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>
#include <k4abt.hpp>

using namespace ci;
using namespace std;
namespace im = ImGui;

#define LOG_CAPTURE_V( stream )	{ if( mLogVerbose ) {	CI_LOG_I( "|v| " << stream ); } }
//#define LOG_CAPTURE_V( stream )	( (void)( 0 ) )

//#define LOG_CAPTURE_PROCESS( stream )	{ CI_LOG_I( "|process| " << stream ); }
#define LOG_CAPTURE_PROCESS( stream )	( (void)( 0 ) )

namespace ck4a {

namespace {

// TODO: set this from CaptureManager (but also might want to set individually, later though)
//const k4a_depth_mode_t DEPTH_MODE = K4A_DEPTH_MODE_NFOV_UNBINNED;
const k4a_depth_mode_t DEPTH_MODE = K4A_DEPTH_MODE_WFOV_2X2BINNED;

// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;


// taken from k4a::device::get_serialnum()
string getSerialNumberAsString( k4a_device_t deviceHandle )
{
	string serialnum;
	size_t buffer = 0;
	k4a_buffer_result_t result = k4a_device_get_serialnum( deviceHandle, &serialnum[0], &buffer );
	if( result == K4A_BUFFER_RESULT_TOO_SMALL && buffer > 1 ) {
		serialnum.resize( buffer );
		result = k4a_device_get_serialnum( deviceHandle, &serialnum[0], &buffer);
		if( result == K4A_BUFFER_RESULT_SUCCEEDED && serialnum[buffer - 1] == 0 )	{
			// std::string expects there to not be as null terminator at the end of its data but
			// k4a_device_get_serialnum adds a null terminator, so we drop the last character of the string after we
			// get the result back.
			serialnum.resize( buffer - 1 );
		}
	}

	CI_ASSERT( result == K4A_BUFFER_RESULT_SUCCEEDED );
	return serialnum;
}

vec2 toVec2( const k4a_float2_t &v )
{
	return vec2( v.v[0], v.v[1] );
}

vec3 toVec3( const k4a_float3_t &v )
{
	return vec3( v.v[0], v.v[1], v.v[2] );
}

quat toQuat( const k4a_quaternion_t &q )
{
	return quat( q.v[0], q.v[1], q.v[2], q.v[3] );
}

ci::log::Level k4aLogLevelToCinder( k4a_log_level_t level )
{
	switch( level ) {
		case K4A_LOG_LEVEL_CRITICAL:	return ci::log::LEVEL_INFO; // TODO: K4A passes this through when devices are initialized, there are some false positives we may need to filter out somehow
		case K4A_LOG_LEVEL_ERROR:		return ci::log::LEVEL_ERROR;
		case K4A_LOG_LEVEL_WARNING:		return ci::log::LEVEL_WARNING;
		case K4A_LOG_LEVEL_INFO:		return ci::log::LEVEL_INFO;
		case K4A_LOG_LEVEL_TRACE:		return ci::log::LEVEL_DEBUG;
		case K4A_LOG_LEVEL_OFF:			return ci::log::LEVEL_VERBOSE; // nothing in ci::log maps to this
		default:						CI_ASSERT_NOT_REACHABLE();

	}

	return ci::log::LEVEL_INFO;
}

const char* imageFormatToString( k4a_image_format_t format )
{
	switch( format ) {
		case K4A_IMAGE_FORMAT_COLOR_MJPG:	return "K4A_IMAGE_FORMAT_COLOR_MJPG";
		case K4A_IMAGE_FORMAT_COLOR_NV12:	return "K4A_IMAGE_FORMAT_COLOR_NV12";
		case K4A_IMAGE_FORMAT_COLOR_YUY2:	return "K4A_IMAGE_FORMAT_COLOR_YUY2";
		case K4A_IMAGE_FORMAT_COLOR_BGRA32:	return "K4A_IMAGE_FORMAT_COLOR_BGRA32";
		case K4A_IMAGE_FORMAT_DEPTH16:		return "K4A_IMAGE_FORMAT_DEPTH16";
		case K4A_IMAGE_FORMAT_IR16:			return "K4A_IMAGE_FORMAT_IR16";
		case K4A_IMAGE_FORMAT_CUSTOM8:		return "K4A_IMAGE_FORMAT_CUSTOM8";
		case K4A_IMAGE_FORMAT_CUSTOM16:		return "K4A_IMAGE_FORMAT_CUSTOM16";
		case K4A_IMAGE_FORMAT_CUSTOM:		return "K4A_IMAGE_FORMAT_CUSTOM";
		default: CI_ASSERT_NOT_REACHABLE();
	}

	return "(unknown)";
}

const char* colorResolutionToString( k4a_color_resolution_t res )
{
	switch( res ) {
		case K4A_COLOR_RESOLUTION_OFF:		return "K4A_COLOR_RESOLUTION_OFF";
		case K4A_COLOR_RESOLUTION_720P:		return "K4A_COLOR_RESOLUTION_720P";
		case K4A_COLOR_RESOLUTION_1080P:	return "K4A_COLOR_RESOLUTION_1080P";
		case K4A_COLOR_RESOLUTION_1440P:	return "K4A_COLOR_RESOLUTION_1440P";
		case K4A_COLOR_RESOLUTION_1536P:	return "K4A_COLOR_RESOLUTION_1536P";
		case K4A_COLOR_RESOLUTION_2160P:	return "K4A_COLOR_RESOLUTION_2160P";
		case K4A_COLOR_RESOLUTION_3072P:	return "K4A_COLOR_RESOLUTION_3072P";
		default: CI_ASSERT_NOT_REACHABLE();
	}

	return "(unknown)";
}

const char* depthModeToString( k4a_depth_mode_t mode )
{
	switch( mode ) {
		case K4A_DEPTH_MODE_OFF:			return "K4A_DEPTH_MODE_OFF";
		case K4A_DEPTH_MODE_NFOV_2X2BINNED:	return "K4A_DEPTH_MODE_NFOV_2X2BINNED";
		case K4A_DEPTH_MODE_NFOV_UNBINNED:	return "K4A_DEPTH_MODE_NFOV_UNBINNED";
		case K4A_DEPTH_MODE_WFOV_2X2BINNED:	return "K4A_DEPTH_MODE_WFOV_2X2BINNED";
		case K4A_DEPTH_MODE_WFOV_UNBINNED:	return "K4A_DEPTH_MODE_WFOV_UNBINNED";
		case K4A_DEPTH_MODE_PASSIVE_IR:		return "K4A_DEPTH_MODE_PASSIVE_IR";
		default: CI_ASSERT_NOT_REACHABLE();
	}

	return "(unknown)";
}

const char* cameraFpsToString( k4a_fps_t fps )
{
	switch( fps ) {
		case K4A_FRAMES_PER_SECOND_5:		return "K4A_FRAMES_PER_SECOND_5";
		case K4A_FRAMES_PER_SECOND_15:		return "K4A_FRAMES_PER_SECOND_15";
		case K4A_FRAMES_PER_SECOND_30:		return "K4A_FRAMES_PER_SECOND_30";
		default: CI_ASSERT_NOT_REACHABLE();
	}

	return "(unknown)";
}

const char* wiredSyncModeToString( k4a_wired_sync_mode_t mode )
{
	switch( mode ) {
		case K4A_WIRED_SYNC_MODE_STANDALONE:		return "K4A_WIRED_SYNC_MODE_STANDALONE";
		case K4A_WIRED_SYNC_MODE_MASTER:		return "K4A_WIRED_SYNC_MODE_MASTER";
		case K4A_WIRED_SYNC_MODE_SUBORDINATE:		return "K4A_WIRED_SYNC_MODE_SUBORDINATE";
		default: CI_ASSERT_NOT_REACHABLE();
	}

	return "(unknown)";
}

//! Creates and returns an RGB32F surface that contains an xy table for mapping depth values from 2d to 3d coordinate space
Surface32f makeTableDepth2dTo3d( const k4a::calibration &calibration )
{
	int width = calibration.depth_camera_calibration.resolution_width;
	int height = calibration.depth_camera_calibration.resolution_height;
	Surface32f surface( width, height, false );

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	auto iter = surface.getIter();
	while( iter.line() ) {
		p.xy.y = (float)iter.y();
		while( iter.pixel() ) {
			p.xy.x = (float)iter.x();
			k4a_calibration_2d_to_3d( &calibration, &p, 1.0f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid );
			//CI_LOG_I( "[" << p.xy.x << ", " << p.xy.y << "]: ray: (" << ray.xyz.x << ", " << ray.xyz.y << ", " << ray.xyz.z << ")" );
			if( valid ) {
				iter.r() = ray.xyz.x;
				iter.g() = ray.xyz.y;
			}
			else {
				iter.r() = nanf("");
				iter.g() = nanf("");
			}

			iter.b() = 0;
		}
	}

	return surface;
}

} // anon

//! Singleton to manager k4a devices
// TODO: Globally store device handles on DeviceInfo
// - in order to query serials and open / close 
// - can only store one k4a::device, so might need to use the raw handle / switch to c api
// - see how k4a handles this. They refresh only?
// - this is going to require moving device init / uninit to Manager, called from CaptureAzureKinect init / uninit
class AzureKinectManager {
public:
	struct DeviceInfo {
		std::string mSerial;
		bool		mOpen = false; // TODO: use this to tell if the device is currently in use
	};

	AzureKinectManager();
	~AzureKinectManager();

	void setLogMessageLevel( k4a_log_level_t level );
	void buildDeviceSerialMap();

	//! Returns the device index with this serial number, of -1 if none was found
	int getDeviceIndex( const std::string &serialNumber ) const;

	static void onLogMessage( void *context, k4a_log_level_t level,	const char *file, const int line, const char *message );

	std::vector<DeviceInfo> mDeviceInfos;
};


AzureKinectManager& manager()
{
	static AzureKinectManager sInstance;
	return sInstance;
}

AzureKinectManager::AzureKinectManager()
{
	setLogMessageLevel( K4A_LOG_LEVEL_WARNING );
	buildDeviceSerialMap();
}

AzureKinectManager::~AzureKinectManager()
{
}

void AzureKinectManager::setLogMessageLevel( k4a_log_level_t level )
{
	k4a_result_t result = k4a_set_debug_message_handler( &AzureKinectManager::onLogMessage, this, level );
	CI_VERIFY( result == K4A_RESULT_SUCCEEDED );
}

void AzureKinectManager::buildDeviceSerialMap()
{
	// FIXME: this won't work when a different process on this PC has already opened the device
	// - not sure what to do here, other than send the serial number over osc and append it to our vector there
	//    - this would mean they should be stored in a map, and it's way more complicated
	// - consider opening a feature request to be able to query the serial number before opening the device

	uint32_t numDevices = (uint32_t)CaptureAzureKinect::getNumDevicesInstalled();
	mDeviceInfos.clear();
	for( uint32_t i = 0; i < numDevices; i++ ) {
		k4a_device_t deviceHandle = NULL;
		if( K4A_RESULT_SUCCEEDED != k4a_device_open( i, &deviceHandle ) )	{
			CI_LOG_E( "Failed to open device at index: " << i );
			continue;
		}

		DeviceInfo info;
		info.mSerial = getSerialNumberAsString( deviceHandle );
		mDeviceInfos.push_back( info );

		k4a_device_close( deviceHandle );
	}

	CI_LOG_I( "total devices: " << mDeviceInfos.size() );
}

int AzureKinectManager::getDeviceIndex( const std::string &serialNumber ) const
{
	for( size_t i = 0; i < mDeviceInfos.size(); i++ ) {
		if( mDeviceInfos[i].mSerial == serialNumber ) {
			return (int)i;
		}
	}

	CI_LOG_W( "no device found with serial number: " << serialNumber );
	return -1;
}

//static 
void AzureKinectManager::onLogMessage( void *context, k4a_log_level_t level, const char *file, const int line, const char *message )
{
	auto cinderLevel = k4aLogLevelToCinder( level );
	if( (int)cinderLevel == (int)ci::log::LEVEL_ERROR ) {
		int blarg = 2;
	}

	ci::log::Entry( cinderLevel, ci::log::Location( CINDER_CURRENT_FUNCTION, file, line ) ) << message;
}

struct CaptureAzureKinect::Data {
	k4a::device		mDevice;
	k4abt::tracker	mTracker;
	k4a_device_configuration_t mDeviceConfig; // stored mostly for debugging reasons

	std::map<string, k4abt_skeleton_t> mSkeletons; //! stored each process so that parsing tweaks can continue while paused
};

// static
int	CaptureAzureKinect::getNumDevicesInstalled()
{
	return (int)k4a::device::get_installed_count();
}

// static
const char* CaptureAzureKinect::getSDKVersionString()
{
	return K4A_VERSION_STR;
}

CaptureAzureKinect::CaptureAzureKinect( CaptureManager *captureManager )
	: CaptureDevice( captureManager ), mData( new Data )
{
	manager(); // init global AzureKinectManager
}

CaptureAzureKinect::~CaptureAzureKinect()
{
	uninit();
}

void CaptureAzureKinect::init( const ma::Info &info )
{
	CI_ASSERT( ! isInitialized() );

	mTimeLastCaptureFailed = -1;
	mTimeLastCapture = -1;

	mId = info.get<string>( "id" );
	mEnabled = info.get( "enabled", mEnabled );
	mUIEnabled = info.get( "ui", mUIEnabled );
	mDepthEnabled = info.get( "depth", mDepthEnabled ); // TODO: pass in a depth / color format instead, or "disabld"
	mColorEnabled = info.get( "color", mColorEnabled );
	mBodyTrackingEnabled = info.get( "bodyTracking", mBodyTrackingEnabled );
	mDebugColor = info.get( "debugColor", mDebugColor );
	mPos = info.get( "pos", mPos );
	mOrientation = info.get( "orientation", mOrientation );

	if( info.contains( "hostId" ) ) {
		mHostId = info.get<string>( "hostId" );
		if( getManager()->getHostId() != mHostId ) {
			CI_LOG_I( "[" << mId << "] hostId '" << mHostId << "' not this process, will run remote." );
			mRemote = true;
		}
	}

	// load device by looking for the following keys in this order:
	// - "serial", "deviceIndex", or if neither exist then use the default
	if( info.contains( "serial" ) ) {
		mSerialNumber = info.get<string>( "serial" );
		mDeviceIndex = manager().getDeviceIndex( mSerialNumber );
		if( mEnabled && mDeviceIndex == -1 && ! mRemote ) {
			// TODO: this will need to be reworked for when devices are running remote
			// - for now, just log and return
			CI_LOG_W( "no device present with the specified serial number (" << mSerialNumber << ")" );
			return;
		}
	}
	else {
		mDeviceIndex = info.get<uint32_t>( "deviceIndex", K4A_DEVICE_DEFAULT );
	}

	if( mRemote ) {
		CI_LOG_I( "remote device '" << mId << "marked as stopped" );
		setStatus( Status::Stopped );
		return;
	}

	if( mEnabled && mDeviceIndex >= getNumDevicesInstalled() ) {
		CI_LOG_E( "device index (" << mDeviceIndex << ") out of range (" << k4a::device::get_installed_count() << ")" );

		setStatus( Status::Failed );
		// TODO: rename this to timeLastInit? and wait on that for autoStart?
		// - this seems nice, because then I can handle on the re-init stuff from CaptureManager
		mTimeNeedsReinit = getManager()->getCurrentTime();
		return;
	}

	if( mEnabled ) {
		try {
			mData->mDevice = k4a::device::open( mDeviceIndex );
			if( mSerialNumber.empty() ) {
				mSerialNumber = mData->mDevice.get_serialnum();
			}
			LOG_CAPTURE_V( "opened device at index: " << mDeviceIndex << ", serial number: " << mSerialNumber );
		}
		catch( exception &exc ) {
			CI_LOG_EXCEPTION( "Failed to open device with index: " << mDeviceIndex, exc );
			mTimeNeedsReinit = getManager()->getCurrentTime();
			setStatus( Status::Failed );
			return;
		}

		// decide which device is master basd on whether it has sync out connected and sync in disconnected
		// TODO: add sanity check in CaptureManager that there aren't two 'masters'
		// - will do this where I find and store the master
		bool syncInConnected, syncOutConnected;
		k4a_result_t result = k4a_device_get_sync_jack( mData->mDevice.handle(), &syncInConnected, &syncOutConnected );
		CI_VERIFY( result == K4A_RESULT_SUCCEEDED );
		mSyncMaster = syncOutConnected && ! syncInConnected;

		if( getManager()->isSyncDevicesEnabled() ) {
			// TODO: expose in CaptureManager gui, get settings from him
			int32_t colorExposureUsec = 8000;  // somewhat reasonable default exposure time
			int32_t powerlineFreq = 2;          // default to a 60 Hz powerline

			try {
				// If you want to synchronize cameras, you need to manually set both their exposures
				mData->mDevice.set_color_control( K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, colorExposureUsec );
				// This setting compensates for the flicker of lights due to the frequency of AC power in your region. If you are in an area with 50 Hz power, this may need to be updated
				mData->mDevice.set_color_control( K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, powerlineFreq );
			}
			catch( exception &exc ) {
				CI_LOG_E( "Failed to set color control for device with index: " << mDeviceIndex );
			}
		}

		setStatus( Status::Stopped );
		LOG_CAPTURE_V( "Device with id: " << mId << ", index: " << mDeviceIndex << " initialized. sync master: " << mSyncMaster );
	}
	else {
		setStatus( Status::Disabled );
		LOG_CAPTURE_V( "Device with id: " << mId << " disabled." );
	}

}

void CaptureAzureKinect::save( ma::Info &info ) const
{
	info["serial"] = mSerialNumber;
	info["deviceIndex"] = mDeviceIndex;
	info["id"] = mId;
	info["enabled"] = mEnabled;
	info["ui"] = mUIEnabled;
	info["depth"] = mDepthEnabled;
	info["color"] = mColorEnabled;
	info["bodyTracking"] = mBodyTrackingEnabled;
	info["debugColor"] = mDebugColor;
	info["pos"] = mPos;
	info["orientation"] = mOrientation;

	if( ! mHostId.empty() ) {
		info["hostId"] = mHostId;
	}
}

void CaptureAzureKinect::uninit()
{
	stop();

	if( mData->mTracker ) {
		mData->mTracker.destroy();
		mData->mTracker = {};
		mCurrentBodyFrame = -1;
	}
	if( mData->mDevice ) {
		mData->mDevice.close();
		mData->mDevice = {};
	}

	clearData();
	setStatus( Status::Uninitialized );
	LOG_CAPTURE_V( "complete." );
}

void CaptureAzureKinect::reinit()
{
	// first serialize current params
	ma::Info info;
	save( info );

	uninit();
	init( info );
}

void CaptureAzureKinect::clearData()
{
	lock_guard<recursive_mutex> lock( mMutexData );

	mColorSurface = {};
	mColorTexture = {};
	mDepthTexture = {};
	mDepthTexture = {};
	mTableDepth2d3dTexture = {};

	mBodies.clear();
	mData->mSkeletons.clear();
}

void CaptureAzureKinect::setEnabled( bool b )
{ 
	if( mEnabled == b ) {
		return;
	}

	mEnabled = b;
	reinit();
}

void CaptureAzureKinect::start()
{
	if( isStarted() ) {
		CI_LOG_W( "already started, returning." );
		return;
	}
	if( ! mEnabled ) {
		CI_LOG_W( "cannot start when disabled, returning." );
		return;
	}
	if( isRemote() ) {
		CI_LOG_W( "Device (" << mId << ") set to run remote, returning." );
		return;
	}

	CI_ASSERT( ! mThread );
	setStatus( Status::Started );
	mThreadShouldQuit = false;
	mThread = make_unique<std::thread>( &CaptureAzureKinect::threadEntry, this );
}

void CaptureAzureKinect::stop()
{
	if( ! isRunning() ) {
		return;
	}

	if( mThread && mThread->joinable() ) {
		LOG_CAPTURE_V( "Joining thread for device: " << mId << "..." );
		mThreadShouldQuit = true;
		if( mThread->joinable() ) {
			mThread->join();
		}
	}
	mThread = {};

	// mDevice and mTracker are created async once started, so delete them here
	if( mData->mTracker ) {
		mData->mTracker.shutdown();
		mData->mTracker = {};
	}
	if( mData->mDevice ) {
		mData->mDevice.stop_cameras();
	}

	clearData();
	setStatus( Status::Stopped );
	LOG_CAPTURE_V( "Camera stopped for device: " << mId );
}

// ----------------------------------------------------------------------------------------------------
// Background procssing thread functions
// ----------------------------------------------------------------------------------------------------

void CaptureAzureKinect::threadEntry()
{
	ci::setThreadName( "CaptureAzureKinect (" + mId + ")" );
	LOG_CAPTURE_V( "background thread running for device: " << mId );

	CI_ASSERT( getStatus() == Status::Started );

	if( mSyncMaster && getManager()->isSyncDevicesEnabled() ) {
		// Wait until CaptureManager tells us that all subordinates have started
		while( ! getManager()->canSyncMasterStart() ) {
			std::this_thread::sleep_for( chrono::milliseconds( 100 ) );
		}
	}

	Timer timer( true );

	// configure stream
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	// TODO: add params for color and depth format and resolution instead (one being disabled)
	// - will set in "capture" section, although maybe merge "device" params into this
	if( mColorEnabled ) {
		deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	}
	if( mDepthEnabled ) {
		deviceConfig.depth_mode = DEPTH_MODE;
	}

	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30; // TODO: expose as param
	deviceConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

	if( getManager()->isSyncDevicesEnabled() ) {
		deviceConfig.subordinate_delay_off_master_usec = 0;     // Must be zero for master, subordinates should only use this if they use a different exposure setting than master's (se green_screen docs)

		// Even if color is disabled, we still need to enable a color buffer (see docs on K4A_WIRED_SYNC_MODE_MASTER)
		if( ! mColorEnabled ) {
			// TODO: mark mColorEnabled = true here
			// - will do this once I can use MJPG and use that for in the texture viewer
			deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
			deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
		}

		if( mSyncMaster ) {
			deviceConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
			//deviceConfig.synchronized_images_only = true; // TODO: might want this if color is enabled (was only set on master in green_screen, not sure why not all
		}
		else {
			deviceConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
			deviceConfig.depth_delay_off_color_usec = MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC * mDeviceIndex;
		}

		// TODO: init powerline and exposure settings
	}

	mData->mDeviceConfig = deviceConfig;

	try {
		mData->mDevice.start_cameras( &deviceConfig );		
		LOG_CAPTURE_V( "Camera started for device: " << mId << " in << " << timer.getSeconds() << " seconds." );
	}
	catch( exception & exc ) {
		CI_LOG_EXCEPTION( "failed to start Camera with id: " << mId, exc );
		mTimeNeedsReinit = getManager()->getCurrentTime();
		return;
	}

	timer.stop();
	timer.start();

	if( mBodyTrackingEnabled ) {
		// TODO: make sure depth is enabled and set to a suitable mode during init()
		// - right now I assume it is
		// https://docs.microsoft.com/en-us/azure/kinect-dk/build-first-body-app#open-device-and-start-the-camera
		try {
			k4a::calibration calibration = mData->mDevice.get_calibration( mData->mDeviceConfig.depth_mode, mData->mDeviceConfig.color_resolution );
			mData->mTracker = k4abt::tracker::create( calibration, K4ABT_TRACKER_CONFIG_DEFAULT );
			mCurrentBodyFrame = 0;
			mTotalBodiesTrackedLastFrame = 0;
			LOG_CAPTURE_V( "\t- created body tracker in " << timer.getSeconds() << " seconds." );
		}
		catch( exception & exc ) {
			CI_LOG_EXCEPTION( "failed to create body tracker for Camera with id: " << mId, exc );
		}
	}

	mTimeStartedRunning = getManager()->getCurrentTime();
	setStatus( Status::Running );
	while( true ) {
		unique_lock<recursive_mutex> lock( mMutexProcess );
		if( mThreadShouldQuit )
			break;

		if( ! mPaused ) {
			Timer timer( true );
			process();
			mLastProcessDuration = timer.getSeconds();
		}
		else {
			std::this_thread::sleep_for( chrono::milliseconds( 500 ) );
		}
	}

	LOG_CAPTURE_V( "background thread ending for device: " << mId );
}

void CaptureAzureKinect::process()
{
	CI_ASSERT( mData->mDevice );
	CI_ASSERT( mEnabled );
	CI_ASSERT( isRunning() );

	// TODO: make this a deviceConfig param
	// - use heartbeat?
	const auto waitTimeout = chrono::milliseconds( 2000 );

	k4a::capture capture;
	if( ! mData->mDevice.get_capture( &capture, waitTimeout ) ) {
		mTimeLastCaptureFailed = getManager()->getCurrentTime();
		mTimeNeedsReinit = mTimeLastCapture;
		CI_LOG_E( "get_capture() failed" );
		return;
	}

	mTimeLastCapture = getManager()->getCurrentTime();

	LOG_CAPTURE_PROCESS( "time: " << mTimeLastCapture );

	// TODO: need to decide how to expose copying depth versus color buffers in settings
	// - right now only copies if ui is ticked
	// - but also don't want to force copying the color buffer if depth texture is needed
	// - color index should also be considered here
	if( mCopyBuffersEnabled ) {
		lock_guard<recursive_mutex> lock( mMutexData );
		
		// TODO: add separate background CpuProfiler
		//CI_PROFILE_CPU( "Capture Buffers (" + mId + ")" );

		// store color image as a Surface8u
		{
			auto image = capture.get_color_image();
			if( image ) {
				uint8_t *data = image.get_buffer();
				auto format = image.get_format();
				if( format == K4A_IMAGE_FORMAT_COLOR_MJPG ) {
					size_t dataSize = image.get_size();
					auto dataSource = DataSourceBuffer::create( make_shared<Buffer>( data, dataSize ), ".jpeg" );
					mColorSurface = Surface8u( loadImage( dataSource, ImageSource::Options(), ".jpeg" ) );
				}
				else {
					int width = image.get_width_pixels();
					int height = image.get_height_pixels();
					int stride = image.get_stride_bytes();

					Surface8u surface( data, width, height, stride, SurfaceChannelOrder::BGRA );
					mColorSurface = surface.clone( true );
				}
			}
		}

		// store depth image as a Channel16u
		{
			auto image = capture.get_depth_image();
			if( image ) {
				uint16_t *data = (uint16_t *)image.get_buffer();
				int width = image.get_width_pixels();
				int height = image.get_height_pixels();
				int stride = image.get_stride_bytes();

				Channel16u channel( width, height, stride, 1, data );
				mDepthChannel = channel.clone( true );
			}
		}

	}

	if( mBodyTrackingEnabled && mData->mTracker ) {
		//CI_PROFILE( "Body Tracking (" + mId + ")" );

		// process body tracking
		bool success = false;
		try {
			success = mData->mTracker.enqueue_capture( capture, waitTimeout );
		}
		catch( exception &exc ) {
			CI_LOG_EXCEPTION( "mTracker.enqueue_capture failed", exc );
			mTimeLastCaptureFailed = getManager()->getCurrentTime();
		}

		if( success ) {
			// successful enqueue - now see if we have a frame ready
			k4abt::frame frame;
			if( mData->mTracker.pop_result( &frame, waitTimeout ) ) {
				// fill mSkeletons with new data from this frame
				uint32_t numBodies = frame.get_num_bodies();
				mData->mSkeletons.clear();
				for( uint32_t bodyIndex = 0; bodyIndex < numBodies; bodyIndex++ ) {
					auto bodyId = frame.get_body_id( bodyIndex );
					k4abt_skeleton_t skeleton;
					frame.get_body_skeleton( bodyIndex, skeleton );
					mData->mSkeletons.insert( { to_string( bodyId ), skeleton } );
				}

				map<string,Body> bodies;
				// copy bodies for processing
				{
					lock_guard<recursive_mutex> lock( mMutexData );

					for( auto &body : mBodies ) {
						body.second.mActive = false;
					}

					bodies = mBodies;
				}

				double currentTime = getManager()->getCurrentTime();
				for( uint32_t bodyIndex = 0; bodyIndex < numBodies; bodyIndex++ ) {
					string bodyId = to_string( frame.get_body_id( bodyIndex ) );

					// see if we already got a body with this id, if not add a new one
					Body* body = nullptr;
					auto bodyIt = bodies.find( bodyId );
					if( bodyIt != bodies.end() ) {
						body = &bodyIt->second;
					}
					else {
						Body newBody;
						newBody.mId = bodyId;
						newBody.mTimeFirstTracked = getManager()->getCurrentTime();
						auto resultIt = bodies.insert( { bodyId, newBody } ).first;
						body = &resultIt->second;
					}

					body->mActive = true;
					body->mTimeLastTracked = getManager()->getCurrentTime();
					body->mFrameLastTracked = mCurrentBodyFrame;

					if( fillBodyFromSkeleton( body, currentTime ) ) {
						getManager()->sendBodyTracked( this, *body );
					}
					else{
						LOG_CAPTURE_PROCESS( "Warning: detected unusable body. Device id: " << mId << ", body id: " << body->getId() );
						bodies.erase( bodyId );
					}
				}

				// copy bodies to synchronized container
				{
					lock_guard<recursive_mutex> lock( mMutexData );
					mBodies = bodies;
					mTotalBodiesTrackedLastFrame = numBodies;
				}

				mCurrentBodyFrame += 1; // keep track of how many body frames we've processed
			}

		}
	}
}

// Returns false if the body should be rejected.
// info on Azure Kinect coordinate space: https://docs.microsoft.com/en-us/azure/kinect-dk/coordinate-systems
bool CaptureAzureKinect::fillBodyFromSkeleton( Body *body, double currentTime )
{
	CI_ASSERT( ! body->mId.empty() );

	const auto &skeleton = mData->mSkeletons.at( body->mId );
	bool hasGoodJoint = false; // only accept a body if it has at last one good joint
	for( int i = 0; i < K4ABT_JOINT_COUNT; i++ ) {
		const auto &jointKinect = skeleton.joints[i];
		JointType jointType = (JointType)i;
		ck4a::Joint &joint = body->mJoints[jointType];

		// note: currently JointType and JointConfidence maps 1:1 to k4abt
		// - if this changes need separate map fns
		joint.mType = jointType; 
		joint.mConfidence = (JointConfidence)jointKinect.confidence_level;

		joint.mPos = toVec3( jointKinect.position );	
		joint.mPos /= 10; // k4a uses millimeters for 3D joint positions, we use centimeters
		joint.mPos = vec3( vec4( joint.mPos, 1 ) * mOrientation ); // rotate relative to room

		joint.mPos *= vec3( -1, -1, 1 ); // flip x and y axes
		joint.mPos += mPos; // translate relative to room


		// DEBUG: hacking pos.x to be a certain shape when tracked, for gesture handwave work
		if( 0 ) {
			float x;

			//x = sin( getManager()->getCurrentTime() * 3.0 );
			//x = lmap<float>( x, -1, 1, 7, 75 );

			x = fmod( getManager()->getCurrentTime() * 1.0, 2.0f ) - 1.0f;
			x = fabsf( x );
			x = lmap<float>( x, 0, 1, 7, 75 );

			joint.mPos.x = x;
		}


		// joint orientation:
		// - start with an orientation to go from kinect camera -> opengl
		// - then apply the rotation that goes from the t-pose image to the depth cam's coordinate system
		auto fromKinectRot = glm::angleAxis( glm::radians( 180.0f ), vec3( 0, 0, 1 ) );
		auto q = fromKinectRot * toQuat( jointKinect.orientation );

		// - at this point, the head is looking down the positive y axis. So we rotate
		//   the head looks down the -z axis
		q *= glm::angleAxis( glm::radians( 90.0f ), vec3( 1, 0, 0 ) );
		q *= glm::angleAxis( glm::radians( -90.0f ), vec3( 0, 0, 1 ) );

		joint.mOrientation = normalize( q );

		// TODO: make sure motion tracking accounts for joints not being present
		if( (int)joint.mConfidence >= (int)JointConfidence::Medium ) {
			hasGoodJoint = true;
			if( currentTime >= 0 ) {
				joint.mMotionTrackerPos.storePos( joint.mPos, currentTime );
				joint.mVelocity = joint.mMotionTrackerPos.calcVelocity();
			}
		}

		//joint.mTimeFirstTracked = mTimeLastCapture;
	}

	if( ! hasGoodJoint ) {
		// no good joint found, reject
		return false;
	}

	// pick a center joint based on what's present
	// TODO: make this configurable
	static vector<JointType> centerJointDandidates = {
		ck4a::JointType::SpineNavel,
		ck4a::JointType::Head,
		ck4a::JointType::SpineChest,
		ck4a::JointType::Neck
	};

	for( const auto &candidateType : centerJointDandidates ) {
		// only use a candidate if the confidence is medium
		auto jointIt = body->mJoints.find( candidateType );
		if( jointIt != body->mJoints.end() && (int)jointIt->second.mConfidence >= (int)JointConfidence::Medium ) {
			body->mCenterJointType = candidateType;
			break;
		}
	}
	if( body->mCenterJointType == JointType::Unknown ) {
		body->mCenterJointType = body->mJoints.begin()->first;
	}

	const float maxDistance = getManager()->getMaxBodyDistance();
	if( maxDistance < 0 ) {
		// max body distance filtering disabled, consider all bodies
		return true;
	}

	auto center = body->getCenterJoint();
	vec2 centerXZ( center->mPos.x, center->mPos.z );
	if( glm::length( centerXZ ) > maxDistance ) {
		// center joint too far away, reject
		return false;
	}

	// This body is a good one.
	return true;
}

// ----------------------------------------------------------------------------------------------------
// Thread-safe  data access
// ----------------------------------------------------------------------------------------------------

std::vector<Body> CaptureAzureKinect::getBodies() const
{
	lock_guard<recursive_mutex> lock_guard( mMutexData );

	vector<Body> result;
	for( const auto &body : mBodies ) {
		result.push_back( body.second );
	}

	return result;
}

// ----------------------------------------------------------------------------------------------------
// methods called from main update loop
// ----------------------------------------------------------------------------------------------------

void CaptureAzureKinect::openRecording( const ci::fs::path &filePath )
{
	// TODO NEXT: open a mkv file and play it, instead of camera
	// GOAL: read a frame and print it to screen
	
	if( ! fs::exists( filePath ) ) {
		CI_LOG_E( "filepath doesn't exist: " << filePath );
		return;
	}
	// https://docs.microsoft.com/en-us/azure/Kinect-dk/record-playback-api
	k4a_playback_t playback_handle = NULL;
	if( k4a_playback_open( filePath.string().c_str(), &playback_handle ) ) {
		CI_LOG_E( "Failed to option k4a playback file at path: " << filePath );
		return;
	}

	uint64_t recording_length = k4a_playback_get_last_timestamp_usec( playback_handle );
	CI_LOG_I( "Recording is " << recording_length / 1000000 << " seconds long." );



	k4a_playback_close( playback_handle );
}

ci::Surface8u CaptureAzureKinect::getColorSurfaceCloned() const
{
	lock_guard<recursive_mutex> lock( mMutexData );
	return mColorSurface.clone( true );
}

ci::Channel16u CaptureAzureKinect::getDepthChannelCloned() const
{
	lock_guard<recursive_mutex> lock( mMutexData );
	return mDepthChannel.clone( true );
}

void CaptureAzureKinect::update()
{
	double currentTime = getManager()->getCurrentTime();

	// TODO: move this to CaptureManager, and use current Status
	//const double heartbeatSeconds = 5.0;
	//if( mTimeNeedsReinit > 0 && currentTime - mTimeNeedsReinit > heartbeatSeconds ) {
	//	mTimeNeedsReinit = currentTime; // update this so we don't try until another heartbeatSeconds
	//	reinit();
	//}

	if( ! mEnabled ) {
		return;
	}

	{
		double maxSeconds = getManager()->getMaxSecondsUntilBodyRemoved();

		// erase elements that are older than max age
		lock_guard<recursive_mutex> lock( mMutexData );
		for( auto it = mBodies.begin(); it != mBodies.end(); /* */ ) {
			auto &body = it->second;
			if( mPaused ) {
				// re-fill body data so the data is using the latest params (calibration, etc)
				if( ! fillBodyFromSkeleton( &body, -1.0 ) ) {
					LOG_CAPTURE_V( "(" << mId << ") body with id: " << it->first << " rejected." );
					it = mBodies.erase( it );
				}
				else {
					++it;
				}
			}
			else if( currentTime - it->second.mTimeLastTracked > maxSeconds ) {
				LOG_CAPTURE_V( "(" << mId << ") body with id: " << it->first << " expiring after " << currentTime - it->second.mTimeLastTracked << " seconds." );
				it = mBodies.erase( it );
			}
			else {
				// TODO: update joint velocities here?
				++it;
			}
		}
	}
	
	if( mCopyBuffersEnabled ) {
		lock_guard<recursive_mutex> lock_guard( mMutexData );

		//MA_PROFILE( "CaptureAzureKinect::update (" + mId + ")" );

		// TODO: profile the update() route versus Texture::create()
		// - I found that the update() was taking +1ms per frame compared to recreating each frame
		if( mColorSurface.getSize() != ivec2( 0 ) ) {
			if( mColorTexture && mColorTexture->getSize() == mColorSurface.getSize() ) {
				mColorTexture->update( mColorSurface );
			}
			else {
				auto format = gl::Texture2d::Format().label( "Capture - color (" + mId + ")" );
				mColorTexture = gl::Texture::create( mColorSurface, format );
			}
		}

		if( mDepthChannel.getSize() != ivec2( 0 ) ) {
			if( mDepthTexture && mDepthTexture->getSize() == mDepthChannel.getSize() ) {
				mDepthTexture->update( mDepthChannel );
			}
			else {
				auto format = gl::Texture2d::Format().label( "Capture - depth (" + mId + ")" )
					.internalFormat( GL_R16UI );
				mDepthTexture = gl::Texture::create( mDepthChannel, format );
			}
		}

		// create conversion table. TODO: make this optional, it is only necessary for point cloud stuff
		if( mDepthEnabled && ! mTableDepth2d3dTexture ) {
			k4a::calibration calibration = mData->mDevice.get_calibration( mData->mDeviceConfig.depth_mode, mData->mDeviceConfig.color_resolution );
			mTableDepth2d3dSurface = makeTableDepth2dTo3d( calibration );
			auto format = gl::Texture2d::Format().label( "Capture - table depth 2d->3d (" + mId + ")" )
				.internalFormat( GL_RGB32F )
				.minFilter( GL_LINEAR ).magFilter( GL_LINEAR );
			//format.dataType(GL_FLOAT); // TODO: remove if not needed
			mTableDepth2d3dTexture = gl::Texture::create( mTableDepth2d3dSurface, format );
		}
	}

	// store profiling information every frame
	mProcessDurationsBuffer[mProcessDurationsBufferIndex++ % mProcessDurationsBuffer.size()] = float( mLastProcessDuration * 1000.0 );
}

void CaptureAzureKinect::updateUI()
{
	if( ! mUIEnabled )
		return;

	if( ! im::Begin( mId.c_str(), &mUIEnabled, ImGuiWindowFlags_HorizontalScrollbar ) ) {
		im::End();
		return;
	}

	enabledUI( false );
	im::Separator();

	im::Text( "status: %s", getStatusAsString() );
	im::Text( "serial: %s", mSerialNumber.c_str() );
	if( ! mHostId.empty() ) {
		const char *remoteOrLocal = isRemote() ? "Remote" : "Local";
		im::Text( "%s host id: %s", remoteOrLocal, mHostId.c_str() );
	}
	if( mData->mDevice ) {
		im::TextColored( mDebugColor, "azure kinect index: %d, serial number: %s", mDeviceIndex, mSerialNumber.c_str() );
		if( im::CollapsingHeader( "extended info", ImGuiTreeNodeFlags_DefaultOpen ) ) {

			// TODO: show entire k4a_device_configuration_t here
			bool syncInConnected, syncOutConnected;
			k4a_result_t result = k4a_device_get_sync_jack( mData->mDevice.handle(), &syncInConnected, &syncOutConnected );
			im::Text( "sync in: %d, sync out: %d, master: %d", syncInConnected, syncOutConnected, mSyncMaster );

			if( ImGui::TreeNode( "Device configuration" ) ) {
				const auto &config = mData->mDeviceConfig;
				im::Text( "color_format: %s", imageFormatToString( config.color_format ) );
				im::Text( "color_resolution: %s", colorResolutionToString( config.color_resolution ) );
				im::Text( "depth_mode: %s", depthModeToString( config.depth_mode ) );
				im::Text( "camera_fps: %s", cameraFpsToString( config.camera_fps ) );
				im::Text( "synchronized_images_only: %d", config.synchronized_images_only );
				im::Text( "depth_delay_off_color_usec: %d", config.depth_delay_off_color_usec );
				im::Text( "wired_sync_mode: %s", wiredSyncModeToString( config.wired_sync_mode ) );
				im::Text( "subordinate_delay_off_master_usec: %d", config.subordinate_delay_off_master_usec );
				im::Text( "disable_streaming_indicator: %d", config.disable_streaming_indicator );
				ImGui::TreePop();
			}

			if( ImGui::TreeNode( "Device Firmware Version Info" ) ) {
				k4a_hardware_version_t versionInfo;
				result = k4a_device_get_version( mData->mDevice.handle(), &versionInfo );
				if( result == K4A_RESULT_SUCCEEDED ) {
					ImGui::Text( "RGB camera: %u.%u.%u", versionInfo.rgb.major, versionInfo.rgb.minor, versionInfo.rgb.iteration );
					ImGui::Text( "Depth camera: %u.%u.%u",
						versionInfo.depth.major,
						versionInfo.depth.minor,
						versionInfo.depth.iteration );
					ImGui::Text( "Audio: %u.%u.%u", versionInfo.audio.major, versionInfo.audio.minor, versionInfo.audio.iteration );

					ImGui::Text( "Build Config: %s", versionInfo.firmware_build == K4A_FIRMWARE_BUILD_RELEASE ? "Release" : "Debug" );
					ImGui::Text( "Signature type: %s",
						versionInfo.firmware_signature == K4A_FIRMWARE_SIGNATURE_MSFT ?
						"Microsoft" :
						versionInfo.firmware_signature == K4A_FIRMWARE_SIGNATURE_TEST ? "Test" : "Unsigned" );

				}
				else {
					im::TextColored( Color( 1, 0, 0 ), "Failed to query Firmware Version info" );
				}
				ImGui::TreePop();
			}
		}
	}
	else if( mEnabled && ! isRemote() ) {
		im::TextColored( Color( 1, 0, 0 ), "null mDevice" );
	}

	if( mTimeStartedRunning >= 0 ) {
		im::Text( "| started running: %.3fs", (float)mTimeStartedRunning );
	}
	if( mTimeLastCapture >= 0 ) {
		im::SameLine();
		im::Text( "| last capture: %.3fs", (float)mTimeLastCapture );
	}
	if( mTimeLastCaptureFailed >= 0 ) {
		float g = glm::clamp<float>( ( getManager()->getCurrentTime() - mTimeLastCaptureFailed ) / 5.0f, 0.0f, 1.0f );
		im::TextColored( Color( 1, g, 0 ), "capture failed at: %.3fs", (float)mTimeLastCaptureFailed );
	}

	im::Separator();

	if( im::Checkbox( "depth", &mDepthEnabled ) ) {
		reinit();
	}
	im::SameLine();
	if( im::Checkbox( "color", &mColorEnabled ) ) {
		reinit();
	}
	if( im::Checkbox( "body tracking", &mBodyTrackingEnabled ) ) {
		reinit();
	}

	im::Checkbox( "log verbose", &mLogVerbose );
	im::ColorEdit3( "debug color", &mDebugColor.r, ImGuiColorEditFlags_Float );

	im::Separator();

	if( im::CollapsingHeader( "Profiling" ) ) {
		//double processDuration = mLastProcessDuration;
		im::Text( "process duration: %0.2fms", float( mLastProcessDuration * 1000.0 ) );
		ImGui::PlotLines( "##process duration lines", mProcessDurationsBuffer.data(), int( mProcessDurationsBuffer.size() ), 0, 0, 0.0f, 120.0f, ImVec2( ImGui::GetContentRegionAvailWidth(), 90 ) );
	}
	if( im::CollapsingHeader( "Calibration", ImGuiTreeNodeFlags_DefaultOpen ) ) {
		im::DragFloat3( "pos", &mPos.x, 0.5f );
		// TODO (maybe): replace this with imguizmo.quat
		// - not sure yet if I want the dependency but might be inevitable
		//if( im::DragFloat4( "orientation", &mOrientation.x, 0.01f ) ) {
		//	mOrientation = glm::normalize( mOrientation );
		//}

		// TODO (maybe): add nudge buttons.
		// - maybe with ArrowButton, or is there an easier way to enable drag on InputFloat?
		// - could make my own wrapper button class that combines a DragFloat with the part of InputFloat that uses ButtonEx( "-/+" ) etc..

		vec3 rotEuler = glm::degrees( glm::eulerAngles( mOrientation ) );
		if( im::DragFloat( "pitch", &rotEuler.x, 0.1f ) ) {
			mOrientation = glm::normalize( quat( glm::radians( rotEuler ) ) );
		}
		if( im::DragFloat( "yaw", &rotEuler.y, 0.11f ) ) {
			mOrientation = glm::normalize( quat( glm::radians( rotEuler ) ) );
		}
		if( im::DragFloat( "roll", &rotEuler.z, 0.11f ) ) {
			mOrientation = glm::normalize( quat( glm::radians( rotEuler ) ) );
		}
	}

	im::Separator();

	if( im::CollapsingHeader( "Recording / Playback", ImGuiTreeNodeFlags_DefaultOpen ) ) {
		if( im::Button( "open" ) ) {
			// TODO: add basic file api
			openRecording( "E:\\Dropbox\\work\\framestore\\sentinal\\kinect_recordings\\output.mkv" );
		}
	}

	if( mBodyTrackingEnabled && im::CollapsingHeader( ( "Body Tracking (bodies: " + to_string( mTotalBodiesTrackedLastFrame ) + ")###Bodies" ).c_str(), ImGuiTreeNodeFlags_DefaultOpen ) ) {
		if( mData->mTracker ) {
			// TODO: make temporal smoothing a deviceConfig param on CaptureManager itself
			// - perhaps devices can override but CaptureManager will have this param that it can set on all devices
			static float mTrackerTemporalSmoothingFactor = 0.6f;
			if( im::SliderFloat( "temporal smoothing", &mTrackerTemporalSmoothingFactor, 0, 1 ) ) {
				mData->mTracker.set_temporal_smoothing( mTrackerTemporalSmoothingFactor );
			}
		}
		else {
			im::Text( "no tracker" );
		}
		im::Separator();
		double currentTime = getManager()->getCurrentTime();

		// make a copy of bodies, sort them by oldest
		auto bodies = getBodies();
		stable_sort( bodies.begin(), bodies.end(),
			[]( const ck4a::Body &a, const ck4a::Body &b ) {
				return a.mTimeFirstTracked < b.mTimeFirstTracked;
			}
		);

		for( int b = 0; b < bodies.size(); b++ ) {
			const auto &body = bodies[b];
			auto bodyColor = getDebugBodyColor( stoi( body.mId ) );
			im::PushStyleColor( ImGuiCol_Text, Color( bodyColor.x, bodyColor.y, bodyColor.z ) );

			if( im::TreeNodeEx( ( "body " + to_string( b ) ).c_str(), ImGuiTreeNodeFlags_DefaultOpen ) ) {
				im::PopStyleColor();
				im::ScopedId idScope( body.mId.c_str() );
				im::Text( "id: %d", body.getId() );
				im::Text( "time tracked: %0.3f", float( currentTime - body.mTimeFirstTracked ) );
				im::Text( "time since last tracked: %0.3f, frames: %d", float( currentTime - body.mTimeLastTracked ), mCurrentBodyFrame - body.mFrameLastTracked );
				im::Text( "center joint type: %s", jointTypeAsString( body.getCenterJointType() ) );

				if( im::TreeNodeEx( "joints" ) ) {
					for( const auto &jt : body.getJoints() ) {
						const auto &joint = jt.second;

						ColorA col = im::GetStyleColorVec4( ImGuiCol_Text );
						if( joint.mConfidence == JointConfidence::Medium ) {
							//col = Color( 0, 1, 0.2f );
						}
						if( joint.mConfidence == JointConfidence::Low ) {
							col *= 0.75f;
						}
						else if( joint.mConfidence == JointConfidence::None ) {
							col = ColorA( 0.6f, 0.2f, 0.2f );
						}
						im::PushStyleColor( ImGuiCol_Text, col );

						im::Text( "%13s: confidence: %d,", joint.getTypeAsString(), (int)joint.mConfidence ); // TODO: make this lin the tree node
						im::SameLine();
						im::Text( "pos: [%+3.1f, %+3.1f, %+3.1f], vel: [%+4.2f, %+4.2f, %+4.2f], speed: %.2f",
							joint.mPos.x, joint.mPos.y, joint.mPos.z,
							joint.mVelocity.x, joint.mVelocity.y, joint.mVelocity.z,
							glm::length( joint.mVelocity )
						);

						if( joint.mType == JointType::Head ) {
							vec3 rotEuler = glm::degrees( glm::eulerAngles( joint.mOrientation ) );
							im::Text( "\t\t\t- orientation: [%+3.2f, %+3.2f, %+3.2f]", rotEuler.x, rotEuler.y, rotEuler.z );
						}

						im::PopStyleColor();
					}
					im::TreePop(); // joints
				}
				im::TreePop(); // body b
			}
			else {
				im::PopStyleColor(); // treenode color
			}
		}
	}


	im::Separator();

	im::Checkbox( "capture buffers", &mCopyBuffersEnabled );

	if( mCopyBuffersEnabled && im::CollapsingHeader( "Buffers", ImGuiTreeNodeFlags_DefaultOpen ) ) {
		ImGuiTreeNodeFlags viewerFlags = ImGuiTreeNodeFlags_DefaultOpen;
		imx::Texture2d( "color", mColorTexture, viewerFlags );
		imx::TextureDepth( "depth", mDepthTexture, viewerFlags );
		imx::Texture2d( "depth table", mTableDepth2d3dTexture, viewerFlags );
	}

	im::End();
}

void CaptureAzureKinect::enabledUI( bool showId )
{
	im::ScopedId idScope( mId.c_str() );

	if( showId ) {
		ImGui::AlignTextToFramePadding();
		if( isRemote() ) {
			im::TextColored( mDebugColor, "%s (R):", mId.c_str() );	
		}
		else {
			im::TextColored( mDebugColor, "%s:", mId.c_str() );	
		}
		im::SameLine();
		im::SetCursorPosX( 160 );
	}

	bool enabled = mEnabled;
	if( im::Checkbox( "enabled", &enabled ) ) {
		setEnabled( enabled );
	}
	// start / stop buttons and paused toggle are disabled when device is disabled
	if( ! mEnabled ) {
		imx::BeginDisabled();
	}

	im::SameLine();
	if( isStarted() ) {
		if( im::Button( "stop" ) ) {
			stop();
		}
	}
	else {
		if( im::Button( "start" ) ) {
			start();
		}
	}

	im::SameLine();
	im::Checkbox( "paused", &mPaused );

	if( ! mEnabled ) {
		imx::EndDisabled();
	}

	im::SameLine();
	im::Checkbox( "ui", &mUIEnabled );
}

// static
void CaptureAzureKinect::managerUI()
{
	if( im::Button( "refresh" ) ) {
		// Well this really doesn't work unless it is synced with the devices
		manager().buildDeviceSerialMap();
	}

	// TODO: combo for manager setLogMessageLevel

	im::Text( "sdk version: %s", CaptureAzureKinect::getSDKVersionString() );
	im::Text( "devices installed: %d", CaptureAzureKinect::getNumDevicesInstalled() );
	const auto &deviceInfos = manager().mDeviceInfos;
	for( int i = 0; i < deviceInfos.size(); i++ ) {
		const auto &info = deviceInfos[i];
		im::Text( "[%d] serial: %s", i, info.mSerial.c_str() );
		if( im::BeginPopupContextItem( ( "track source preview##" + to_string( i ) ).c_str() ) ) {
			if( im::Button( "copy to clipboard" ) ) {
				im::SetClipboardText( info.mSerial.c_str() );
			}
			im::EndPopup();
		}
	}
}

} // namespace ck4a
