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

#include "ck4a/CaptureManager.h"

#include "mason/imx/ImGuiStuff.h"

#include "cinder/Log.h"
#include "cinder/osc/Osc.h"

using namespace ci;
using namespace std;
namespace im = ImGui;

#define LOG_NETWORK_V( stream )	{ if( sLogNetworkVerbose ) {	CI_LOG_I( "|v| " << stream ); } }
//#define LOG_NETWORK_V( stream )	( (void)( 0 ) )

namespace ck4a {

namespace {

bool sLogNetworkVerbose = false;

} // anon

CaptureManager::CaptureManager()
{
}

CaptureManager::~CaptureManager()
{
	uninit();
}

void CaptureManager::init( const ma::Info& info )
{
	mEnabled = info.get( "enabled", mEnabled );
	mAutoStart = info.get( "autoStart", mAutoStart );
	mUIEnabled = info.get( "ui", mUIEnabled );
	mVerboseLogging = info.get( "verboseLogging", mVerboseLogging );
	mSyncDevicesEnabled = info.get( "syncDevices", mSyncDevicesEnabled );
	mMaxSecondsUntilBodyRemoved = info.get( "maxSecondsUntilBodyRemoved", mMaxSecondsUntilBodyRemoved );
	mMaxBodyDistance = info.get( "maxBodyDistance", mMaxBodyDistance );

	// parse hosts
	mNetworkingEnabled = info.get<bool>( "networkingEnabled", mNetworkingEnabled );
	mHostId = info.get<string>( "hostId" ); // the id for this instance host

	auto hosts = info.get<vector<ma::Info>>( "hosts" );

	for( const auto &hostInfo : hosts ) {
		mHosts.emplace_back();
		auto &host = mHosts.back();
		host.mId = hostInfo.get<string>( "id" );
		host.mMaster = hostInfo.get( "master", false );
		host.mIpAddress = hostInfo.get<string>( "ipAddress", "127.0.0.1" );
		host.mReceivePort = hostInfo["receivePort"];
		//host.mSendPort = hostInfo["sendPort"];
	}

	if( ! mEnabled ) {
		CI_LOG_I( "disabled." );	
		return;
	}

	if( mNetworkingEnabled ) {
		initNetworking();
	}
	else {
		CI_LOG_I( "networking disabled" );
		// TODO: error reporting
	}


	auto devices = info.get<vector<ma::Info>>( "devices" );
	bool haveSyncMaster = false;
	for( const auto &deviceInfo : devices ) {
		auto device = make_shared<CaptureAzureKinect>( this );
		device->init( deviceInfo );

		// sanity check that we have exactly 1 sync master
		if( mSyncDevicesEnabled ) {
			if( device->isSyncMaster() ) {
				if( haveSyncMaster ) {
					CI_LOG_E( "Multiple sync masters, check sync cable configuration." );
				}
				haveSyncMaster = true;
			}
		}
		mCaptureDevices.push_back( device );
	}

	if( mSyncDevicesEnabled && ! haveSyncMaster ) {
		CI_LOG_E( "Sync devices mode enabled, but no sync master found. Check sync cable configuratoin.");
	}

	CI_LOG_I( "initialized " << mCaptureDevices.size() << " devices." );

	if( mAutoStart ) {
		CI_LOG_I( "starting all devices marked enabled.." );
		for( const auto device : mCaptureDevices ) {
			if( device->isEnabled() ) {
				device->start();
			}
		}
	}
}

void CaptureManager::save( ma::Info& info ) const
{
	info["enabled"] = mEnabled;
	info["ui"] = mUIEnabled;
	info["verboseLogging"] = mVerboseLogging; // TODO: make this a log level?
	info["syncDevices"] = mSyncDevicesEnabled;	
	info["heartbeatSeconds"] = mHeartbeatSeconds;
	info["maxSecondsUntilBodyRemoved"] = mMaxSecondsUntilBodyRemoved;
	info["maxBodyDistance"] = mMaxBodyDistance;

	std::vector<ma::Info> devices;
	for( const auto &device : mCaptureDevices ) {
		ma::Info deviceInfo;
		device->save( deviceInfo );
		devices.push_back( deviceInfo );
	}

	info.set( "devices", devices );
}

void CaptureManager::uninit()
{
	for( const auto &device : mCaptureDevices ) {
		device->uninit();
	}

	mCaptureDevices.clear();
	mOSCSender = nullptr;
}

void CaptureManager::clearData()
{
	for( const auto &device : mCaptureDevices ) {
		device->clearData();
	}
}

void CaptureManager::setEnabled( bool b )
{
	if( mEnabled == b ) {
		CI_LOG_W( "already" << string( b ? "enabled" : "disabled" ) );
		return;
	}

	mEnabled = b;

	// TODO: should re-init in this case?
	// - Don't have the config info though, would need to serialize that

	if( mNetworkingEnabled && ( ! mOSCSender || ! mOSCReceiver ) ) {
		initNetworking();
	}

	if( mEnabled ) {
		if( mAutoStart ) {
			startAll();
		}
	}
	else {
		stopAll();
	}
}

void CaptureManager::startAll()
{
	if( ! mEnabled ) {
		CI_LOG_W( "already enabeld." );
		return;
	}

	mEnabled = true;
	for( const auto &device : mCaptureDevices ) {
		if( device->isEnabled() ) {
			device->start();
		}
	}
}

void CaptureManager::stopAll()
{
	for( const auto &device : mCaptureDevices ) {
		device->stop();
	}

	mEnabled = false;
}

void CaptureManager::setPaused( bool b )
{
	mPaused = b;
	for( const auto &device : mCaptureDevices ) {
		device->setPaused( mPaused );
	}
}

CaptureAzureKinectRef CaptureManager::getDevice( const std::string &deviceId ) const
{
	for( const auto &device : mCaptureDevices ) {
		if( device->getId() == deviceId ) {
			return device;
		}
	}

	// return null device
	return {};
}

bool CaptureManager::isAnyDeviceRunning() const
{
	for( const auto &device : mCaptureDevices ) {
		if( device->isEnabled() && device->isRunning() ) {
			return true;
		}
	}

	return false;
}

bool CaptureManager::canSyncMasterStart() const
{
	if( ! mSyncDevicesEnabled ) {
		return true;
	}

	// Check if all subordinates are running, if one isn't then return false
	// TODO: probably also want to allow it to start if something has failed, but not sure yet about that
	// - could just start the failed one later and hope for the best, counting on a manual global camera restart later to syncrhonize
	for( const auto &device : mCaptureDevices ) {
		if( ! device->isEnabled() )
			continue;
		if( device->isSyncMaster() )
			continue;

		if( device->getStatus() != CaptureDevice::Status::Running ) {
			// TODO: add some log v for this
			//CI_LOG_I( "blocking master from starting because device '" << device->getId() << "' has status: " << device->getStatusAsString() );
			return false;
		}
	}

	return true;
}

// TODO: consider using chrono::time::now throughout
double CaptureManager::getCurrentTime() const
{
	return app::getElapsedSeconds();
}

void CaptureManager::keyDown( app::KeyEvent &event )
{
	bool handled = true;
	if( event.getChar() == 'p' ) {
		setPaused( ! isPaused() );
		CI_LOG_I( "paused: " << isPaused() );
	}
}

void CaptureManager::update()
{
	if( ! mEnabled ) {
		return;
	}

	updateNetworking();

	for( const auto &device : mCaptureDevices ) {
		device->update();
	}
}

// ----------------------------------------------------------------------------------------------------
// Networking
// ----------------------------------------------------------------------------------------------------

static bool sSendHeartBeat = true;
static string sTestMessage = "blarg";
static vec2 sTestVec;
static double mTimeLastHeartBeatSent = -1.0;

void CaptureManager::initNetworking()
{
	int hostIndex = -1;
	int masterHostIndex = -1;
	mMasterHost = false;

	for( int i = 0; i < mHosts.size(); i++ ) {
		const auto &host = mHosts[i];

		if( host.mMaster ) {
			if( masterHostIndex >= 0 ) {
				CI_LOG_E( "sanity check failed: already have a master host at index: " << masterHostIndex );
			}

			masterHostIndex  = i;
		}

		if( mHostId == host.mId ) {
			hostIndex = i;
			if( host.mMaster ) {
				mMasterHost = true;
			}
		}
	}


	if( hostIndex < 0 || hostIndex >= mHosts.size() ) {
		CI_LOG_E( "invalid host id: " << mHostId );
		return;
	}

	CI_LOG_I( "host id: " << mHostId << ", is master: " << mMasterHost );

	const auto &host = mHosts[hostIndex];

	// init OSC Receiver
	try {
		mOSCReceiver = make_unique<osc::ReceiverUdp>( host.mReceivePort );

		initOSCListeners();

		mOSCReceiver->bind(); // note to self, bind() must be called before listen()..

		// UDP opens the socket and "listens" accepting any message from any endpoint. The listen
		// function takes an error handler for the underlying socket. Any errors that would
		// call this function are because of problems with the socket or with the remote message.
		mOSCReceiver->listen(
			[host]( asio::error_code error, asio::ip::udp::endpoint endpoint ) -> bool {
			if( error ) {
				CI_LOG_E( "Error Listening: " << error.message() << " val: " << error.value() << " endpoint: " << endpoint );
				return false;
			}
			else {
				// I'm not sure if this can ever happen, but I'm logging it anyway
				CI_LOG_W( "(" << host.mId << ") received error message from endpoint: " << endpoint << ", but missing asio error.." );
				return true;
			}
		} );

		CI_LOG_I( "connected receiver for host (" << host.mId << ") to port: " << host.mReceivePort );
	}
	catch( exception &exc ) {
		CI_LOG_EXCEPTION( "Failed to init OSC Receiver for host (" << host.mId << ") on port: " << host.mReceivePort, exc );
	}

	if( mMasterHost ) {
		// init OSC Sender - master to subordinate
		// TODO: broadcast

		// hard-coding dest. host / port for the moment
		const std::string destinationHost = "127.0.0.1";
		const uint16_t destinationPort = 10001;

		try {
			//mOSCSender = make_unique<osc::SenderUdp>( host.mReceivePort, destinationHost, destinationPort );
			mOSCSender = make_unique<osc::SenderUdp>( 0, destinationHost, destinationPort );
			mOSCSender->bind();

			CI_LOG_I( "connected master OSC Sender (" << mHostId << ") sender" );
		}
		catch( exception &exc ) {
			CI_LOG_EXCEPTION( "(" << mHostId << ") Failed to init OSC Sender", exc );
		}

	}
	else {
		// init OSC Sender - subordinate to master
		const auto &masterHost = mHosts[masterHostIndex];
		try {
			mOSCSender = make_unique<osc::SenderUdp>( 0, masterHost.mIpAddress, masterHost.mReceivePort );
			mOSCSender->bind();

			CI_LOG_I( "connected sender from subordinate host (" << host.mId << ") to master (" << masterHost.mId << ")" );
		}
		catch( exception &exc ) {
			CI_LOG_EXCEPTION( "Failed to create and bind OSC Sender", exc );
		}
	}
}

void CaptureManager::initOSCListeners()
{
	if( ! mOSCReceiver ) {
		CI_LOG_E( "null receiver" );
		return;
	}

	mOSCReceiver->removeAllListeners();

	//mOSCReceiver->setListener( "/test/message",
	//	[&]( const osc::Message &msg ) {
	//		CI_LOG_I( "/test/message: " << msg );
	//	}
	//);

#if 0
	mOSCReceiver->setListener( "*", 
		[&]( const osc::Message &msg ) {
			CI_LOG_I( "wildcard: " << msg );
		}
	);
#endif

	if( mMasterHost ) {
		// TODO: update this address to be /capture/device/status
		mOSCReceiver->setListener( "/device/status",
			[this]( const osc::Message &msg ) {
				string deviceId = msg.getArgString( 0 );
				string status = msg.getArgString( 1 );
				CI_LOG_I( "device with id: " << deviceId << " set to status: " << status );

				onRemoteDeviceStatusChanged( deviceId, statusFromString( status ) ); 
			}
		);

		// TODO NEXT: figure out to what extent I can parse this data
		mOSCReceiver->setListener( "/capture/device/*",
			[this]( const osc::Message &msg ) {
				CI_LOG_I( "received message: " << msg );
			}
		);

	}
}

void CaptureManager::updateNetworking()
{
	if( sSendHeartBeat ) {
		// TODO: will rethink this once full networked setup is in
		// - actually want to be receiving heartbeats from all remote clients to the master
		// - right now using this to test receiving in unreal
		if( mTimeLastHeartBeatSent < 0 || app::getElapsedSeconds() - mTimeLastHeartBeatSent > mHeartbeatSeconds )  {
			mTimeLastHeartBeatSent = app::getElapsedSeconds();

			osc::Message msg( "/capture/host/" + mHostId + "/heartbeat" );
			msg.append( (float)mTimeLastHeartBeatSent );
			sendMessage( msg );
		}
	}
}

void CaptureManager::sendTestMessage()
{
	osc::Message msg( "/test/message" );
	msg.append( sTestMessage );
	sendMessage( msg );
}

void CaptureManager::sendTestValue()
{
	osc::Message msg( "/test/value" );
	msg.append( sTestVec.x );
	msg.append( sTestVec.y );
	sendMessage( msg );
}

#define DEBUG_OSC_MESSAGES 0

void CaptureManager::sendMessage( const osc::Message &msg )
{
	if( ! mOSCSender ) {
		LOG_NETWORK_V( "Error: null OSC Sender" );
		return;
	}

	lock_guard<mutex> lock( mMutexSender );

#if DEBUG_OSC_MESSAGES
	// FIXME: figure out why this msg print isn't working
	CI_LOG_I( "time: " << getCurrentTime() << "] sending message:\n" << msg );
	//CI_LOG_I( "\t- arg 0: " << msg.getArg<float>( 0 ) );
#endif

	mOSCSender->send( msg,
		[]( asio::error_code error ) {
			CI_LOG_E( "\t- error sending message: " << error.message() << ", value: " << error.value() );
		},
		[=] {
			//LOG_NETWORK_V( "\t- successfully sent message at time: " << getCurrentTime() );
		}
	);
}

void appendToMessage( osc::Message &msg, const vec3 &v )
{
	msg.append( v.x );
	msg.append( v.y );
	msg.append( v.z );
}

// note: called from Capture process thread
// TODO: try sending as a bundle
void CaptureManager::sendBodyTracked( const CaptureDevice *device, Body body )
{
	// master host doesn't need to send bodies, it will collect them all
	if( mMasterHost ) {
		return;
	}

	// capture/device/{deviceId}
	// send body over the wire

	string bodyAddress = "/capture/device/" + device->getId() + "/body/" + body.getId();

	osc::Message msg( bodyAddress );
	msg.append( body.isActive() );

	sendMessage( msg );

	// first: joint by joint
	// joint addresses: .../body/{bodyId}/joints/{jointId}
	for( const auto &jp : body.getJoints() ) {
		const auto &joint = jp.second;

		osc::Message msg( bodyAddress + "/joints/" + to_string( (int)jp.first ) );
		appendToMessage( msg, joint.mPos );
		sendMessage( msg );
	}

}

void CaptureManager::onLocalDeviceStatusChanged( CaptureDevice *device )
{
	CI_LOG_I( "device: " << device->getId() << ", status: " << device->getStatusAsString() );

	CI_ASSERT( ! device->isRemote() );

	if( ! mMasterHost && mOSCSender ) {
		osc::Message msg( "/device/status" );
		msg.append( device->getId() );
		msg.append( device->getStatusAsString() );
		sendMessage( msg );
	}
}

void CaptureManager::onRemoteDeviceStatusChanged( string id, CaptureDevice::Status status )
{
	auto device = getDevice( id );
	if( ! device ) {
		CI_LOG_E( "no device with id: " << id );
		return;
	}

	CI_ASSERT( device->isRemote() );

	CI_LOG_I( "setting remote Device with id: " << id << " to status: " << statusToString( status ) );
	device->setStatus( status );
}

// ----------------------------------------------------------------------------------------------------
// UI
// ----------------------------------------------------------------------------------------------------

void CaptureManager::updateUI()
{
	// draw UIs for all devices if they are enabled
	for( const auto &device : mCaptureDevices ) {
		device->updateUI();
	}

	if( ! mUIEnabled ) {
		return;
	}

	if( ! im::Begin( "CaptureManager", &mUIEnabled ) ) {
		im::End();
		return;
	}

	static bool sLogVerboseAll = false;
	if( im::Checkbox( "log all verbose", &sLogVerboseAll ) ) {
		for( const auto &device : mCaptureDevices ) {
			device->setLogVerboseEnabled( sLogVerboseAll );
		}
	}

	bool enabled = mEnabled;
	if( im::Checkbox( "enabled", &enabled ) ) {
		setEnabled( enabled );
	}

	if( ! mEnabled ) {
		imx::BeginDisabled();
	}

	im::SameLine();
	bool paused = mPaused;
	if( im::Checkbox( "paused", &paused ) ) {
		setPaused( paused );
	}

	im::SameLine();

	if( isAnyDeviceRunning() ) {
		if( im::Button( "stop all" ) ) {
			stopAll();
		}
	}
	else {
		if( im::Button( "start all" ) ) {
			startAll();
		}
	}

	if( ! mEnabled ) {
		imx::EndDisabled();
	}

	im::Checkbox( "auto start", &mAutoStart );

	// TODO: make these checkboxes (will need to re-init networking / devices
	im::Value( "networking enabled", mNetworkingEnabled );
	im::Value( "sync devices", mSyncDevicesEnabled );


	im::DragFloat( "body max distance", &mMaxBodyDistance, 0.5f, -1.0f, 10000.0f );

	float maxSeconds = (float)mMaxSecondsUntilBodyRemoved;
	if( im::DragFloat( "body max seconds until removed", &maxSeconds, 0.001f, 0.0f, 10e6f, "%.4f" ) ) {
		mMaxSecondsUntilBodyRemoved = (double)maxSeconds;
	}

	if( mEnabled ) {
		if( im::CollapsingHeader( "Azure Kinect Manager", ImGuiTreeNodeFlags_DefaultOpen ) ) {
			CaptureAzureKinect::managerUI();
		}
	}

 	im::Separator();

	for( const auto &device : mCaptureDevices ) {
		device->enabledUI();
	}

	if( im::CollapsingHeader( "Networking", ImGuiTreeNodeFlags_DefaultOpen ) ) {
		if( mMasterHost ) {
			im::PushStyleColor( ImGuiCol_Text, Color( 0, 1, 0 ) );
		}
		im::Text( "host id: %s", mHostId.c_str() );
		im::Text( "ip address: %s", ci::System::getIpAddress().c_str() );
		if( im::BeginPopupContextItem( "ip address" ) ) {
			if( im::Button( "copy to clipboard" ) ) {
				im::SetClipboardText( ci::System::getIpAddress().c_str() );
			}
			im::EndPopup();
		}

		if( mMasterHost ) {
			im::PopStyleColor();
		}

		im::Checkbox( "log verbose##networking", &sLogNetworkVerbose );
		if( im::Button( "re-init" ) ) {
			// TODO: move these methods to an uninitNetworking() method
			mOSCReceiver->close();
			mOSCReceiver = {};
			mOSCSender->close();
			mOSCSender = {};

			// TODO: get this working. currently getting an Error Listening..
			initNetworking();
		}
		im::SameLine();
		if( im::Button( "init listeners" ) ) {
			initOSCListeners();
		}

		if( mOSCSender ) {
			im::Checkbox( "heartbeat", &sSendHeartBeat );

			im::SameLine();
			float heartbeatSeconds = (float)mHeartbeatSeconds;
			if( im::DragFloat( "seconds", &heartbeatSeconds, 0.2f, 0.01f, 10e6f ) ) {
				mHeartbeatSeconds = (double)heartbeatSeconds;
			}
			im::Text( "time last heartbeat sent: %0.3f", (float)mTimeLastHeartBeatSent );
			im::InputText( "##test msg", &sTestMessage );
			im::SameLine();
			if( im::Button( "test message" ) ) {
				sendTestMessage();
			}
			if( im::DragFloat2( " test value", &sTestVec.x ) ) {
				sendTestValue();
			}
		}
		else {
			im::TextColored( Color( 1, 1, 0 ), "no OSC Sender" );
		}

		if( im::TreeNodeEx( "Hosts" ) ) {
			for( int i = 0; i < mHosts.size(); i++ ) {
				const auto &host = mHosts[i];
				if( im::TreeNodeEx( host.mId.c_str(), ImGuiTreeNodeFlags_DefaultOpen ) ) {
					im::Text( "ip: %s, port: %d", host.mIpAddress.c_str(), host.mReceivePort );
					im::TreePop();
				}
			}
			im::TreePop();
		}

		if( im::TreeNodeEx( "Sockets" ) ) {
			if( mOSCSender ) {
				if( im::TreeNodeEx( "Sender", ImGuiTreeNodeFlags_DefaultOpen ) ) {
					im::Text( "local address: %s:%d", mOSCSender->getLocalAddress().address().to_string().c_str(), mOSCSender->getLocalAddress().port() );
					im::Text( "remote address: %s:%d", mOSCSender->getRemoteAddress().address().to_string().c_str(), mOSCSender->getRemoteAddress().port() );
					im::TreePop();
				}
			}

			if( mOSCReceiver ) {
				if( im::TreeNodeEx( "Receiver", ImGuiTreeNodeFlags_DefaultOpen ) ) {
					im::Text( "local endpoint: %s:%d", mOSCReceiver->getLocalEndpoint().address().to_string().c_str(), mOSCReceiver->getLocalEndpoint().port() );
					if( im::TreeNodeEx( "Listeners", ImGuiTreeNodeFlags_DefaultOpen ) ) {
						auto listeners = mOSCReceiver->getListeners();
						for( const auto &l : listeners ) {
							im::BulletText( "%s", l.first.c_str() );
						}

						im::TreePop();
					}
					im::TreePop();
				}
			}

			im::TreePop();
		}
	}

	im::End();
}

void CaptureManager::enabledUI()
{
	im::ScopedId idScope( "ck4a::CaptureManager" );

	im::Checkbox( "CaptureManager", &mEnabled ); im::SameLine();
	im::Checkbox( "ui", &mUIEnabled );
}

} // namespace ck4a
