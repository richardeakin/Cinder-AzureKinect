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

#include "ck4a/CaptureDevice.h"
#include "ck4a/CaptureManager.h"

#include "mason/imx/ImGuiStuff.h"

using namespace ci;
using namespace std;
namespace im = ImGui;

namespace ck4a {

const static map<CaptureDevice::Status, const char*> sStatusMap = {
	{ CaptureDevice::Status::Uninitialized, "Uninitialized" },
	{ CaptureDevice::Status::Disabled, "Disabled" },
	{ CaptureDevice::Status::Stopped, "Stopped" },
	{ CaptureDevice::Status::Started, "Started" },
	{ CaptureDevice::Status::Running, "Running" },
	{ CaptureDevice::Status::Failed, "Failed" },
};

const char* statusToString( CaptureDevice::Status status )
{
	CI_ASSERT( sStatusMap.count( status ) );
	return sStatusMap.at( status );
}

CaptureDevice::Status statusFromString( const std::string &statusStr )
{
	for( const auto &mp : sStatusMap ) {
		if( mp.second == statusStr ) {
			return mp.first;
		}
	}

	CI_ASSERT_NOT_REACHABLE();
	return CaptureDevice::Status::Failed;
}

const char* CaptureDevice::getStatusAsString() const
{ 
	return statusToString( mStatus );
}

void CaptureDevice::setStatus( Status status )
{ 
	if( mStatus == status ) {
		return;
	}

	mStatus = status;

	if( ! isRemote() ) {
		// TODO: I think this needs to happen on the main thread
		// - if so can just use app's io_service for now
		getManager()->onLocalDeviceStatusChanged( this );
	}
}

} // namespace ck4a
