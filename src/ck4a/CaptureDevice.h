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

#include "mason/Info.h"

#include <atomic>

namespace ck4a {

class CaptureManager;

// TODO: move base interface to this class
// - will add CaptureDeviceNetwork
class CaptureDevice {
public:
	enum class Status {
		Uninitialized,
		Disabled,
		Stopped,
		Started,
		Running,
		Failed = -1000
	};

	CaptureDevice( CaptureManager *manager )
		: mManager( manager )
	{}

	CaptureManager* getManager() const	{ return mManager; }

	void setStatus( Status status );
	Status getStatus() const				{ return mStatus; }
	const char* getStatusAsString() const;

	const std::string&	getId() const				{ return mId; }

	bool isInitialized() const	{ return ( (int)getStatus() >= (int)Status::Stopped ); }
	bool isStarted() const		{ return ( (int)getStatus() >= (int)Status::Started ); }
	bool isRunning() const		{ return ( getStatus() == Status::Running ); }
	bool isRemote() const		{ return mRemote; }

protected:
	std::string		mId;

	bool mRemote = false; //! Gets marked by CaptureDevice::init when host id does not match CaptureManager's

private:
	CaptureManager*			mManager = nullptr;
	std::atomic<Status>		mStatus = Status::Uninitialized;
};

const char* statusToString( CaptureDevice::Status status );
CaptureDevice::Status statusFromString( const std::string &statusStr );

} // namespace ck4a
