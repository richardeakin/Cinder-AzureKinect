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

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/quaternion.hpp"

#define DC_MOTION_TRACKING_ENABLED 1

#if DC_MOTION_TRACKING_ENABLED
#include "mason/MotionTracker.h"
#endif

#include <map>
#include <string>

#if defined( DC_DEBUG_RELEASE )
#pragma optimize ( "", off )
#endif

namespace ck4a {

using glm::vec2;
using glm::vec3;
using glm::quat;

//! platform-agnostic joint types (although they are derived from azure kinect)
//! https://docs.microsoft.com/en-us/azure/kinect-dk/body-joints
enum class JointType {
	Pelvis			= 0,
	SpineNavel		= 1,
	SpineChest		= 2,
	Neck			= 3,
	ClavicleLeft	= 4,
	ShoulderLeft	= 5,
	ElbowLeft		= 6,
	WristLeft		= 7,
	HandLeft		= 8,
	HandTipLeft		= 9,
	ThumbLeft		= 10,
	ClavicleRight	= 11,
	ShoulderRight	= 12,
	ElbowRight		= 13,
	WristRight		= 14,
	HandRight		= 15,
	HandTipRight	= 16,
	ThumbRight		= 17,
	HipLeft			= 18,
	KneeLeft		= 19,
	AnkleLeft		= 20,
	FootLeft		= 21,
	HipRight		= 22,
	KneeRight		= 23,
	AnkleRight		= 24,
	FootRight		= 25,
	Head			= 26,
	Nose			= 27,
	EyeLeft			= 28,
	EarLeft			= 29,
	EyeRight		= 30,
	EarRight		= 31,
	Count			= 32,
	Unknown = 1000
};

enum class JointConfidence {
	None			= 0,
	Low				= 1,
	Medium			= 2,
	High			= 3,
	Count			= 4
};

//!
const char*  jointTypeAsString( JointType jointType );
//!
JointType	 jointTypeFromString( const std::string &jointType );
//!
JointType	 getParentJointType( JointType jointType );
//! All Joints in string form, in the same order as their enum value. Useful for GUI selection.
const std::vector<std::string>&	allJointNames();

//! Returns one of 6 unique colors depending on i
vec3		 getDebugBodyColor( int i );
//!
vec3		 getDebugBodyColor( const std::string &bodyId );

struct Joint {
	JointType		mType;
	JointConfidence mConfidence;
	bool			mActive = false;
	vec3			mPos; //! cm
	vec3			mVelocity; //! cm / s
	quat			mOrientation;
	double			mTimeFirstTracked = -1;

#if DC_MOTION_TRACKING_ENABLED
	// TODO: consider moving this out of this lower layer and into the Gestures.cpp stuff
	// - it's usually only used for a couple / few joints
	ma::MotionTracker<vec3>		mMotionTrackerPos;
#endif

	// Helpers
	bool		isActive() const			{ return mConfidence != JointConfidence::None; }
	bool		isGood() const				{ return (int)mConfidence >= (int)JointConfidence::Medium; }
	const char*	getTypeAsString() const		{ return jointTypeAsString( mType ); }
	JointType	getParentJointType() const	{ return ck4a::getParentJointType( mType ); }
	vec3		getDir() const				{ return glm::normalize( vec3( mOrientation * glm::vec4( 0, 0, -1, 1 ) ) ); 	}

	bool	isHand() const { return mType == JointType::HandLeft || mType == JointType::HandRight; }
	// TODO: add for all left/right types
};

class Body {
  public:

	const std::string&  getId() const					{ return mId; }
	//int	  getIndex() const				{ return mIndex; }
	bool  isActive() const				{ return mActive; }
	float getDistanceToCamera() const	{ return mDistanceToCamera; }
	//! Time the Body was first detected. 
	double getTimeFirstTracked() const	{ return mTimeFirstTracked; }

	size_t	getNumJoints() const							{ return mJoints.size(); }
	const std::map<JointType, Joint>&	getJoints() const	{ return mJoints; }

	//! Returns nullptr if no Joint of this type
	const Joint*	getJoint( JointType type ) const;
	//! Returns the joint used as center of body, or nullptr if none was found
	const Joint*	getCenterJoint() const		{ return getJoint( mCenterJointType ); }
	//!
	JointType		getCenterJointType() const	{ return mCenterJointType; }

	void			merge( const Body &other );

	// TODO: remove friends / make data private.
	// - will do after body resolving is fleshed out
	// - add setters for necessary data, or make struct

  //private:
	std::string			mId;
	bool				mActive = false; //! If this body has been processed since the last update
	float				mDistanceToCamera = -1; // TODO: remove if unused
	double				mTimeFirstTracked = -1;
	double				mTimeLastTracked = -1;
	uint64_t			mFrameLastTracked = -1;
	JointType			mCenterJointType = JointType::Unknown; // will pick another if this isn't present

	std::map<JointType, Joint>	mJoints;
};

enum class PlaybackStatus {
	NotLoaded,
	Ready,
	EndOfFile,
	Failed
};

const char* playbackStatusToString( PlaybackStatus status );

//! Corresponds to k4a_depth_mode_t
//! https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___enumerations_ga3507ee60c1ffe1909096e2080dd2a05d.html
enum class DepthMode {
	Off = 0,		//! K4A_DEPTH_MODE_OFF
	NFovBinned,		//! K4A_DEPTH_MODE_NFOV_2X2BINNED
	NFovUnbinned,	//! K4A_DEPTH_MODE_NFOV_UNBINNED
	WFovBinned,		//! K4A_DEPTH_MODE_WFOV_2X2BINNED
	WFovUnbinned,	//! K4A_DEPTH_MODE_WFOV_UNBINNED
	PassiveIR,		//! K4A_DEPTH_MODE_PASSIVE_IR
	Count
};

const std::string&	modeToString( DepthMode mode );
DepthMode			modeFromString( const std::string &str );
//! Returns a vector<string> for using in imgui / config
const std::vector<std::string>& getDepthModeLabels();

} // namespace ck4a
