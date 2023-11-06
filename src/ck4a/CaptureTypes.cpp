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

#include "ck4a/CaptureTypes.h"
#include "cinder/CinderAssert.h"
#include "cinder/Log.h"

#include <array>

namespace ck4a {

// --------------------------------------------
// Body
// --------------------------------------------

const Joint* Body::getJoint( JointType type ) const
{
	auto it = mJoints.find( type );
	if( it != mJoints.end() ) {
		return &it->second;
	}

	return nullptr;
}

void Body::merge( const Body &other, double currentTime )
{
	for( auto &kv : mJoints ) {
		auto &joint = kv.second;
		const Joint *otherJoint = other.getJoint( joint.mType );
		if( otherJoint ) {
			// if two joints are the same confidence, mix them
			// otherwise use the one with higher confidence
			if( otherJoint->mConfidence == joint.mConfidence ) {
				joint.mPos = glm::mix( joint.mPos, otherJoint->mPos, 0.5f );

				//joint.mOrientation = glm::mix( joint.mOrientation, otherJoint->mOrientation, 0.5f );
				joint.mOrientation = glm::slerp( joint.mOrientation, otherJoint->mOrientation, 0.5f );
			}
			else if( (int)otherJoint->mConfidence > (int)joint.mConfidence ) {
				joint.mPos = otherJoint->mPos;
				joint.mOrientation = otherJoint->mOrientation;
			}
		}
	}
}

void Body::initJointFilters( float freq, float minCutoff, float beta, float dCuttoff )
{
	CI_LOG_I( "id: " << mId );

	for( auto &kv : mJoints ) {
		kv.second.mPosFiltered = FilteredVec3( kv.second.mPosFiltered.get(), freq, minCutoff, beta, dCuttoff );
	}
}

void Body::update( double currentTime, const SmoothParams &params )
{
	// pick a center joint based on what's present
	// TODO: make this configurable (pass in candidates)
	static std::vector<JointType> centerJointDandidates = {
		ck4a::JointType::SpineNavel,
		ck4a::JointType::Head,
		ck4a::JointType::SpineChest,
		ck4a::JointType::Neck
	};

	for( const auto &candidateType : centerJointDandidates ) {
		// only use a candidate if the confidence is medium
		auto jointIt = mJoints.find( candidateType );
		if( jointIt != mJoints.end() && (int)jointIt->second.mConfidence >= (int)JointConfidence::Medium ) {
			mCenterJointType = candidateType;
			break;
		}
	}
	if( mCenterJointType == JointType::Unknown ) {
		mCenterJointType = mJoints.begin()->first;
	}

	if( params.mSmoothJoints ) {
		for( auto &kv : mJoints ) {
			auto &joint = kv.second;
#if( CK4A_FILTER_TYPE == CK4A_FILTER_TYPE_ONE_EURO )
			joint.updateSmoothedPos( currentTime );
#else
			joint.updateSmoothedPos( params.mLowPassAlpha );
#endif
		}
	}

}


//Rectf Body::calcBounds() const
//{
//	Rectf result = Rectf::zero();
//	bool empty = true;
//	for( const auto &kv : mJoints ) {
//		const auto &joint = kv.second;
//		vec2 pos = joint.getPos();
//		if( empty ) {
//			result = Rectf( pos, pos );
//			empty = false;
//		}
//		else {
//			result.include( joint.getPos() );
//		}
//		int blarg = 2;
//	}
//
//	return result;
//}


// --------------------------------------------
// Joint string conversions
// --------------------------------------------

const char* jointTypeAsString( JointType jointType )
{
	switch( jointType ) {
		case JointType::Pelvis:			return "Pelvis";
		case JointType::SpineNavel:		return "SpineNavel";
		case JointType::SpineChest:		return "SpineChest";
		case JointType::Neck:			return "Neck";
		case JointType::ClavicleLeft:	return "ClavicleLeft";
		case JointType::ShoulderLeft:	return "ShoulderLeft";
		case JointType::ElbowLeft:		return "ElbowLeft";
		case JointType::WristLeft:		return "WristLeft";
		case JointType::HandLeft:		return "HandLeft";
		case JointType::HandTipLeft:	return "HandTipLeft";
		case JointType::ThumbLeft:		return "ThumbLeft";
		case JointType::ClavicleRight:	return "ClavicleRight";
		case JointType::ShoulderRight:	return "ShoulderRight";
		case JointType::ElbowRight:		return "ElbowRight";
		case JointType::WristRight:		return "WristRight";
		case JointType::HandRight:		return "HandRight";
		case JointType::HandTipRight:	return "HandTipRight";
		case JointType::ThumbRight:		return "ThumbRight";
		case JointType::HipLeft:		return "HipLeft";
		case JointType::KneeLeft:		return "KneeLeft";
		case JointType::AnkleLeft:		return "AnkleLeft";
		case JointType::FootLeft:		return "FootLeft";
		case JointType::HipRight:		return "HipRight";
		case JointType::KneeRight:		return "KneeRight";
		case JointType::AnkleRight:		return "AnkleRight";
		case JointType::FootRight:		return "FootRight";
		case JointType::Head:			return "Head";
		case JointType::Nose:			return "Nose";
		case JointType::EyeLeft:		return "EyeLeft";
		case JointType::EarLeft:		return "EarLeft";
		case JointType::EyeRight:		return "EyeRight";
		case JointType::EarRight:		return "EarRight";
		default: break;
	}

	return "(Unknown)";
}

JointType jointTypeFromString( const std::string &jointType )
{
		 if( "Pelvis" )			return JointType::Pelvis;
	else if( "SpineNavel" )		return JointType::SpineNavel;
	else if( "SpineChest" )		return JointType::SpineChest;
	else if( "Neck" )			return JointType::Neck;
	else if( "ClavicleLeft" )	return JointType::ClavicleLeft;
	else if( "ShoulderLeft" )	return JointType::ShoulderLeft;	
	else if( "ElbowLeft" )		return JointType::ElbowLeft;
	else if( "WristLeft" )		return JointType::WristLeft;
	else if( "HandLeft" )		return JointType::HandLeft;
	else if( "HandTipLeft" )	return JointType::HandTipLeft;
	else if( "ThumbLeft" )		return JointType::ThumbLeft;
	else if( "ClavicleRight" )	return JointType::ClavicleRight;
	else if( "ShoulderRight" )	return JointType::ShoulderRight;
	else if( "ElbowRight" )		return JointType::ElbowRight;
	else if( "WristRight" )		return JointType::WristRight;
	else if( "HandRight" )		return JointType::HandRight;
	else if( "HandTipRight" )	return JointType::HandTipRight;
	else if( "ThumbRight" )		return JointType::ThumbRight;
	else if( "HipLeft" )		return JointType::HipLeft;
	else if( "KneeLeft" )		return JointType::KneeLeft;
	else if( "AnkleLeft" )		return JointType::AnkleLeft;
	else if( "FootLeft" )		return JointType::FootLeft;
	else if( "HipRight" )		return JointType::HipRight;
	else if( "KneeRight" )		return JointType::KneeRight;
	else if( "AnkleRight" )		return JointType::AnkleRight;
	else if( "FootRight" )		return JointType::FootRight;
	else if( "Head" )			return JointType::Head;
	else if( "Nose" )			return JointType::Nose;
	else if( "EyeLeft" )		return JointType::EyeLeft;
	else if( "EarLeft" )		return JointType::EarLeft;
	else if( "EyeRight" )		return JointType::EyeRight;
	else if( "EarRight" )		return JointType::EarRight;

	return JointType::Unknown;
}

JointType getParentJointType( JointType jointType )
{
	switch( jointType ) {
		case JointType::Pelvis:			return JointType::Count; // doesn't connect to anything
		case JointType::SpineNavel:		return JointType::Pelvis;
		case JointType::SpineChest:		return JointType::SpineNavel;
		case JointType::Neck:			return JointType::SpineChest;
		case JointType::ClavicleLeft:	return JointType::SpineChest;
		case JointType::ShoulderLeft:	return JointType::ClavicleLeft;
		case JointType::ElbowLeft:		return JointType::ShoulderLeft;
		case JointType::WristLeft:		return JointType::ElbowLeft;
		case JointType::HandLeft:		return JointType::WristLeft;
		case JointType::HandTipLeft:	return JointType::HandLeft;
		case JointType::ThumbLeft:		return JointType::WristLeft;
		case JointType::ClavicleRight:	return JointType::SpineChest;
		case JointType::ShoulderRight:	return JointType::ClavicleRight;
		case JointType::ElbowRight:		return JointType::ShoulderRight;
		case JointType::WristRight:		return JointType::ElbowRight;
		case JointType::HandRight:		return JointType::WristRight;
		case JointType::HandTipRight:	return JointType::HandRight;
		case JointType::ThumbRight:		return JointType::WristRight;
		case JointType::HipLeft:		return JointType::Pelvis;
		case JointType::KneeLeft:		return JointType::HipLeft;
		case JointType::AnkleLeft:		return JointType::KneeLeft;
		case JointType::FootLeft:		return JointType::AnkleLeft;
		case JointType::HipRight:		return JointType::Pelvis;
		case JointType::KneeRight:		return JointType::HipRight;
		case JointType::AnkleRight:		return JointType::KneeRight;
		case JointType::FootRight:		return JointType::AnkleRight;
		case JointType::Head:			return JointType::Neck;
		case JointType::Nose:			return JointType::Head;
		case JointType::EyeLeft:		return JointType::Head;
		case JointType::EarLeft:		return JointType::Head;
		case JointType::EyeRight:		return JointType::Head;
		case JointType::EarRight:		return JointType::Head;
		default: break;
	}

	return JointType::Unknown;
}

const std::vector<std::string>&	allJointNames()
{
	static std::vector<std::string> sJointNames;
	if( sJointNames.empty() ) {
		for( size_t i = 0; i < (size_t)JointType::Count; i++ ) {
			sJointNames.push_back( jointTypeAsString( (JointType)i ) );
		}
	}

	return sJointNames;
}

void Joint::updateSmoothedPos( double currentTime )
{
	mPosFiltered.set( mPos, currentTime );
}

namespace {

const std::array<vec3, 6> sBodyColors = {
	vec3( 1, 0, 0 ),
	vec3( 0, 1, 0 ),
	vec3( 0, 0, 1 ),
	vec3( 1, 1, 0 ),
	vec3( 0, 1, 1 ),
	vec3( 1, 0, 1 )
};

const std::vector<std::string> sDepthModeLabels = {
	"Off",
	"NFovBinned",
	"NFovUnbinned",
	"WFovBinned",
	"WFovUnbinned",
	"PassiveIR"
};

} // anon

vec3 getDebugBodyColor( int i )
{
	return sBodyColors[i % sBodyColors.size()];
}

vec3 getDebugBodyColor( const std::string &bodyId )
{
	static std::hash<std::string> sHasher;

	int h = sHasher( bodyId );
	return sBodyColors[h % sBodyColors.size()];
}

const char* playbackStatusToString( PlaybackStatus status )
{
	switch( status ) {
		case PlaybackStatus::NotLoaded:	return "NotLoaded";
		case PlaybackStatus::Ready:		return "Ready";
		case PlaybackStatus::EndOfFile:	return "EndOfFile";
		case PlaybackStatus::Failed:	return "Failed";
		default: break;
	}

	return "(unknown)";
}

const std::vector<std::string>& getDepthModeLabels()
{
	return sDepthModeLabels;
}

const std::string& modeToString( DepthMode mode )
{
	size_t index = (size_t)mode;
	CI_ASSERT( index < sDepthModeLabels.size() );
	return sDepthModeLabels.at( index );
}

DepthMode modeFromString( const std::string &str )
{
	if( str == "Off" )					return DepthMode::Off;
	else if( str == "NFovBinned" )		return DepthMode::NFovBinned;
	else if( str == "NFovUnbinned" )	return DepthMode::NFovUnbinned;
	else if( str == "WFovBinned" )		return DepthMode::WFovBinned;
	else if( str == "WFovUnbinned" )	return DepthMode::WFovUnbinned;
	else if( str == "PassiveIR" )		return DepthMode::PassiveIR;

	CI_ASSERT_NOT_REACHABLE();
	return DepthMode::Count;
}

} // namespace ck4a
