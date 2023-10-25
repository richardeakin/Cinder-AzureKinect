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

// original source: https://cristal.univ-lille.fr/~casiez/1euro/1efilter.cc

#pragma once

#include "cinder/Vector.h"

namespace ck4a {

template <typename T = double>
struct low_pass_filter {
	low_pass_filter() : hatxprev(0), xprev(0), hadprev(false) {}
	T operator()(T x, T alpha) {
		T hatx;
		if(hadprev) {
			hatx = alpha * x + (1-alpha) * hatxprev;
		} else {
			hatx = x;
			hadprev = true;
		}
		hatxprev = hatx;
		xprev = x;
		return hatx;
	}
	T hatxprev;
	T xprev;
	bool hadprev;
};

template <typename T = double, typename timestamp_t = double>
struct one_euro_filter {
	one_euro_filter(double _freq = 60.0, T _mincutoff = T(0), T _beta = T(0), T _dcutoff = T(0) )
		: freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff), last_time_(-1)
	{}
	T operator()(T x, timestamp_t t = -1)
	{
		T dx = 0;

		if(last_time_ != -1 && t != -1 && t != last_time_) {
			freq = 1.0 / (t - last_time_);
		}
		last_time_ = t;

		if(xfilt_.hadprev)
			dx = (x - xfilt_.xprev) * freq;

		T edx = dxfilt_(dx, alpha(dcutoff));
		T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));
		return xfilt_(x, alpha(cutoff));
	}

	double freq;
	T mincutoff, beta, dcutoff;
private:
	T alpha(T cutoff)
	{
		T tau = 1.0 / (2 * M_PI * cutoff);
		T te = 1.0 / freq;
		return 1.0 / (1.0 + tau / te);
	}

	timestamp_t last_time_;
	low_pass_filter<T> xfilt_, dxfilt_;
};

// TODO: add other constructor params
template <typename T>
struct FilteredValue {
	FilteredValue( float initialValue = T( 0 ) )
		: mValue( initialValue )
	{}

	void set( const T &value )
	{
		mValue = value;
		// TODO: clear filter
	}

	void set( const T &value, double currentTime )
	{
		mValue = mFilter( value, currentTime );
	}

	T mValue;
	one_euro_filter<T, double>	mFilter;
};

// TODO: add overloads to FilteredValue that work for glm::vec types instead (currently only works for float and double)
using glm::vec3;

struct FilteredVec3 {
	FilteredVec3( const vec3 &initialValue = vec3( 0 ) )
		: mX( initialValue.x ), mY( initialValue.y ), mZ( initialValue.z )
	{}

	void set( const vec3 &v )
	{
		mX.set( v.x );
		mY.set( v.y );
		mZ.set( v.z );
	}

	void set( const vec3 &v, double currentTime )
	{
		mX.set( v.x, currentTime );
		mY.set( v.y, currentTime );
		mZ.set( v.z, currentTime );
	}

	vec3 get() const
	{
		return { 
			mX.mValue,
			mY.mValue,
			mZ.mValue
		};
	}

	FilteredValue<float> mX;
	FilteredValue<float> mY;
	FilteredValue<float> mZ;
};

} // namespace mason