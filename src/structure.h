#pragma once

#ifndef structure_h
#define structure_h

#include "ofMain.h"
#include <ST/CaptureSession.h>


class Structure : public ST::CaptureSessionDelegate  {
public:

	// Thread-safe accessors for the stored CaptureSession samples
	ST::DepthFrame lastDepthFrame();
	ST::ColorFrame lastVisibleFrame();
	ST::InfraredFrame lastInfraredFrame();
	ST::AccelerometerEvent lastAccelerometerEvent();
	ST::GyroscopeEvent lastGyroscopeEvent();
	uint16_t* getShiftedDepth();

	Structure();
	
	void startThread();
	std::mutex mut;

	void calculateDepthTransform();

	ST::CaptureSessionSettings Structure::getSettings();

	// CaptureSession callbacks
	void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event);
	void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample&);

private:
	
	ST::DepthFrame _lastDepthFrame;
	ST::ColorFrame _lastVisibleFrame;
	ST::InfraredFrame _lastInfraredFrame;
	ST::AccelerometerEvent _lastAccelerometerEvent;
	ST::GyroscopeEvent _lastGyroscopeEvent;

	float depth_row[640 * 480];
	uint16_t depth[640 * 480];
	ofMatrix4x4 pose;
	
	double inv_depth_fx, inv_depth_fy;
	double depth_cx, depth_cy;
	double depth_Tx, depth_Ty;
	double rgb_fx, rgb_fy;
	double rgb_cx, rgb_cy;
	double rgb_Tx, rgb_Ty;

	bool intrinsics;

};

#endif /* structure_h */
