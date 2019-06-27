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
	
	void startThread();

	std::mutex mut;

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
};

#endif /* structure_h */
