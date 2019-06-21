#ifndef structure_h
#define structure_h

#include "ofMain.h"
#include <ST/CaptureSession.h>


class Structure : public ST::CaptureSessionDelegate  {
public:
	~Structure();

	// Thread-safe accessors for the stored CaptureSession samples
	ST::DepthFrame lastDepthFrame();
	ST::ColorFrame lastVisibleFrame();
	ST::InfraredFrame lastInfraredFrame();
	ST::AccelerometerEvent lastAccelerometerEvent();
	ST::GyroscopeEvent lastGyroscopeEvent();

	ST::CaptureSessionEventId lastCaptureSessionEvent();

	// CaptureSession callbacks
	void captureSessionEventDidOccur(ST::CaptureSession *, ST::CaptureSessionEventId) override;
	void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample&) override;

private:
	std::atomic<ST::CaptureSessionEventId> _lastCaptureSessionEvent {ST::CaptureSessionEventId::Disconnected};

	ST::DepthFrame _lastDepthFrame;
	ST::ColorFrame _lastVisibleFrame;
	ST::InfraredFrame _lastInfraredFrame;
	ST::AccelerometerEvent _lastAccelerometerEvent;
	ST::GyroscopeEvent _lastGyroscopeEvent;
	std::mutex mut;
};

#endif /* structure_h */
