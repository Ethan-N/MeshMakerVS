#include "structure.h"
#include <ST/CaptureSession.h>

	void Structure::captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) {
		_lastCaptureSessionEvent.store(event);
	}

	void Structure::captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {
		std::unique_lock<std::mutex> u(mut);
		// Depth/visible/infrared frames can arrive individually or as part of a SynchronizedFrames sample
		
		if (sample.depthFrame.isValid()) {
			_lastDepthFrame = sample.depthFrame;
		}
		if (sample.visibleFrame.isValid()) {
			_lastVisibleFrame = sample.visibleFrame;
		}
		if (sample.infraredFrame.isValid()) {
			_lastInfraredFrame = sample.infraredFrame;
		}
		if (sample.type == ST::CaptureSessionSample::Type::AccelerometerEvent) {
			_lastAccelerometerEvent = sample.accelerometerEvent;
		}
		if (sample.type == ST::CaptureSessionSample::Type::GyroscopeEvent) {
			_lastGyroscopeEvent = sample.gyroscopeEvent;
		}
		mut.unlock();
	}

	ST::CaptureSessionEventId Structure::lastCaptureSessionEvent() {
		return _lastCaptureSessionEvent.load();
	}

	ST::DepthFrame Structure::lastDepthFrame() {
		std::unique_lock<std::mutex> u(mut);
		return _lastDepthFrame;
	}

	ST::ColorFrame Structure::lastVisibleFrame() {
		std::unique_lock<std::mutex> u(mut);
		return _lastVisibleFrame;
	}

	ST::InfraredFrame Structure::lastInfraredFrame() {
		std::unique_lock<std::mutex> u(mut);
		return _lastInfraredFrame;
	}

	ST::AccelerometerEvent Structure::lastAccelerometerEvent() {
		std::unique_lock<std::mutex> u(mut);
		return _lastAccelerometerEvent;
	}

	ST::GyroscopeEvent Structure::lastGyroscopeEvent() {
		std::unique_lock<std::mutex> u(mut);
		return _lastGyroscopeEvent;
	}

	Structure::~Structure() {
		ofLog() << "Waiting to stop...";
		//waitForThread();
		ofLog() << "Stopped!";

	}
