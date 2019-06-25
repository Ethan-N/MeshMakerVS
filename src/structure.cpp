#include "structure.h"
#include <ST/CaptureSession.h>

	void Structure::captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) {
		printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
		switch (event) {
		case ST::CaptureSessionEventId::Booting: break;
		case ST::CaptureSessionEventId::Ready:
			printf("Starting streams...\n");
			session->startStreaming();
			break;
		case ST::CaptureSessionEventId::Disconnected:
		case ST::CaptureSessionEventId::Error:
			printf("Capture session error\n");
			break;
		default:
			printf("Capture session event unhandled\n");
		}
	}

	void Structure::captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {
		std::unique_lock<std::mutex> u(mut);
		switch (sample.type) {
		case ST::CaptureSessionSample::Type::DepthFrame:
			_lastDepthFrame = sample.depthFrame;
			break;
		case ST::CaptureSessionSample::Type::VisibleFrame:
			_lastVisibleFrame = sample.visibleFrame;
			break;
		case ST::CaptureSessionSample::Type::InfraredFrame:
			_lastInfraredFrame = sample.infraredFrame;
			break;
		case ST::CaptureSessionSample::Type::SynchronizedFrames:
			_lastDepthFrame = sample.depthFrame;
			_lastVisibleFrame = sample.visibleFrame;
			//_lastInfraredFrame = sample.infraredFrame;
			break;
		case ST::CaptureSessionSample::Type::AccelerometerEvent:
			_lastAccelerometerEvent = sample.accelerometerEvent;
			break;
		case ST::CaptureSessionSample::Type::GyroscopeEvent:
			_lastGyroscopeEvent = sample.gyroscopeEvent;
			break;
		default:
			printf("Sample type unhandled\n");
		}
	};

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


	void run(Structure* st) {
		ST::CaptureSession session;
		session.setDelegate(st);
		if (!session.startMonitoring(st->getSettings())) {
			printf("Failed to initialize capture session!\n");
			exit(1);
		}

		/* Loop forever. The SessionDelegate receives samples on a background thread
		while streaming. */
		while (true) {
			ofSleepMillis(1000);
		}
	};

	void Structure::startThread(){
		std::thread t(run, this);
		t.detach();
	}


	ST::CaptureSessionSettings Structure::getSettings() {
		ST::CaptureSessionSettings settings;
		settings.source = ST::CaptureSessionSourceId::StructureCore;
		settings.structureCore.depthEnabled = true;
		settings.structureCore.visibleEnabled = true;
		settings.structureCore.infraredEnabled = true;
		settings.structureCore.accelerometerEnabled = true;
		settings.structureCore.gyroscopeEnabled = true;
		settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
		settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_200Hz;
		settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Hybrid;

		return settings;
	}



	Structure::~Structure() {
		ofLog() << "Waiting to stop...";
		//waitForThread();
		ofLog() << "Stopped!";

	}
