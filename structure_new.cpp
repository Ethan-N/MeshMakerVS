#include <ST/CaptureSession.h>
#include "ofApp.h"
#include "structure.h"

//Structure Core Code to put data into frames
//========================================================================
struct SessionDelegate : ST::CaptureSessionDelegate {
	void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) override {
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

	void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) override {
		//printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
		switch (sample.type) {
		case ST::CaptureSessionSample::Type::DepthFrame:
			printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
			break;
		case ST::CaptureSessionSample::Type::VisibleFrame:
			printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
			break;
		case ST::CaptureSessionSample::Type::InfraredFrame:
			printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
			break;
		case ST::CaptureSessionSample::Type::SynchronizedFrames:
			//printf("Synchronized frames: depth %dx%d visible %dx%d infrared %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height(), sample.visibleFrame.width(), sample.visibleFrame.height(), sample.infraredFrame.width(), sample.infraredFrame.height());
			mut.lock();
			lastDepthFrame = sample.depthFrame;
			lastVisibleFrame = sample.visibleFrame;
			mut.unlock();
			//lastInfraredFrame = sample.infraredFrame;
			if(!intrinsics){
				inv_depth_fx = 1.0 / lastDepthFrame.intrinsics().fx;
				inv_depth_fy = 1.0 / lastDepthFrame.intrinsics().fy;
				depth_cx = lastDepthFrame.intrinsics().cx, depth_cy = lastDepthFrame.intrinsics().cy;
				rgb_fx = lastVisibleFrame.intrinsics().fx, rgb_fy = lastVisibleFrame.intrinsics().fy;
				rgb_cx = lastVisibleFrame.intrinsics().cx, rgb_cy = lastVisibleFrame.intrinsics().cy;
				intrinsics = true;
			}
			break;
		case ST::CaptureSessionSample::Type::AccelerometerEvent:
			//printf("Accelerometer event: [% .5f % .5f % .5f]\n", sample.accelerometerEvent.acceleration().x, sample.accelerometerEvent.acceleration().y, sample.accelerometerEvent.acceleration().z);
			//x_accel = sample.accelerometerEvent.acceleration().x;
			break;
		case ST::CaptureSessionSample::Type::GyroscopeEvent:
			//printf("Gyroscope event: [% .5f % .5f % .5f]\n", sample.gyroscopeEvent.rotationRate().x, sample.gyroscopeEvent.rotationRate().y, sample.gyroscopeEvent.rotationRate().z);
			break;
		default:
			printf("Sample type unhandled\n");
		}
	}
};


void run() {
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

	SessionDelegate delegate;
	ST::CaptureSession session;
	session.setDelegate(&delegate);
	if (!session.startMonitoring(settings)) {
		printf("Failed to initialize capture session!\n");
		exit(1);
	}

	/* Loop forever. The SessionDelegate receives samples on a background thread
	while streaming. */
	while (true) {
		ofSleepMillis(1000);
	}
}

void startThread() {
	std::thread t(run);
	t.detach();
}
