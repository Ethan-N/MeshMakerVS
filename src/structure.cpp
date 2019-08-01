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

	uint16_t * Structure::getShiftedDepth()
	{
		return depth;
	}

	Structure::Structure()
	{
		ofMatrix4x4 camera(0.999941, 0.0101288, 0.00390053, 0.0210626,
			-0.0100631, 0.999813, -0.016518, -0.000203676,
			-0.00406711, 0.0164778, 0.999856, -0.00250867,
			0, 0, 0, 1);
		pose.makeTranslationMatrix(0.0210626, -0.000203676, -0.00250867);
		pose *= camera;

		intrinsics = false;
		depth_Tx = 0.0, depth_Ty = 0.0;
		rgb_Tx = 0.0, rgb_Ty = 0.0;
	}

	void Structure::calculateDepthTransform() {
		if(!intrinsics){
			inv_depth_fx = 1.0 / _lastDepthFrame.intrinsics().fx;
			inv_depth_fy = 1.0 / _lastDepthFrame.intrinsics().fy;
			depth_cx = _lastDepthFrame.intrinsics().cx, depth_cy = _lastDepthFrame.intrinsics().cy;
			rgb_fx = _lastVisibleFrame.intrinsics().fx, rgb_fy = _lastVisibleFrame.intrinsics().fy;
			rgb_cx = _lastVisibleFrame.intrinsics().cx, rgb_cy = _lastVisibleFrame.intrinsics().cy;
			intrinsics = true;
		}

		std::unique_lock<std::mutex> u(mut);
		memcpy(depth_row, _lastDepthFrame.depthInMillimeters(), sizeof(float)*640*480);
		u.unlock();

		int w = _lastDepthFrame.width();
		int h = _lastDepthFrame.height();

		for (unsigned v = 0; v < 480; ++v)
		{
			for (unsigned u = 0; u < 640; ++u)
			{
				float raw_depth = depth_row[u+v*640];
				if (raw_depth!=raw_depth)
					continue;

				double depth_val = raw_depth/1000.0;

				/// @todo Combine all operations into one matrix multiply on (u,v,d)
				// Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
				ofVec4f xyz_depth(((u - depth_cx)*depth_val - depth_Tx) * inv_depth_fx,
					((v - depth_cy)*depth_val - depth_Ty) * inv_depth_fy,
					depth_val,
					1);

				// Transform to RGB camera frame
				//ofVec4f xyz_rgb = pose * xyz_depth;
				float xyz_row2 = (pose.getPtr()[8]*xyz_depth.x + pose.getPtr()[9]*xyz_depth.y + pose.getPtr()[10]*xyz_depth.z + pose.getPtr()[11]*xyz_depth.w);
				// Project to (u,v) in RGB image
				double inv_Z = 1.0 / xyz_row2;
				int u_rgb = (rgb_fx*
					(pose.getPtr()[0]*xyz_depth.x + pose.getPtr()[1]*xyz_depth.y + pose.getPtr()[2]*xyz_depth.z + pose.getPtr()[3]*xyz_depth.w) + rgb_Tx)*inv_Z + rgb_cx + 0.5;
				int v_rgb = (rgb_fy*
					(pose.getPtr()[4]*xyz_depth.x + pose.getPtr()[5]*xyz_depth.y + pose.getPtr()[6]*xyz_depth.z + pose.getPtr()[7]*xyz_depth.w) + rgb_Ty)*inv_Z + rgb_cy + 0.5;

				if (u_rgb < 0 || u_rgb >= 640 ||
					v_rgb < 0 || v_rgb >= 480)
					continue;

				uint16_t& reg_depth = depth[v_rgb*w + u_rgb];
				uint16_t new_depth = 1000.0*xyz_row2;

				// Validity and Z-buffer checks
				if (reg_depth == 0 || reg_depth > new_depth)
					reg_depth = new_depth;
			}
		}
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
		settings.structureCore.infraredEnabled = false;
		settings.structureCore.accelerometerEnabled = false;
		settings.structureCore.gyroscopeEnabled = false;
		//settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
		settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
		settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_100Hz;
		settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Long;
		settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::Off;
		//settings.applyExpensiveCorrection = true;

		return settings;
	}
