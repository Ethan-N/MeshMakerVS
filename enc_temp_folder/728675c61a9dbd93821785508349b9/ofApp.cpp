#include "ofApp.h"
#include <ST/CaptureSession.h>

//Variable Declarations, mostly for camera
mutex mut;
ST::DepthFrame lastDepthFrame;
ST::ColorFrame lastVisibleFrame;
ST::InfraredFrame lastInfraredFrame;
double lastRenderedTimestamp;
ofMatrix4x4 pose;
double inv_depth_fx, inv_depth_fy;
double depth_cx, depth_cy;
double depth_Tx, depth_Ty;
double rgb_fx, rgb_fy;
double rgb_cx, rgb_cy;
double rgb_Tx, rgb_Ty;

bool intrinsics;

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
			//lastInfraredFrame = sample.infraredFrame;
			if(!intrinsics){
				inv_depth_fx = 1.0 / lastDepthFrame.intrinsics().fx;
				inv_depth_fy = 1.0 / lastDepthFrame.intrinsics().fy;
				depth_cx = lastDepthFrame.intrinsics().cx, depth_cy = lastDepthFrame.intrinsics().cy;
				rgb_fx = lastVisibleFrame.intrinsics().fx, rgb_fy = lastVisibleFrame.intrinsics().fy;
				rgb_cx = lastVisibleFrame.intrinsics().cx, rgb_cy = lastVisibleFrame.intrinsics().cy;
				intrinsics = true;
			}
			mut.unlock();
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
	//settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
	settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
	settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_200Hz;
	settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Short;
	//settings.structureCore.infraredMode = ST::StructureCoreInfraredMode::LeftCameraOnly;


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

void cameraThread() {
	std::thread t(run);
	t.detach();
}


//--------------------------------------------------------------
void ofApp::setup(){
	lastRenderedTimestamp = 0;

	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();

	ofMatrix4x4 camera(0.999941, 0.0101288, 0.00390053, 0.0210626,
		-0.0100631, 0.999813, -0.016518, -0.000203676,
		-0.00406711, 0.0164778, 0.999856, -0.00250867,
		0, 0, 0, 1);

	// Threaded OSC Receive
	receiver.startThread();

	cam.lookAt(ofVec3f(0), ofVec3f(0, 1, 0));

	pose.makeTranslationMatrix(0.0210626, -0.000203676, -0.00250867);

	pose *= camera;

	//cam.lookAt(ofVec3f(0), ofVec3f(0, 0, 1));

	intrinsics = false;
	depth_Tx = 0.0, depth_Ty = 0.0;
	rgb_Tx = 0.0, rgb_Ty = 0.0;

	// On macOS, a run loop is required for the Structure Core driver to function.

	cameraThread();
}
//--------------------------------------------------------------
void ofApp::update(){
	
	std::stringstream strm;
	strm << "fps: " << ofGetFrameRate();
	ofSetWindowTitle(strm.str());


	// Get the position of the Tracker
	Orientation7 cor = receiver.getCamera();
	Orientation7 camTriggerPosition = receiver.getPreviousCameraTrigger();
	Orientation7 controller = receiver.getController();

	/*
	
	cam.setOrientation(cor.quat);
	ofLog() << cor.quat;
	cam.setPosition(cor.pos*150);
	cam.setFov(receiver.getFov()); // Can also set this in the main view
	*/

	
	if(lastDepthFrame.isValid() && lastRenderedTimestamp != lastDepthFrame.timestamp()){
		lastRenderedTimestamp = lastDepthFrame.timestamp();

		mut.lock();
		memcpy(depth_row, lastDepthFrame.depthInMillimeters(), sizeof(float)*640*480);
		memcpy(colors, lastVisibleFrame.rgbData(), sizeof(uint8_t)*640*480*3);
		mut.unlock();

		int w = lastDepthFrame.width();
		int h = lastDepthFrame.height();
		float threshold = 5;

		uint16_t* depth = new uint16_t[2*480*640];
				
		int raw_index = 0;
		for (unsigned v = 0; v < 480; ++v)
		{
			for (unsigned u = 0; u < 640; ++u, ++raw_index)
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

				uint16_t& reg_depth = depth[v_rgb*lastVisibleFrame.width() + u_rgb];
				uint16_t new_depth = 1000.0*xyz_row2;
				
				// Validity and Z-buffer checks
				if (reg_depth == 0 )
					reg_depth = new_depth;
			}
		}
		

		int index = 0;
		for (int x=0; x<w; ++x) {
			for (int y=0; y<h; ++y) {
				if (depth[x + w * y] != 0) {

					float* point = points[(x + w * y)].getPtr();
					*point = -x;
					*(point+1) = -y;
					*(point+2) = -depth[x + w * y];

					uint8_t* col_pointer = &colors[(x + w * y) * 3];
					color[(x + w * y)].set(*col_pointer / 255.0, *(col_pointer + 1) / 255.0, *(col_pointer + 2) / 255.0, 1.0);

					if (x == 0 or y == 0)
						continue;

					int left_ind = x - 1 + w * y;
					int diag_ind = x - 1 + w * (y - 1);
					int bottom_ind = x + w * (y - 1);

					if (abs(depth[x + w * y] - depth[diag_ind]) < threshold) {
						//Triangle 1, Top Left
						if (abs(depth[x + w * y] - depth[left_ind]) < threshold and abs(depth[left_ind] - depth[diag_ind]) < threshold) {
							faces[index] = left_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = diag_ind;

							index += 3;
						}
						//Triangle 2, Bottom Right
						if (abs(depth[x + w * y] - depth[bottom_ind]) < threshold and abs(depth[bottom_ind] - depth[diag_ind]) < threshold) {

							faces[index] = bottom_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = diag_ind;

							index += 3;
						}
					}
				}
			}
		}
		vbo.setVertexData( &points[0], 640*480, GL_DYNAMIC_DRAW );
		vbo.setColorData( &color[0], 640*480, GL_DYNAMIC_DRAW );
		vbo.setIndexData( &faces[0], index, GL_DYNAMIC_DRAW );
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0,0,0);
	cam.begin();
	ofPushMatrix();
	//ofTranslate(300, 0, 1200);
	ofTranslate(300,300);
	vbo.drawElements(GL_TRIANGLES, sizeof(faces));
	ofPopMatrix();

	// draw the dam controller
	ofNode n;
	Orientation7 controller = receiver.getController();
	n.setOrientation(controller.quat);
	n.setPosition(controller.pos);
	n.setScale((controller.trigger + 1) * 0.001);
	n.draw();

	cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
