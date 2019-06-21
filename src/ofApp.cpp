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
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
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

	cam.disableMouseInput();
	cam.lookAt(ofVec3f(0), ofVec3f(0, 1, 0));

	pose.makeTranslationMatrix(0.0210626, -0.000203676, -0.00250867);

	pose *= camera;

	//cam.lookAt(ofVec3f(0), ofVec3f(0, 0, 1));

	mesh.setMode(OF_PRIMITIVE_TRIANGLES);

	mesh.disableIndices();

	intrinsics = false;
	depth_Tx = 0.0, depth_Ty = 0.0;
	rgb_Tx = 0.0, rgb_Ty = 0.0;

	// On macOS, a run loop is required for the Structure Core driver to function.

	std::thread(run).detach();
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
		int threshold = 15;

		mesh.clear();

		uint16_t* depth = new uint16_t[2*480*640];


		int raw_index = 0;
		for (unsigned v = 0; v < 480; ++v)
		{
			for (unsigned u = 0; u < 640; ++u, ++raw_index)
			{
				float raw_depth = depth_row[u+v*640];
				if (!std::isfinite(raw_depth))
					continue;

				double depth_val = raw_depth/1000.0;

				/// @todo Combine all operations into one matrix multiply on (u,v,d)
				// Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
				ofVec4f xyz_depth(((u - depth_cx)*depth_val - depth_Tx) * inv_depth_fx,
					((v - depth_cy)*depth_val - depth_Ty) * inv_depth_fy,
					depth_val,
					1);

				// Transform to RGB camera frame
				ofVec4f xyz_rgb = pose * xyz_depth;

				// Project to (u,v) in RGB image
				double inv_Z = 1.0 / xyz_rgb[2];
				int u_rgb = (rgb_fx*xyz_rgb[0] + rgb_Tx)*inv_Z + rgb_cx + 0.5;
				int v_rgb = (rgb_fy*xyz_rgb[1] + rgb_Ty)*inv_Z + rgb_cy + 0.5;

				if (u_rgb < 0 || u_rgb >= 640 ||
					v_rgb < 0 || v_rgb >= 480)
					continue;

				uint16_t& reg_depth = depth[v_rgb*lastVisibleFrame.width() + u_rgb];
				uint16_t new_depth = 1000.0*xyz_rgb[2];
				// Validity and Z-buffer checks
				if (reg_depth == 0 || reg_depth > new_depth)
					reg_depth = new_depth;
			}
		}

		for (int x=1; x<w; ++x) {
			for (int y=1; y<h; ++y) {
				if(!(depth[x+w*y]==0) and depth[x+w*y]==depth[x+w*y]){

					ofColor start_color = ofColor(colors[(x+w*y)*3], colors[(x+w*y)*3+1], colors[(x+w*y)*3+2]);

					float left_point = depth[x-1+w*y];
					ofColor left_color = ofColor(colors[((x-1)+w*y)*3], colors[((x-1)+w*y)*3+1], colors[((x-1)+w*y)*3+2]);

					float diag_point = depth[x-1+w*(y-1)];
					ofColor diag_color = ofColor(colors[((x-1)+w*(y-1))*3], colors[((x-1)+w*(y-1))*3+1], colors[((x-1)+w*(y-1))*3+2]);

					float bottom_point = depth[x+w*(y-1)];
					ofColor bottom_color = ofColor(colors[(x+w*(y-1))*3], colors[(x+w*(y-1))*3+1], colors[(x+w*(y-1))*3+2]);

					if (depth[x-1+w*y]==0 or depth[x-1+w*y]!=depth[x-1+w*y]){
						left_point = depth[x+w*y];
					}
					if (depth[x-1+w*(y-1)]==0 or depth[x-1+w*(y-1)]!=depth[x-1+w*(y-1)]){
						diag_point = depth[x+w*y];
					}
					if (depth[x+w*(y-1)]==0 or depth[x+w*(y-1)]!=depth[x+w*(y-1)]){
						bottom_point = depth[x+w*y];
					}

					//Triangle 1, Top Left
					if(abs(depth[x+w*y]-left_point) < threshold and abs(depth[x+w*y]-diag_point) < threshold and abs(left_point-diag_point) < threshold){

						mesh.addVertex(ofVec3f(-x, -y, -depth[x+w*y]));
						mesh.addVertex(ofVec3f(-(x-1), -y, -left_point));
						mesh.addVertex(ofVec3f(-(x-1), -(y-1), -diag_point));

						mesh.addColor(start_color);
						mesh.addColor(left_color);
						mesh.addColor(diag_color);

					}
					//Triangle 2, Bottom Right
					if(abs(depth[x+w*y]-bottom_point) < threshold and abs(depth[x+w*y]-diag_point) < threshold and abs(bottom_point-diag_point) < threshold){
						mesh.addVertex(ofVec3f(-x, -y, -depth[x+w*y]));
						mesh.addVertex(ofVec3f(-x, -(y-1), -bottom_point));
						mesh.addVertex(ofVec3f(-(x-1), -(y-1), -diag_point));
						mesh.addColor(start_color);
						mesh.addColor(bottom_color);
						mesh.addColor(diag_color);
					}
				}
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0,0,0);
	mut.lock();
	cam.begin();
	ofPushMatrix();
	//ofTranslate(300, 0, 1200);
	mesh.draw();
	ofPopMatrix();
	mut.unlock();

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
