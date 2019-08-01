#include "ofApp.h"
#include <ST/CaptureSession.h>

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();
	ofSetVerticalSync(false);

	cam.move(0, .24, 0);
	cam.setNearClip(.35);
	cam.setFarClip(10);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX(), cam.getY(), cam.getZ());
	depth_cam.setNearClip(.58);
	depth_cam.setFarClip(8);

	w = 1280;
	h = 960;

	// Threaded OSC Receive
	receiver.startThread();

	controller.setScale(.01);

	st.startThread();
	draw_mesh = false;

	curve_count = 0;
	circlenum = 0;
	circles.resize(100000);
	circles.updateGpu();
	ofLog() << circles.getMatrix(0);
	pressed = false;

	auto deviceList = ofxBlackmagic::Iterator::getDeviceList();

	//Technically can handle more than one input, but only sets input to last BlackMagic device
	for(auto device : deviceList) {
		auto input_dev = shared_ptr<ofxBlackmagic::Input>(new ofxBlackmagic::Input());

		static int index = 0;
		auto mode = index++ == 0 ? bmdModeHD1080p25 : bmdModeHD1080p24;
		input_dev->startCapture(device, mode);
		input = input_dev;
	}

	fbo.allocate(ofGetWidth(), ofGetHeight());
}
//--------------------------------------------------------------
void ofApp::update(){

	std::stringstream strm;
	strm << "fps: " << ofGetFrameRate();
	ofSetWindowTitle(strm.str());

	// Get the position of the Tracker
	Orientation7 cor = receiver.getCamera();

	cam.setOrientation(cor.quat);
	cam.setPosition(cor.pos);

	Orientation7 control = receiver.getController();
	controller.setOrientation(control.quat);
	ofVec3f old_pos = controller.getPosition();

	if (control.trigger > 0 && !pressed) {
		pressed = true;
		curve_count = 1;
		positions[0] = control.pos;
		controller.setPosition(control.pos);
		circles.setMatrix(circlenum, controller.getLocalTransformMatrix());
		circles.setColor(circlenum, ofColor::fromHsb(255*control.trigger, 255, 255));
		circles.updateGpu();
		circlenum += 1;
	}
	else if (curve_count < 3 && control.trigger > 0) {
		positions[curve_count]  = control.pos;
		curve_count += 1;
	}
	else if (control.trigger > 0) {
		if (curve_count == 3) {
			positions[3] = control.pos;
			curve_count += 1;
		}
		else {
			positions[0] = positions[1];
			positions[1] = positions[2];
			positions[2] = positions[3];
			positions[3] = control.pos;
		}
		for (int i = 1; i < 11; i++) {
			controller.setPosition(ofInterpolateCatmullRom(positions[0], positions[1], positions[2], positions[3], i * .1));
			circles.setMatrix(circlenum, controller.getGlobalTransformMatrix());
			circles.setColor(circlenum, ofColor::fromHsb(255 * control.trigger, 255, 255));
			circles.updateGpu();
			circlenum += 1;
		}
	}
	else if (control.trigger == 0 && pressed) {
		pressed = false;
	}

	input->update();

	if(st.lastDepthFrame().isValid()){
		std::fill(depth, depth+w*h, 0);
		memcpy(depth, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*w*h);
		
		index = 0;
		depth_cam.setFov(80);
		ofVec3f CameraVec;
		glm::mat4 inv_MVPmatrix = glm::inverse(depth_cam.getModelViewProjectionMatrix());
		float* vec_point = glm::value_ptr(inv_MVPmatrix);
		for (int y=0; y<h; ++y) {
			for (int x=0; x<w; ++x) {
				if (depth[x + w * y] != 0 && depth[x + w * y] < 8000 && depth[x + w * y] == depth[x + w * y]) {
					CameraVec.x = 2.0f * x / w - 1.0f;
					//Height difference due to 720p from SDI output of camera
					CameraVec.y = 1.0f - 2.0f * y / 720.0;
					CameraVec.z = ofNormalize(depth[x + w * y]/1000.0, .35, 10);


					float* CameraXYZ = CameraVec.getPtr();
					float* point = points[(x + w * y)].getPtr();
					

					//Slight optimization over standard matrix multiplication
					float w_vec = (*(vec_point+3) * *CameraXYZ + *(vec_point+7) * *(CameraXYZ+1) + *(vec_point+11) * *(CameraXYZ+2) + *(vec_point+15));

					*point = (*vec_point * *CameraXYZ + *(vec_point+4) * *(CameraXYZ+1) + *(vec_point+8) * *(CameraXYZ+2) + *(vec_point+12)) / w_vec;
					*(point + 1) = (*(vec_point+1) * *CameraXYZ + *(vec_point+5) * *(CameraXYZ+1) + *(vec_point+9) * *(CameraXYZ+2) + *(vec_point+13)) / w_vec;
					*(point + 2) =  (*(vec_point+2) * *CameraXYZ + *(vec_point+6) * *(CameraXYZ+1) + *(vec_point+10) * *(CameraXYZ+2) + *(vec_point+14)) / w_vec;
					//auto world = inv_MVPmatrix * CameraXYZ;
					//points[x + w * y] = glm::vec3(world) / world.w;

					faces[index] = x + w * y;
					index += 1;
					
				}
			}
		}
		vbo.setVertexData(&points[0], w*h, GL_DYNAMIC_DRAW);
		vbo.setIndexData(&faces[0], index, GL_DYNAMIC_DRAW);
	}

	depth_cam.move(0, -.24, 0.06);

	fbo.begin();
	ofClear(0,0,0,255);
	glDepthMask(GL_FALSE);  
	input->draw(0, 0, 1280, 720);
	glDepthMask(GL_TRUE); 


	cam.setFov(80); 
	cam.begin();
	if(draw_mesh)
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	vbo.drawElements(GL_POINTS, index);
	if(draw_mesh)
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	cam.end();

	//depth_cam.setFov(receiver.getFov());
	depth_cam.setFov(33.75);
	depth_cam.begin();
	circles.draw();
	depth_cam.end();
	depth_cam.move(0, .24, -.06);
	fbo.end();

	spout.sendTexture(fbo.getTexture(), "composition");           
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
	ofClear(0,0,0,255);
	fbo.draw(0,0);
}

//--------------------------------------------------------------
void ofApp::exit() {
	spout.exit();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
	case 'm':
		draw_mesh = !draw_mesh;
		break;
	case 'e':
		circles.clear();
		circlenum = 0;
		circles.updateGpu();
		break;
	}
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
