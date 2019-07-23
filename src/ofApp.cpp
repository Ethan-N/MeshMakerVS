#include "ofApp.h"
#include <ST/CaptureSession.h>


//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();

	depth_cam.setFov(70.); 
	depth_cam.setNearClip(.35);
	depth_cam.setFarClip(10);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX()+.125, cam.getY(), cam.getZ());

	cam.setFov(82.5); // this is overwritten by the osc receiver
	cam.setNearClip(.35);
	cam.setFarClip(10);

	w = 1280;
	h = 960;
	threshold = .15;

	// Threaded OSC Receive
	receiver.startThread();

	controller.setScale(.005);

	st.startThread();

	auto deviceList = ofxBlackmagic::Iterator::getDeviceList();

	//Technically can handle more than one input, but only sets input to last BlackMagic device
	for(auto device : deviceList) {
		auto input_dev = shared_ptr<ofxBlackmagic::Input>(new ofxBlackmagic::Input());

		static int index = 0;
		auto mode = index++ == 0 ? bmdModeHD1080p25 : bmdModeHD1080p24;
		input_dev->startCapture(device, mode);
		input = input_dev;
	}
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
	//cam.setFov(receiver.getFov()); // Can also set this in the main view


	Orientation7 control = receiver.getController();
	controller.setOrientation(control.quat);
	controller.setPosition(control.pos);

	input->update();
	

	if(st.lastDepthFrame().isValid()){

		memcpy(colors, st.lastVisibleFrame().rgbData(), sizeof(uint8_t)*640*480*3);
		cameraRGB.loadData(colors, 640, 480, GL_RGB);

		std::fill(depth, depth+w*h, 0);
		memcpy(depth, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*w*h);
		cameraDepth.loadData(depth, w, h, GL_RED);
		
		index = 0;
		depth_cam.setFov(70);
		glm::mat4 inv_MVPmatrix = glm::inverse(depth_cam.getModelViewProjectionMatrix());
		float* vec_point = glm::value_ptr(inv_MVPmatrix);
		for (int y=0; y<h; ++y) {
			for (int x=0; x<w; ++x) {
				if (depth[x + w * y] != 0 && depth[x + w * y] == depth[x + w * y]) {

					ofVec3f CameraVec;
					CameraVec.x = 2.9f * x / w - 1.45f;
					CameraVec.y = 1.3f - 2.6f * y / h;
					CameraVec.z = ofNormalize(depth[x + w * y]/1000.0, .35, 10);

					float* CameraXYZ = CameraVec.getPtr();
					float* point = points[(x + w * y)].getPtr();

					float w_vec = (*(vec_point+3) * *CameraXYZ + *(vec_point+7) * *(CameraXYZ+1) + *(vec_point+11) * *(CameraXYZ+2) + *(vec_point+15));
					*point = (*vec_point * *CameraXYZ + *(vec_point+4) * *(CameraXYZ+1) + *(vec_point+8) * *(CameraXYZ+2) + *(vec_point+12)) / w_vec;
					*(point+1) = (*(vec_point+1) * *CameraXYZ + *(vec_point+5) * *(CameraXYZ+1) + *(vec_point+9) * *(CameraXYZ+2) + *(vec_point+13)) / w_vec;
					*(point+2) = (*(vec_point+2) * *CameraXYZ + *(vec_point+6) * *(CameraXYZ+1) + *(vec_point+10) * *(CameraXYZ+2) + *(vec_point+14)) / w_vec;

					//auto world = inv_MVPmatrix * CameraXYZ;
					//points[x + w * y] = glm::vec3(world) / world.w;
					
					int left_ind = x - 1 + w * y;
					int diag_ind = x - 1 + w * (y - 1);
					int top_ind = x + w * (y - 1);

					if (points[x + w * y].distance(points[diag_ind]) < threshold) {
						//Triangle 1, Bottom Left
						if (points[x + w * y].distance(points[left_ind]) < threshold and points[left_ind].distance(points[diag_ind]) < threshold) {
							faces[index] = diag_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = left_ind;

							index += 3;
						}
						//Triangle 2, Top Right
						if (points[x + w * y].distance(points[top_ind]) < threshold and points[top_ind].distance(points[diag_ind]) < threshold) {
							faces[index] = diag_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = top_ind;

							index += 3;
						}
					}
				}
			}
		}
		vbo.setVertexData(&points[0], w*h, GL_DYNAMIC_DRAW);
		vbo.setIndexData(&faces[0], index, GL_DYNAMIC_DRAW);
	}
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(255, 255, 255);
	glDepthMask(GL_FALSE);  
	ofSetColor(255,255,255);
	cameraRGB.draw(-ofGetWidth()*.24, -ofGetHeight()*.24, 0, ofGetWidth()*1.45, ofGetHeight()*1.45);
	ofSetColor(255,255,255);
	glDepthMask(GL_TRUE); 

	ofSetColor(255, 255, 255);
	cam.setFov(82.5); 
	cam.begin();
	//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	vbo.drawElements(GL_TRIANGLES, index);
	//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	cam.end();

	//depth_cam.setFov(receiver.getFov());
	depth_cam.setFov(30);
	if (flipped)
		controller.begin();
	else
		depth_cam.begin();
	ofSetColor(255, 0, 255);
	for (ofNode n : nodes) {
		n.draw();
	}
	if (flipped) {
		cam.draw();
		ofSetColor(0, 0, 255);
		depth_cam.draw();
		controller.end();
	}
	else{
		controller.draw();
		depth_cam.end();
	}
	ofSetColor(255);
	input->draw(0, 0, ofGetWidth(), ofGetHeight());
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	ofNode n;
	switch (key) {
		case ' ':
			n.setPosition(controller.getGlobalPosition());
			n.setOrientation(controller.getGlobalOrientation());
			n.setScale(.005);
			nodes.push_back(n); 
			break;
		case 'f':
			flipped = true;
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
