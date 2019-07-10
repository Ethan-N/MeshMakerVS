#include "ofApp.h"
#include <ST/CaptureSession.h>


//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();

	cam.setFov(85.); // this is overwritten by the osc receiver
	cam.setNearClip(.01);
	cam.setFarClip(100);

	controller.setFov(85.);
	controller.setNearClip(.01);
	controller.setFarClip(100);

	depth_cam.setFov(45.); // this is overwritten by the osc receiver
	depth_cam.setNearClip(.01);
	depth_cam.setFarClip(100);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX()+.125, cam.getY(), cam.getZ());

	// Threaded OSC Receive
	receiver.startThread();

	//cam.disableMouseInput();
	controller.setScale(.01);

	st.startThread();
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
	cam.setFov(receiver.getFov()); // Can also set this in the main view


	Orientation7 control = receiver.getController();
	controller.setOrientation(control.quat);
	controller.setPosition(control.pos);

	index = 0;
	if(st.lastDepthFrame().isValid()){

		int w = st.lastDepthFrame().width();
		int h = st.lastDepthFrame().height();

		memcpy(colors, st.lastVisibleFrame().rgbData(), sizeof(uint8_t)*640*480*3);
		cameraRGB.loadData(colors, 640, 480, GL_RGB);

		std::fill(depth, depth+w*h, 0);
		memcpy(depth, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*w*h);
		cameraDepth.loadData(depth, 1280, 960, GL_RED);
		
		float threshold = .1;
		ofRectangle view(0, 0, ofGetWidth(), ofGetHeight());
		for (int x=0; x<w; ++x) {
			for (int y=0; y<h; ++y) {
				if (depth[x + w * y] != 0 && depth[x + w * y] == depth[x + w * y]) {
					ofVec3f screen_point(x, y, depth[x + w * y] / 1000.);

					points[x + w * y] = depth_cam.screenToWorld(screen_point, view);

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
	//cameraRGB.draw(0, 0, 0, ofGetWidth(), ofGetHeight());
	ofSetColor(255,255,255);
	cameraDepth.draw(ofGetWidth()*.13, ofGetHeight()*.13, 0, ofGetWidth()*.87, ofGetHeight()*.87);
	glDepthMask(GL_TRUE); 
	depth_cam.begin();
	//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	//vbo.drawElements(GL_TRIANGLES, index);
	depth_cam.end();
	//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
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
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	ofNode n;
	switch (key) {
		case ' ':
			n.setScale(.01);
			n.setPosition(controller.getGlobalPosition());
			n.setOrientation(controller.getGlobalOrientation());
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
