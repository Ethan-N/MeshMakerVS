#include "ofApp.h"
#include <ST/CaptureSession.h>


//--------------------------------------------------------------
void ofApp::setup() {
	lastRenderedTimestamp = 0;

	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();
	//ofDisableAlphaBlending();

	cam.setFov(70.); // this is overwritten by the osc receiver
	cam.setNearClip(.1);
	cam.setFarClip(30);

	rgb_cam.setFov(70.); // this is overwritten by the osc receiver
	rgb_cam.setNearClip(.1);
	rgb_cam.setFarClip(30);
	
	rgb_cam.setParent(cam, true);
	rgb_cam.setPosition(cam.getX()+.01, cam.getY(), cam.getZ());

	depth_cam.setFov(70.); // this is overwritten by the osc receiver
	depth_cam.setNearClip(.1);
	depth_cam.setFarClip(30);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX()+.005, cam.getY(), cam.getZ());

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


	if(st.lastDepthFrame().isValid() && lastRenderedTimestamp != st.lastDepthFrame().timestamp()){
		lastRenderedTimestamp = st.lastDepthFrame().timestamp();

		int w = st.lastDepthFrame().width();
		int h = st.lastDepthFrame().height();

		memcpy(colors, st.lastVisibleFrame().rgbData(), sizeof(uint8_t)*640*480*3);
		cameraRGB.loadData(colors, 640, 480, GL_RGB);
		memcpy(depth, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*640*480);

		int index = 0;
		float threshold = 1.5;
		ofRectangle view(0, 0, ofGetWidth(), ofGetHeight());
		for (int x=0; x<w; ++x) {
			for (int y=0; y<h; ++y) {
				if (depth[x + w * y] != 0 && depth[x + w * y]==depth[x + w * y]) {
					//float* point = points[(x + w * y)].getPtr();
					//*point = x*2;
					//*(point+1) = y*2;
					//*(point+2) = -depth[x + w * y]/1000.0;
					ofVec3f screen_point(x*2, y*2, depth[x + w * y] / 1000.);
					points[(x + w * y)] = depth_cam.screenToWorld(screen_point, view);

					int left_ind = x - 1 + w * y;
					int diag_ind = x - 1 + w * (y - 1);
					int top_ind = x + w * (y - 1);

					if (abs(depth[x + w * y] - depth[diag_ind]) < threshold) {
						//Triangle 1, Bottom Left
						if (abs(depth[x + w * y] - depth[left_ind]) < threshold and abs(depth[left_ind] - depth[diag_ind]) < threshold) {
							faces[index] = diag_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = left_ind;

							index += 3;
						}
						//Triangle 2, Top Right
						if (abs(depth[x + w * y] - depth[top_ind]) < threshold and abs(depth[top_ind] - depth[diag_ind]) < threshold) {

							faces[index] = diag_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = top_ind;

							index += 3;
						}
					}
				}
			}
		}
		vbo.setVertexData(&points[0], w*h, GL_DYNAMIC_DRAW );
		vbo.setIndexData(&faces[0], index, GL_DYNAMIC_DRAW );
		std::fill(depth, depth+w*h, 0);
	}
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(255, 255, 255);
	glDepthMask(GL_FALSE);  
	ofSetColor(255,255,255);
	cameraRGB.draw(0, 0, 0, ofGetWidth(), ofGetHeight());
	glDepthMask(GL_TRUE); 
	cam.begin();
	vbo.drawElements(GL_TRIANGLES, sizeof(faces));
	ofSetColor(255, 0, 255);
	//rgb_cam.transformGL();
	//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	//rgb_cam.restoreTransformGL();
		for (ofNode n : nodes) {
			n.draw();
		}
	controller.draw();

	cam.end();
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
