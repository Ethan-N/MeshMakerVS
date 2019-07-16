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

	// Threaded OSC Receive
	receiver.startThread();

	controller.setScale(.005);

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
	//cam.setFov(receiver.getFov()); // Can also set this in the main view


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
		cameraDepth.loadData(depth, w, h, GL_RED);

		float threshold = .15;
		depth_cam.setFov(70);
		auto inv_MVPmatrix = glm::inverse(depth_cam.getModelViewProjectionMatrix());
		for (int y=0; y<h; ++y) {
			for (int x=0; x<w; ++x) {
				if (depth[x + w * y] != 0 && depth[x + w * y] == depth[x + w * y]) {

					glm::vec4 CameraXYZ;
					CameraXYZ.x = 2.9f * x / w - 1.45f;
					CameraXYZ.y = 1.3f - 2.6f * y / h;
					CameraXYZ.z = ofNormalize(depth[x + w * y]/1000.0, .35, 10);
					CameraXYZ.w = 1;

					auto world = inv_MVPmatrix * CameraXYZ;
					points[x + w * y] = glm::vec3(world) / world.w;
					
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
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	vbo.drawElements(GL_TRIANGLES, index);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
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
