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

	pos_node.setParent(cam, true);
	pos_node.setPosition(cam.getX()+.06, cam.getY(), cam.getZ());
	
	// Threaded OSC Receive
	receiver.startThread();

	cam.disableMouseInput();

	cam.setFov(70.); // this is overwritten by the osc receiver
	cam.setNearClip(0.05);
	cam.setFarClip(30);

	st.startThread();
}
//--------------------------------------------------------------
void ofApp::update(){

	std::stringstream strm;
	strm << "fps: " << ofGetFrameRate();
	ofSetWindowTitle(strm.str());

	cameraRGB.loadData(colors, 640, 480, GL_RGB);

	// Get the position of the Tracker
	Orientation7 cor = receiver.getCamera();
	Orientation7 camTriggerPosition = receiver.getPreviousCameraTrigger();
	Orientation7 controller = receiver.getController();

	cam.setOrientation(cor.quat);
	cam.setPosition(cor.pos);
	cam.setFov(receiver.getFov()); // Can also set this in the main view



	if(st.lastDepthFrame().isValid() && lastRenderedTimestamp != st.lastDepthFrame().timestamp()){
		lastRenderedTimestamp = st.lastDepthFrame().timestamp();

		int w = st.lastDepthFrame().width();
		int h = st.lastDepthFrame().height();

		st.calculateDepthTransform();
		uint16_t* depth = st.getShiftedDepth();

		memcpy(colors, st.lastVisibleFrame().rgbData(), sizeof(uint8_t)*640*480*3);
		ofVec3f pos = pos_node.getGlobalPosition();
		int index = 0;
		float threshold = 100;
		for (int x=0; x<w; ++x) {
			for (int y=0; y<h; ++y) {
				if (depth[x + w * y] != 0) {
					float* point = points[(x + w * y)].getPtr();
					*point = pos[0]+(x*2.0-640)/500.0;
					*(point+1) = pos[1]-(y*2.0-480)/500.0;
					*(point+2) = pos[2]-depth[x + w * y]/1000.0;

					uint8_t* col_pointer = &colors[(x + w * y) * 3];
					color[(x + w * y)].set(*col_pointer / 1000.0, *(col_pointer + 1) / 1000.0, *(col_pointer + 2) / 1000.0, 1.0);

					if (x == 0 or y == 0)
						continue;

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
		vbo.setVertexData( &points[0], 640*480, GL_DYNAMIC_DRAW );
		vbo.setColorData( &color[0], 640*480, GL_DYNAMIC_DRAW );
		vbo.setIndexData( &faces[0], index, GL_DYNAMIC_DRAW );
		std::fill(depth, depth+480*640, 0);
	}
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0,0,0);
	glDepthMask(GL_FALSE);  
	cameraRGB.draw(0, 0, 0, ofGetWidth(), ofGetHeight());
	glDepthMask(GL_TRUE); 
	cam.begin();
	vbo.drawElements(GL_TRIANGLES, sizeof(faces));
		for (ofNode n : nodes) {
			ofSetColor(255, 255, 255);
			n.draw();
		}

	cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	ofNode n;
	switch (key) {
		case ' ':
			n.setScale(.01);
			n.setPosition(cam.getGlobalPosition());
			n.setOrientation(cam.getGlobalOrientation());
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
