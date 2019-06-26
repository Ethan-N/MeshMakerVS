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

	ofMatrix4x4 camera(0.999941, 0.0101288, 0.00390053, 0.0210626,
		-0.0100631, 0.999813, -0.016518, -0.000203676,
		-0.00406711, 0.0164778, 0.999856, -0.00250867,
		0, 0, 0, 1);

	// Threaded OSC Receive
	receiver.startThread();

	cam.disableMouseInput();

	pose.makeTranslationMatrix(0.0210626, -0.000203676, -0.00250867);

	pose *= camera;

	sphere.setRadius(20);

	intrinsics = false;
	depth_Tx = 0.0, depth_Ty = 0.0;
	rgb_Tx = 0.0, rgb_Ty = 0.0;

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
	cam.setPosition(cor.pos*150);
	cam.setFov(receiver.getFov()); // Can also set this in the main view



	if(st.lastDepthFrame().isValid() && lastRenderedTimestamp != st.lastDepthFrame().timestamp()){
		lastRenderedTimestamp = st.lastDepthFrame().timestamp();

		if(!intrinsics){
			inv_depth_fx = 1.0 / st.lastDepthFrame().intrinsics().fx;
			inv_depth_fy = 1.0 / st.lastDepthFrame().intrinsics().fy;
			depth_cx = st.lastDepthFrame().intrinsics().cx, depth_cy = st.lastDepthFrame().intrinsics().cy;
			rgb_fx = st.lastVisibleFrame().intrinsics().fx, rgb_fy = st.lastVisibleFrame().intrinsics().fy;
			rgb_cx = st.lastVisibleFrame().intrinsics().cx, rgb_cy = st.lastVisibleFrame().intrinsics().cy;
			intrinsics = true;
		}

		memcpy(depth_row, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*640*480);
		memcpy(colors, st.lastVisibleFrame().rgbData(), sizeof(uint8_t)*640*480*3);

		int w = st.lastDepthFrame().width();
		int h = st.lastDepthFrame().height();
		float threshold = 10;

		uint16_t* depth = new uint16_t[2*480*640];

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
					int top_ind = x + w * (y - 1);

					if (abs(depth[x + w * y] - depth[diag_ind]) < threshold) {
						//Triangle 1, Bottom Left
						if (abs(depth[x + w * y] - depth[left_ind]) < threshold and abs(depth[left_ind] - depth[diag_ind]) < threshold) {
							faces[index] = left_ind;
							faces[index + 1] = x + w * y;
							faces[index + 2] = diag_ind;

							index += 3;
						}
						//Triangle 2, Top Right
						if (abs(depth[x + w * y] - depth[top_ind]) < threshold and abs(depth[top_ind] - depth[diag_ind]) < threshold) {

							faces[index] = top_ind;
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
	ofSetColor(255, 255, 255);
	sphere.setPosition(300, 300, 0);
	sphere.draw();
	cameraRGB.draw(300, 300, -400);
	ofTranslate(100, 100, 0);
	vbo.drawElements(GL_TRIANGLES, sizeof(faces));
	cam.begin();
	ofPushMatrix();
		//ofTranslate(0, 0, 800);
		for (ofNode n : nodes) {
			ofSetColor(255, 255, 255);
			n.draw();
		}
	ofPopMatrix();

	cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	ofNode n;
	switch (key) {
		case ' ':
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
