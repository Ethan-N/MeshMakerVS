#include "ofApp.h"
#include <ST/CaptureSession.h>

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();
	ofSetVerticalSync(false);
	glEnable(GL_PROGRAM_POINT_SIZE);

	cam.setNearClip(.35);
	cam.setFarClip(10);
	cam.setFov(40);
	cam.setAspectRatio(1.7777777);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX(), cam.getY(), cam.getZ()-.06);
	depth_cam.setNearClip(.35);
	depth_cam.setFarClip(10);
	depth_cam.setFov(32.5);

	w = 1280;
	h = 960;

	// Threaded OSC Receive
	receiver.startThread();
	receiver.setDelay(.095);

	controller.setScale(.005);

	st.startThread();

	draw_mesh = false;
	triangles = true;

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

	//float scale = receiver.getScale();
	float scale = 2000.0;

	image_diag = sqrt(pow(1280.0 / (2.0*scale), 2) + pow(720.0 / (2.0*scale), 2));
	vert_fov = 46.0;
	focus_len = (720.0 / (2.0*scale) / 2.0) / tan(vert_fov * 3.1415 / 180.0 / 2.0);


	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			abs_height[x + w * y] = abs(720.0 / (2.0*scale) - (y/1.33333333)/scale);
			abs_width[x + w * y] = abs(1280.0 / (2.0*scale) - x/scale);
			pixel_base[x + w * y] = sqrt(pow(focus_len, 2) + pow(abs_width[x + w * y], 2));
			pixel_angle[x + w * y] = atan2(abs_height[x + w * y], pixel_base[x + w * y]);
			pixel_focus[x + w * y] = sqrt(pow(pixel_base[x + w * y], 2) + pow(abs_height[x + w * y], 2));
			pixel_base_ang[x + w * y] = asin(focus_len/pixel_base[x + w * y]);
		}
	}

	threshold = .001;
	
	string words = "Testing Text";
	text.load("impact.ttf", 100, true, false, true);
	//Expand bounding box
	textfbo.allocate(text.stringWidth(words),text.getAscenderHeight()-text.getDescenderHeight()*1.5, GL_RGBA);

	fbo.allocate(ofGetWidth(), ofGetHeight());
}
//--------------------------------------------------------------
void ofApp::update(){

	//string words = receiver.getText();
	string words = "Testing Text";

	textfbo.begin();
	ofClear(255,255,255, 1.0);
	ofSetColor(255);
	text.drawString(words, 0, text.getAscenderHeight()-text.getDescenderHeight()*1.0);
	textfbo.end();
	box.set(textfbo.getWidth(), text.getAscenderHeight()-text.getDescenderHeight()*1.5, .1, 1, 2, false);
	box.mapTexCoordsFromTexture(textfbo.getTexture());
	box.setSideColor(box.SIDE_FRONT, ofColor(255, 255, 255, 0.0));
	box.setScale(.0003);

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
		
		count = 0;
		for (int y=0; y<h; ++y) {
			for (int x=0; x<w; ++x) {
				if (depth[x + w * y] != 0 && depth[x + w * y] < 8000 && depth[x + w * y] == depth[x + w * y]) {
					float* point = points[(x + w * y)].getPtr();

					float obj_height = (depth[x + w * y]/1000.0 + pixel_focus[x + w * y]) *  abs_height[x + w * y] / pixel_focus[x + w * y];
					float actual_pixel_base = sqrt(pow(depth[x + w * y]/1000.0 + pixel_focus[x + w * y],2) - pow(obj_height,2)) - pixel_base[x + w * y];

					points[(x + w * y)] = cam.getPosition();
					if (x >= w / 2.0)
						points[(x + w * y)] += (cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) * cam.getXAxis();
					else
						points[(x + w * y)] += -(cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) * cam.getXAxis();

					if (y >= h / 2.0)
						points[(x + w * y)] += -obj_height * cam.getYAxis();
					else
						points[(x + w * y)] += obj_height * cam.getYAxis();

					points[(x + w * y)] += -sin(pixel_base_ang[x + w * y]) * actual_pixel_base * cam.getZAxis();

					faces[count] = x + w * y;
					count += 1;
				}
			}
		}
		vbo.setVertexData(&points[0], w*h, GL_DYNAMIC_DRAW);
		vbo.setIndexData(&faces[0], count, GL_DYNAMIC_DRAW);
	}

	fbo.begin();
	ofClear(0,0,0,255);
	glDepthMask(GL_FALSE);  
	input->draw(0, 0, 1280, 720);
	glDepthMask(GL_TRUE); 

	//Circles and Mesh need to be in different cameras for glColorMask to Work

	//cam.tiltDeg(-receiver.getDelay());
	cam.tiltDeg(-3.0);
	//cam.setFov(receiver.getFov()); 
	cam.setFov(85.0); 
	cam.begin();

	if(!draw_mesh)
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	vbo.drawElements(GL_POINTS, count);
	if(!draw_mesh)
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	cam.end();
	cam.tiltDeg(3.0);
	//cam.tiltDeg(receiver.getDelay());

	box.setPosition(control.pos);
	box.setOrientation(control.quat);

	depth_cam.begin();
	
	textfbo.getTexture().bind();
	box.draw();
	textfbo.getTexture().unbind();


	circles.draw();
	depth_cam.end();
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
	case 't':
		triangles = !triangles;
		break;
	case 'e':
		circles.clear();
		circlenum = 0;
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
