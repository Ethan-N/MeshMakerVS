#pragma once

#include "ofxBlackMagic.h"
#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxSpout2Sender.h"
#include "receiver.h"
#include "msgDelay.h"
#include "structure.h"
#include "circles.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

	void exit();

    ofVbo vbo;
    ofCamera cam;

	Structure st;

	ofIndexType faces[1280 * 960];
	ofVec3f points[1280 * 960];
	float depth[1280 * 960];
	int count;

	int w;
	int h;
	
	bool draw_mesh;

	ofCamera depth_cam;
	ofCamera controller;

	ofTrueTypeFont text;
	ofFbo textfbo;
	ofBoxPrimitive box;

	vector<ofBoxPrimitive> boxes;
	vector<ofFbo> fbos;
	vector<string> strings;
	string words;

    Receiver receiver;

	ofstream writesave;

	bool pressed;
	int curve_count;
	ofVec3f positions[4];

	Circles circles;
	int circlenum;

	float image_diag;
	float vert_fov;
	float horiz_fov;
	float focus_len;

	float abs_height[1280 * 960];
	float abs_width[1280 * 960];
	float pixel_base[1280 * 960];
	float pixel_angle[1280 * 960];
	float pixel_focus[1280 * 960];
	float pixel_base_ang[1280 * 960];

	shared_ptr<ofxBlackmagic::Input> input;

	ofFbo fbo;
	ofShader mesh_shader;
	ofxSpout2::Sender spout;
};
