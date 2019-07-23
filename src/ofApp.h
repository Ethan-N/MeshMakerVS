#pragma once

#include "ofxBlackMagic.h"
#include "ofMain.h"
#include "ofxOsc.h"
#include "receiver.h"
#include "msgDelay.h"
#include "structure.h"

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

	ofTexture cameraDepth;
    ofVbo vbo;
    ofCamera cam;

	bool flipped;

	Structure st;

	uint8_t colors[640 * 480 * 3];

	ofIndexType faces[1280 * 960 * 6];
	ofVec3f points[1280 * 960];
	ofFloatColor color[640 * 480];
	float depth[1280 * 960];
	int index;

	int w;
	int h;
	float threshold;

	ofCamera depth_cam;
	ofCamera controller;

	vector<ofNode> nodes;
    Receiver receiver;

	shared_ptr<ofxBlackmagic::Input> input;
};
