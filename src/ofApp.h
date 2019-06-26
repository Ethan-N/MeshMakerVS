#pragma once

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

	double inv_depth_fx, inv_depth_fy;
	double depth_cx, depth_cy;
	double depth_Tx, depth_Ty;
	double rgb_fx, rgb_fy;
	double rgb_cx, rgb_cy;
	double rgb_Tx, rgb_Ty;

	bool intrinsics;
    
    ofTexture myDepthTexture;
    ofVbo vbo;
    ofEasyCam cam;

	Structure st;

	double lastRenderedTimestamp;
	ofMatrix4x4 pose;
    
	float depth_row[640 * 480];
	uint8_t colors[640 * 480 * 3];

	ofIndexType faces[640 * 480 * 6];
	ofVec3f points[640 * 480];
	ofFloatColor color[640 * 480];

	vector<ofNode> nodes;

    Receiver receiver;
    MsgDelay<Orientation7> camDelay;
    // Caching the previous frame make it easy to get the delta
    uint64_t previousMicroseconds = 0;
    // This is some hacky junk for rotating the camera
    double previousPressTime;// last time the camera was pressed
    ofNode previousControllerAtPressTime;
};
