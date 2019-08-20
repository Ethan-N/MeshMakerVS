//
//  receiver.h
//  ar-facepalm
//
//  Created by Charles Holbrow on 12/7/18.
//

#ifndef receiver_h
#define receiver_h

#include "ofMain.h"
#include "ofxOsc.h"
#include "msgDelay.h"

struct Orientation7 {
    ofVec3f pos = ofVec3f(1, 1, 1);
    ofQuaternion quat;
    float trigger;
	float trackpad_x;
	float trackpad_y;
};

class Receiver : public ofThread {
public:
    ~Receiver();
    void threadedFunction();
    Orientation7 getCamera();
    double getFov();
    double getScale();
    double getDelay();
	void setDelay(double value);
    Orientation7 getController();
    Orientation7 getPreviousCameraTrigger();
    double getBigwigPosition();
    int getBigwigLevel();
	string getText();
protected:
    MsgDelay<Orientation7> cameraMessages;
    ofxOscReceiver oscReceiver;
    Orientation7 cameraState;
    Orientation7 controllerState;
    double fov = 70;
    double scale = 1.;
    double delay = 0.00;
    double bitwigPosition = 0;
    int bitwigLevel = 0;
	string text;
    // if the camera controlled by /controller/1 (not /tracker/0), then store the
    // position of the last trigger press on the /controller/1 here.
    Orientation7 previousCameraTrigger;
};

#endif /* receiver_h */
