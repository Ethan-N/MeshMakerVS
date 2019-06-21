#include "receiver.h"

Receiver::~Receiver() {
    ofLog() << "Waiting for OSC Receiver thread to stop...";
    waitForThread();
    ofLog() << "...OSC Receiver thread stopped!";
    
}

void Receiver::threadedFunction() {
    oscReceiver.setup(9999);
    while(isThreadRunning()) {
        while(oscReceiver.hasWaitingMessages()){
            ofxOscMessage m;
            oscReceiver.getNextMessage(m);
            auto addr = m.getAddress();
            if (addr == "/tracker/0" || addr == "/controller/1") {
                lock();
                // Convert the message to 3d vector and a quaternion
                ofVec3f pos = ofVec3f(m.getArgAsFloat(0) * scale,
                                      m.getArgAsFloat(1) * scale,
                                      m.getArgAsFloat(2) * scale);
                float w, x, y, z;
                w = m.getArgAsFloat(3);
                x = m.getArgAsFloat(4);
                y = m.getArgAsFloat(5);
                z = m.getArgAsFloat(6);
                ofQuaternion quat = ofQuaternion(x, y, z, w);
                
                // When I was getting the quaternion from the python script I used these
                // two quaternions so that the tracker would work positioned on the flash
                // show of a camera with the tracker led pointing forward.
                // When I switched to using the openvr script to get the quaternion
                // these messed things up (made the FOV look up twoard the sky when
                // mounted on the flash shoe)
                //static const ofQuaternion r1 = ofQuaternion(90, ofVec3f(1, 0, 0));
                //static const ofQuaternion r2 = ofQuaternion(180, ofVec3f(0, 1, 0));
                //quat = r1 * r2 * quat; // rotate quat by r2 and then by r1  (I think)
                
                // rotate down slightly to make it easier to point the controller
                //static const ofQuaternion rController = ofQuaternion(-45, ofVec3f(1, 0, 0));
                //quat = rController * quat;
                
                ofNode tracker, camera;
                tracker.setPosition(pos);
                tracker.setOrientation(quat);
                camera.setParent(tracker);
                //camera.setPosition(0, -0.15, 0); // This is for when using tracker on camera
                
                Orientation7 result;
                result.pos = camera.getGlobalPosition();
                result.quat = camera.getGlobalOrientation();
                result.trigger = m.getArgAsFloat(7);
                
                double time = static_cast<double>(ofGetElapsedTimeMicros() * 0.000001);
                cameraMessages.add(time, result);
                unlock();
            } else if (addr == "/controller/0") {
                lock();
                ofVec3f pos = ofVec3f(m.getArgAsFloat(0) * scale,
                                      m.getArgAsFloat(1) * scale,
                                      m.getArgAsFloat(2) * scale);
                float w, x, y, z;
                w = m.getArgAsFloat(3);
                x = m.getArgAsFloat(4);
                y = m.getArgAsFloat(5);
                z = m.getArgAsFloat(6);
                ofQuaternion quat = ofQuaternion(x, y, z, w);
                ofNode controller, pointer;
                controller.setPosition(pos);
                controller.setOrientation(quat);
                pointer.setParent(controller);
                //pointer.setPosition(0, -70.f, -30.f); // offset for the flash shoe
                
                controllerState.pos = pointer.getGlobalPosition();
                controllerState.quat = pointer.getGlobalOrientation();
                controllerState.trigger = m.getArgAsFloat(7);
                unlock();
            } else if (addr == "/fov") {
                lock();
                fov = m.getArgAsFloat(0);
                unlock();
            } else if (addr == "/scale") {
                lock();
                scale = m.getArgAsFloat(0);
                unlock();
            } else if (addr == "/delay") {
                lock();
                delay = m.getArgAsFloat(0);
                unlock();
            } else if (addr == "/transport/position") {
                lock();
                bitwigPosition = m.getArgAsDouble(0);
                unlock();
            } else if (addr == "/track/master/meter") {
                lock();
                bitwigLevel = m.getArgAsInt(0);
                unlock();
            }
        }
        ofSleepMillis(1);
    }
    if (oscReceiver.isListening()) oscReceiver.stop();
}

Orientation7 Receiver::getCamera() {
    double time = static_cast<double>(ofGetElapsedTimeMicros() * 0.000001);
    lock(); // --- LOCK ---
    
    time -= delay;
    // only update the state if we got a new message;
    if (cameraMessages.hasMessageAt(time)) cameraState = cameraMessages.getMessageAt(time);
    Orientation7 result = cameraState;
    
    // keep track of the position of the most recent camera press
    // note that this is a little buggy -- it only works if getCamera is being
    // caled regularly. The alternative is having it arrive early.
    if (result.trigger >= .9f) {
        previousCameraTrigger = result;
    }
    
    unlock(); // --- UNLOCK ---
    return result;
}

Orientation7 Receiver::getController() {
    Orientation7 result;
    lock();
    result = controllerState;
    unlock();
    return result;
}

double Receiver::getFov() {
    double result;
    lock();
    result = fov;
    unlock();
    return result;
}

double Receiver::getScale() {
    double result;
    lock();
    result = scale;
    unlock();
    return result;
}

double Receiver::getDelay() {
    double result;
    lock();
    result = delay;
    unlock();
    return delay;
}

Orientation7 Receiver::getPreviousCameraTrigger() {
    Orientation7 result;
    lock();
    result = previousCameraTrigger;
    unlock();
    return result;
}

double Receiver::getBigwigPosition() {
    double result;
    lock();
    result = bitwigPosition;
    unlock();
    return result;
}

int Receiver::getBigwigLevel() {
    int result;
    lock();
    result = bitwigLevel;
    unlock();
    return result;
}
