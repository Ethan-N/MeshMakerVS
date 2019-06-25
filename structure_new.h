#pragma once
#include <ST/CaptureSession.h>
#include "ofMain.h"

#ifndef structure_h
#define structure_h

mutex mut;

bool intrinsics;
ST::DepthFrame lastDepthFrame;
ST::ColorFrame lastVisibleFrame;
ST::InfraredFrame lastInfraredFrame;
double inv_depth_fx, inv_depth_fy;
double depth_cx, depth_cy;
double depth_Tx, depth_Ty;
double rgb_fx, rgb_fy;
double rgb_cx, rgb_cy;
double rgb_Tx, rgb_Ty;

#endif /* structure_h */