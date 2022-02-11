// Copyright 2022 Theo Grant
#pragma once
#include <vector>
#include "Algorithms.h"
#include "Components/Breadcrumb.h"
#include "ofMain.h"

class ofApp : public ofBaseApp {
  vector<Boid> boids;
  vector<Breadcrumb> breadcrumbs;
  int numberOfUpdates = 0;
  bool noInput = true;
  bool wanderOrNo;
  bool face;
  bool seek;
  bool flock = false;
  Rigidbody targetPoint;

 public:
  void setup();
  void update();
  void draw();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
};
