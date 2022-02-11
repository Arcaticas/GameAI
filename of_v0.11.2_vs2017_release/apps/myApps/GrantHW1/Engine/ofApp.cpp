// Copyright 2022 Theo Grant
#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
  ofBackground(0, 0, 0);
  boids.push_back(Boid(40, 40, 100, 0, ofColor::white));
}

//--------------------------------------------------------------
void ofApp::update() {
  numberOfUpdates++;

  if (boids.size() > 0) {
    for (int i = 0; i < boids.size(); i++) {
      boids[i].Update();
      if (numberOfUpdates % 20 == 0) {
        breadcrumbs.push_back(Breadcrumb(boids[i].m_rigidbody.m_pos.x,
                                         boids[i].m_rigidbody.m_pos.y,
                                         boids[i].m_color));
      }
      if (numberOfUpdates % 5 == 0) {
        // std::cout << boids[0].m_rigidbody.m_rotationalAccel<< "\n";
      }
    }
  }

  if (breadcrumbs.size() > 0) {
    if (breadcrumbs[0].CheckLife()) {
      breadcrumbs.erase(breadcrumbs.begin());
    }
  }

  DynamicSteeringOutput temp;

  if (flock) {
    if (boids.size() > 1) {
      std::vector<Rigidbody*> bodies;
      for (int i = 0; i < boids.size(); i++) {
        bodies.push_back(&boids[i].m_rigidbody);
      }
      boids[0].ApplyDynmaticSteering(Algorithms::DynamicWander(
          boids[0].m_rigidbody, 90, boids[0].m_maxAccel,
          boids[0].m_maxRotAccel));

      Algorithms::DynamicFlock(bodies, boids[0].m_maxAccel, boids[0].m_maxSpeed,
                               50, 10000, 1, 50, 5, 1);
    }
  } else {
    if (noInput) {
      if (boids[0].m_rigidbody.m_pos.x > 750 &&
          boids[0].m_rigidbody.m_pos.y <= 40) {
        boids[0].m_rigidbody.m_vel.x = 0;
        boids[0].m_rigidbody.m_vel.y = 100;
      } else if (boids[0].m_rigidbody.m_pos.x > 750 &&
                 boids[0].m_rigidbody.m_pos.y > 750) {
        boids[0].m_rigidbody.m_vel.x = -100;
        boids[0].m_rigidbody.m_vel.y = 0;
      } else if (boids[0].m_rigidbody.m_pos.x < 40 &&
                 boids[0].m_rigidbody.m_pos.y > 750) {
        boids[0].m_rigidbody.m_vel.x = 0;
        boids[0].m_rigidbody.m_vel.y = -100;
      } else if (boids[0].m_rigidbody.m_pos.x < 40 &&
                 boids[0].m_rigidbody.m_pos.y < 40) {
        boids[0].m_rigidbody.m_vel.x = 100;
        boids[0].m_rigidbody.m_vel.y = 0;
      }
    } else {
      if (wanderOrNo) {
        if (face) {
          temp = Algorithms::DynamicWanderFace(boids[0].m_rigidbody, 1,
                                               boids[0].m_maxAccel,
                                               boids[0].m_maxRotAccel);
        } else {
          temp = Algorithms::DynamicWander(boids[0].m_rigidbody, 1,
                                           boids[0].m_maxAccel,
                                           boids[0].m_maxRotAccel);
        }
      } else {
        if (seek) {
          temp.Accel = Algorithms::DynamicSeek(boids[0].m_rigidbody,
                                               targetPoint, boids[0].m_maxAccel)
                           .Accel;

          boids[0].m_rigidbody.m_orientation = Algorithms::SnapToTarget(
              boids[0].m_rigidbody.m_orientation, boids[0].m_rigidbody.m_vel);

        } else {
          temp.Accel = Algorithms::DynamicArrivePM(
                           boids[0].m_rigidbody, targetPoint, 200, 20,
                           boids[0].m_maxAccel, boids[0].m_maxSpeed, .5)
                           .Accel;
          boids[0].m_rigidbody.m_orientation = Algorithms::SnapToTarget(
              boids[0].m_rigidbody.m_orientation, boids[0].m_rigidbody.m_vel);
        }
      }
    }
  }

  boids[0].m_rigidbody.m_accel = temp.Accel;
  boids[0].m_rigidbody.m_rotationalAccel = temp.AngularAccel;
}

//--------------------------------------------------------------
void ofApp::draw() {
  for (int i = 0; i < boids.size(); i++) {
    boids[i].DrawBoid();
  }

  for (int i = 0; i < breadcrumbs.size(); i++) {
    breadcrumbs[i].DrawBreadcrumb();
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
  if (key == 'q') {
    wanderOrNo = true;
    face = true;
  } else if (key == 'e') {
    wanderOrNo = true;
    face = false;
  } else if (key == OF_KEY_TAB) {
    flock = true;
    boids.push_back(Boid(600, 400, ofColor::red));
    boids.push_back(Boid(500, 400, ofColor::red));
    boids.push_back(Boid(500, 420, ofColor::red));
    boids.push_back(Boid(300, 480, ofColor::red));
    boids.push_back(Boid(100, 420, ofColor::red));
    boids.push_back(Boid(550, 400, ofColor::red));
    boids.push_back(Boid(500, 450, ofColor::red));
    boids.push_back(Boid(490, 400, ofColor::red));
    boids.push_back(Boid(350, 420, ofColor::red));
    boids.push_back(Boid(550, 410, ofColor::red));
    boids.push_back(Boid(510, 420, ofColor::red));
    boids.push_back(Boid(350, 480, ofColor::red));
    boids.push_back(Boid(150, 420, ofColor::red));
    boids.push_back(Boid(570, 410, ofColor::red));
    boids.push_back(Boid(570, 450, ofColor::red));
    boids.push_back(Boid(490, 400, ofColor::red));
  }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  noInput = false;
  targetPoint.m_pos.x = x;
  targetPoint.m_pos.y = y;
  if (button == 0) {
    seek = true;
  } else if (button == 2) {
    seek = false;
  }
  }

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {}
