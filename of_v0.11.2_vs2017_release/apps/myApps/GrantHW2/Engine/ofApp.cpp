// Copyright 2022 Theo Grant
#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
  ofBackground(0, 0, 0);
  boids.push_back(Boid(700, 500, 0, 0, ofColor::white));

  graph = new unsigned char*[18];
  unsigned char graphCustom[18][18] = {
      {0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},  // 0

      {0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 1

      {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 2

      {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 3

      {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 4

      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  // 5

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},  // 6

      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // 7

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  // 8

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},  // 9

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},  // 10

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  // 11

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0},  // 12

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 13

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  // 14

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 15

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},  // 16

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // 17
  };
  unsigned char graphCustom2[13][13] = {
      {0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0},  // 0

      {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 1

      {0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 2

      {0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0},  // 3

      {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},  // 4

      {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},  // 5

      {0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0},  // 6

      {0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 4, 10},  // 7

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 18, 0, 0},  // 8

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0},  // 9

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 10

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 16, 0, 0},  // 11

      {0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 0, 0},  // 12

  };

  // Converting to int**
  for (int i = 0; i < 18; i++) {
    graph[i] = new unsigned char[18];
    for (int j = 0; j < 18; j++) {
      graph[i][j] = graphCustom[i][j];
    }
  }

  big.CreateWall(5050, 30, 100);
  big.CreateWall(3005, 30, 100);
  big.CreateWall(585, 30, 100);
  big.CreateWall(1350, 30, 100);
}

//--------------------------------------------------------------
void ofApp::update() {
  numberOfUpdates++;

  // Boid management
  if (boids.size() > 0) {
    for (int i = 0; i < boids.size(); i++) {
      boids[i].Update();
      if (numberOfUpdates % 50 == 0) {
        breadcrumbs.push_back(Breadcrumb(boids[i].m_rigidbody.m_pos.x,
                                         boids[i].m_rigidbody.m_pos.y,
                                         boids[i].m_color));
      }
    }
  }

  // Breadcrumb management
  if (breadcrumbs.size() > 0) {
    if (breadcrumbs[0].CheckLife()) {
      breadcrumbs.erase(breadcrumbs.begin());
    }
  }

  if (!boids[0].pathing_nodes.empty()) {
    int* coords =
        GraphMaker::NodeToWorld(boids[0].pathing_nodes.top(), 100, 800);
    Rigidbody current_target;
    current_target.m_pos.x = coords[0];
    current_target.m_pos.y = coords[1];

    boids[0].m_rigidbody.m_accel =
        Algorithms::DynamicSeek(boids[0].m_rigidbody, current_target, 100)
            .Accel;
    boids[0].m_rigidbody.m_rotationalAccel =
        Algorithms::LookWhereYouAreGoing(boids[0].m_rigidbody, 1, .5, .5, 1).AngularAccel;

    if (boids[0].m_rigidbody.DistanceTo(current_target) < 10) {
      boids[0].pathing_nodes.pop();
    }
  } else {
    boids[0].m_rigidbody.m_vel.x = 0;
    boids[0].m_rigidbody.m_vel.y = 0;
    boids[0].m_rigidbody.m_accel.x = 0;
    boids[0].m_rigidbody.m_accel.y = 0;
  }
}

//--------------------------------------------------------------
void ofApp::draw() {
  for (int i = 0; i < boids.size(); i++) {
    boids[i].DrawBoid();
  }

  for (int i = 0; i < breadcrumbs.size(); i++) {
    breadcrumbs[i].DrawBreadcrumb();
  }
  ofDrawRectangle(GraphMaker::NodeToWorld(5555, 100, 800)[0],
                  GraphMaker::NodeToWorld(5553, 100, 800)[1], 170, 170);

  ofDrawRectangle(GraphMaker::NodeToWorld(3510, 100, 800)[0],
                  GraphMaker::NodeToWorld(3508, 100, 800)[1], 170, 170);

  ofDrawRectangle(GraphMaker::NodeToWorld(1090, 100, 800)[0],
                  GraphMaker::NodeToWorld(1088, 100, 800)[1], 170, 170);

  ofDrawRectangle(GraphMaker::NodeToWorld(1855, 100, 800)[0],
                  GraphMaker::NodeToWorld(1853, 100, 800)[1], 170, 170);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  /*while (current_node != src) {
    std::cout << output[current_node] << '\n';
    int* coords = GraphMaker::NodeToWorld( 4020, 100, 800);
    boids[0].m_rigidbody.m_pos.x = coords[0];
    boids[0].m_rigidbody.m_pos.y = coords[1];
    current_node = output[current_node];
  }*/


  /*for (int i = 0; i < 4020;i++) {
    if (output[i]>0)
        std::cout << output[i] << '\n';
  }*/

  std::cout << GraphMaker::WorldToNode(x, y, 100, 800) << '\n';
  int start = GraphMaker::WorldToNode(boids[0].m_rigidbody.m_pos.x,
                                      boids[0].m_rigidbody.m_pos.y, 100, 800);
  int end = GraphMaker::WorldToNode(x, y, 100, 800);
  std::cout << ofGetElapsedTimeMillis() << '\n';

  output = Algorithms::aStarE(big.data, start, end, big.total_nodes);
  std::cout << ofGetElapsedTimeMillis() << '\n';

  if (output != nullptr) {
    boids[0].pathing_nodes =
        Algorithms::OrderPathfinding(output, start, end, big.total_nodes);
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
