// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_COMPONENTS_RIGIDBODY_H_
#define GRANTHW1_ENGINE_COMPONENTS_RIGIDBODY_H_
#include "math/ofVec2f.h"
#include "ofMathConstants.h"

struct Rigidbody {
 public:
  ofVec2f m_pos;
  ofVec2f m_vel;
  ofVec2f m_accel;

  float m_orientation = 0;
  float m_rotationalVel = 0;
  float m_rotationalAccel = 0;

  float m_weight = 1;

  int DistanceTo(Rigidbody target) {
    return sqrt((m_pos.x - target.m_pos.x) * (m_pos.x - target.m_pos.x) +
                (m_pos.y - target.m_pos.y) * (m_pos.y - target.m_pos.y));
  }
};

#endif
