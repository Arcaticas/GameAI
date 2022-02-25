// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_COMPONENTS_BOID_H_
#define GRANTHW1_ENGINE_COMPONENTS_BOID_H_

#include <Components/SteeringOutputs.h>
#include <ofColor.h>

#include "Rigidbody.h"
#include <stack>

class Boid {
 public:
  Rigidbody m_rigidbody;
  float m_maxSpeed = 25 * 1;
  float m_maxAccel = 100 * 1;
  float m_maxRotVel = .5;
  float m_maxRotAccel = .5;
  ofColor m_color;

  std::stack<int> pathing_nodes;

  Boid();
  Boid(float i_xPos, float i_yPos);
  Boid(float i_xPos, float i_yPos, ofColor i_color);
  Boid(float i_xPos, float i_yPos, float i_angularVel);
  Boid(float i_xPos, float i_yPos, float i_angularVel, ofColor i_color);
  Boid(float i_xPos, float i_yPos, float i_xVel, float i_yVel);
  Boid(float i_xPos, float i_yPos, float i_xVel, float i_yVel, ofColor i_color);

  void DrawBoid();
  void Update();

  void ApplyDynmaticSteering(DynamicSteeringOutput i_input);
};
#endif  // GRANTHW1_ENGINE_COMPONENTS_BOID_H_
