// Copyright 2022 Theo Grant
#include "Boid.h"

#include <app/ofAppRunner.h>
#include <math/ofMath.h>

#include "ofGraphics.h"

Boid::Boid() {
  m_rigidbody.m_pos.x = 400;
  m_rigidbody.m_pos.y = 400;
  m_rigidbody.m_vel.x = 0;
  m_rigidbody.m_vel.y = 0;

  m_color = ofColor(abs(255.0 * ofRandomf()), abs(255.0 * ofRandomf()),
                    abs(255.0 * ofRandomf()));
}

Boid::Boid(float i_xPos, float i_yPos) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_vel.x = 0;
  m_rigidbody.m_vel.y = 0;

  m_color = ofColor(abs(255.0 * ofRandomf()), abs(255.0 * ofRandomf()),
                    abs(255.0 * ofRandomf()));
}

Boid::Boid(float i_xPos, float i_yPos, ofColor i_color) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_vel.x = 0;
  m_rigidbody.m_vel.y = 0;

  m_color = i_color;
}

Boid::Boid(float i_xPos, float i_yPos, float i_angularVel) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_rotationalVel = i_angularVel;

  m_color = ofColor(abs(255.0 * ofRandomf()), abs(255.0 * ofRandomf()),
                    abs(255.0 * ofRandomf()));
}

Boid::Boid(float i_xPos, float i_yPos, float i_angularVel, ofColor i_color) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_rotationalVel = i_angularVel;

  m_color = i_color;
}

Boid::Boid(float i_xPos, float i_yPos, float i_xVel, float i_yVel) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_vel.x = i_xVel;
  m_rigidbody.m_vel.y = i_yVel;

  m_color = ofColor(abs(255.0 * ofRandomf()), abs(255.0 * ofRandomf()),
                    abs(255.0 * ofRandomf()));
}

Boid::Boid(float i_xPos, float i_yPos, float i_xVel, float i_yVel,
           ofColor i_color) {
  m_rigidbody.m_pos.x = i_xPos;
  m_rigidbody.m_pos.y = i_yPos;
  m_rigidbody.m_vel.x = i_xVel;
  m_rigidbody.m_vel.y = i_yVel;

  m_color = i_color;
}

void Boid::ApplyDynmaticSteering(DynamicSteeringOutput i_input) {
  m_rigidbody.m_accel += i_input.Accel;
  m_rigidbody.m_rotationalAccel += i_input.AngularAccel;
}

void Boid::DrawBoid() {
  ofSetColor(m_color);
  ofDrawCircle(m_rigidbody.m_pos.x, m_rigidbody.m_pos.y, 15);

  float pointX = 30 * sin(m_rigidbody.m_orientation);
  float pointY = 30 * cos(m_rigidbody.m_orientation);

  float sideX = 15 * cos(m_rigidbody.m_orientation);
  float sideY = 15 * sin(m_rigidbody.m_orientation);

  ofDrawTriangle(
      glm::vec2(m_rigidbody.m_pos.x - sideX, m_rigidbody.m_pos.y + sideY),
      glm::vec2(m_rigidbody.m_pos.x + sideX, m_rigidbody.m_pos.y - sideY),
      glm::vec2(m_rigidbody.m_pos.x - pointX, m_rigidbody.m_pos.y - pointY));
}

void Boid::Update() {
  // Limits linear acceleration
  if (m_rigidbody.m_accel.length() > m_maxAccel) {
    m_rigidbody.m_accel.scale(m_maxAccel);
  }
  m_rigidbody.m_vel.x += m_rigidbody.m_accel.x * ofGetLastFrameTime();
  m_rigidbody.m_vel.y += m_rigidbody.m_accel.y * ofGetLastFrameTime();
  // Limits rotational acceleration
  if (abs(m_rigidbody.m_rotationalAccel) > m_maxRotAccel) {
    if (m_rigidbody.m_rotationalAccel > 0)
      m_rigidbody.m_rotationalAccel = m_maxRotAccel;
    else
      m_rigidbody.m_rotationalAccel = -m_maxRotAccel;
  }
  m_rigidbody.m_rotationalVel +=
      m_rigidbody.m_rotationalAccel * ofGetLastFrameTime();
  // Limits velocity
  if (m_rigidbody.m_vel.length() > m_maxSpeed) {
    m_rigidbody.m_vel.scale(m_maxSpeed);
  }
  m_rigidbody.m_pos.x += m_rigidbody.m_vel.x * ofGetLastFrameTime();
  m_rigidbody.m_pos.y += m_rigidbody.m_vel.y * ofGetLastFrameTime();
  // Limit rotational velocity
  if (abs(m_rigidbody.m_rotationalVel) > m_maxRotVel) {
    if (m_rigidbody.m_rotationalVel > 0)
      m_rigidbody.m_rotationalVel = m_maxRotVel;
    else
      m_rigidbody.m_rotationalVel = -m_maxRotVel;
  }
  m_rigidbody.m_orientation +=
      m_rigidbody.m_rotationalVel * ofGetLastFrameTime();

  /// Endless looping code - looking back at this comment, this sounds
  /// terrifying
  if (m_rigidbody.m_pos.x > 815) {
    m_rigidbody.m_pos.x = -10;
  } else if (m_rigidbody.m_pos.x < -15) {
    m_rigidbody.m_pos.x = 810;
  }

  if (m_rigidbody.m_pos.y > 815) {
    m_rigidbody.m_pos.y = -10;
  } else if (m_rigidbody.m_pos.y < -15) {
    m_rigidbody.m_pos.y = 810;
  }
}
