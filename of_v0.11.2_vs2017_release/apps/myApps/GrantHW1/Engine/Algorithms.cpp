// Copyright 2022 Theo Grant
#include "Algorithms.h"

#include <algorithm>
#include <vector>
#include <ofMain.h>

namespace Algorithms {
float SnapToTarget(float currentOrientation, ofVec2f velocity) {
  if (velocity.length() > 0) {
    return atan2(-velocity.x, -velocity.y);
  } else {
    return currentOrientation;
  }
}

KinematicSteeringOutput KinematicSeek(Rigidbody characterRB, Rigidbody targetRB,
                                      float maxSpeed) {
  KinematicSteeringOutput result = KinematicSteeringOutput();
  result.Velocity = targetRB.m_pos - characterRB.m_pos;
  result.Velocity = result.Velocity.normalize();
  result.Velocity *= maxSpeed;

  result.Rotation = 0;
  return result;
}

KinematicSteeringOutput KinematicFlee(Rigidbody characterRB, Rigidbody targetRB,
                                      float maxSpeed) {
  KinematicSteeringOutput result = KinematicSteeringOutput();
  result.Velocity = targetRB.m_pos - characterRB.m_pos;
  result.Velocity = result.Velocity.normalize();
  result.Velocity *= -maxSpeed;

  result.Rotation = 0;
  return result;
}

DynamicSteeringOutput DynamicSeek(Rigidbody characterRB, Rigidbody targetRB,
                                  float maxAccel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = targetRB.m_pos - characterRB.m_pos;
  result.Accel = result.Accel.normalize();
  result.Accel *= maxAccel;

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicArrivePM(Rigidbody characterRB, Rigidbody targetRB,
                                      float slowRadius, float targetRadius,
                                      float maxAccel, float maxSpeed,
                                      float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = targetRB.m_pos - characterRB.m_pos;

  if (result.Accel.length() > slowRadius) {
    result.Accel.normalize();
    result.Accel *= maxAccel;
    result.AngularAccel = 0;
  } else if (result.Accel.length() > targetRadius) {
    float lerp =
        (result.Accel.length() - targetRadius) / (slowRadius - targetRadius);
    float newMaxSpeed = lerp * maxSpeed;
    ofVec2f targetVec = result.Accel;
    targetVec.scale(newMaxSpeed);
    ofVec2f velocityDif = targetVec - characterRB.m_vel;
    result.Accel = velocityDif / timeToTarget;
    result.AngularAccel = 0;
  } else {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  }

  return result;
}

DynamicSteeringOutput DynamicWander(Rigidbody characterRB, float MaxRate,
                                    float maxAccel, float MaxRotAccel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  float clampedOrientation = ofWrapRadians(characterRB.m_orientation);
  float thetaSize = abs(MaxRate - clampedOrientation);
  thetaSize *= ofRandomf();
  thetaSize += clampedOrientation;

  ofVec2f normal = ofVec2f(-sin(thetaSize), -cos(thetaSize));

  normal *= 10;  // offset
  Rigidbody target;
  target.m_pos = characterRB.m_pos + normal;

  result.Accel = DynamicSeek(characterRB, target, maxAccel).Accel;

  result.AngularAccel = LookWhereYouAreGoing(characterRB, 1, .2, MaxRotAccel,
  .1).AngularAccel;

  return result;
}

DynamicSteeringOutput DynamicWanderFace(Rigidbody characterRB, float MaxRate,
                                    float maxAccel, float MaxRotAccel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  float clampedOrientation = ofWrapRadians(characterRB.m_orientation);
  float thetaSize = abs(MaxRate - clampedOrientation);
  thetaSize *= ofRandomf();
  thetaSize += clampedOrientation;

  ofVec2f normal = ofVec2f(-sin(thetaSize), -cos(thetaSize));

  normal *= 10;  // offset
  Rigidbody target;
  target.m_pos = characterRB.m_pos + normal;

  result.Accel = DynamicSeek(characterRB, target, maxAccel).Accel;

  result.AngularAccel =
      DynamicFace(characterRB, target, 1, .1, MaxRotAccel, .01).AngularAccel;
  return result;
}

DynamicSteeringOutput DynamicFlee(Rigidbody characterRB, Rigidbody targetRB,
                                  float maxAccel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = targetRB.m_pos - characterRB.m_pos;
  result.Accel = result.Accel.normalize();
  result.Accel *= -maxAccel;

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicEvade(Rigidbody characterRB, Rigidbody targetRB,
                                   float maxAccel, float personalRadius,
                                   float decay) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f distance = targetRB.m_pos - characterRB.m_pos;

  if (distance.length() > personalRadius) {
    result.Accel = DynamicFlee(characterRB, targetRB, 0).Accel;
  } else {
    float repulsion = decay / (distance.length() * distance.length());
    float strength = min(repulsion, maxAccel);
    result.Accel = DynamicFlee(characterRB, targetRB, strength).Accel;
  }

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicSeperation(Rigidbody characterRB,
                                        std::vector<Rigidbody*> targetRBs,
                                        float maxAccel, float personalRadius,
                                        float decay) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f output = ofVec2f();
  for (int i = 0; i < targetRBs.size(); i++) {
    output += DynamicEvade(characterRB, *targetRBs[i], maxAccel, personalRadius,
                           decay)
                  .Accel;
  }
  if (output.length() > maxAccel) {
    output.scale(maxAccel);
  }
  result.Accel = output;
  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicAlign(Rigidbody characterRB, Rigidbody targetRB,
                                   float slowTheta, float targetTheta,
                                   float maxRot, float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();

  float difference = targetRB.m_orientation - characterRB.m_orientation;

  int sign = 1;
  if (difference < 0) {
    sign = -1;
  }

  difference = ofWrapRadians(difference);

  float targetRotationVel;

  float thetaSize = abs(difference);
  if (thetaSize < targetTheta) {
    targetRotationVel = 0;

  } else if (thetaSize > slowTheta) {
    targetRotationVel = maxRot;
  } else {
    float lerpFactor = thetaSize / slowTheta;

    targetRotationVel = maxRot * lerpFactor;
  }

  float rotVelDiff = (targetRotationVel - characterRB.m_rotationalVel);
  float rotAccel = rotVelDiff / timeToTarget;
  result.AngularAccel = rotAccel * sign;

  result.Accel.x = 0;
  result.Accel.y = 0;
  return result;
}

DynamicSteeringOutput DynamicFace(Rigidbody characterRB, Rigidbody targetRB,
                                  float slowTheta, float targetTheta,
                                  float maxRot, float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f direction = targetRB.m_pos - characterRB.m_pos;

  if (direction.length() < .001) {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  } else {
    Rigidbody target;
    target.m_orientation = atan2(-direction.x, -direction.y);

    result = DynamicAlign(characterRB, target, slowTheta, targetTheta, maxRot,
                          timeToTarget);
  }

  return result;
}

DynamicSteeringOutput LookWhereYouAreGoing(Rigidbody characterRB,
                                           float slowTheta, float targetTheta,
                                           float maxRot, float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  if (characterRB.m_vel.length() < .001) {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  } else {
    Rigidbody target;
    target.m_orientation = atan2(-characterRB.m_vel.y, -characterRB.m_vel.x);
    result = DynamicAlign(characterRB, target, slowTheta, targetTheta, maxRot,
                          timeToTarget);
  }
  return result;
}

DynamicSteeringOutput DynamicVelocityMatching(Rigidbody characterRB,
                                              float maxAccel, ofVec2f targetVel,
                                              float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f velocityDif = targetVel - characterRB.m_vel;
  result.Accel = velocityDif / timeToTarget;
  result.AngularAccel = 0;

  return result;
}

DynamicSteeringOutput DynamicRotationMatching(Rigidbody characterRB,
                                              float maxRotAccel,
                                              float targetRotVel,
                                              float timeToTarget) {
  DynamicSteeringOutput result = DynamicSteeringOutput();

  float rotVelDiff = (targetRotVel - characterRB.m_rotationalVel);
  float rotAccel = rotVelDiff / timeToTarget;
  result.AngularAccel = rotVelDiff;

  result.Accel.x = 0;
  result.Accel.y = 0;
  return result;
}

void DynamicFlock(std::vector<Rigidbody*> allRBs,
                  float maxAccel, float maxSpeed,
                  float personalRadius, float decay,
                  float tttVel, float slowRadius,
                  float targetRadius, float tttArrive) {
  DynamicSteeringOutput output;
  DynamicSteeringOutput finalOutput;
  finalOutput.AngularAccel = 0;
  float seperationWeight = 5;
  float velocityWeight = .4;
  float arriveWeight = .6;
  ofVec2f totalVel;
  Rigidbody centroid;
  float totalWeight = 0;
  allRBs[0]->m_weight = 40;
  for (int i = 0; i < allRBs.size(); i++) {
    // velocity * weight
    totalVel.x += allRBs[i]->m_vel.x * allRBs[i]->m_weight;
    totalVel.y += allRBs[i]->m_vel.y * allRBs[i]->m_weight;
    totalWeight += allRBs[i]->m_weight;
  }
  totalVel.x /= totalWeight;
  totalVel.y /= totalWeight;
  totalWeight = 0;
  for (int i = 0; i < allRBs.size(); i++) {
    // velocity * weight
    centroid.m_pos.x += allRBs[i]->m_pos.x * allRBs[i]->m_weight;
    centroid.m_pos.y += allRBs[i]->m_pos.y * allRBs[i]->m_weight;
    totalWeight += allRBs[i]->m_weight;
  }
  centroid.m_pos.x /= totalWeight;
  centroid.m_pos.y /= totalWeight;
  for (int i = 0; i < allRBs.size(); i++) {
    output =
        DynamicSeperation(*allRBs[i], allRBs, maxAccel, personalRadius, decay);
    finalOutput.Accel += output.Accel * seperationWeight;
    finalOutput.AngularAccel += output.AngularAccel * seperationWeight;

    output = DynamicVelocityMatching(*allRBs[i], maxAccel, totalVel, tttVel);
    finalOutput.Accel += output.Accel * velocityWeight;
    finalOutput.AngularAccel += output.AngularAccel * velocityWeight;

    output = DynamicArrivePM(*allRBs[i], centroid, slowRadius, targetRadius,
                             maxAccel, maxSpeed, tttArrive);
    finalOutput.Accel += output.Accel * arriveWeight;
    finalOutput.AngularAccel += output.AngularAccel * arriveWeight;
    // std::cout << centroid.m_pos.x << " " << centroid.m_pos.y << "\n";
    allRBs[i]->m_accel += finalOutput.Accel;
    allRBs[i]->m_rotationalAccel += finalOutput.AngularAccel;
  }
  std::cout << allRBs[0]->m_pos.x << " " << allRBs[0]->m_pos.y << "\n";
}

}  // namespace Algorithms
