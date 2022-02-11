// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_ALGORITHMS_H_
#define GRANTHW1_ENGINE_ALGORITHMS_H_
#include <vector>
#include "Components/Boid.h"
#include "Components/SteeringOutputs.h"

namespace Algorithms {
float SnapToTarget(float currentOrientation, ofVec2f velocity);

KinematicSteeringOutput KinematicSeek(Rigidbody character, Rigidbody target,
                                      float maxSpeed);

KinematicSteeringOutput KinematicFlee(Rigidbody characterRB, Rigidbody targetRB,
                                      float maxSpeed);

DynamicSteeringOutput DynamicSeek(Rigidbody characterRB, Rigidbody targetRB,
                                  float maxAccel);

DynamicSteeringOutput DynamicArrivePM(Rigidbody characterRB, Rigidbody targetRB,
                                      float slowRadius, float targetRadius,
                                      float maxAccel, float maxSpeed,
                                      float timeToTarget);

DynamicSteeringOutput DynamicWander(Rigidbody characterRB, float MaxRate,
                                    float maxAccel, float MaxRotAccel);

DynamicSteeringOutput DynamicWanderFace(Rigidbody characterRB, float MaxRate,
                                        float maxAccel, float MaxRotAccel);

DynamicSteeringOutput DynamicFlee(Rigidbody characterRB, Rigidbody targetRB,
                                  float maxAccel);

DynamicSteeringOutput DynamicEvade(Rigidbody characterRB, Rigidbody targetRB,
                                   float maxAccel, float personalRadius,
                                   float decay);

DynamicSteeringOutput DynamicSeperation(Rigidbody characterRB,
                                        std::vector<Rigidbody*> targetRBs,
                                        float maxAccel, float personalRadius,
                                        float decay);

DynamicSteeringOutput DynamicAlign(Rigidbody characterRB, Rigidbody targetRB,
                                   float slowTheta, float targetTheta,
                                   float maxRot, float timeToTarget);

DynamicSteeringOutput DynamicFace(Rigidbody characterRB, Rigidbody targetRB,
                                  float slowTheta, float targetTheta,
                                  float maxRot, float timeToTarget);

DynamicSteeringOutput LookWhereYouAreGoing(Rigidbody characterRB,
                                           float slowTheta, float targetTheta,
                                           float maxRot, float timeToTarget);

DynamicSteeringOutput DynamicVelocityMatching(Rigidbody characterRB,
                                              float maxAccel, ofVec2f targetVel,
                                              float timeToTarget);

DynamicSteeringOutput DynamicRotationMatching(Rigidbody characterRB,
                                              float maxRotAccel,
                                              float targetRotVel,
                                              float timeToTarget);

void DynamicFlock(std::vector<Rigidbody*> allRBs, float maxAccel,
                  float maxSpeed, float personalRadius, float decay,
                  float tttVel, float slowRadius, float targetRadius,
                  float tttArrive);

}  // namespace Algorithms
#endif  // _GRANTHW1_ENGINE_ALGORITHMS_H_
