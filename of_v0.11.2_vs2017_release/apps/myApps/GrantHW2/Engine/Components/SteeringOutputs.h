// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_COMPONENTS_STEERINGOUTPUTS_H_
#define GRANTHW1_ENGINE_COMPONENTS_STEERINGOUTPUTS_H_
#include "math/ofVec2f.h"

struct KinematicSteeringOutput {
  ofVec2f Velocity;
  float Rotation;
};
struct DynamicSteeringOutput {
  ofVec2f Accel;
  float AngularAccel;
};
#endif  // GRANTHW1_ENGINE_COMPONENTS_STEERINGOUTPUTS_H_
