// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_ALGORITHMS_H_
#define GRANTHW1_ENGINE_ALGORITHMS_H_
#include <vector>
#include <stack>
#include "Components/Boid.h"
#include "Components/SteeringOutputs.h"

namespace Algorithms {
float SnapToTarget(float current_orientation, ofVec2f velocity);

KinematicSteeringOutput KinematicSeek(Rigidbody character, Rigidbody target,
                                      float max_speed);

KinematicSteeringOutput KinematicFlee(Rigidbody character_rb,
                                      Rigidbody target_rb, float max_speed);

DynamicSteeringOutput DynamicSeek(Rigidbody character_rb, Rigidbody target_rb,
                                  float max_accel);

DynamicSteeringOutput DynamicArrivePM(Rigidbody character_rb,
                                      Rigidbody target_rb, float slow_radius,
                                      float target_radius, float max_accel,
                                      float max_speed, float time_to_target);

DynamicSteeringOutput DynamicWander(Rigidbody character_rb, float max_rate,
                                    float max_accel, float max_rot_accel);

DynamicSteeringOutput DynamicWanderFace(Rigidbody character_rb, float max_rate,
                                        float max_accel, float max_rot_accel);

DynamicSteeringOutput DynamicFlee(Rigidbody character_rb, Rigidbody target_rb,
                                  float max_accel);

DynamicSteeringOutput DynamicEvade(Rigidbody character_rb, Rigidbody target_rb,
                                   float max_accel, float personal_radius,
                                   float decay);

DynamicSteeringOutput DynamicSeperation(Rigidbody character_rb,
                                        std::vector<Rigidbody*> target_rbs,
                                        float max_accel, float personal_radius,
                                        float decay);

DynamicSteeringOutput DynamicAlign(Rigidbody character_rb, Rigidbody target_rb,
                                   float slow_theta, float target_theta,
                                   float max_rot, float time_to_target);

DynamicSteeringOutput DynamicFace(Rigidbody character_rb, Rigidbody target_rb,
                                  float slow_theta, float target_theta,
                                  float max_rot, float time_to_target);

DynamicSteeringOutput LookWhereYouAreGoing(Rigidbody character_rb,
                                           float slow_theta, float target_theta,
                                           float max_rot, float time_to_target);

DynamicSteeringOutput DynamicVelocityMatching(Rigidbody character_rb,
                                              float max_accel,
                                              ofVec2f targetVel,
                                              float time_to_target);

DynamicSteeringOutput DynamicRotationMatching(Rigidbody character_rb,
                                              float max_rot_accel,
                                              float targetRotVel,
                                              float time_to_target);

void DynamicFlock(std::vector<Rigidbody*> all_rbs, float max_accel,
                  float max_speed, float personal_radius, float decay,
                  float ttt_vel, float slow_radius, float target_radius,
                  float ttt_arrive);

int* Dijkstra(unsigned char** graph, int src, int target, int total_nodes);

int* aStarE(unsigned char** graph, int src, int target, int total_nodes);

int* aStarM(unsigned char** graph, int src, int target, int total_nodes);

std::stack<int> OrderPathfinding(int* pathfinding_output, int src, int target,
                                 int total_nodes);

int ManhattanDistance(int srcNode, int targetNode, int graph_width);

int EuclideanDistance(int current_node, int target_node, int graph_width);

}  // namespace Algorithms
#endif  // _GRANTHW1_ENGINE_ALGORITHMS_H_
