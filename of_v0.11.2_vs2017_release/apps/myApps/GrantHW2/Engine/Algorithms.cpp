// Copyright 2022 Theo Grant
#include "Algorithms.h"

#include <ofMain.h>

#include <algorithm>
#include <vector>

namespace Algorithms {
float SnapToTarget(float current_orientation, ofVec2f velocity) {
  if (velocity.length() > 0) {
    return atan2(-velocity.x, -velocity.y);
  } else {
    return current_orientation;
  }
}

KinematicSteeringOutput KinematicSeek(Rigidbody character_rb,
                                      Rigidbody target_rb, float max_speed) {
  KinematicSteeringOutput result = KinematicSteeringOutput();
  result.Velocity = target_rb.m_pos - character_rb.m_pos;
  result.Velocity = result.Velocity.normalize();
  result.Velocity *= max_speed;

  result.Rotation = 0;
  return result;
}

KinematicSteeringOutput KinematicFlee(Rigidbody character_rb,
                                      Rigidbody target_rb, float max_speed) {
  KinematicSteeringOutput result = KinematicSteeringOutput();
  result.Velocity = target_rb.m_pos - character_rb.m_pos;
  result.Velocity = result.Velocity.normalize();
  result.Velocity *= -max_speed;

  result.Rotation = 0;
  return result;
}

DynamicSteeringOutput DynamicSeek(Rigidbody character_rb, Rigidbody target_rb,
                                  float max_accel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = target_rb.m_pos - character_rb.m_pos;
  result.Accel = result.Accel.normalize();
  result.Accel *= max_accel;

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicArrivePM(Rigidbody character_rb,
                                      Rigidbody target_rb, float slow_radius,
                                      float target_radius, float max_accel,
                                      float max_speed, float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = target_rb.m_pos - character_rb.m_pos;

  if (result.Accel.length() > slow_radius) {
    result.Accel.normalize();
    result.Accel *= max_accel;
    result.AngularAccel = 0;
  } else if (result.Accel.length() > target_radius) {
    float lerp =
        (result.Accel.length() - target_radius) / (slow_radius - target_radius);
    float newMaxSpeed = lerp * max_speed;
    ofVec2f targetVec = result.Accel;
    targetVec.scale(newMaxSpeed);
    ofVec2f velocityDif = targetVec - character_rb.m_vel;
    result.Accel = velocityDif / time_to_target;
    result.AngularAccel = 0;
  } else {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  }

  return result;
}

DynamicSteeringOutput DynamicWander(Rigidbody character_rb, float max_rate,
                                    float max_accel, float max_rot_accel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  float clampedOrientation = ofWrapRadians(character_rb.m_orientation);
  float thetaSize = abs(max_rate - clampedOrientation);
  thetaSize *= ofRandomf();
  thetaSize += clampedOrientation;

  ofVec2f normal = ofVec2f(-sin(thetaSize), -cos(thetaSize));

  normal *= 10;  // offset
  Rigidbody target;
  target.m_pos = character_rb.m_pos + normal;

  result.Accel = DynamicSeek(character_rb, target, max_accel).Accel;

  result.AngularAccel =
      LookWhereYouAreGoing(character_rb, 1, .2, max_rot_accel, .1).AngularAccel;

  return result;
}

DynamicSteeringOutput DynamicWanderFace(Rigidbody character_rb, float max_rate,
                                        float max_accel, float max_rot_accel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  float clampedOrientation = ofWrapRadians(character_rb.m_orientation);
  float thetaSize = abs(max_rate - clampedOrientation);
  thetaSize *= ofRandomf();
  thetaSize += clampedOrientation;

  ofVec2f normal = ofVec2f(-sin(thetaSize), -cos(thetaSize));

  normal *= 10;  // offset
  Rigidbody target;
  target.m_pos = character_rb.m_pos + normal;

  result.Accel = DynamicSeek(character_rb, target, max_accel).Accel;

  result.AngularAccel =
      DynamicFace(character_rb, target, 1, .1, max_rot_accel, .01).AngularAccel;
  return result;
}

DynamicSteeringOutput DynamicFlee(Rigidbody character_rb, Rigidbody target_rb,
                                  float max_accel) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  result.Accel = target_rb.m_pos - character_rb.m_pos;
  result.Accel = result.Accel.normalize();
  result.Accel *= -max_accel;

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicEvade(Rigidbody character_rb, Rigidbody target_rb,
                                   float max_accel, float personal_radius,
                                   float decay) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f distance = target_rb.m_pos - character_rb.m_pos;

  if (distance.length() > personal_radius) {
    result.Accel = DynamicFlee(character_rb, target_rb, 0).Accel;
  } else {
    float repulsion = decay / (distance.length() * distance.length());
    float strength = min(repulsion, max_accel);
    result.Accel = DynamicFlee(character_rb, target_rb, strength).Accel;
  }

  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicSeperation(Rigidbody character_rb,
                                        std::vector<Rigidbody*> target_rbs,
                                        float max_accel, float personal_radius,
                                        float decay) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f output = ofVec2f();
  for (int i = 0; i < target_rbs.size(); i++) {
    output += DynamicEvade(character_rb, *target_rbs[i], max_accel,
                           personal_radius, decay)
                  .Accel;
  }
  if (output.length() > max_accel) {
    output.scale(max_accel);
  }
  result.Accel = output;
  result.AngularAccel = 0;
  return result;
}

DynamicSteeringOutput DynamicAlign(Rigidbody character_rb, Rigidbody target_rb,
                                   float slow_theta, float target_theta,
                                   float max_rot, float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();

  float difference = target_rb.m_orientation - character_rb.m_orientation;

  int sign = 1;
  if (difference < 0) {
    sign = -1;
  }

  difference = ofWrapRadians(difference);

  float targetRotationVel;

  float thetaSize = abs(difference);
  if (thetaSize < target_theta) {
    targetRotationVel = 0;

  } else if (thetaSize > slow_theta) {
    targetRotationVel = max_rot;
  } else {
    float lerpFactor = thetaSize / slow_theta;

    targetRotationVel = max_rot * lerpFactor;
  }

  float rotVelDiff = (targetRotationVel - character_rb.m_rotationalVel);
  float rotAccel = rotVelDiff / time_to_target;
  result.AngularAccel = rotAccel * sign;

  result.Accel.x = 0;
  result.Accel.y = 0;
  return result;
}

DynamicSteeringOutput DynamicFace(Rigidbody character_rb, Rigidbody target_rb,
                                  float slow_theta, float target_theta,
                                  float max_rot, float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f direction = target_rb.m_pos - character_rb.m_pos;

  if (direction.length() < .001) {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  } else {
    Rigidbody target;
    target.m_orientation = atan2(-direction.x, -direction.y);

    result = DynamicAlign(character_rb, target, slow_theta, target_theta,
                          max_rot, time_to_target);
  }

  return result;
}

DynamicSteeringOutput LookWhereYouAreGoing(Rigidbody character_rb,
                                           float slow_theta, float target_theta,
                                           float max_rot,
                                           float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  if (character_rb.m_vel.length() < .001) {
    result.Accel.x = 0;
    result.Accel.y = 0;
    result.AngularAccel = 0;
  } else {
    Rigidbody target;
    target.m_orientation = atan2(-character_rb.m_vel.y, -character_rb.m_vel.x);
    result = DynamicAlign(character_rb, target, slow_theta, target_theta,
                          max_rot, time_to_target);
  }
  return result;
}

DynamicSteeringOutput DynamicVelocityMatching(Rigidbody character_rb,
                                              float max_accel,
                                              ofVec2f targetVel,
                                              float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();
  ofVec2f velocityDif = targetVel - character_rb.m_vel;
  result.Accel = velocityDif / time_to_target;
  result.AngularAccel = 0;

  return result;
}

DynamicSteeringOutput DynamicRotationMatching(Rigidbody character_rb,
                                              float max_rot_accel,
                                              float targetRotVel,
                                              float time_to_target) {
  DynamicSteeringOutput result = DynamicSteeringOutput();

  float rotVelDiff = (targetRotVel - character_rb.m_rotationalVel);
  float rotAccel = rotVelDiff / time_to_target;
  result.AngularAccel = rotVelDiff;

  result.Accel.x = 0;
  result.Accel.y = 0;
  return result;
}

void DynamicFlock(std::vector<Rigidbody*> all_rbs, float max_accel,
                  float max_speed, float personal_radius, float decay,
                  float ttt_vel, float slow_radius, float target_radius,
                  float ttt_arrive) {
  DynamicSteeringOutput output;
  DynamicSteeringOutput final_output;
  final_output.AngularAccel = 0;
  float seperationWeight = 5;
  float velocity_weight = .4;
  float arrive_weight = .6;
  ofVec2f total_vel;
  Rigidbody centroid;
  float total_weight = 0;
  all_rbs[0]->m_weight = 40;
  for (int i = 0; i < all_rbs.size(); i++) {
    // velocity * weight
    total_vel.x += all_rbs[i]->m_vel.x * all_rbs[i]->m_weight;
    total_vel.y += all_rbs[i]->m_vel.y * all_rbs[i]->m_weight;
    total_weight += all_rbs[i]->m_weight;
  }
  total_vel.x /= total_weight;
  total_vel.y /= total_weight;
  total_weight = 0;
  for (int i = 0; i < all_rbs.size(); i++) {
    // velocity * weight
    centroid.m_pos.x += all_rbs[i]->m_pos.x * all_rbs[i]->m_weight;
    centroid.m_pos.y += all_rbs[i]->m_pos.y * all_rbs[i]->m_weight;
    total_weight += all_rbs[i]->m_weight;
  }
  centroid.m_pos.x /= total_weight;
  centroid.m_pos.y /= total_weight;
  for (int i = 0; i < all_rbs.size(); i++) {
    output = DynamicSeperation(*all_rbs[i], all_rbs, max_accel, personal_radius,
                               decay);
    final_output.Accel += output.Accel * seperationWeight;
    final_output.AngularAccel += output.AngularAccel * seperationWeight;

    output =
        DynamicVelocityMatching(*all_rbs[i], max_accel, total_vel, ttt_vel);
    final_output.Accel += output.Accel * velocity_weight;
    final_output.AngularAccel += output.AngularAccel * velocity_weight;

    output = DynamicArrivePM(*all_rbs[i], centroid, slow_radius, target_radius,
                             max_accel, max_speed, ttt_arrive);
    final_output.Accel += output.Accel * arrive_weight;
    final_output.AngularAccel += output.AngularAccel * arrive_weight;
    // std::cout << centroid.m_pos.x << " " << centroid.m_pos.y << "\n";
    all_rbs[i]->m_accel += final_output.Accel;
    all_rbs[i]->m_rotationalAccel += final_output.AngularAccel;
  }
  std::cout << all_rbs[0]->m_pos.x << " " << all_rbs[0]->m_pos.y << "\n";
}

int* Dijkstra(unsigned char** graph, int src, int target, int total_nodes) {
  int* cost_to = new int[total_nodes];
  int* previous_node = new int[total_nodes];
  bool* visted = new bool[total_nodes];

  for (int i = 0; i < total_nodes; i++) {
    cost_to[i] = INT_MAX;
    visted[i] = false;
  }

  cost_to[src] = 0;

  for (int count = 0; count < total_nodes - 1; count++) {
    int min = INT_MAX;
    int min_index = -1;

    for (int i = 0; i < total_nodes; i++) {
      if (visted[i] == false && cost_to[i] <= min) {
        min = cost_to[i];
        min_index = i;
      }
    }
    // If tries to enter space with no connection.
    if (min_index == -1) {
      return nullptr;
    }
    int u = min_index;
    visted[u] = true;

    for (int v = 0; v < total_nodes; v++) {
      if (!visted[v] && graph[u][v] && cost_to[u] != INT_MAX &&
          cost_to[u] + graph[u][v] < cost_to[v]) {
        cost_to[v] = cost_to[u] + graph[u][v];
        previous_node[v] = u;
        if (v == target) {
          return previous_node;
        }
      }
    }
  }

  return nullptr;
}


int* aStarE(unsigned char** graph, int src, int target, int total_nodes) {
  int* cost_to = new int[total_nodes];
  int* previous_node = new int[total_nodes];
  bool* visted = new bool[total_nodes];

  for (int i = 0; i < total_nodes; i++) {
    cost_to[i] = INT_MAX;
    visted[i] = false;
  }

  cost_to[src] = 0;

  for (int count = 0; count < total_nodes - 1; count++) {
    int min = INT_MAX;
    int min_index = -1;
    int hValue;

    for (int i = 0; i < total_nodes; i++) {
      hValue = EuclideanDistance(i, target, sqrt(total_nodes));
      if (visted[i] == false && cost_to[i] + hValue <= min &&
          cost_to[i] != INT_MAX) {
        min = cost_to[i] + hValue;
        min_index = i;
      }
    }
    // If tries to enter space with no connection.
    if (min_index == -1) {
      return nullptr;
    }
    int u = min_index;
    visted[u] = true;

    for (int v = 0; v < total_nodes; v++) {
      hValue = EuclideanDistance(v, target, sqrt(total_nodes));
      if (!visted[v] && graph[u][v] && cost_to[u] != INT_MAX &&
          cost_to[u] + graph[u][v] + hValue < cost_to[v]) {
        cost_to[v] = cost_to[u] + graph[u][v] + hValue;
        previous_node[v] = u;
        if (v == target) {
          return previous_node;
        }
      }
    }
  }

  return nullptr;
}

int* aStarM(unsigned char** graph, int src, int target, int total_nodes) {
  int* cost_to = new int[total_nodes];
  int* previous_node = new int[total_nodes];
  bool* visted = new bool[total_nodes];

  for (int i = 0; i < total_nodes; i++) {
    cost_to[i] = INT_MAX;
    visted[i] = false;
  }

  cost_to[src] = 0;

  for (int count = 0; count < total_nodes - 1; count++) {
    int min = INT_MAX;
    int min_index = -1;
    int hValue;

    for (int i = 0; i < total_nodes; i++) {
      hValue = ManhattanDistance(i, target, sqrt(total_nodes));
      if (visted[i] == false && cost_to[i] + hValue <= min &&
          cost_to[i] != INT_MAX) {
        min = cost_to[i] + hValue;
        min_index = i;
      }
    }
    // If tries to enter space with no connection.
    if (min_index == -1) {
      return nullptr;
    }
    int u = min_index;
    visted[u] = true;

    for (int v = 0; v < total_nodes; v++) {
      hValue = ManhattanDistance(v, target, sqrt(total_nodes));
      if (!visted[v] && graph[u][v] && cost_to[u] != INT_MAX &&
          cost_to[u] + graph[u][v] + hValue < cost_to[v]) {
        cost_to[v] = cost_to[u] + graph[u][v] + hValue;
        previous_node[v] = u;
        if (v == target) {
          return previous_node;
        }
      }
    }
  }

  return nullptr;
}

std::stack<int> OrderPathfinding(int* pathfinding_output, int src, int target,
                                 int total_nodes) {
  int current_node = target;
  stack<int> output;
  while (current_node != src) {
    output.push(pathfinding_output[current_node]);
    current_node = pathfinding_output[current_node];
  }
  return output;
}

int ManhattanDistance(int current_node, int target_node, int graph_width) {
  int current_x = current_node % graph_width;
  int current_y = current_node / graph_width;
  int target_x = target_node % graph_width;
  int target_y = target_node / graph_width;

  return abs(current_x - target_x) + abs(current_y - target_y);
}

int EuclideanDistance(int current_node, int target_node, int graph_width) {
  int current_x = current_node % graph_width;
  int current_y = current_node / graph_width;
  int target_x = target_node % graph_width;
  int target_y = target_node / graph_width;

  return sqrt((current_x - target_x) * (current_x - target_x) +
              (current_y - target_y) * (current_y - target_y));
}

}  // namespace Algorithms
