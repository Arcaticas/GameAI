// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_COMPONENTS_GRAPHMAKER_H_
#define GRANTHW1_ENGINE_COMPONENTS_GRAPHMAKER_H_

class GraphMaker {
 public:
  int total_nodes;
  unsigned char** data;

  GraphMaker();
  explicit GraphMaker(int i_width);
  void CreateWall(int corner_node, int wall_width, int graph_width);

  static int* NodeToWorld(int node, int graph_width, int screen_resolution);
  static int WorldToNode(int i_x, int i_y, int graph_width,
                         int screen_resolution);
};
#endif  // GRANTHW1_ENGINE_COMPONENTS_GRAPHMAKER_H_
