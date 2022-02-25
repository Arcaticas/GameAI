// Copyright 2022 Theo Grant
#include "GraphMaker.h"

GraphMaker::GraphMaker() {}

GraphMaker::GraphMaker(int i_width) {
  total_nodes = i_width * i_width;

  data = new unsigned char*[total_nodes];

  for (int i = 0; i < total_nodes; i++) {
    data[i] = new unsigned char[total_nodes];

    for (int j = 0; j < total_nodes; j++) {
      if (i + 1 == j && i % i_width != i_width - 1) {
        data[i][j] = 2;
      } else if (i - 1 == j && i % i_width != 0) {
        data[i][j] = 2;
      } else if (i + i_width == j) {
        data[i][j] = 2;
      } else if (i - i_width == j) {
        data[i][j] = 2;
      } else {
        data[i][j] = 0;
      }
    }
  }
}

void GraphMaker::CreateWall(int corner_node, int wall_width, int graph_width) {
  for (int i = corner_node + 1; i < corner_node + wall_width - 1; i++) {
    data[i][i + graph_width] = 0;
    data[i + graph_width][i] = 0;

    data[i + ((wall_width - 1) * graph_width)]
        [i + ((wall_width - 1) * graph_width) - graph_width] = 0;

    data[i + ((wall_width - 1) * graph_width) - graph_width]
        [i + ((wall_width - 1) * graph_width)] = 0;
  }
  for (int i = corner_node + graph_width;
       i < corner_node + (wall_width - 1) * graph_width; i += graph_width) {
    data[i][i + 1] = 0;
    data[i + 1][i] = 0;

    data[i + wall_width - 1][(i + wall_width - 1) - 1] = 0;

    data[(i + wall_width - 1) - 1][i + wall_width - 1] = 0;
  }
}

int* GraphMaker::NodeToWorld(int node, int graph_width, int screen_resolution) {
  int scale = screen_resolution / graph_width;
  int x_grid = node % graph_width;
  int y_grid = node / graph_width;
  int* output = new int[2];
  output[0] = x_grid * scale;
  output[1] = y_grid * scale;

  return output;
}

int GraphMaker::WorldToNode(int i_x, int i_y, int graph_width,
                            int screen_resolution) {
  int scale = screen_resolution / graph_width;
  int x_grid = i_x / scale;
  int y_grid = i_y / scale;

  return (y_grid * graph_width) + x_grid;
}
