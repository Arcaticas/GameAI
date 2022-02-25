// Copyright 2022 Theo Grant
#ifndef GRANTHW1_ENGINE_COMPONENTS_BREADCRUMB_H_
#define GRANTHW1_ENGINE_COMPONENTS_BREADCRUMB_H_
#include <ofColor.h>

class Breadcrumb {
 public:
  float startTime = 0.0;
  float posX;
  float posY;

  ofColor m_color;

  Breadcrumb(float i_x, float i_y, ofColor i_color);
  void DrawBreadcrumb();
  bool CheckLife();
};
#endif  // GRANTHW1_ENGINE_COMPONENTS_BREADCRUMB_H_
