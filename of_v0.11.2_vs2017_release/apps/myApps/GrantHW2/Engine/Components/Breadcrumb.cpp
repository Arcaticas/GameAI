// Copyright 2022 Theo Grant
#include "Breadcrumb.h"
#include "graphics/ofGraphics.h"
#include "utils/ofUtils.h"

Breadcrumb::Breadcrumb(float i_x, float i_y, ofColor i_color) {
  posX = i_x;
  posY = i_y;
  m_color = i_color;
  startTime = ofGetElapsedTimef();
}

void Breadcrumb::DrawBreadcrumb() {
  ofSetColor(m_color);
  ofDrawCircle(posX, posY, 5);
}

bool Breadcrumb::CheckLife() {
  if (ofGetElapsedTimef() - startTime > 30) {
    return true;
  } else {
    return false;
  }
}
