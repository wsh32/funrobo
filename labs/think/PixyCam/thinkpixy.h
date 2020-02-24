#include <SPI.h>
#include <Pixy.h>

struct PixyCamData {
  boolean isDetected;
  float x, y, w, h, a, theta;
};
