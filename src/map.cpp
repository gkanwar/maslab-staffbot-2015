#include "map.h"

#include <cmath>


// Rasterize a line into our grid
// startGX, startGY, endGX, endGY all in grid units
void Map::fillLine(int startGX, int startGY, int endGX, int endGY, MapElement elt) {
  // Currently only supports lines in increments of 45 degrees
  if (startGX == endGX) {
    if (startGY < endGY) {
      for (int y = startGY; y <= endGY; ++y) {
        grid[startGX][y] = elt;
      }
    }
    else {
      for (int y = endGY; y <= startGY; ++y) {
        grid[startGX][y] = elt;
      }
    }
  }
  else if (startGY == endGY) {
    if (startGX < endGX) {
      for (int x = startGX; x <= endGX; ++x) {
        grid[x][startGY] = elt;
      }
    }
    else {
      for (int x = endGX; x <= startGX; ++x) {
        grid[x][startGY] = elt;
      }
    }
  }
  else if (abs(endGX-startGX) == abs(endGY-startGY)) { // 45-degree line
    int loopStartX = (startGX < endGX) ? startGX : endGX;
    int loopEndX = (startGX < endGX) ? endGX : startGX;
    int loopStartY = (startGY < endGY) ? startGY : endGY;
    int loopEndY = (startGY < endGY) ? endGY : startGY;
    int y = loopStartY;
    for (int x = loopStartX; x <= loopEndX; ++x) {
      assert(y <= loopEndY);
      grid[x][y] = elt;
      ++y;
    }
  }
  else {
    assert(false);
  }
}
