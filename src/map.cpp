#include "map.h"

#include <cmath>

#include "render.h"


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
      rassert(y <= loopEndY);
      grid[x][y] = elt;
      ++y;
    }
  }
  else {
    rassert(false) << "Invalid line angle";
  }
}

void Map::renderMap() {
  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      if (grid[i][j] == WALL) {
        drawRect(i*TILE_SIZE, j*TILE_SIZE, TILE_SIZE, TILE_SIZE, 0.5, 0.5, 0.5);
      }
      else if (grid[i][j] == PLATFORM) {
        drawRect(i*TILE_SIZE, j*TILE_SIZE, TILE_SIZE, TILE_SIZE, 1.0, 1.0, 0.0);
      }
    }
  }
}
