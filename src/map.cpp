#include "map.h"

#include <cmath>

#include "render.h"

void Map::initGrid(vector<Wall> walls, vector<Wall> platforms) {
  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      grid[i][j] = NONE;
    }
  }

  for (Wall w : walls) {
    checkPoint(w.startX, w.startY);
    checkPoint(w.endX, w.endY);
    fillLine(toGridCoord(w.startX), toGridCoord(w.startY),
             toGridCoord(w.endX), toGridCoord(w.endY), WALL);
  }

  for (Wall p : platforms) {
    checkPoint(p.startX, p.startY);
    checkPoint(p.endX, p.endY);
    // TODO: This assumes platform extends for the entire wall segment
    fillLine(toGridCoord(p.startX), toGridCoord(p.startY),
             toGridCoord(p.endX), toGridCoord(p.endY), PLATFORM);
  }

  buildClosestMap();
}

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
    int y = (startGX < endGX) ? startGY : endGY;
    int yInc = (startGX < endGX && startGY < endGY ||
                startGX > endGX && startGY > endGY) ? 1 : -1;
    for (int x = loopStartX; x <= loopEndX; ++x) {
      grid[x][y] = elt;
      y += yInc;
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

Vector Map::computeClosestWall(int gridX, int gridY) {
  rassert(gridX >= 0 && gridX < GRID_SIZE &&
          gridY >= 0 && gridY < GRID_SIZE);
  double closestErr = -1;
  Vector closest;
  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      if (grid[i][j] == Map::WALL ||
          grid[i][j] == Map::PLATFORM) {
        double err = distSq(gridX, gridY, i, j);
        if (closestErr < 0 || err < closestErr) {
          closestErr = err;
          closest = Vector(i*TILE_SIZE, j*TILE_SIZE);
        }
      }
      // if (closestErr > 0 && (j-gridY)*TILE_SIZE > closestErr) break;
    }
    // if (closestErr > 0 && (i-gridX)*TILE_SIZE > closestErr) break;
  }
  rassert(closestErr >= 0);
  return closest;
}

void Map::buildClosestMap() {
  for (int i = 0; i < GRID_SIZE; ++i) {
    cout << i << endl;
    for (int j = 0; j < GRID_SIZE; ++j) {
      closestMap[i][j] = computeClosestWall(i, j);
      // Sanity check
      checkPoint(closestMap[i][j].x, closestMap[i][j].y);
    }
  }
}
