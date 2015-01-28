#ifndef MAP_H
#define MAP_H

#include <vector>

#include "error.h"
#include "util.h"

#define TILE_SIZE 0.1
#define GRID_SIZE 101

using namespace std;


struct Wall {
  // In meters
  double startX, startY, endX, endY;
  Wall(double startX, double startY, double endX, double endY)
      : startX(startX), startY(startY), endX(endX), endY(endY) {}
};

class Map {
 public:
  enum MapElement { NONE, WALL, PLATFORM };

  Map(vector<Wall> walls, vector<Wall> platforms) {
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

  MapElement getMapElement(double x, double y) const {
    checkPoint(x, y);
    int xCoord = toGridCoord(x);
    int yCoord = toGridCoord(y);
    return grid[xCoord][yCoord];
  }

  void renderMap();

  bool isValidPoint(double x, double y) const {
    return (x >= 0 &&
            y >= 0 &&
            x < GRID_SIZE*TILE_SIZE &&
            y < GRID_SIZE*TILE_SIZE);
  }

  Vector clampPoint(Vector pt) const {
    double x = pt.x;
    double y = pt.y;
    if (x < 0) x = 0;
    else if (x >= GRID_SIZE*TILE_SIZE) x = (GRID_SIZE-1)*TILE_SIZE;
    if (y < 0) y = 0;
    else if (y >= GRID_SIZE*TILE_SIZE) y = (GRID_SIZE-1)*TILE_SIZE;
    return Vector(x, y);
  }

  Vector getClosest(double x, double y) const {
    return closestMap[toGridCoord(x)][toGridCoord(y)];
  }

  int toGridCoord(double x) const {
    int gc = (int)(x/TILE_SIZE);
    rassert(gc >= 0 && gc < GRID_SIZE);
    return gc;
  }
  
  bool isObstacle(int gridX, int gridY) const {
    rassert(gridX >= 0 && gridX < GRID_SIZE &&
            gridY >= 0 && gridY < GRID_SIZE)
        << "Invalid grid point: " << gridX << "," << gridY;
    return grid[gridX][gridY] == WALL ||
        grid[gridX][gridY] == PLATFORM;
  }

 private:
  void fillLine(int startX, int startY, int endX, int endY, MapElement elt);

  inline void checkPoint(double x, double y) const {
    rassert(isValidPoint(x, y))
        << "Point (" << x << "," << y << ") is invalid coords ("
        << toGridCoord(x) << "," << toGridCoord(y) << ")";
  }

  Vector computeClosestWall(int gridX, int gridY);
  void buildClosestMap();

  // Grid of 501x501 tiles of size 0.1m x 0.1m, i.e. a 50mx50m grid
  // Index (i,j) has lower left (i*0.1m, j*0.1m)
  MapElement grid[GRID_SIZE][GRID_SIZE];

  Vector closestMap[GRID_SIZE][GRID_SIZE];
};

#endif
