#ifndef MAP_H
#define MAP_H

#include <cassert>
#include <vector>

#define TILE_SIZE 0.1
#define GRID_SIZE 500

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
    for (int i = 0; i <= GRID_SIZE; ++i) {
      for (int j = 0; j <= GRID_SIZE; ++j) {
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
  }

  MapElement getMapElement(double x, double y) {
    checkPoint(x, y);
    int xCoord = toGridCoord(x);
    int yCoord = toGridCoord(y);
    return grid[xCoord][yCoord];
  }

 private:
  void fillLine(int startX, int startY, int endX, int endY, MapElement elt);

  inline int toGridCoord(double x) {
    return (int)(x/TILE_SIZE + 0.5);
  }

  inline void checkPoint(double x, double y) {
    assert(x >= 0 &&
           y >= 0 &&
           x <= GRID_SIZE*TILE_SIZE &&
           y <= GRID_SIZE*TILE_SIZE);
  }

  // Grid of 500x500 tiles of size 0.1m x 0.1m
  // Index (i,j) is centered at (i*0.1m, j*0.1m)
  MapElement grid[GRID_SIZE+1][GRID_SIZE+1];
};

#endif
