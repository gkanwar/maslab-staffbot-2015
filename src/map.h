#ifndef MAP_H
#define MAP_H

#include <fstream>
#include <vector>

#include "error.h"
#include "robot.h"
#include "util.h"

#define TILE_SIZE 0.1
#define GRID_SIZE 101
#define METERS_PER_UNIT 0.6096

using namespace std;


struct Wall {
  // In meters
  double startX, startY, endX, endY;
  Wall(double startX, double startY, double endX, double endY)
      : startX(startX), startY(startY), endX(endX), endY(endY) {}
};

struct Stack {
  enum Color { RED, GREEN };
  double x, y;
  Color blocks[3];
  Stack(double x, double y, Color bottom, Color middle, Color top) :
      x(x), y(y) {
    blocks[0] = bottom;
    blocks[1] = middle;
    blocks[2] = top;
  }
};

struct Point {
  double x, y;
  Point(double x, double y) : x(x), y(y) {}
};

struct HomeBase {
  vector<Point> points;
  HomeBase() : points({}) {}
  HomeBase(vector<Point> points) : points(points) {}
};

class Map {
 public:
  enum MapElement { NONE, WALL, PLATFORM };

  Map(vector<Wall> walls, vector<Wall> platforms, RobotPose initPose) : initPose(initPose) {
    initGrid(walls, platforms);
  }

  Map(string filename) : initPose(0, 0, 0) {
    ifstream file;
    file.open(filename);
    vector<Wall> walls;
    vector<Wall> platforms;
    while (!file.eof()) {
      char line[1024];
      file.getline(line, 1024);
      if (file.fail()) continue;
      string str(line);
      istringstream iss(str);
      vector<string> tokens;
      string token;
      while(getline(iss, token, ',')) {
        tokens.push_back(token);
      }
      rassert(tokens.size() > 0);
      if (tokens[0] == "L") {
        rassert(tokens.size() == 3);
        initPose = RobotPose(unitsToMeters(stoi(tokens[1])),
                             unitsToMeters(stoi(tokens[2])), 0);
      }
      else if (tokens[0] == "W") {
        rassert(tokens.size() == 5);
        walls.emplace_back(unitsToMeters(stoi(tokens[1])),
                           unitsToMeters(stoi(tokens[2])),
                           unitsToMeters(stoi(tokens[3])),
                           unitsToMeters(stoi(tokens[4])));
      }
      else if (tokens[0] == "S") {
        rassert(tokens.size() == 6);
        stacks.emplace_back(unitsToMeters(stoi(tokens[1])),
                            unitsToMeters(stoi(tokens[2])),
                            (tokens[3] == "R") ? Stack::RED : Stack::GREEN,
                            (tokens[4] == "R") ? Stack::RED : Stack::GREEN,
                            (tokens[5] == "R") ? Stack::RED : Stack::GREEN);
      }
      else if (tokens[0] == "P") {
        rassert(tokens.size() == 5);
        platforms.emplace_back(unitsToMeters(stoi(tokens[1])),
                               unitsToMeters(stoi(tokens[2])),
                               unitsToMeters(stoi(tokens[3])),
                               unitsToMeters(stoi(tokens[4])));
      }
      else if (tokens[0] == "H") {
        int numPts = stoi(tokens[1]);
        rassert(tokens.size() == 2*numPts + 2);
        vector<Point> hbPoints;
        for (int i = 0; i < numPts; ++i) {
          hbPoints.emplace_back(unitsToMeters(stoi(tokens[2+2*i])),
                                unitsToMeters(stoi(tokens[3+2*i])));
        }
        homeBase = HomeBase(hbPoints);
      }
      else {
        rassert(false) << "Invalid map token: " << tokens[0];
      }
    }

    initGrid(walls, platforms);
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

  RobotPose getInitPose() const {
    return initPose;
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
  // Import map helper: convert map units to meters
  double unitsToMeters(double units) {
    return units * METERS_PER_UNIT;
  }

  void initGrid(vector<Wall> walls, vector<Wall> platforms);

  void fillLine(int startX, int startY, int endX, int endY, MapElement elt);

  inline void checkPoint(double x, double y) const {
    rassert(isValidPoint(x, y))
        << "Point (" << x << "," << y << ") is invalid coords ("
        << toGridCoord(x) << "," << toGridCoord(y) << ")";
  }

  Vector computeClosestWall(int gridX, int gridY);
  void buildClosestMap();

  RobotPose initPose;
  HomeBase homeBase;
  vector<Stack> stacks;

  // Grid of 501x501 tiles of size 0.1m x 0.1m, i.e. a 50mx50m grid
  // Index (i,j) has lower left (i*0.1m, j*0.1m)
  MapElement grid[GRID_SIZE][GRID_SIZE];

  Vector closestMap[GRID_SIZE][GRID_SIZE];
};

#endif
