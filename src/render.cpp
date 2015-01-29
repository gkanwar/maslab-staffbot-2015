#include "render.h"

#ifndef EDISON

#include <chrono>
#include <cstdlib>
#include <GL/glut.h>
#include <thread>
#include <vector>

#include "error.h"
#include "map.h"

namespace {

struct RenderRect {
  double x, y, w, h;
  double r, g, b;
};

bool init = false;
bool frameDirty = false;
bool waitKey = true;
vector<RenderRect> rects;
pthread_t glutThread;

double boundsLeft = 0.0;
double boundsRight = GRID_SIZE*TILE_SIZE;
double boundsBottom = 0.0;
double boundsTop = GRID_SIZE*TILE_SIZE;

void* glutThreadFunc(void* args) {
  glutMainLoop();
  return NULL;
}

void render() {
  if (frameDirty) {
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(boundsLeft, boundsRight, boundsBottom, boundsTop, 1, -1);

    for (RenderRect r : rects) {
      glColor3f(r.r, r.g, r.b);
      glRectf(r.x, r.y, r.x+r.w, r.y+r.h);
    }

    rects.clear();

    glutSwapBuffers();
    frameDirty = false;
  }

  glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 'a': {
      double delta = (boundsRight - boundsLeft)*0.1;
      boundsLeft -= delta;
      boundsRight -= delta;
      break;
    }
    case 'd': {
      double delta = (boundsRight - boundsLeft)*0.1;
      boundsLeft += delta;
      boundsRight += delta;
      break;
    }
    case 'w': {
      double delta = (boundsTop - boundsBottom)*0.1;
      boundsTop += delta;
      boundsBottom += delta;
      break;
    }
    case 's': {
      double delta = (boundsTop - boundsBottom)*0.1;
      boundsTop -= delta;
      boundsBottom -= delta;
      break;
    }
    case 'q': {
      double deltaWidth = (boundsRight - boundsLeft)*0.1;
      double deltaHeight = (boundsTop - boundsBottom)*0.1;
      boundsTop += deltaHeight;
      boundsBottom -= deltaHeight;
      boundsLeft -= deltaWidth;
      boundsRight += deltaWidth;
      break;
    }
    case 'e': {
      double deltaWidth = (boundsRight - boundsLeft)*0.1;
      double deltaHeight = (boundsTop - boundsBottom)*0.1;
      boundsTop -= deltaHeight;
      boundsBottom += deltaHeight;
      boundsLeft += deltaWidth;
      boundsRight -= deltaWidth;
      break;
    }
    case 'r': {
      boundsLeft = 0.0;
      boundsRight = GRID_SIZE*TILE_SIZE;
      boundsBottom = 0.0;
      boundsTop = GRID_SIZE*TILE_SIZE;
      break;
    }
    case 'c': {
      waitKey = false;
      break;
    }
  }
}

// Should be called at the start of every render func
void initRender() {
  if (!init) {
    init = true;

    char* argv[3] = {"robot", "-display", ":0"};
    int argc = 3;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutCreateWindow("robot");
    glutReshapeWindow(RENDER_WIDTH, RENDER_HEIGHT);
    glutDisplayFunc(render);
    glutKeyboardFunc(keyboard);

    pthread_create(&glutThread, NULL, glutThreadFunc, NULL);
  }
}

}  // anonymous namespace
#endif

void drawFrame(bool waitForKey) {
#ifndef EDISON
  frameDirty = true;
  waitKey = waitForKey;
  while (frameDirty || waitKey) {
    this_thread::sleep_for(chrono::milliseconds(10));
  }
#endif
}

void drawRect(double x, double y, double w, double h, double r, double g, double b) {
#ifndef EDISON
  initRender();

  RenderRect rect;
  rect.x = x;
  rect.y = y;
  rect.w = w;
  rect.h = h;
  rect.r = r;
  rect.g = g;
  rect.b = b;
  rects.push_back(rect);
#endif
}

void joinRenderThread() {
#ifndef EDISON
  pthread_join(glutThread, NULL);
#endif
}

