#include "render.h"

#include <chrono>
#include <cstdlib>
#include <GL/glut.h>
#include <thread>
#include <vector>

#include "error.h"

namespace {

struct RenderRect {
  int x, y, w, h;
  double r, g, b;
};

bool init = false;
bool frameDirty = false;
vector<RenderRect> rects;
pthread_t glutThread;

void* glutThreadFunc(void* args) {
  glutMainLoop();
  return NULL;
}

void render() {
  if (frameDirty) {
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    for (RenderRect r : rects) {
      glColor3f(r.r, r.g, r.b);
      glRectf(2*r.x/(double)RENDER_WIDTH - 1.0,
              2*r.y/(double)RENDER_HEIGHT - 1.0,
              2*(r.x+r.w)/(double)RENDER_WIDTH - 1.0,
              2*(r.y+r.h)/(double)RENDER_HEIGHT - 1.0);
    }

    rects.clear();

    glutSwapBuffers();
    frameDirty = false;
  }

  glutPostRedisplay();
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

    pthread_create(&glutThread, NULL, glutThreadFunc, NULL);
  }
}

}  // anonymous namespace

void drawFrame() {
  frameDirty = true;
  while (frameDirty) {
    this_thread::sleep_for(chrono::milliseconds(10));
  }
}

void drawRect(int x, int y, int w, int h, double r, double g, double b) {
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
}

void joinRenderThread() {
  pthread_join(glutThread, NULL);
}
