#ifndef RENDER_H
#define RENDER_H

// Format of color int
#define R_MASK 0xff000000
#define G_MASK 0x00ff0000
#define B_MASK 0x0000ff00
#define A_MASK 0x000000ff

#define RENDER_WIDTH 640
#define RENDER_HEIGHT 480

void drawRect(int x, int y, int w, int h, double r, double g, double b);
void joinRenderThread();

#endif
