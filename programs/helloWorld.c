/*
  Simple hello world for raycastlib. Renders a single raycasted frame into
  terminal as ASCII.

  author: Miloslav Ciz
  license: CC0 1.0, public domain
*/

#define RCL_PIXEL_FUNCTION pixelFunc /* tell the library the name of our
                                        function with which we write pixels to
                                        the screen */
#define RCL_COMPUTE_FLOOR_DEPTH 0    /* turn off what we don't need, to help
                                        performance */
#define RCL_COMPUTE_CEILING_DEPTH 0

#include "../raycastlib.h"           // now include raycastlib itself

#include <stdio.h>                   // for IO

#define SCREEN_W 80
#define SCREEN_H 40

#define PIXELS_TOTAL ((SCREEN_W + 1) * SCREEN_H + 1)

char screen[(SCREEN_W + 1) * SCREEN_H + 1]; /* our ASCII screen, with extra
                                               space for newlines and
                                               terminating 0 */

/* Function that will tell the library height of square at each coordinates.
   We may implement it however we like, it may e.g. read the height out of a
   level array. Here we simply compute the height procedurally, without needing
   extra data. */
RCL_Unit heightAt(int16_t x, int16_t y)
{
  return (x < 0 || x > 10 || y < 0 || y > 10) ?
    2 * RCL_UNITS_PER_SQUARE : // two library units, imagine e.g. 2 meters     
    0;
}

/* This is the function we write pixels to our ASCII screen with. Above we have
   told the library it should call this function during rendering. */
void pixelFunc(RCL_PixelInfo *p)
{
  char c = ' ';

  if (p->isWall)
  {
    switch (p->hit.direction)
    {
      case 1: 
      case 2:  c = '#'; break;
      case 0:
      case 3:  c = '/'; break;
      default:          break;
    }
  }
  else // floor/ceiling
    c = p->isFloor ? '.' : ':';

  screen[p->position.y * (SCREEN_W + 1) + p->position.x] = c;
}

int main()
{
  for (int i = 0; i < PIXELS_TOTAL; ++i) // prefill screen with newlines
    screen[i] = '\n';

  screen[PIXELS_TOTAL - 1] = 0;          // terminate the string

  RCL_Camera camera;                     // camera to specify our view
  RCL_initCamera(&camera);

  // set up the camera:
  camera.position.x = 5 * RCL_UNITS_PER_SQUARE;
  camera.position.y = 6 * RCL_UNITS_PER_SQUARE;
  camera.direction = 5 * RCL_UNITS_PER_SQUARE / 6; // 4/5 of full angle
  camera.resolution.x = SCREEN_W;
  camera.resolution.y = SCREEN_H;

  RCL_RayConstraints constraints;        /* this struct tell the library more
                                            details about how it should cast
                                            rays */
  RCL_initRayConstraints(&constraints);
  constraints.maxHits = 1;               // we don't need more than 1 hit here
  constraints.maxSteps = 40;             // max squares a ray will travel



  /* This will start the rendering itself. The library will start calling our
     pixelFunc to render one frame. You can also try to use the complex
     rendering function, the result should be practically the same:

     RCL_renderComplex(camera,heightAt,0,0,constraints); */
  RCL_renderSimple(camera,heightAt,0,0,constraints);

  puts(screen); // print out the rendered frame

  return 0;     // done!
}
