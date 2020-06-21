/*
  Raycasting terminal test.

  author: Miloslav Ciz
  license: CC0 1.0, public domain
*/

#define RCL_PIXEL_FUNCTION pixelFunc // set our pixel functio
#define RCL_COMPUTE_FLOOR_DEPTH 0    // turn off what we don't need
#define RCL_COMPUTE_CEILING_DEPTH 0

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "../raycastlib.h"

#define LEVEL_W 20
#define LEVEL_H 15

#define SCREEN_W 80
#define SCREEN_H 40

#define FRAME_OFFSET 20 // number of newlines printed before each frame

#define PIXELS_TOTAL (FRAME_OFFSET+ (SCREEN_W + 1) * SCREEN_H + 1)

char pixels[FRAME_OFFSET + (SCREEN_W + 1) * SCREEN_H + 1];
RCL_Camera camera;

const int8_t level[LEVEL_W * LEVEL_H] = // here 1 means wall, 0 floor
{
/*                      11  13  15  17  19 
  0 1 2 3 4 5 6 7 8 9 10  12  14  16  18  */
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0, // 0
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0, // 1
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0, // 2
  1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0, // 3
  0,0,0,0,0,0,0,0,0,1,0,0,1,1,1,0,0,1,0,0, // 4
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0, // 5
  1,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0, // 6
  0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0, // 7
  0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1, // 8
  0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0, // 9
  0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1, // 10
  0,0,0,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,1, // 11
  0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0, // 12
  0,0,0,0,0,1,0,0,0,1,1,1,0,0,1,0,0,0,0,0, // 13
  0,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0  // 14
};

/*
  Function that says the floor height at each square. We do it by reading the
  anove level array.
*/
RCL_Unit heightAt(int16_t x, int16_t y)
{
  int32_t index = y * LEVEL_W + x;

  if (index < 0 || (index >= LEVEL_W * LEVEL_H))
    return RCL_UNITS_PER_SQUARE * 2;

  return level[y * LEVEL_W + x] * RCL_UNITS_PER_SQUARE * 2;
}

static const char asciiShades[] = "HXi/;,.               ";

void pixelFunc(RCL_PixelInfo *p)
{
  char c = ' ';

  uint8_t shade = 3;

  shade -= RCL_min(3,p->depth / RCL_UNITS_PER_SQUARE);

  if (p->isWall)
  {
    switch (p->hit.direction)
    {
      case 0:  shade += 2;
      case 1:  shade += p->texCoords.y / 512;
               c = asciiShades[shade];
               break;
      case 2:  c = 'o'; break;
      case 3:
      default: c = '.'; break;
    }
  }

  pixels[FRAME_OFFSET + p->position.y * (SCREEN_W + 1) + p->position.x] = c;
}

void draw()
{
  memset(pixels,'\n',PIXELS_TOTAL);
  pixels[PIXELS_TOTAL - 1] = 0; // terminate string

  RCL_RayConstraints c;

  RCL_initRayConstraints(&c);

  c.maxHits = 1;
  c.maxSteps = 40;

  #if 1
    RCL_renderSimple(camera,heightAt,0,0,c);
  #else
    /* Here you can try using the complex rendering function. The result should
       be practically the same. */

    RCL_renderComplex(camera,heightAt,0,0,c);
  #endif

  puts(pixels);
}

int dx = 1;
int dy = 0;
int dr = 1;
int frame = 0;

int main()
{
  RCL_initCamera(&camera);
  camera.position.x = 2 * RCL_UNITS_PER_SQUARE;
  camera.position.y = 2 * RCL_UNITS_PER_SQUARE;
  camera.direction = 0;
  camera.resolution.x = SCREEN_W;
  camera.resolution.y = SCREEN_H;

  for (int i = 0; i < 10000; ++i)
  {
    draw();

    int squareX = RCL_divRoundDown(camera.position.x,RCL_UNITS_PER_SQUARE);
    int squareY = RCL_divRoundDown(camera.position.y,RCL_UNITS_PER_SQUARE);
 
    if (rand() % 100 == 0)
    {
      dx = 1 - rand() % 3;
      dy = 1 - rand() % 3;
      dr = 1 - rand() % 3;
    }

    while (heightAt(squareX + dx,squareY + dy) > 0)
    {
      dx = 1 - rand() % 3;
      dy = 1 - rand() % 3;
      dr = 1 - rand() % 3;
    }

    camera.position.x += dx * 200;
    camera.position.y += dy * 200;
    camera.direction += dr * 10;

    camera.height = RCL_UNITS_PER_SQUARE + RCL_sin(frame * 16) / 2;

    usleep(100000);

    frame++;
  }

  return 0;
}
