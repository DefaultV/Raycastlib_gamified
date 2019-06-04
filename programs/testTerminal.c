/*
  Raycasting terminal test.

  author: Miloslav Ciz
  license: CC0
*/

#define RCL_PIXEL_FUNCTION pixelFunc
#define RCL_COMPUTE_WALL_TEXCOORDS 0
#define RCL_COMPUTE_FLOOR_DEPTH 0
#define RCL_COMPUTE_CEILING_DEPTH 0

#include <stdio.h>
#include "../raycastlib.h"
#include <unistd.h>
#include <stdlib.h>

#define LEVEL_W 20
#define LEVEL_H 15

#define SCREEN_W 80
#define SCREEN_H 40

char pixels[SCREEN_W * SCREEN_H];
RCL_Camera camera;

const int8_t level[LEVEL_W * LEVEL_H] =
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

RCL_Unit heightAt(int16_t x, int16_t y)
{
  int32_t index = y * LEVEL_W + x;

  if (index < 0 || (index >= LEVEL_W * LEVEL_H))
    return RCL_UNITS_PER_SQUARE * 2;

  return level[y * LEVEL_W + x] * RCL_UNITS_PER_SQUARE * 2;
}

void pixelFunc(RCL_PixelInfo *p)
{
  char c = ' ';

  if (p->isWall)
  {
    switch (p->hit.direction)
    {
      case 0:  c = 'X'; break;
      case 1:  c = '#'; break;
      case 2:  c = 'o'; break;
      case 3:
      default: c = '.'; break;
    }
  }

  pixels[p->position.y * SCREEN_W + p->position.x] = c;
}

void draw()
{
  for (int i = 0; i < 15; ++i)
    printf("\n");

  RCL_RayConstraints c;

  RCL_initRayConstraints(&c);

  c.maxHits = 1;
  c.maxSteps = 40;

  //RCL_renderSimple(camera,heightAt,0,0,c);
  RCL_renderComplex(camera,heightAt,0,0,c);

  for (int j = 0; j < SCREEN_H; ++j)
  {
    for (int i = 0; i < SCREEN_W; ++i)
      printf("%c",pixels[j * SCREEN_W + i]);   

    printf("\n");
  }
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

    camera.height = RCL_UNITS_PER_SQUARE + RCL_sinInt(frame * 16) / 2;

    usleep(100000);

    frame++;
  }

  return 0;
}
