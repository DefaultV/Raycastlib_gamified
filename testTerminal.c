#include <stdio.h>
#include "raycastlib.h"
#include <unistd.h>
#include <stdlib.h>

#define LEVEL_W 20
#define LEVEL_H 15

#define SCREEN_W 80
#define SCREEN_H 40

char pixels[SCREEN_W * SCREEN_H];
Camera camera;

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

Unit heightAt(int16_t x, int16_t y)
{
  int32_t index = y * LEVEL_W + x;

  if (index < 0 || (index >= LEVEL_W * LEVEL_H))
    return UNITS_PER_SQUARE * 2;

  return level[y * LEVEL_W + x] * UNITS_PER_SQUARE * 2;
}

void pixelFunc(PixelInfo *p)
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
  for (int i = 0; i < 10; ++i)
  printf("\n");

  RayConstraints c;

  initRayConstraints(&c);

  c.maxHits = 1;
  c.maxSteps = 40;
  c.computeTextureCoords = 0;


  //renderSimple(camera,heightAt,0,pixelFunc,0,c);
  render(camera,heightAt,0,0,pixelFunc,c);

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
  initCamera(&camera);
  camera.position.x = 2 * UNITS_PER_SQUARE;
  camera.position.y = 2 * UNITS_PER_SQUARE;
  camera.direction = 0;
  camera.resolution.x = SCREEN_W;
  camera.resolution.y = SCREEN_H;

  for (int i = 0; i < 10000; ++i)
  {
    draw();

    int squareX = divRoundDown(camera.position.x,UNITS_PER_SQUARE);
    int squareY = divRoundDown(camera.position.y,UNITS_PER_SQUARE);
 
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

    camera.height = UNITS_PER_SQUARE + sinInt(frame * 16) / 2;

    usleep(100000);

    frame++;
  }

  return 0;
}
