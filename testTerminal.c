#include <stdio.h>
#include "raycastlib.h"
#include <unistd.h>

#define LEVEL_W 20
#define LEVEL_H 15

#define SCREEN_W 60
#define SCREEN_H 20

char pixels[SCREEN_W * SCREEN_H];
Camera camera;

int8_t level[LEVEL_W * LEVEL_H] =
{
/*                      11  13  15  17  19 
  0 1 2 3 4 5 6 7 8 9 10  12  14  16  18  */
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, // 0
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, // 1
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, // 2
  0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, // 3
  0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0, // 4
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 5
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 6
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 7
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 8
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 9
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 10
  0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0, // 11
  0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0, // 12
  0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0, // 13
  0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0  // 14
};

Unit heightAt(int16_t x, int16_t y)
{
  if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H)
    return UNITS_PER_SQUARE * 2;

return 0;
//  return level[y * LEVEL_W + x] * UNITS_PER_SQUARE * 2;
}

void pixelFunc(PixelInfo *p)
{
  char c = '.';

  if (p->isWall)
  {
    switch (p->hit.direction)
    {
      case 0:  c = 'X'; break;
      case 1:  c = '#'; break;
      case 2:  c = 'p'; break;
      case 3:
      default: c = 'o'; break;
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

  renderSimple(camera,heightAt,0,pixelFunc,0,c);

  for (int j = 0; j < SCREEN_H; ++j)
  {
    for (int i = 0; i < SCREEN_W; ++i)
      printf("%c",pixels[j * SCREEN_W + i]);   

    printf("\n");
  }
}

int main()
{
  initCamera(&camera);
  camera.position.x = 2 * UNITS_PER_SQUARE;
  camera.position.y = 2 * UNITS_PER_SQUARE;
  camera.direction = -UNITS_PER_SQUARE / 3; //0;//(3 * UNITS_PER_SQUARE) / 4;
  camera.resolution.x = SCREEN_W;
  camera.resolution.y = SCREEN_H;

  for (int i = 0; i < 100; ++i)
  {
    draw();
    camera.position.x += 100;
    usleep(100000);
  }

  return 0;
}
