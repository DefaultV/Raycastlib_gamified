#include "raycastlib.h"
#include <stdio.h>

void logVector2D(Vector2D v)
{
  printf("[%d,%d]\n",v.x,v.y);
}

void logRay(Ray r)
{
  printf("ray:\n");
  printf("  start: ");
  logVector2D(r.start);
  printf("  dir: ");
  logVector2D(r.direction);
}

void logHitResult(HitResult h)
{
  printf("hit:\n");\
  printf("  sqaure: ");
  logVector2D(h.square);
  printf("  pos: ");
  logVector2D(h.position);
  printf("  dist: %d", h.distance);
  printf("  texcoord: %d", h.textureCoord);
}

int16_t testArrayFunc(int16_t x, int16_t y)
{
  return (x < 0 || y < 0 || x > 9 || y > 9) ? 1 : 0;
}

/**
  Simple automatic test function.
*/

int testSingleRay(Unit startX, Unit startY, Unit dirX, Unit dirY,
  int16_t expectSquareX, int16_t expectSquareY, int16_t expectPointX,
  int16_t expectPointY, int16_t tolerateError)
{
  Ray r;

  r.start.x     = startX;
  r.start.y     = startY;
  r.direction.x = dirX;
  r.direction.y = dirY;

  printf("- casting ray:\n");
  logRay(r);

  HitResult h = castRay(r,testArrayFunc,20);
  
  printf("- result:\n");
  logHitResult(h);

  int result = 
    h.square.x   == expectSquareX &&
    h.square.y   == expectSquareY &&
    h.position.x <= expectPointX + tolerateError &&
    h.position.x >= expectPointX - tolerateError &&
    h.position.y <= expectPointY + tolerateError &&
    h.position.y >= expectPointY - tolerateError;

  if (result)
    printf("\nOK\n");
  else
    printf("\nFAIL\n");

  return result;
}

int main()
{
  printf("Testing raycastlib.\n"); 

  if (!testSingleRay(
    3 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2,
    4 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2,
    100, 50,
    10, 7,
    10240, 7936,
    16))
    return 1; 

  return 0;
}
