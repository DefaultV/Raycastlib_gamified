/**
  Tests for raycastlib.

  license: CC0
*/

#include <stdio.h>
#include "raycastlib.h"

int16_t testArrayFunc(int16_t x, int16_t y)
{
  if (x > 12 || y > 12)
    return x * y;

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
    printf("\nOK\n\n");
  else
    printf("\nFAIL\n\n");

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

  if (!testSingleRay(
    0,
    0,
    100, 100,
    10, 9,
    10240, 10239,
    16))
    return 1; 

  for (Unit i = -UNITS_PER_SQUARE; i <= UNITS_PER_SQUARE; i += 64)
  {
    Unit v = sinInt(i);

    logVector2D(angleToDirection(i));

    //for (int j = 0; j < (v + UNITS_PER_SQUARE) / 64; ++j)
    //  printf(".");

    //printf("\n");
  }

  return 0;
}
