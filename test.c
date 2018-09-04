/**
  Tests for raycastlib.

  license: CC0
*/

#define RAYCASTLIB_PROFILE

#include <stdio.h>
#include "raycastlib.h"
#include <sys/time.h>

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

  HitResult h = castRay(r,testArrayFunc);
  
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

int testSingleMapping(Unit posX, Unit posY, Unit posZ, uint32_t resX,
  uint32_t resY, Unit camX, Unit camY, Unit camZ, Unit camDir, Unit fov,
  Unit expectX, Unit expectY, Unit expectZ)
{
  int result;

  Camera c;

  c.resolution.x = resX;
  c.resolution.y = resY;
  c.position.x = camY;
  c.position.y = camY;
  c.direction = camDir;
  c.height = posZ;
  c.fovAngle = fov;

  Vector2D pos;
  Unit height;

  pos.x = posX;
  pos.y = posY;
  height = posZ;
 
  PixelInfo p;

  printf("- mapping pixel: %d %d %d\n",posX,posY,posZ);

  p = mapToScreen(pos,height,c);

  printf("- result:\n");
  logPixelInfo(p);

  result = p.position.x == expectX && p.position.y == expectY &&
    p.depth == expectZ;

  if (result)
    printf("\nOK\n\n");
  else
    printf("\nFAIL\n\n");

  return result;
}

// returns milliseconds
long measureTime(void (*func)(void))
{
  long start, end;
  struct timeval timecheck;

  gettimeofday(&timecheck, NULL);
  start = (long) timecheck.tv_sec * 1000 + (long) timecheck.tv_usec / 1000;

  func();

  gettimeofday(&timecheck, NULL);
  end = (long) timecheck.tv_sec * 1000 + (long) timecheck.tv_usec / 1000;

  return end - start;
}

void benchCastRays()
{
  Ray r;

  r.start.x = UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2;
  r.start.y = 2 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 4;

  Vector2D directions[8];

  for (int i = 0; i < 8; ++i)
    directions[i] = angleToDirection(UNITS_PER_SQUARE / 8 * i);

  for (int i = 0; i < 1000000; ++i)
  {
    r.direction = directions[i % 8];
    castRay(r,testArrayFunc);
  }
}

void benchmarkMapping()
{
  Camera c;

  c.resolution.x = 1024;
  c.resolution.y = 768;
  c.position.x = UNITS_PER_SQUARE / 2;
  c.position.y = UNITS_PER_SQUARE * 2;
  c.direction = UNITS_PER_SQUARE / 8;
  c.height = 0;
  c.fovAngle = UNITS_PER_SQUARE / 2;

  PixelInfo p;

  Vector2D pos;
  Unit height;

  pos.x = -1024 * UNITS_PER_SQUARE;
  pos.y = -512 * UNITS_PER_SQUARE;
  height = 0;
 
  for (int i = 0; i < 1000000; ++i)
  {
    p = mapToScreen(pos,height,c);

    pos.x += 4;
    pos.y += 8;
    height = (height + 16) % 1024;
  }
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
    10240, 10240,
    16))
    return 1; 

  if (!testSingleRay(
    400,
    6811,
    -629,805,
    -1, 7,
    -1, 7325,
    16))
    return 1;

  if (!testSingleRay(
    -4 * UNITS_PER_SQUARE - UNITS_PER_SQUARE / 2,
    7 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 3,
    100,-100,
    0, 2,
    1, 2900,
    16))
    return 1;

  if (!testSingleMapping(
    -UNITS_PER_SQUARE,
    0,
    UNITS_PER_SQUARE / 2,
    1280,
    640,
    0,
    0,
    0,
    UNITS_PER_SQUARE / 2,
    UNITS_PER_SQUARE / 4,
    1280, // shouldn't be half?
    320,
    1024
    ))
    return -1;

  for (Unit i = -UNITS_PER_SQUARE; i <= UNITS_PER_SQUARE; i += 64)
  {
    Unit v = sinInt(i);

    logVector2D(angleToDirection(i));

    //for (int j = 0; j < (v + UNITS_PER_SQUARE) / 64; ++j)
    //  printf(".");

    //printf("\n");
  }

  printf("benchmark:\n");

  long t;
  t = measureTime(benchCastRays);
  printf("cast 1000000 rays: %ld ms\n",t);

  t = measureTime(benchmarkMapping);
  printf("map point to screen 1000000 times: %ld ms\n",t);
 
  printf("\n"); 
  printProfile();

  return 0;
}
