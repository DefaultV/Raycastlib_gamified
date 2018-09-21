/**
  Tests for raycastlib.

  license: CC0
*/

#define RCL_PROFILE

#define RCL_HORIZONTAL_FOV RCL_UNITS_PER_SQUARE / 2

#define RCL_PIXEL_FUNCTION pixelFunc

#include <stdio.h>
#include "raycastlib.h"
#include <sys/time.h>

RCL_Unit testArrayFunc(int16_t x, int16_t y)
{
  if (x > 12 || y > 12)
    return x * y;

  return (x < 0 || y < 0 || x > 9 || y > 9) ? 1 : 0;
}

RCL_Unit testArrayFunc2(int16_t x, int16_t y)
{
  return testArrayFunc(x,y) + 10 * RCL_UNITS_PER_SQUARE;
}

/**
  Simple automatic test function.
*/

int testSingleRay(RCL_Unit startX, RCL_Unit startY, RCL_Unit dirX, RCL_Unit dirY,
  int16_t expectSquareX, int16_t expectSquareY, int16_t expectPointX,
  int16_t expectPointY, int16_t tolerateError)
{
  RCL_Ray r;

  r.start.x     = startX;
  r.start.y     = startY;
  r.direction.x = dirX;
  r.direction.y = dirY;

  printf("- casting ray:\n");
  RCL_logRay(r);

  RCL_HitResult h = RCL_castRay(r,testArrayFunc);
  
  printf("- result:\n");
  RCL_logHitResult(h);

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

int testSingleMapping(RCL_Unit posX, RCL_Unit posY, RCL_Unit posZ, uint32_t resX,
  uint32_t resY, RCL_Unit camX, RCL_Unit camY, RCL_Unit camZ, RCL_Unit camDir,
  RCL_Unit expectX, RCL_Unit expectY, RCL_Unit expectZ)
{
  int result;

  RCL_Camera c;

  c.resolution.x = resX;
  c.resolution.y = resY;
  c.position.x = camX;
  c.position.y = camY;
  c.direction = camDir;
  c.height = camZ;

  RCL_Vector2D pos;
  RCL_Unit height;

  pos.x = posX;
  pos.y = posY;
  height = posZ;
 
  RCL_PixelInfo p;

  printf("- mapping pixel: %d %d %d\n",posX,posY,posZ);

  p = RCL_mapToScreen(pos,height,c);

  printf("- result:\n");
  RCL_logPixelInfo(p);

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
  RCL_Ray r;

  r.start.x = RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;
  r.start.y = 2 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 4;

  RCL_Vector2D directions[8];

  for (int i = 0; i < 8; ++i)
    directions[i] = RCL_angleToDirection(RCL_UNITS_PER_SQUARE / 8 * i);

  for (int i = 0; i < 1000000; ++i)
  {
    r.direction = directions[i % 8];
    RCL_castRay(r,testArrayFunc);
  }
}

void benchmarkMapping()
{
  RCL_Camera c;

  c.resolution.x = 1024;
  c.resolution.y = 768;
  c.position.x = RCL_UNITS_PER_SQUARE / 2;
  c.position.y = RCL_UNITS_PER_SQUARE * 2;
  c.direction = RCL_UNITS_PER_SQUARE / 8;
  c.height = 0;

  RCL_PixelInfo p;

  RCL_Vector2D pos;
  RCL_Unit height;

  pos.x = -1024 * RCL_UNITS_PER_SQUARE;
  pos.y = -512 * RCL_UNITS_PER_SQUARE;
  height = 0;
 
  for (int i = 0; i < 1000000; ++i)
  {
    p = RCL_mapToScreen(pos,height,c);

    pos.x += 4;
    pos.y += 8;
    height = (height + 16) % 1024;
  }
}

int countPixels = 0;
uint32_t *pixelCounts = 0;
RCL_Camera countCamera;
int countOK = 1;

void pixelFunc(RCL_PixelInfo *p)
{
  if (countPixels)
  {
    if (p->position.x >= countCamera.resolution.x || p->position.x < 0 ||
        p->position.y >= countCamera.resolution.y || p->position.y < 0)
    {
      printf("ERROR: writing pixel outside screen at %d %d!\n",
        p->position.x,p->position.y);
   
      countOK = 0;
    }
    else
      pixelCounts[p->position.y * countCamera.resolution.x + p->position.x]++;
  }
}

int testPixelCount(RCL_Unit camX, RCL_Unit camY, RCL_Unit camZ,
  RCL_Unit camDir, RCL_Unit camShear, uint16_t camResX, uint16_t camResY,
  int complexRender)
{
  printf("Counting rendered pixels...\n");

  RCL_RayConstraints constraints;
  RCL_Camera c;

  RCL_initRayConstraints(&constraints);
  RCL_initCamera(&c);
  c.position.x = camX;
  c.position.y = camY;
  c.direction = camDir;
  c.shear = camShear;
  c.height = camZ;
  c.resolution.x = camResX;
  c.resolution.y = camResY;

  uint32_t pixels[camResX * camResY];

  for (uint32_t i = 0; i < camResX * camResY; ++i)
    pixels[i] = 0;

  pixelCounts = pixels;
  countCamera = c;
  countPixels = 1;

  countOK = 1;

  if (complexRender)
    RCL_renderComplex(c,testArrayFunc,testArrayFunc2,0,constraints);
  else
    RCL_renderSimple(c,testArrayFunc,0,0,constraints);

  for (uint32_t y = 0; y < camResY; ++y)
    for (uint32_t x = 0; x < camResX; ++x)
    {
      uint32_t index = y * camResX + x;

      if (pixels[index] != 1)
      {
        printf("ERROR: pixel at %d %d written %d times!\n",x,y,pixels[index]);
        countOK = 0;
      }
    }

  return countOK;
}

void benchmarkRender()
{
  RCL_Camera c;

  c.resolution.x = 640;
  c.resolution.y = 300;
  c.position.x = 10;
  c.position.y = 12;
  c.direction = 100;
  c.height = 200;

  RCL_RayConstraints constraints;

  constraints.maxHits = 10;
  constraints.maxSteps = 12;

  countPixels = 0;

  for (int i = 0; i < 100; ++i)
    RCL_renderComplex(c,testArrayFunc,testArrayFunc2,0,constraints);
}

int main()
{
  printf("Testing raycastlib.\n"); 

  if (!testSingleRay(
    3 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2,
    4 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2,
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
    -4 * RCL_UNITS_PER_SQUARE - RCL_UNITS_PER_SQUARE / 2,
    7 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 3,
    100,-100,
    0, 2,
    1, 2900,
    16))
    return 1;

  printf("testing perspective scale...\n");

  for (RCL_Unit i = 1; i < 100; ++i)
  {
    RCL_Unit size = i * 3;
    RCL_Unit distance = i * 6 + 200;

    RCL_Unit scaled = RCL_perspectiveScale(size,distance);
    RCL_Unit distance2 = RCL_perspectiveScaleInverse(size,scaled);

    if (RCL_absVal(distance - distance2 > 2))
      printf("ERROR: distance: %d, distance inverse: %d\n",distance,distance2);
  }

  printf("OK\n");

  if (!testPixelCount(
    RCL_UNITS_PER_SQUARE / 2,
    RCL_UNITS_PER_SQUARE / 2,
    RCL_UNITS_PER_SQUARE / 2,
    0,
    0,
    128,
    64,
    1))
    return 1;

  if (!testPixelCount(
    3 * RCL_UNITS_PER_SQUARE + 100,
    4 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 3,
    RCL_UNITS_PER_SQUARE,
    312,
    0,
    200,
    63,
    0))
    return 1;

  if (!testPixelCount(
    - RCL_UNITS_PER_SQUARE,
    0,
    300,
    -600,
    -120,
    64,
    68,
    1))
    return 1;

  printf("OK\n");

/*
  if (!testSingleMapping(
    -RCL_UNITS_PER_SQUARE,
    0,
    RCL_UNITS_PER_SQUARE / 2,
    1280,
    640,
    0,
    0,
    0,
    RCL_UNITS_PER_SQUARE / 2,
    640,
    0,
    1024
    ))
    return -1;
*/
  printf("benchmark:\n");

  long t;
  t = measureTime(benchCastRays);
  printf("cast 1000000 rays: %ld ms\n",t);

  t = measureTime(benchmarkMapping);
  printf("map point to screen 1000000 times: %ld ms\n",t);

  t = measureTime(benchmarkRender);
  printf("render 100 times: %ld ms\n",t);

  printf("\n"); 
  printProfile();

  return 0;
}
