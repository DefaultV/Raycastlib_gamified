#ifndef RAYCASTLIB_H
#define RAYCASTLIB_H

#include <stdint.h>

/**
  author: Miloslav "drummyfish" Ciz
  license: CC0

  - Game field's bottom left corner is at [0,0].
  - X axis goes right.
  - Y axis goes up.
  - Each game square is UNITS_PER_SQUARE * UNITS_PER_SQUARE.
  - Angles are in Units, 0 means pointing right (x+) and positively rotates
    clockwise, a full angle has UNITS_PER_SQUARE Units.
  */

#define UNITS_PER_SQUARE 1024

typedef int32_t Unit;   /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. */

#define logVector2D(v)\
  printf("[%d,%d]\n",v.x,v.y);

#define logRay(r){\
  printf("ray:\n");\
  printf("  start: ");\
  logVector2D(r.start);\
  printf("  dir: ");\
  logVector2D(r.direction);}\

#define logHitResult(h){\
  printf("hit:\n");\
  printf("  sqaure: ");\
  logVector2D(h.square);\
  printf("  pos: ");\
  logVector2D(h.position);\
  printf("  dist: %d\n", h.distance);\
  printf("  texcoord: %d\n", h.textureCoord);}\

/// Position in 2D space.
typedef struct
{
  int32_t y;
  int32_t x;
} Vector2D;

typedef struct
{
  Vector2D start;
  Vector2D direction;
} Ray;

typedef struct
{
  Vector2D square;     ///< Collided square coordinates.
  Vector2D position;   ///< Exact collision position in Units.
  Unit     distance;   /**< Euclidean distance to the hit position, or -1 if
                            no collision happened. */
  Unit     textureCoord;    /**< Normalized (0 to UNITS_PER_SQUARE - 1) texture
                                 coordinate. */
  uint8_t  direction;       ///< Direction of hit.
} HitResult;

/**
 Casts a single ray and returns the first collision result.

 @param Ray       Ray to be cast.
 @param arrayFunc Function that for x and y array coordinates (in squares, NOT
                  Units) returns a type of square (just a number) - transition
                  between two squares of different types (values) is considered
                  a collision).
 @param maxSteps  Maximum number of steps (in squares) to trace the ray.
 @return          The first collision result.
 */

/**
 Like castRaysMultiHit but only returns the first hit.
 */

HitResult castRay(Ray ray, int16_t (*arrayFunc)(int16_t, int16_t),
  uint16_t maxSteps);

/**
 Casts a single ray and returns a list of collisions.
 */
void castRayMultiHit(Ray ray, int16_t (*arrayFunc)(int16_t x, int16_t y),
  uint16_t maxSteps, HitResult *hitResults, uint16_t *hitResultsLen,
  uint16_t maxHits);

Vector2D angleToDirection(Unit angle);
Unit cosInt(Unit input);
Unit sinInt(Unit input);

/// Normalizes given vector to have UNITS_PER_SQUARE length.
Vector2D normalize(Vector2D v);

/// Computes a cos of an angle between two vectors.
Unit vectorsAngleCos(Vector2D v1, Vector2D v2);

uint16_t sqrtInt(uint32_t value);
Unit dist(Vector2D p1, Vector2D p2);
Unit len(Vector2D v);

/**
 Converts an angle in whole degrees to an angle in Units that this library
 uses.
 */   
Unit degreesToUnitsAngle(int16_t degrees);

///< Computes the change in size of an object due to perspective.
Unit perspectiveScale(Unit originalSize, Unit distance, Unit fov);

/**
 Casts rays for given camera view and for each hit calls a user provided
 function.
 */
void castRaysMultiHit(
  Vector2D position, Unit directionAngle, Unit fovAngle, uint16_t resolution,
  int16_t (*arrayFunc)(int16_t x, int16_t y),
  void (*hitFunc)(uint16_t pos, HitResult hit, uint16_t hitNo, Ray r),
  uint16_t maxHits, uint16_t maxSteps);

//=============================================================================
// privates

Unit clamp(Unit value, Unit valueMin, Unit valueMax)
{
  if (value < valueMin)
    return valueMin;

  if (value > valueMax)
    return valueMax;

  return value;
}

// Bhaskara's cosine approximation formula
#define trigHelper(x) (UNITS_PER_SQUARE *\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 - 4 * (x) * (x)) /\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 + (x) * (x)))

Unit cosInt(Unit input)
{
  input = input % UNITS_PER_SQUARE;

  if (input < 0)
    input = UNITS_PER_SQUARE + input;

  if (input < UNITS_PER_SQUARE / 4)
    return trigHelper(input);
  else if (input < UNITS_PER_SQUARE / 2)
    return -1 * trigHelper(UNITS_PER_SQUARE / 2 - input);
  else if (input < 3 * UNITS_PER_SQUARE / 4)
    return -1 * trigHelper(input - UNITS_PER_SQUARE / 2);
  else
    return trigHelper(UNITS_PER_SQUARE - input);
}

#undef trigHelper

Unit sinInt(Unit input)
{
  return cosInt(input - UNITS_PER_SQUARE / 4);
}

Vector2D angleToDirection(Unit angle)
{
  Vector2D result;

  result.x = cosInt(angle);
  result.y = -1 * sinInt(angle);

  return result;
}

uint16_t sqrtInt(uint32_t value)
{
  uint32_t result = 0;

  uint32_t a  = value;
  uint32_t b = 1u << 30;

  while (b > a)
    b >>= 2;

  while (b != 0)
  {
    if (a >= result + b)
    {
      a -= result + b;
      result = result +  2 * b;
    }

    b >>= 2;
    result >>= 1;
  }

  return result;
}

Unit dist(Vector2D p1, Vector2D p2)
{
  int32_t dx = p2.x - p1.x;
  int32_t dy = p2.y - p1.y;

  dx = dx * dx;
  dy = dy * dy;

  return sqrtInt(((uint32_t) dx) + ((uint32_t) dy));
}

Unit len(Vector2D v)
{
  v.x *= v.x;
  v.y *= v.y;

  return sqrtInt(((uint32_t) v.x) + ((uint32_t) v.y));
}

int8_t pointIsLeftOfRay(Vector2D point, Ray ray)
{
  int dX    = point.x - ray.start.x;
  int dY    = point.y - ray.start.y;
  return (ray.direction.x * dY - ray.direction.y * dX) > 0;
         // ^ Z component of cross-product
}

/**
  Casts a ray within a single square, to collide with the square borders.  
 */
void castRaySquare(Ray localRay, Vector2D *nextCellOffset,
  Vector2D *collisionPointOffset)
{
  nextCellOffset->x = 0;
  nextCellOffset->y = 0;

  Ray criticalLine = localRay;

  #define helper(c1,c2,n)\
    {\
      nextCellOffset->c1 = n;\
      collisionPointOffset->c1 = criticalLine.start.c1 - localRay.start.c1;\
      collisionPointOffset->c2 = \
        (((int32_t) collisionPointOffset->c1) * localRay.direction.c2) /\
        ((localRay.direction.c1 == 0) ? 1 : localRay.direction.c1);\
      Unit nextC2 = localRay.start.c2 + collisionPointOffset->c2;\
    }

  #define helper2(n1,n2,c)\
    if (pointIsLeftOfRay(localRay.start,criticalLine) == c)\
      helper(y,x,n1)\
    else\
      helper(x,y,n2)
      
  if (localRay.direction.x > 0)
  {
    criticalLine.start.x = UNITS_PER_SQUARE - 1;

    if (localRay.direction.y > 0)
    {
      // top right
      criticalLine.start.y = UNITS_PER_SQUARE - 1;
      helper2(1,1,1)
    }
    else
    {
      // bottom right
      criticalLine.start.y = 0;
      helper2(-1,1,0)
    }
  }
  else
  {
    criticalLine.start.x = 0;

    if (localRay.direction.y > 0)
    {
      // top left
      criticalLine.start.y = UNITS_PER_SQUARE - 1;
      helper2(1,-1,0)
    }
    else
    {
      // bottom left
      criticalLine.start.y = 0;
      helper2(-1,-1,1)
    }
  }

  #undef helper2
  #undef helper

  collisionPointOffset->x += nextCellOffset->x;
  collisionPointOffset->y += nextCellOffset->y;
}

void castRayMultiHit(Ray ray, int16_t (*arrayFunc)(int16_t, int16_t),
  uint16_t maxSteps, HitResult *hitResults, uint16_t *hitResultsLen,
  uint16_t maxHits)
{
  Vector2D initialPos = ray.start;
  Vector2D currentPos = ray.start;

  Vector2D currentSquare;

  currentSquare.x = ray.start.x / UNITS_PER_SQUARE;
  currentSquare.y = ray.start.y / UNITS_PER_SQUARE;

  *hitResultsLen = 0;

  int16_t squareType = arrayFunc(currentSquare.x,currentSquare.y);

  Vector2D no, co; // next cell offset, collision offset

  for (uint16_t i = 0; i < maxSteps; ++i)
  {
    int16_t currentType = arrayFunc(currentSquare.x,currentSquare.y);

    if (currentType != squareType)
    {
      // collision

      HitResult h;

      h.position = currentPos;
      h.square   = currentSquare;
      h.distance = dist(initialPos,currentPos);

      if (no.y > 0)
        h.direction = 0;
      else if (no.x > 0)
        h.direction = 1;
      else if (no.y < 0)
        h.direction = 2;
      else
        h.direction = 3;

      hitResults[*hitResultsLen] = h;

      *hitResultsLen += 1;

      squareType = currentType;

      if (*hitResultsLen >= maxHits)
        break;
    }

    ray.start.x = currentPos.x % UNITS_PER_SQUARE;
    ray.start.y = currentPos.y % UNITS_PER_SQUARE;

    castRaySquare(ray,&no,&co);

    currentSquare.x += no.x;
    currentSquare.y += no.y;

    currentPos.x += co.x;
    currentPos.y += co.y;
  }
}

HitResult castRay(Ray ray, int16_t (*arrayFunc)(int16_t, int16_t),
  uint16_t maxSteps)
{
  HitResult result;
  uint16_t  len;

  castRayMultiHit(ray,arrayFunc,maxSteps,&result,&len,1);

  if (len == 0)
    result.distance = -1;

  return result;
}

void castRaysMultiHit(
  Vector2D position, Unit directionAngle, Unit fovAngle, uint16_t resolution,
  int16_t (*arrayFunc)(int16_t x, int16_t y),
  void (*hitFunc)(uint16_t pos, HitResult hit, uint16_t hitNo, Ray r),
  uint16_t maxHits, uint16_t maxSteps)
{
  uint16_t fovHalf = fovAngle / 2;

  Vector2D dir1 = angleToDirection(directionAngle - fovHalf);
  Vector2D dir2 = angleToDirection(directionAngle + fovHalf);

  Unit dX = dir2.x - dir1.x;
  Unit dY = dir2.y - dir1.y;

  HitResult hits[maxHits];
  uint16_t hitCount;

  Ray r;
  r.start = position;

  for (uint8_t i = 0; i < resolution; ++i)
  {
    r.direction.x = dir1.x + (dX * i) / resolution;
    r.direction.y = dir1.y + (dY * i) / resolution;

    castRayMultiHit(r,arrayFunc,maxSteps,hits,&hitCount,maxHits);

    for (uint8_t j = 0; j < hitCount; ++j)
      hitFunc(i,hits[j],j,r);
  }
}

Vector2D normalize(Vector2D v)
{
  Vector2D result;

  Unit l = len(v);

  result.x = (v.x * UNITS_PER_SQUARE) / l;
  result.y = (v.y * UNITS_PER_SQUARE) / l;

  return result;
}

Unit vectorsAngleCos(Vector2D v1, Vector2D v2)
{
  v1 = normalize(v1);
  v2 = normalize(v2);

  return (v1.x * v2.x + v1.y * v2.y) / UNITS_PER_SQUARE;
}

Unit degreesToUnitsAngle(int16_t degrees)
{
  return (degrees * UNITS_PER_SQUARE) / 360;
}

Unit perspectiveScale(Unit originalSize, Unit distance, Unit fov)
{
  distance *= fov;
  distance = distance == 0 ? 1 : distance; // prevent division by zero

  return originalSize / distance; 
}

#endif
