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

  */

#define UNITS_PER_SQUARE 1024

typedef int32_t Unit;   /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. */

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

HitResult castRay(Ray ray, int16_t (*arrayFunc)(int16_t, int16_t),
  uint16_t maxSteps);

/**
 Casts a single ray and returns a list of collisions.
 */

void castRayMultiHit(Ray ray, int16_t (*arrayFunc)(int16_t, int16_t),
  uint16_t maxSteps, HitResult *hitResults, uint16_t *hitResultsLen,
  uint16_t maxHits);

//=============================================================================
// privates

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
  Unit dx = p2.x - p1.x;
  Unit dy = p2.y - p1.y;
  return sqrtInt(((uint16_t) dx * dx) + ((uint16_t) dy * dy));
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
      collisionPointOffset->c2 =\
        (collisionPointOffset->c1 * localRay.direction.c2) /\
        (localRay.direction.c1 == 0 ? 1 : localRay.direction.c1);\
    }

  #define helper2(n1,n2,c)\
    if (pointIsLeftOfRay(localRay.start,criticalLine) == c)\
      helper(y,x,n1)\
    else\
      helper(x,y,n2)
      
  if (localRay.direction.x > 0)
  {
    criticalLine.start.x = UNITS_PER_SQUARE;

    if (localRay.direction.y > 0)
    {
      // top right
      criticalLine.start.y = UNITS_PER_SQUARE;
      helper2(1,1,1)
    }
    else
    {
      // bottom right
      criticalLine.start.y = -1;
      helper2(-1,1,0)
    }
  }
  else
  {
    criticalLine.start.x = -1;

    if (localRay.direction.y > 0)
    {
      // top left
      criticalLine.start.y = UNITS_PER_SQUARE;
      helper2(1,-1,0)
    }
    else
    {
      // bottom left
      criticalLine.start.y = -1;
      helper2(-1,-1,1)
    }
  }

  #undef helper2
  #undef helper
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

      hitResults[*hitResultsLen] = h;

      *hitResultsLen += 1;

      squareType = currentType;

      if (*hitResultsLen >= maxHits)
        break;
    }

    ray.start.x = currentPos.x % UNITS_PER_SQUARE;
    ray.start.y = currentPos.y % UNITS_PER_SQUARE;

    Vector2D no, co;

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

#endif
