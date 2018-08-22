#include <stdint.h>
#include <stdio.h>

/**
  author: Miloslav "drummyfish" Ciz
  license: CC0

  - Game field's bottom left corner is at [0,0].
  - X axis goes right.
  - Y axis goes up.
  - Each game square is UNITS_PER_SQUARE * UNITS_PER_SQUARE.

  */

#define UNITS_PER_SQUARE 1024

typedef int64_t Unit;   /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. */

/// Position in 2D space.
typedef struct
{
  Unit y;
  Unit x;
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
} HitResult;

HitResult castRay(Ray ray, int (*collisionFunc)(int, int),
  unsigned int maxSteps);

//=============================================================================
// privates

#define logVector2D(v) printf("[%d,%d]\n",v.x,v.y)
#define logRay(r)      printf("{\n");\
                       logVector2D(r.start);\
                       logVector2D(r.direction);\
                       printf("}\n");

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

HitResult castRay(Ray ray, int (*collisionFunc)(int, int),
  unsigned int maxSteps)
{
  HitResult result;

  result.distance = -1;
  result.square.x = ray.start.x / UNITS_PER_SQUARE;
  result.square.y = ray.start.y / UNITS_PER_SQUARE;
  result.position = ray.start;

  for (uint16_t i = 0; i < maxSteps; ++i)
  {
    ray.start.x = result.position.x % UNITS_PER_SQUARE;
    ray.start.y = result.position.y % UNITS_PER_SQUARE;

    Vector2D no, co;

    castRaySquare(ray,&no,&co);

    result.square.x += no.x;
    result.square.y += no.y;

    result.position.x += co.x;
    result.position.y += co.y;
  }

  return result;
}

int aaa(int x, int y)
{
  return (x < 0 || y < 0 || x > 10 || y > 10) ? 1 : 0;
}

int main()
{
  Ray r;

  r.start.x     = 4 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2;
  r.start.y     = 4 * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2;
  r.direction.x = 100;
  r.direction.y = 50;
 
  logRay(r);
 
  castRay(r,aaa,10);

  return 0;
}
