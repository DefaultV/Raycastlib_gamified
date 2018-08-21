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
  
 */
void castRaySquare(Ray globalRay, Vector2D *nextCellOffset,
  Vector2D *collisionPointOffset)
{
  globalRay.start.x %= UNITS_PER_SQUARE;
  globalRay.start.y %= UNITS_PER_SQUARE;

  nextCellOffset->x = 0;
  nextCellOffset->y = 0;

  Ray criticalLine = globalRay;

  #define helper(c1,c2,n)\
    {\
      nextCellOffset->c1 = n;\
      collisionPointOffset->c1 = criticalLine.start.c1 - globalRay.start.c1;\
      collisionPointOffset->c2 =\
        (collisionPointOffset->c1 * globalRay.direction.c2) /\
        (globalRay.direction.c1 == 0 ? 1 : globalRay.direction.c1);\
    }

  #define helper2(n1,n2,c)\
    if (pointIsLeftOfRay(globalRay.start,criticalLine) == c)\
      helper(y,x,n1)\
    else\
      helper(x,y,n2)
      
  if (globalRay.direction.x > 0)
  {
    criticalLine.start.x = UNITS_PER_SQUARE - 1;

    if (globalRay.direction.y > 0)
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

    if (globalRay.direction.y > 0)
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
}

int main()
{
  Ray r;
  Vector2D no;
  Vector2D co; 

  r.start.x     = 10;
  r.start.y     = 10;
  r.direction.x = -200;
  r.direction.y = -200;
 
  logRay(r);
 
  castRaySquare(r,&no,&co);

  logVector2D(no);
  logVector2D(co);

  return 0;
}
