#ifndef RAYCASTLIB_H
#define RAYCASTLIB_H

/**
  raycastlib - Small C header-only raycasting library for embedded and low
  performance computers, such as Arduino. Only uses integer math and stdint
  standard library.

  author: Miloslav "drummyfish" Ciz
  license: CC0

  - Game field's bottom left corner is at [0,0].
  - X axis goes right.
  - Y axis goes up.
  - Each game square is UNITS_PER_SQUARE * UNITS_PER_SQUARE.
  - Angles are in Units, 0 means pointing right (x+) and positively rotates
    clockwise, a full angle has UNITS_PER_SQUARE Units.
  */

#include <stdio.h>

#include <stdint.h>

#define UNITS_PER_SQUARE 1024 ///< No. of Units in a side of a spatial square.

typedef int32_t Unit;   /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. This effectively
                             serves the purpose of a fixed-point arithmetic. */

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

typedef struct
{
  Vector2D position;
  Unit direction;
  Vector2D resolution;
  Unit fovAngle;
  Unit height;
} Camera;

typedef struct
{
  Vector2D position; ///< On-screen position.
  int8_t isWall;     ///< Whether the pixel is a wall or a floor(/ceiling).
  Unit depth;        ///< Corrected depth.
  HitResult hit;     ///< Corresponding ray hit.
} PixelInfo;

typedef struct
{
  uint16_t maxHits;
  uint16_t maxSteps;
} RayConstraints;

/**
 Function used to retrieve the cells of the rendered scene. It should return
 a "type" of given square as an integer (e.g. square height) - between squares
 that return different numbers there is considered to be a collision.
*/ 
typedef Unit (*ArrayFunction)(int16_t x, int16_t y);

typedef void (*PixelFunction)(PixelInfo info);

typedef void
  (*ColumnFunction)(HitResult *hits, uint16_t hitCount, uint16_t x, Ray ray);

/**
 Simple-interface function to cast a single ray.

 @return          The first collision result.
 */
HitResult castRay(Ray ray, ArrayFunction arrayFunc);

/**
 Casts a single ray and returns a list of collisions.
 */
void castRayMultiHit(Ray ray, ArrayFunction arrayFunc, HitResult *hitResults,
  uint16_t *hitResultsLen, RayConstraints constraints);

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
void castRaysMultiHit(Camera cam, ArrayFunction arrayFunc,
  ColumnFunction columnFunc, RayConstraints constraints);

void render(Camera cam, ArrayFunction arrayFunc, PixelFunction pixelFunc,
  RayConstraints constraints);

//=============================================================================
// privates

#ifdef RAYCASTLIB_PROFILE
  // function call counters for profiling
  uint32_t profile_sqrtInt = 0;
  uint32_t profile_clamp = 0;
  uint32_t profile_cosInt = 0;
  uint32_t profile_angleToDirection = 0;
  uint32_t profile_dist = 0;
  uint32_t profile_len = 0;
  uint32_t profile_pointIsLeftOfRay = 0;
  uint32_t profile_castRaySquare = 0;
  uint32_t profile_castRayMultiHit = 0; 
  uint32_t profile_castRay = 0;
  uint16_t profile_normalize = 0;
  uint16_t profile_vectorsAngleCos = 0;

  #define profileCall(c) profile_##c += 1

  #define printProfile() {\
    printf("profile:\n");\
    printf("  sqrtInt: %d\n",profile_sqrtInt);\
    printf("  clamp: %d\n",profile_clamp);\
    printf("  cosInt: %d\n",profile_cosInt);\
    printf("  angleToDirection: %d\n",profile_angleToDirection);\
    printf("  dist: %d\n",profile_dist);\
    printf("  len: %d\n",profile_len);\
    printf("  pointIsLeftOfRay: %d\n",profile_pointIsLeftOfRay);\
    printf("  castRaySquare: %d\n",profile_castRaySquare);\
    printf("  castRayMultiHit : %d\n",profile_castRayMultiHit);\
    printf("  castRay: %d\n",profile_castRay);\
    printf("  normalize: %d\n",profile_normalize);\
    printf("  vectorsAngleCos: %d\n",profile_vectorsAngleCos); }
#else
  #define profileCall(c)
#endif

Unit clamp(Unit value, Unit valueMin, Unit valueMax)
{
  profileCall(clamp);

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
  profileCall(cosInt);

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
  profileCall(angleToDirection);

  Vector2D result;

  result.x = cosInt(angle);
  result.y = -1 * sinInt(angle);

  return result;
}

uint16_t sqrtInt(uint32_t value)
{
  profileCall(sqrtInt);

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
  profileCall(dist);

  int32_t dx = p2.x - p1.x;
  int32_t dy = p2.y - p1.y;

  dx = dx * dx;
  dy = dy * dy;

  return sqrtInt((uint32_t) (dx + dy));
}

Unit len(Vector2D v)
{
  profileCall(len);

  v.x *= v.x;
  v.y *= v.y;

  return sqrtInt(((uint32_t) v.x) + ((uint32_t) v.y));
}

int8_t pointIsLeftOfRay(Vector2D point, Ray ray)
{
  profileCall(pointIsLeftOfRay);

  Unit dX    = point.x - ray.start.x;
  Unit dY    = point.y - ray.start.y;
  return (ray.direction.x * dY - ray.direction.y * dX) > 0;
         // ^ Z component of cross-product
}

/**
  Casts a ray within a single square, to collide with the square borders.  
 */
void castRaySquare(Ray localRay, Vector2D *nextCellOff, Vector2D *collOff)
{
  profileCall(castRaySquare);

  nextCellOff->x = 0;
  nextCellOff->y = 0;

  Ray criticalLine = localRay;

  #define helper(c1,c2,n)\
    {\
      nextCellOff->c1 = n;\
      collOff->c1 = criticalLine.start.c1 - localRay.start.c1;\
      collOff->c2 = \
        (((int32_t) collOff->c1) * localRay.direction.c2) /\
        ((localRay.direction.c1 == 0) ? 1 : localRay.direction.c1);\
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

  collOff->x += nextCellOff->x;
  collOff->y += nextCellOff->y;
}

void castRayMultiHit(Ray ray, ArrayFunction arrayFunc, HitResult *hitResults,
  uint16_t *hitResultsLen, RayConstraints constraints)
{
  profileCall(castRayMultiHit);

  Vector2D initialPos = ray.start;
  Vector2D currentPos = ray.start;

  Vector2D currentSquare;

  currentSquare.x = ray.start.x / UNITS_PER_SQUARE;
  currentSquare.y = ray.start.y / UNITS_PER_SQUARE;

  *hitResultsLen = 0;

  int16_t squareType = arrayFunc(currentSquare.x,currentSquare.y);

  Vector2D no, co; // next cell offset, collision offset

  no.x = 0;        // just to supress a warning
  no.y = 0;

  for (uint16_t i = 0; i < constraints.maxSteps; ++i)
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

      if (*hitResultsLen >= constraints.maxHits)
        break;
    }

    ray.start.x = currentPos.x % UNITS_PER_SQUARE;
    ray.start.y = currentPos.y % UNITS_PER_SQUARE;

    castRaySquare(ray,&no,&co);

    currentSquare.x += no.x;
    currentSquare.y += no.y;

    // offset into the next cell
    currentPos.x += co.x;
    currentPos.y += co.y;
  }
}

HitResult castRay(Ray ray, ArrayFunction arrayFunc)
{
  profileCall(castRay);

  HitResult result;
  uint16_t  len;
  RayConstraints c;

  c.maxSteps = 1000;
  c.maxHits = 1;

  castRayMultiHit(ray,arrayFunc,&result,&len,c);

  if (len == 0)
    result.distance = -1;

  return result;
}

void castRaysMultiHit(Camera cam, ArrayFunction arrayFunc,
  ColumnFunction columnFunc, RayConstraints constraints)
{
  uint16_t fovHalf = cam.fovAngle / 2;

  Vector2D dir1 = angleToDirection(cam.direction - fovHalf);
  Vector2D dir2 = angleToDirection(cam.direction + fovHalf);

  Unit dX = dir2.x - dir1.x;
  Unit dY = dir2.y - dir1.y;

  HitResult hits[constraints.maxHits];
  Ray rays[constraints.maxHits];
  uint16_t hitCount;

  Ray r;
  r.start = cam.position;

  for (uint16_t i = 0; i < cam.resolution.x; ++i)
  {
    r.direction.x = dir1.x + (dX * i) / cam.resolution.x;
    r.direction.y = dir1.y + (dY * i) / cam.resolution.x;

    castRayMultiHit(r,arrayFunc,hits,&hitCount,constraints);

    columnFunc(hits,hitCount,i,r);
  }
}

PixelFunction _pixelFunction = 0;
ArrayFunction _arrayFunction = 0;
Camera _camera;

void _columnFunction(HitResult *hits, uint16_t hitCount, uint16_t x, Ray ray)
{
  int32_t y = _camera.resolution.y - 1; // on screen y, will only go upwards

  Unit worldZPrev = -1 * _camera.height;

  uint16_t middleRow = _camera.resolution.y / 2;

  for (uint32_t j = 0; j < hitCount; ++j)
  {
    HitResult hit = hits[j];

    /* FIXME/TODO: The adjusted (=orthogonal, camera-space) distance could
       possibly be computed more efficiently by not computing Euclidean
       distance at all, but rather compute the distance of the collision
       point from the projection plane (line). */

    Unit dist = // adjusted distance
      (hit.distance * vectorsAngleCos(angleToDirection(_camera.direction),
      ray.direction)) / UNITS_PER_SQUARE;

    dist = dist == 0 ? 1 : dist; // prevent division by zero

    Unit wallHeight = _arrayFunction(hit.square.x,hit.square.y);

    Unit worldZ2 = -1 * _camera.height + wallHeight;

    Unit z1Screen = middleRow -
      perspectiveScale(
        (worldZPrev * _camera.resolution.y) / UNITS_PER_SQUARE,
        dist,
        1);

    Unit z2Screen = middleRow -
      perspectiveScale(
        (worldZ2 * _camera.resolution.y) / UNITS_PER_SQUARE,
        dist,
        1);

    PixelInfo p;
    p.position.x = x;

    p.isWall = 0;

    // draw floor until the wall
    for (int32_t i = y; i > z1Screen; --i)
    {
      p.position.y = i;
      _pixelFunction(p);  
    }

    // draw the wall

    p.isWall = 1;

    for (int32_t i = z1Screen < y ? z1Screen : y; i > z2Screen; --i)
    {
      p.position.y = i;
      p.hit = hit;
      _pixelFunction(p);
    }

    y = z2Screen;
    worldZPrev = worldZ2;
  }
}

void render(Camera cam, ArrayFunction arrayFunc, PixelFunction pixelFunc,
  RayConstraints constraints)
{
  _pixelFunction = pixelFunc;
  _arrayFunction = arrayFunc;
  _camera = cam;
  castRaysMultiHit(cam,arrayFunc,_columnFunction,constraints);
}

Vector2D normalize(Vector2D v)
{
  profileCall(normalize);

  Vector2D result;

  Unit l = len(v);

  result.x = (v.x * UNITS_PER_SQUARE) / l;
  result.y = (v.y * UNITS_PER_SQUARE) / l;

  return result;
}

Unit vectorsAngleCos(Vector2D v1, Vector2D v2)
{
  profileCall(vectorsAngleCos);

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
return (originalSize * UNITS_PER_SQUARE) / distance;

  distance *= fov;
  distance = distance == 0 ? 1 : distance; // prevent division by zero

  return originalSize / distance; 
}

#endif
