#ifndef RAYCASTLIB_H
#define RAYCASTLIB_H

/**
  raycastlib - Small C header-only raycasting library for embedded and low
  performance computers, such as Arduino. Only uses integer math and stdint
  standard library.

  Check the defines below to fine-tune accuracy vs performance! Don't forget
  to compile with optimizations.

  author: Miloslav "drummyfish" Ciz
  license: CC0
  version: 0.1

  - Game field's bottom left corner is at [0,0].
  - X axis goes right in the ground plane.
  - Y axis goes up in the ground plane.
  - Height means the Z (vertical) coordinate.
  - Each game square is UNITS_PER_SQUARE * UNITS_PER_SQUARE points.
  - Angles are in Units, 0 means pointing right (x+) and positively rotates
    clockwise. A full angle has UNITS_PER_SQUARE Units.
*/

#include <stdint.h>

#ifndef RAYCAST_TINY /** Turns on super efficient version of this library. Only
                         use if neccesarry, looks ugly. */
  #define UNITS_PER_SQUARE 1024 ///< N. of Units in a side of a spatial square.
  typedef int32_t Unit; /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. This effectively
                             serves the purpose of a fixed-point arithmetic. */
  #define UNIT_INFINITY 5000000;
#else
  #define UNITS_PER_SQUARE 16
  typedef int16_t Unit;
  #define UNIT_INFINITY 32000;
  #define USE_DIST_APPROX 2
#endif

#ifndef USE_COS_LUT
#define USE_COS_LUT 0 /**< type of look up table for cos function:
                           0: none (compute)
                           1: 64 items
                           2: 128 items */
#endif

#ifndef USE_DIST_APPROX
#define USE_DIST_APPROX 0 /**< What distance approximation to use:
                            0: none (compute full Euclidean distance)
                            1: accurate approximation
                            2: octagonal approximation (LQ) */
#endif

#ifndef ROLL_TEXTURE_COORDS
#define ROLL_TEXTURE_COORDS 1 /**< Says whether rolling doors should also roll
                                   the texture coordinates along (mostly
                                   desired for doors). */
#endif

#ifndef VERTICAL_FOV
#define VERTICAL_FOV (UNITS_PER_SQUARE / 2)
#endif

#ifndef HORIZONTAL_FOV
#define HORIZONTAL_FOV (UNITS_PER_SQUARE / 4)
#endif

#define HORIZONTAL_FOV_HALF (HORIZONTAL_FOV / 2)

#ifndef CAMERA_COLL_RADIUS
#define CAMERA_COLL_RADIUS UNITS_PER_SQUARE / 4
#endif

#ifndef CAMERA_COLL_HEIGHT_BELOW
#define CAMERA_COLL_HEIGHT_BELOW UNITS_PER_SQUARE
#endif 

#ifndef CAMERA_COLL_HEIGHT_ABOVE
#define CAMERA_COLL_HEIGHT_ABOVE (UNITS_PER_SQUARE / 3)
#endif

#ifndef CAMERA_COLL_STEP_HEIGHT
#define CAMERA_COLL_STEP_HEIGHT (UNITS_PER_SQUARE / 2)
#endif

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

#define logPixelInfo(p){\
  printf("pixel:\n");\
  printf("  position: ");\
  logVector2D(p.position);\
  printf("  depth: %d\n", p.depth);\
  printf("  wall: %d\n", p.isWall);\
  printf("  textCoordY: %d\n", p.textureCoordY);\
  printf("  hit: ");\
  logHitResult(p.hit);\
  }\

/// Position in 2D space.
typedef struct
{
  Unit x;
  Unit y;
} Vector2D;

typedef struct
{
  Vector2D start;
  Vector2D direction;
} Ray;

typedef struct
{
  Unit     distance;     /**< Euclidean distance to the hit position, or -1 if
                              no collision happened. */
  uint8_t  direction;    ///< Direction of hit.
  Unit     textureCoord; ///< Normalized (0 to UNITS_PER_SQUARE - 1) tex coord.
  Vector2D square;       ///< Collided square coordinates.
  Vector2D position;     ///< Exact collision position in Units.
  Unit     arrayValue;   /**  Value returned by array function (most often this
                              will be the floor height). */
  Unit     type;         /**< Integer identifying type of square (number
                              returned by type function, e.g. texture index).*/
  Unit     doorRoll;     ///< Holds value of door roll.
} HitResult;

// TODO: things like FOV could be constants to make them precomp. and faster?

typedef struct
{
  Vector2D position;
  Unit direction;
  Vector2D resolution;
  int16_t shear; /* Shear offset in pixels (0 => no shear), can simulate
                    looking up/down. */
  Unit height;
} Camera;

/**
  Holds an information about a single rendered pixel (for a pixel function
  that works as a fragment shader.
*/
typedef struct
{
  Vector2D position;  ///< On-screen position.
  int8_t isWall;      ///< Whether the pixel is a wall or a floor/ceiling.
  int8_t isFloor;     ///< Whether the pixel is floor or ceiling.
  int8_t isHorizon;   ///< Whether the pixel is floor going towards horizon.
  Unit depth;         ///< Corrected depth.
  HitResult hit;      ///< Corresponding ray hit.
  Unit textureCoordY; ///< Normalized (0 to UNITS_PER_SQUARE - 1) tex coord.
} PixelInfo;

typedef struct
{
  uint16_t maxHits;
  uint16_t maxSteps;
  uint8_t computeTextureCoords; ///< Turns texture coords on/off.
} RayConstraints;

/**
  Function used to retrieve some information about cells of the rendered scene.
  It should return a characteristic of given square as an integer (e.g. square
  height, texture index, ...) - between squares that return different numbers
  there is considered to be a collision.

  This function should be as fast as possible as it will typically be called
  very often.
*/ 
typedef Unit (*ArrayFunction)(int16_t x, int16_t y);

/**
  Function that renders a single pixel at the display. It is handed an info
  about the pixel it should draw.

  This function should be as fast as possible as it will typically be called
  very often.
*/
typedef void (*PixelFunction)(PixelInfo *info);

typedef void
  (*ColumnFunction)(HitResult *hits, uint16_t hitCount, uint16_t x, Ray ray);

/**
  Simple-interface function to cast a single ray.
  @return          The first collision result.
*/
HitResult castRay(Ray ray, ArrayFunction arrayFunc);

/**
  Maps a single point in the world to the screen (2D position + depth).
*/
PixelInfo mapToScreen(Vector2D worldPosition, Unit height, Camera camera);

/**
  Casts a single ray and returns a list of collisions.

  @param ray ray to be cast
  @param arrayFunc function that will be used to determine collisions (hits)
         with the ray (squares for which this function returns different values
         are considered to have a collision between them), this will typically
         be a function returning floor height
  @param typeFunc optional (can be 0) function - if provided, it will be used
         to mark the hit result with the number returned by this function
         (it can be e.g. a texture index)
  @param hitResults array in which the hit results will be stored (has to be
         preallocated with at space for at least as many hit results as
         maxHits specified with the constraints parameter)
  @param hitResultsLen in this variable the number of hit results will be
         returned
  @param constraints specifies constraints for the ray cast
*/
void castRayMultiHit(Ray ray, ArrayFunction arrayFunc, ArrayFunction typeFunc, 
  HitResult *hitResults, uint16_t *hitResultsLen, RayConstraints constraints);

Vector2D angleToDirection(Unit angle);

/**
Cos function.

@param input to cos in Units (UNITS_PER_SQUARE = 2 * pi = 360 degrees)
@return normalized output in Units (from -UNITS_PER_SQUARE to UNITS_PER_SQUARE)
*/
Unit cosInt(Unit input);

Unit sinInt(Unit input);

/// Normalizes given vector to have UNITS_PER_SQUARE length.
Vector2D normalize(Vector2D v);

/// Computes a cos of an angle between two vectors.
Unit vectorsAngleCos(Vector2D v1, Vector2D v2);

uint16_t sqrtInt(Unit value);
Unit dist(Vector2D p1, Vector2D p2);
Unit len(Vector2D v);

/**
  Converts an angle in whole degrees to an angle in Units that this library
  uses.
*/   
Unit degreesToUnitsAngle(int16_t degrees);

///< Computes the change in size of an object due to perspective.
Unit perspectiveScale(Unit originalSize, Unit distance);

/**
  Casts rays for given camera view and for each hit calls a user provided
  function.
*/
void castRaysMultiHit(Camera cam, ArrayFunction arrayFunc,
  ArrayFunction typeFunction, ColumnFunction columnFunc,
  RayConstraints constraints);

/**
  Using provided functions, renders a complete complex camera view.

  This function should render each screen pixel exactly once.

  @param cam camera whose view to render
  @param floorHeightFunc function that returns floor height (in Units)
  @param ceilingHeightFunc same as floorHeightFunc but for ceiling, can also be
                           0 (no ceiling will be rendered)
  @param typeFunction function that says a type of square (e.g. its texture
                     index), can be 0 (no type in hit result)
  @param pixelFunc callback function to draw a single pixel on screen
  @param constraints constraints for each cast ray
*/
void render(Camera cam, ArrayFunction floorHeightFunc,
  ArrayFunction ceilingHeightFunc, ArrayFunction typeFunction,
  PixelFunction pixelFunc, RayConstraints constraints);

/**
  Renders given camera view, with help of provided functions. This function is
  simpler and faster than render(...) and is meant to be rendering scenes
  with simple 1-intersection raycasting. The render(...) function can give more
  accurate results than this function, so it's to be considered even for simple
  scenes.

  Additionally this function supports rendering rolling doors.

  This function should render each screen pixel exactly once.

  @param rollFunc function that for given square says its door roll in Units
         (0 = no roll, UNITS_PER_SQUARE = full roll right, -UNITS_PER_SQUARE =
         full roll left), can be zero (no rolling door, rendering should also
         be faster as fewer intersections will be tested)
*/
void renderSimple(Camera cam, ArrayFunction floorHeightFunc,
  ArrayFunction typeFunc, PixelFunction pixelFunc, ArrayFunction rollFunc,
  RayConstraints constraints);

/**
  Function that moves given camera and makes it collide with walls and
  potentially also floor and ceilings. It's meant to help implement player
  movement.

  @param camera camera to move
  @param planeOffset offset to move the camera in
  @param heightOffset height offset to move the camera in
  @param floorHeightFunc function used to retrieve the floor height
  @param ceilingHeightFunc function for retrieving ceiling height, can be 0
                           (camera won't collide with ceiling)
  @param computeHeight whether to compute height - if false (0), floor and
                       ceiling functions won't be used and the camera will
                       only collide horizontally with walls (good for simpler
                       game, also faster)
  @param force if true, forces to recompute collision even if position doesn't
               change
*/
void moveCameraWithCollision(Camera *camera, Vector2D planeOffset,
  Unit heightOffset, ArrayFunction floorHeightFunc,
  ArrayFunction ceilingHeightFunc, int8_t computeHeight, int8_t force);

void initCamera(Camera *camera);
void initRayConstraints(RayConstraints *constraints);

//=============================================================================
// privates

// global helper variables, for precomputing stuff etc.
PixelFunction _pixelFunction = 0;
Camera _camera;
Unit _horizontalDepthStep = 0; 
Unit _startFloorHeight = 0;
Unit _startCeilHeight = 0;
Unit _camResYLimit = 0;
Unit _middleRow = 0;
ArrayFunction _floorFunction = 0;
ArrayFunction _ceilFunction = 0;
uint8_t _computeTextureCoords = 0;
Unit _fHorizontalDepthStart = 0;
Unit _cHorizontalDepthStart = 0;
int16_t _cameraHeightScreen = 0;
ArrayFunction _rollFunction = 0; // says door rolling

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
  uint32_t profile_absVal = 0;
  uint32_t profile_normalize = 0;
  uint32_t profile_vectorsAngleCos = 0;
  uint32_t profile_perspectiveScale = 0;
  uint32_t profile_wrap = 0;
  uint32_t profile_divRoundDown = 0;
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
    printf("  vectorsAngleCos: %d\n",profile_vectorsAngleCos);\
    printf("  absVal: %d\n",profile_absVal);\
    printf("  perspectiveScale: %d\n",profile_perspectiveScale);\
    printf("  wrap: %d\n",profile_wrap);\
    printf("  divRoundDown: %d\n",profile_divRoundDown); }
#else
  #define profileCall(c)
#endif

Unit clamp(Unit value, Unit valueMin, Unit valueMax)
{
  profileCall(clamp);

  if (value >= valueMin)
  {
    if (value <= valueMax)
      return value;
    else
      return valueMax;
  }
  else
    return valueMin;
}

Unit absVal(Unit value)
{
  profileCall(absVal);

  return value >= 0 ? value : -1 * value;
}

/// Like mod, but behaves differently for negative values.
Unit wrap(Unit value, Unit mod)
{
  profileCall(wrap);

  return value >= 0 ? (value % mod) : (mod + (value % mod) - 1);
}

/// Performs division, rounding down, NOT towards zero.
Unit divRoundDown(Unit value, Unit divisor)
{
  profileCall(divRoundDown);

  return value / divisor - ((value >= 0) ? 0 : 1);
}

// Bhaskara's cosine approximation formula
#define trigHelper(x) (((Unit) UNITS_PER_SQUARE) *\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 - 4 * (x) * (x)) /\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 + (x) * (x)))

#if USE_COS_LUT == 1

  #ifdef RAYCAST_TINY
  const Unit cosLUT[64] =
  {
    16,14,11,6,0,-6,-11,-14,-15,-14,-11,-6,0,6,11,14
  };
  #else
  const Unit cosLUT[64] =
  {
    1024,1019,1004,979,946,903,851,791,724,649,568,482,391,297,199,100,0,-100,
    -199,-297,-391,-482,-568,-649,-724,-791,-851,-903,-946,-979,-1004,-1019,
    -1023,-1019,-1004,-979,-946,-903,-851,-791,-724,-649,-568,-482,-391,-297,
    -199,-100,0,100,199,297,391,482,568,649,724,791,851,903,946,979,1004,1019
  };
  #endif

#elif USE_COS_LUT == 2
const Unit cosLUT[128] =
{
  1024,1022,1019,1012,1004,993,979,964,946,925,903,878,851,822,791,758,724,
  687,649,609,568,526,482,437,391,344,297,248,199,150,100,50,0,-50,-100,-150,
  -199,-248,-297,-344,-391,-437,-482,-526,-568,-609,-649,-687,-724,-758,-791,
  -822,-851,-878,-903,-925,-946,-964,-979,-993,-1004,-1012,-1019,-1022,-1023,
  -1022,-1019,-1012,-1004,-993,-979,-964,-946,-925,-903,-878,-851,-822,-791,
  -758,-724,-687,-649,-609,-568,-526,-482,-437,-391,-344,-297,-248,-199,-150,
  -100,-50,0,50,100,150,199,248,297,344,391,437,482,526,568,609,649,687,724,
  758,791,822,851,878,903,925,946,964,979,993,1004,1012,1019,1022
};
#endif

Unit cosInt(Unit input)
{
  profileCall(cosInt);

  // TODO: could be optimized with LUT

  input = wrap(input,UNITS_PER_SQUARE);

#if USE_COS_LUT == 1

  #ifdef RAYCAST_TINY
    return cosLUT[input];
  #else
    return cosLUT[input / 16];
  #endif

#elif USE_COS_LUT == 2
  return cosLUT[input / 8];
#else
  if (input < UNITS_PER_SQUARE / 4)
    return trigHelper(input);
  else if (input < UNITS_PER_SQUARE / 2)
    return -1 * trigHelper(UNITS_PER_SQUARE / 2 - input);
  else if (input < 3 * UNITS_PER_SQUARE / 4)
    return -1 * trigHelper(input - UNITS_PER_SQUARE / 2);
  else
    return trigHelper(UNITS_PER_SQUARE - input);
#endif
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

uint16_t sqrtInt(Unit value)
{
  profileCall(sqrtInt);

#ifdef RAYCAST_TINY
  uint16_t result = 0;
  uint16_t a = value;
  uint16_t b = 1u << 14;
#else
  uint32_t result = 0;
  uint32_t a = value;
  uint32_t b = 1u << 30;
#endif

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

  Unit dx = p2.x - p1.x;
  Unit dy = p2.y - p1.y;

#if USE_DIST_APPROX == 2
  // octagonal approximation

  dx = absVal(dx);
  dy = absVal(dy);

  return dy > dx ? dx / 2 + dy : dy / 2 + dx;
#elif USE_DIST_APPROX == 1
  // more accurate approximation

  Unit a, b, result;

  dx = dx < 0 ? -1 * dx : dx;
  dy = dy < 0 ? -1 * dy : dy;

  if (dx < dy)
  {
     a = dy;
     b = dx;
  }
  else
  {
     a = dx;
     b = dy;
  }

  result = a + (44 * b) / 102;

  if (a < (b << 4))
    result -= (5 * a) / 128;

  return result;

#else
  dx = dx * dx;
  dy = dy * dy;

  return sqrtInt((Unit) (dx + dy));
#endif
}

Unit len(Vector2D v)
{
  profileCall(len);

  Vector2D zero;
  zero.x = 0;
  zero.y = 0;

  return dist(zero,v);
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
void castRaySquare(Ray *localRay, Vector2D *nextCellOff, Vector2D *collOff)
{
  profileCall(castRaySquare);

  nextCellOff->x = 0;
  nextCellOff->y = 0;

  Ray criticalLine;
  criticalLine.start = localRay->start;
  criticalLine.direction = localRay->direction;

  #define helper(c1,c2,n)\
    {\
      nextCellOff->c1 = n;\
      collOff->c1 = criticalLine.start.c1 - localRay->start.c1;\
      collOff->c2 = \
        (((Unit) collOff->c1) * localRay->direction.c2) /\
        ((localRay->direction.c1 == 0) ? 1 : localRay->direction.c1);\
    }

  #define helper2(n1,n2,c)\
    if (pointIsLeftOfRay(localRay->start,criticalLine) == c)\
      helper(y,x,n1)\
    else\
      helper(x,y,n2)
      
  if (localRay->direction.x > 0)
  {
    criticalLine.start.x = UNITS_PER_SQUARE - 1;

    if (localRay->direction.y > 0)
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

    if (localRay->direction.y > 0)
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

void castRayMultiHit(Ray ray, ArrayFunction arrayFunc, ArrayFunction typeFunc, 
  HitResult *hitResults, uint16_t *hitResultsLen, RayConstraints constraints)
{
  profileCall(castRayMultiHit);

  Vector2D initialPos = ray.start;
  Vector2D currentPos = ray.start;

  Vector2D currentSquare;

  currentSquare.x = divRoundDown(ray.start.x, UNITS_PER_SQUARE);
  currentSquare.y = divRoundDown(ray.start.y,UNITS_PER_SQUARE);

  *hitResultsLen = 0;

  Unit squareType = arrayFunc(currentSquare.x,currentSquare.y);

  Vector2D no, co; // next cell offset, collision offset

  no.x = 0;        // just to supress a warning
  no.y = 0;
  co.x = 0;
  co.y = 0;

  for (uint16_t i = 0; i < constraints.maxSteps; ++i)
  {
    Unit currentType = arrayFunc(currentSquare.x,currentSquare.y);

    if (currentType != squareType)
    {
      // collision

      HitResult h;

      h.arrayValue = currentType;
      h.doorRoll = 0;
      h.position = currentPos;
      h.square   = currentSquare;
      h.distance = dist(initialPos,currentPos);

      if (typeFunc != 0)
        h.type = typeFunc(currentSquare.x,currentSquare.y);

      if (no.y > 0)
      {
        h.direction = 0;
        h.textureCoord = constraints.computeTextureCoords ?
          wrap(currentPos.x,UNITS_PER_SQUARE) : 0;
      }
      else if (no.x > 0)
      {
        h.direction = 1;
        h.textureCoord = constraints.computeTextureCoords ?
          wrap(UNITS_PER_SQUARE - currentPos.y,UNITS_PER_SQUARE) : 0;
      }
      else if (no.y < 0)
      {
        h.direction = 2;
        h.textureCoord = constraints.computeTextureCoords ?
          wrap(UNITS_PER_SQUARE - currentPos.x,UNITS_PER_SQUARE) : 0;
      }
      else
      {
        h.direction = 3;
        h.textureCoord = constraints.computeTextureCoords ?
          wrap(currentPos.y,UNITS_PER_SQUARE) : 0;
      }

      if (_rollFunction != 0)
        h.doorRoll = _rollFunction(currentSquare.x,currentSquare.y);

      hitResults[*hitResultsLen] = h;

      *hitResultsLen += 1;

      squareType = currentType;

      if (*hitResultsLen >= constraints.maxHits)
        break;
    }

    ray.start.x = wrap(currentPos.x,UNITS_PER_SQUARE);
    ray.start.y = wrap(currentPos.y,UNITS_PER_SQUARE);

    castRaySquare(&ray,&no,&co);

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
  uint16_t len;
  RayConstraints c;

  c.maxSteps = 1000;
  c.maxHits = 1;

  castRayMultiHit(ray,arrayFunc,0,&result,&len,c);

  if (len == 0)
    result.distance = -1;

  return result;
}

void castRaysMultiHit(Camera cam, ArrayFunction arrayFunc,
  ArrayFunction typeFunction, ColumnFunction columnFunc,
  RayConstraints constraints)
{
  Vector2D dir1 = angleToDirection(cam.direction - HORIZONTAL_FOV_HALF);
  Vector2D dir2 = angleToDirection(cam.direction + HORIZONTAL_FOV_HALF);

  Unit dX = dir2.x - dir1.x;
  Unit dY = dir2.y - dir1.y;

  HitResult hits[constraints.maxHits];
  uint16_t hitCount;

  Ray r;
  r.start = cam.position;

  Unit currentDX = 0;
  Unit currentDY = 0;

  for (int16_t i = 0; i < cam.resolution.x; ++i)
  {
    r.direction.x = dir1.x + currentDX / cam.resolution.x;
    r.direction.y = dir1.y + currentDY / cam.resolution.x;

    castRayMultiHit(r,arrayFunc,typeFunction,hits,&hitCount,constraints);

    columnFunc(hits,hitCount,i,r);

    currentDX += dX;
    currentDY += dY;
  }
}

/**
  Helper function that determines intersection with both ceiling and floor.
*/
Unit _floorCeilFunction(int16_t x, int16_t y)
{
  // TODO: adjust also for RAYCAST_TINY

  Unit f = _floorFunction(x,y);

  if (_ceilFunction == 0)
    return f;

  Unit c = _ceilFunction(x,y);

#ifndef RAYCAST_TINY
  return ((f & 0x0000ffff) << 16) | (c & 0x0000ffff);
#else
  return ((f & 0x00ff) << 8) | (c & 0x00ff);
#endif
}

Unit _floorHeightNotZeroFunction(int16_t x, int16_t y)
{
  return _floorFunction(x,y) == 0 ? 0 :
    (x & 0x00FF) | ((y & 0x00FF) << 8);
    // ^ this makes collisions between all squares - needed for rolling doors
}

Unit adjustDistance(Unit distance, Camera *camera, Ray *ray)
{
  /* FIXME/TODO: The adjusted (=orthogonal, camera-space) distance could
     possibly be computed more efficiently by not computing Euclidean
     distance at all, but rather compute the distance of the collision
     point from the projection plane (line). */

  Unit result =
    (distance *
     vectorsAngleCos(angleToDirection(camera->direction),ray->direction)) /
     UNITS_PER_SQUARE;

  return result == 0 ? 1 : result;
                // ^ prevent division by zero
}

void _columnFunction(HitResult *hits, uint16_t hitCount, uint16_t x, Ray ray)
{
  // last written Y position, can never go backwards
  Unit fPosY = _camera.resolution.y;
  Unit cPosY = -1;

  // world coordinates
  Unit fZ1World = _startFloorHeight;
  Unit cZ1World = _startCeilHeight;

  PixelInfo p;
  p.position.x = x;

  Unit i;

  // we'll be simulatenously drawing the floor and the ceiling now  
  for (Unit j = 0; j <= hitCount; ++j)
  {                          // ^ = add extra iteration for horizon plane
    int8_t drawingHorizon = j == hitCount;

    HitResult hit;
    Unit distance;

    Unit fWallHeight = 0, cWallHeight = 0;
    Unit fZ2World = 0,    cZ2World = 0;
    Unit fZ1Screen = 0,   cZ1Screen = 0;
    Unit fZ2Screen = 0,   cZ2Screen = 0;

    if (!drawingHorizon)
    {
      hit = hits[j];
      distance = adjustDistance(hit.distance,&_camera,&ray);

      fWallHeight = _floorFunction(hit.square.x,hit.square.y);
      fZ2World = fWallHeight - _camera.height;
      fZ1Screen = _middleRow - perspectiveScale(
        (fZ1World * _camera.resolution.y) / UNITS_PER_SQUARE,distance);
      fZ2Screen = _middleRow - perspectiveScale(
        (fZ2World * _camera.resolution.y) / UNITS_PER_SQUARE,distance);

      if (_ceilFunction != 0)
      {
        cWallHeight = _ceilFunction(hit.square.x,hit.square.y);
        cZ2World = cWallHeight - _camera.height;
        cZ1Screen = _middleRow - perspectiveScale(
          (cZ1World * _camera.resolution.y) / UNITS_PER_SQUARE,distance);
        cZ2Screen = _middleRow - perspectiveScale(
          (cZ2World * _camera.resolution.y) / UNITS_PER_SQUARE,distance);
      }
    }
    else
    {
      fZ1Screen = _middleRow;
      cZ1Screen = _middleRow + 1;
    }

    Unit limit;

    #define VERTICAL_DEPTH_MULTIPLY 2

    #define drawHorizontal(pref,l1,l2,comp,inc)\
      p.depth += absVal(pref##Z1World) * VERTICAL_DEPTH_MULTIPLY;\
      limit = clamp(pref##Z1Screen,l1,l2);\
      for (i = pref##PosY inc 1; i comp##= limit; inc##inc i)\
      {\
        p.position.y = i;\
        p.depth += _horizontalDepthStep;\
        _pixelFunction(&p);\
      }\
      if (pref##PosY comp limit)\
        pref##PosY = limit;

    p.isWall = 0;
    p.isHorizon = drawingHorizon;

    // draw floor until wall
    p.isFloor = 1;
    p.depth = (_fHorizontalDepthStart - fPosY) * _horizontalDepthStep;
    drawHorizontal(f,cPosY + 1,_camera.resolution.y,>,-)
                    // ^ purposfully allow outside screen bounds here

    if (_ceilFunction != 0 || drawingHorizon)
    {
      // draw ceiling until wall
      p.isFloor = 0;
      p.depth = (cPosY - _cHorizontalDepthStart) * _horizontalDepthStep;
      drawHorizontal(c,-1,fPosY - 1,<,+)
                    // ^ purposfully allow outside screen bounds here
    }

    #undef drawHorizontal
    #undef VERTICAL_DEPTH_MULTIPLY

    if (!drawingHorizon) // don't draw walls for horizon plane
    {
      #define drawVertical(pref,l1,l2,comp,inc)\
        {\
          limit = clamp(pref##Z2Screen,l1,l2);\
          Unit wallLength = pref##Z2Screen - pref##Z1Screen - 1;\
          wallLength = wallLength != 0 ? wallLength : 1;\
          Unit wallPosition = absVal(pref##Z1Screen - pref##PosY) inc (-1);\
          for (i = pref##PosY inc 1; i comp##= limit; inc##inc i)\
          {\
            p.position.y = i;\
            p.hit = hit;\
            p.textureCoordY = (wallPosition * UNITS_PER_SQUARE) / wallLength; \
            wallPosition++;\
            _pixelFunction(&p);\
          }\
          if (pref##PosY comp limit)\
            pref##PosY = limit;\
          pref##Z1World = pref##Z2World; /* for the next iteration */\
        }

      p.isWall = 1;
      p.depth = distance;
      p.isFloor = 1;

      // draw floor wall

      if (fPosY > 0) // still pixels left?
      {
        p.isFloor = 1;
        drawVertical(f,cPosY + 1,_camera.resolution.y,>,-)
      }               // ^ purposfully allow outside screen bounds here

      // draw ceiling wall

      if (_ceilFunction != 0 && cPosY < _camResYLimit) // still pixels left?
      {
        p.isFloor = 0;
        drawVertical(c,-1,fPosY - 1,<,+)
      }             // ^ puposfully allow outside screen bounds here 

      #undef drawVertical
    }
  }
}

void _columnFunctionSimple(HitResult *hits, uint16_t hitCount, uint16_t x,
  Ray ray)
{
  int16_t y = 0;
  int16_t wallHeightScreen = 0;
  int16_t coordHelper = 0;
  int16_t wallStart = _middleRow;
  int16_t wallEnd = _middleRow;
  int16_t heightOffset = 0;

  Unit dist = 1;

  PixelInfo p;
  p.position.x = x;

  if (hitCount > 0)
  {
    HitResult hit = hits[0];

    uint8_t goOn = 1;

    if (_rollFunction != 0)
    {
      if (hit.arrayValue == 0)
      {
        // standing inside door square, looking out => move to the next hit

        if (hitCount > 1)
          hit = hits[1];
        else
          goOn = 0;
      }
      else
      {
        // normal hit, check the door roll

        int8_t unrolled = hit.doorRoll >= 0 ?
          hit.doorRoll > hit.textureCoord :
          hit.textureCoord > UNITS_PER_SQUARE + hit.doorRoll;

        if (unrolled)
        {
          goOn = 0;

          if (hitCount > 1) /* should probably always be true (hit on square
                               exit) */
          {
            if (hit.direction % 2 != hits[1].direction % 2)
            {
              // hit on the inner side
              hit = hits[1];
              goOn = 1;
            }
            else if (hitCount > 2)
            {
              // hit on the opposite side
              hit = hits[2];
              goOn = 1;
            }
          }
        }
      }
    }

    p.hit = hit;

    if (goOn)
    {
      dist = adjustDistance(hit.distance,&_camera,&ray);

      int16_t wallHeightWorld = _floorFunction(hit.square.x,hit.square.y);

      wallHeightScreen = perspectiveScale((wallHeightWorld *
        _camera.resolution.y) / UNITS_PER_SQUARE,dist);

      int16_t normalizedWallHeight = wallHeightWorld != 0 ?
        ((UNITS_PER_SQUARE * wallHeightScreen) / wallHeightWorld) : 0;

      heightOffset = perspectiveScale(_cameraHeightScreen,dist);

      wallStart = _middleRow - wallHeightScreen + heightOffset +
                  normalizedWallHeight;

      coordHelper = -1 * wallStart;
      coordHelper = coordHelper >= 0 ? coordHelper : 0;

      wallEnd = clamp(wallStart + wallHeightScreen,0,_camResYLimit);
      wallStart = clamp(wallStart,0,_camResYLimit);
    }
  }

  // draw ceiling

  p.isWall = 0;
  p.isFloor = 0;
  p.isHorizon = 1;
  p.depth = 1;

  while (y < wallStart)
  {
    p.position.y = y;
    _pixelFunction(&p);
    ++y;
    p.depth += _horizontalDepthStep;
  }

  // draw wall

  p.isWall = 1;
  p.isFloor = 1;
  p.depth = dist;

#if ROLL_TEXTURE_COORDS == 1 
  p.hit.textureCoord -= p.hit.doorRoll;
#endif

  while (y < wallEnd)
  {
    p.position.y = y;

    if (_computeTextureCoords)
      p.textureCoordY = (coordHelper * UNITS_PER_SQUARE) / wallHeightScreen;

    _pixelFunction(&p);

    ++y;
    ++coordHelper;
  }

  // draw floor

  p.isWall = 0;
  p.depth = _middleRow * _horizontalDepthStep;

  while (y < _camera.resolution.y)
  {
    p.position.y = y;
    _pixelFunction(&p);
    ++y;
    p.depth -= _horizontalDepthStep;
  }
}

void render(Camera cam, ArrayFunction floorHeightFunc,
  ArrayFunction ceilingHeightFunc, ArrayFunction typeFunction,
  PixelFunction pixelFunc, RayConstraints constraints)
{
  _pixelFunction = pixelFunc;
  _floorFunction = floorHeightFunc;
  _ceilFunction = ceilingHeightFunc;
  _camera = cam;
  _camResYLimit = cam.resolution.y - 1;

  int16_t halfResY = cam.resolution.y / 2;

  _middleRow = halfResY + cam.shear;

  _fHorizontalDepthStart = _middleRow + halfResY;
  _cHorizontalDepthStart = _middleRow - halfResY;

  _computeTextureCoords = constraints.computeTextureCoords;

  _startFloorHeight = floorHeightFunc(
    divRoundDown(cam.position.x,UNITS_PER_SQUARE),
    divRoundDown(cam.position.y,UNITS_PER_SQUARE)) -1 * cam.height;

  _startCeilHeight = 
    ceilingHeightFunc != 0 ?
      ceilingHeightFunc(
        divRoundDown(cam.position.x,UNITS_PER_SQUARE),
        divRoundDown(cam.position.y,UNITS_PER_SQUARE)) -1 * cam.height
      : UNIT_INFINITY;

  // TODO
  _horizontalDepthStep = (12 * UNITS_PER_SQUARE) / cam.resolution.y; 

  castRaysMultiHit(cam,_floorCeilFunction,typeFunction,
    _columnFunction,constraints);
}

void renderSimple(Camera cam, ArrayFunction floorHeightFunc,
  ArrayFunction typeFunc, PixelFunction pixelFunc, ArrayFunction rollFunc,
  RayConstraints constraints)
{
  _pixelFunction = pixelFunc;
  _floorFunction = floorHeightFunc;
  _camera = cam;
  _camResYLimit = cam.resolution.y - 1;
  _middleRow = cam.resolution.y / 2;
  _computeTextureCoords = constraints.computeTextureCoords;
  _rollFunction = rollFunc;

  _cameraHeightScreen =
    (_camera.resolution.y * (_camera.height - UNITS_PER_SQUARE)) /
    UNITS_PER_SQUARE;

  // TODO
  _horizontalDepthStep = (12 * UNITS_PER_SQUARE) / cam.resolution.y; 

  constraints.maxHits = 

  _rollFunction == 0 ?
    1 : // no door => 1 hit is enough 
    3;  // for correctly rendering rolling doors we'll need 3 hits (NOT 2)

  castRaysMultiHit(cam,_floorHeightNotZeroFunction,typeFunc,
    _columnFunctionSimple, constraints);
}

Vector2D normalize(Vector2D v)
{
  profileCall(normalize);

  Vector2D result;

  Unit l = len(v);

  l = l != 0 ? l : 1;

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

PixelInfo mapToScreen(Vector2D worldPosition, Unit height, Camera camera)
{
  // TODO: precompute some stuff that's constant in the frame

  PixelInfo result;

  Unit d = dist(worldPosition,camera.position);

  Vector2D toPoint;

  toPoint.x = worldPosition.x - camera.position.x;
  toPoint.y = worldPosition.y - camera.position.y;

  Vector2D cameraDir = angleToDirection(camera.direction);

  result.depth = // adjusted distance
    (d * vectorsAngleCos(cameraDir,toPoint)) / UNITS_PER_SQUARE;

  result.position.y = camera.resolution.y / 2 -
    (camera.resolution.y *
     perspectiveScale(height - camera.height,result.depth)) / UNITS_PER_SQUARE
    + camera.shear;

  Unit middleColumn = camera.resolution.x / 2;

  Unit a = sqrtInt(d * d - result.depth * result.depth);

  Ray r;
  r.start = camera.position;
  r.direction = cameraDir;

  if (!pointIsLeftOfRay(worldPosition,r))
    a *= -1;

  Unit cos = cosInt(HORIZONTAL_FOV_HALF);

  Unit b =
    (result.depth * sinInt(HORIZONTAL_FOV_HALF)) / (cos == 0 ? 1 : cos);
    // sin/cos = tan

  result.position.x = (a * middleColumn) / (b == 0 ? 1 : b);
  result.position.x = middleColumn - result.position.x;

  return result;
}

Unit degreesToUnitsAngle(int16_t degrees)
{
  return (degrees * UNITS_PER_SQUARE) / 360;
}

Unit perspectiveScale(Unit originalSize, Unit distance)
{
  profileCall(perspectiveScale);

  return distance != 0 ?
   (originalSize * UNITS_PER_SQUARE) /
      ((VERTICAL_FOV * 2 * distance) / UNITS_PER_SQUARE)
   : 0;
}

void moveCameraWithCollision(Camera *camera, Vector2D planeOffset,
  Unit heightOffset, ArrayFunction floorHeightFunc,
  ArrayFunction ceilingHeightFunc, int8_t computeHeight, int8_t force)
{
  // TODO: have the cam coll parameters precomputed as macros? => faster

  int8_t movesInPlane = planeOffset.x != 0 || planeOffset.y != 0;
  int16_t xSquareNew, ySquareNew;

  if (movesInPlane || force)
  {
    Vector2D corner; // BBox corner in the movement direction
    Vector2D cornerNew;

    int16_t xDir = planeOffset.x > 0 ? 1 : (planeOffset.x < 0 ? -1 : 0);
    int16_t yDir = planeOffset.y > 0 ? 1 : (planeOffset.y < 0 ? -1 : 0);

    corner.x = camera->position.x + xDir * CAMERA_COLL_RADIUS;
    corner.y = camera->position.y + yDir * CAMERA_COLL_RADIUS;

    int16_t xSquare = divRoundDown(corner.x,UNITS_PER_SQUARE);
    int16_t ySquare = divRoundDown(corner.y,UNITS_PER_SQUARE);

    cornerNew.x = corner.x + planeOffset.x;
    cornerNew.y = corner.y + planeOffset.y;

    xSquareNew = divRoundDown(cornerNew.x,UNITS_PER_SQUARE);
    ySquareNew = divRoundDown(cornerNew.y,UNITS_PER_SQUARE);

    Unit bottomLimit = camera->height - CAMERA_COLL_HEIGHT_BELOW +
      CAMERA_COLL_STEP_HEIGHT;
    Unit topLimit = camera->height + CAMERA_COLL_HEIGHT_ABOVE;

    // checks a single square for collision against the camera
    #define collCheck(dir,s1,s2)\
    if (computeHeight)\
    {\
      Unit height = floorHeightFunc(s1,s2);\
      if (height > bottomLimit)\
        dir##Collides = 1;\
      else if (ceilingHeightFunc != 0)\
      {\
        height = ceilingHeightFunc(s1,s2);\
        if (height < topLimit)\
          dir##Collides = 1;\
      }\
    }\
    else\
      dir##Collides = floorHeightFunc(s1,s2) > CAMERA_COLL_STEP_HEIGHT;

    // check a collision against non-diagonal square
    #define collCheckOrtho(dir,dir2,s1,s2,x)\
    if (dir##SquareNew != dir##Square)\
    {\
      collCheck(dir,s1,s2)\
    }\
    if (!dir##Collides)\
    { /* now also check for coll on the neighbouring square */ \
      int16_t dir2##Square2 = divRoundDown(corner.dir2 - dir2##Dir *\
        CAMERA_COLL_RADIUS * 2,UNITS_PER_SQUARE);\
      if (dir2##Square2 != dir2##Square)\
      {\
        if (x)\
          collCheck(dir,dir##SquareNew,dir2##Square2)\
        else\
          collCheck(dir,dir2##Square2,dir##SquareNew)\
      }\
    }

    int8_t xCollides = 0;
    collCheckOrtho(x,y,xSquareNew,ySquare,1)

    int8_t yCollides = 0;
    collCheckOrtho(y,x,xSquare,ySquareNew,0)

    #define collHandle(dir)\
    if (dir##Collides)\
      cornerNew.dir = (dir##Square) * UNITS_PER_SQUARE + UNITS_PER_SQUARE / 2\
      + dir##Dir * (UNITS_PER_SQUARE / 2) - dir##Dir;\

    if (!xCollides && !yCollides) /* if non-diagonal collision happend, corner
                                     collision can't happen */
    {
      if (xSquare != xSquareNew && ySquare != ySquareNew) // corner?
      {
        int8_t xyCollides = 0;
        collCheck(xy,xSquareNew,ySquareNew)
        
        if (xyCollides)
        {
          // normally should slide, but let's KISS
          cornerNew = corner;
        }
      }
    }

    collHandle(x)
    collHandle(y)

    #undef collCheck
    #undef collHandle

    camera->position.x = cornerNew.x - xDir * CAMERA_COLL_RADIUS;
    camera->position.y = cornerNew.y - yDir * CAMERA_COLL_RADIUS;
  }

  if (computeHeight && (movesInPlane || heightOffset != 0 || force))
  {
    camera->height += heightOffset;

    int16_t xSquare1 =
      divRoundDown(camera->position.x - CAMERA_COLL_RADIUS,UNITS_PER_SQUARE);
    int16_t xSquare2 =
      divRoundDown(camera->position.x + CAMERA_COLL_RADIUS,UNITS_PER_SQUARE);
    int16_t ySquare1 =
      divRoundDown(camera->position.y - CAMERA_COLL_RADIUS,UNITS_PER_SQUARE);
    int16_t ySquare2 =
      divRoundDown(camera->position.y + CAMERA_COLL_RADIUS,UNITS_PER_SQUARE);

    Unit bottomLimit = floorHeightFunc(xSquare1,ySquare1);
    Unit topLimit = ceilingHeightFunc != 0 ?
      ceilingHeightFunc(xSquare1,ySquare1) : UNIT_INFINITY;

    Unit height;

    #define checkSquares(s1,s2)\
    {\
      height = floorHeightFunc(xSquare##s1,ySquare##s2);\
      bottomLimit = bottomLimit < height ? height : bottomLimit;\
      height = ceilingHeightFunc != 0 ?\
        ceilingHeightFunc(xSquare##s1,ySquare##s2) : UNIT_INFINITY;\
      topLimit = topLimit > height ? height : topLimit;\
    }

    if (xSquare2 != xSquare1)
      checkSquares(2,1)

    if (ySquare2 != ySquare1)
      checkSquares(1,2)

    if (xSquare2 != xSquare1 && ySquare2 != ySquare1)
      checkSquares(2,2)

    camera->height = clamp(camera->height,
      bottomLimit + CAMERA_COLL_HEIGHT_BELOW,
      topLimit - CAMERA_COLL_HEIGHT_ABOVE);
    #undef checkSquares
  }
}

void initCamera(Camera *camera)
{
  camera->position.x = 0;
  camera->position.y = 0;
  camera->direction = 0;
  camera->resolution.x = 20;
  camera->resolution.y = 15;
  camera->shear = 0;
  camera->height = UNITS_PER_SQUARE;
}

void initRayConstraints(RayConstraints *constraints)
{
  constraints->maxHits = 1;
  constraints->maxSteps = 20;
  constraints->computeTextureCoords = 1;
}

#endif
