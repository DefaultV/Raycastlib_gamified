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

#include <stdint.h>

#ifndef RAYCAST_TINY
  #define UNITS_PER_SQUARE 1024 ///< N. of Units in a side of a spatial square.
  typedef int32_t Unit; /**< Smallest spatial unit, there is UNITS_PER_SQUARE
                             units in a square's length. This effectively
                             serves the purpose of a fixed-point arithmetic. */
  #define UNIT_INFINITY 5000000;

  typedef int32_t int_maybe32_t;
  typedef uint32_t uint_maybe32_t;
#else
  #define UNITS_PER_SQUARE 64
  typedef int16_t Unit;
  #define UNIT_INFINITY 32767;
  typedef int16_t int_maybe32_t;
  typedef uint16_t uint_maybe32_t;
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
  Vector2D square;       ///< Collided square coordinates.
  Vector2D position;     ///< Exact collision position in Units.
  Unit     distance;     /**< Euclidean distance to the hit position, or -1 if
                            no collision happened. */
  Unit     textureCoord; ///< Normalized (0 to UNITS_PER_SQUARE - 1) tex coord.
  Unit     type;         ///< Integer identifying type of square.
  uint8_t  direction;    ///< Direction of hit.
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
typedef void (*PixelFunction)(PixelInfo info);

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

uint16_t sqrtInt(uint_maybe32_t value);
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
*/
void moveCameraWithCollision(Camera *camera, Vector2D planeOffset,
  Unit heightOffset, ArrayFunction floorHeightFunc,
  ArrayFunction ceilingHeightFunc, int8_t computeHeight);

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

inline Unit absVal(Unit value)
{
  profileCall(absVal);

  return value >= 0 ? value: -1 * value;
}

/// Like mod, but behaves differently for negative values.
inline Unit wrap(Unit value, Unit mod)
{
  profileCall(wrap);

  return value >= 0 ? (value % mod) : (mod + (value % mod) - 1);
}

/// Performs division, rounding down, NOT towards zero.
inline Unit divRoundDown(Unit value, Unit divisor)
{
  profileCall(divRoundDown);

  return value / divisor - ((value >= 0) ? 0 : 1);
}

// Bhaskara's cosine approximation formula
#define trigHelper(x) (((Unit) UNITS_PER_SQUARE) *\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 - 4 * (x) * (x)) /\
  (UNITS_PER_SQUARE / 2 * UNITS_PER_SQUARE / 2 + (x) * (x)))

Unit cosInt(Unit input)
{
  profileCall(cosInt);

  // TODO: could be optimized with LUT

  input = wrap(input,UNITS_PER_SQUARE);

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

uint16_t sqrtInt(uint_maybe32_t value)
{
  profileCall(sqrtInt);

  uint_maybe32_t result = 0;
  uint_maybe32_t a = value;

#ifdef RAYCAST_TINY
  uint_maybe32_t b = 1u << 14;
#else
  uint_maybe32_t b = 1u << 30;
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

  int_maybe32_t dx = p2.x - p1.x;
  int_maybe32_t dy = p2.y - p1.y;

  dx = dx * dx;
  dy = dy * dy;

  return sqrtInt((uint_maybe32_t) (dx + dy));
}

Unit len(Vector2D v)
{
  profileCall(len);

  v.x *= v.x;
  v.y *= v.y;

  return sqrtInt(((uint_maybe32_t) v.x) + ((uint_maybe32_t) v.y));
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
        (((int_maybe32_t) collOff->c1) * localRay.direction.c2) /\
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

      hitResults[*hitResultsLen] = h;

      *hitResultsLen += 1;

      squareType = currentType;

      if (*hitResultsLen >= constraints.maxHits)
        break;
    }

    ray.start.x = wrap(currentPos.x,UNITS_PER_SQUARE);
    ray.start.y = wrap(currentPos.y,UNITS_PER_SQUARE);

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

  for (uint16_t i = 0; i < cam.resolution.x; ++i)
  {
    r.direction.x = dir1.x + (dX * i) / cam.resolution.x;
    r.direction.y = dir1.y + (dY * i) / cam.resolution.x;

    castRayMultiHit(r,arrayFunc,typeFunction,hits,&hitCount,constraints);

    columnFunc(hits,hitCount,i,r);
  }
}

// global helper variables, for precomputing stuff etc.
PixelFunction _pixelFunction = 0;
Camera _camera;
Unit _floorDepthStep = 0; 
Unit _startFloorHeight = 0;
Unit _startCeilHeight = 0;
int_maybe32_t _camResYLimit = 0;
Unit _middleRow = 0;
ArrayFunction _floorFunction = 0;
ArrayFunction _ceilFunction = 0;
uint8_t _computeTextureCoords = 0;
Unit _fogStartYBottom = 0;
Unit _fogStartYTop = 0;

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

  return ((f & 0x0000ffff) << 16) | (c & 0x0000ffff);
}

void _columnFunction(HitResult *hits, uint16_t hitCount, uint16_t x, Ray ray)
{
  int_maybe32_t y = _camResYLimit; // screen y (for floor), will only go up
  int_maybe32_t y2 = 0;            // screen y (for ceil), will only fo down

  Unit worldZPrev = _startFloorHeight;
  Unit worldZPrevCeil = _startCeilHeight;

  PixelInfo p;
  p.position.x = x;

  #define VERTICAL_DEPTH_MULTIPLY 2

  // we'll be simulatenously drawing the floor and the ceiling now  
  for (uint_maybe32_t j = 0; j < hitCount; ++j)
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

    Unit wallHeight = _floorFunction(hit.square.x,hit.square.y);

    Unit worldZ2 = wallHeight - _camera.height;

    int_maybe32_t z1Screen = _middleRow - perspectiveScale(
      (worldZPrev * _camera.resolution.y) / UNITS_PER_SQUARE,dist);

    int_maybe32_t z1ScreenNoClamp = z1Screen;

    z1Screen = clamp(z1Screen,0,_camResYLimit);
    z1Screen = z1Screen > y2 ? z1Screen : y2;

    int_maybe32_t z2Screen = _middleRow - perspectiveScale(
      (worldZ2 * _camera.resolution.y) / UNITS_PER_SQUARE,dist);

    int_maybe32_t wallScreenHeightNoClamp = z2Screen - z1ScreenNoClamp;

    wallScreenHeightNoClamp = wallScreenHeightNoClamp != 0 ?
      wallScreenHeightNoClamp : 1;

    z2Screen = clamp(z2Screen,0,_camResYLimit);
    z2Screen = z2Screen > y2 ? z2Screen : y2;

    // make the same variables for ceiling

    Unit wallHeightCeil = 0;
    Unit worldZ2Ceil = 0;
    int_maybe32_t z1ScreenCeil = 0;
    int_maybe32_t z1ScreenCeilNoClamp = 0; 
    int_maybe32_t z2ScreenCeil = 0;

    if (_ceilFunction != 0)
    {
      wallHeightCeil = _ceilFunction != 0 ?
        _ceilFunction(hit.square.x,hit.square.y) : 0;

      worldZ2Ceil = wallHeightCeil - _camera.height;

      z1ScreenCeil = _middleRow - perspectiveScale(
        (worldZPrevCeil * _camera.resolution.y) / UNITS_PER_SQUARE,dist);

      z1ScreenCeilNoClamp = z1ScreenCeil;

      z1ScreenCeil = clamp(z1ScreenCeil,0,_camResYLimit);
      z1ScreenCeil = z1ScreenCeil < y ? z1ScreenCeil : y;

      z2ScreenCeil = _middleRow - perspectiveScale(
        (worldZ2Ceil * _camera.resolution.y) / UNITS_PER_SQUARE,dist);
    }

    int_maybe32_t wallScreenHeightCeilNoClamp =
      z2ScreenCeil - z1ScreenCeilNoClamp;

    wallScreenHeightCeilNoClamp = wallScreenHeightCeilNoClamp != 0 ?
      wallScreenHeightCeilNoClamp : 1;

    z2ScreenCeil = clamp(z2ScreenCeil,0,_camResYLimit);
    z2ScreenCeil = z2ScreenCeil < y ? z2ScreenCeil : y;

    int_maybe32_t zTop = z1Screen < z2Screen ? z1Screen : z2Screen;

    int_maybe32_t zBottomCeil = z1ScreenCeil > z2ScreenCeil ?
      z1ScreenCeil : z2ScreenCeil;

    if (zTop <= zBottomCeil)
      zBottomCeil = zTop; // walls on ceiling and floor met

    // draw floor until wall

    p.isWall = 0;
    p.isFloor = 1;

    Unit floorCameraDiff = absVal(worldZPrev) * VERTICAL_DEPTH_MULTIPLY;

    for (int_maybe32_t i = y; i > z1Screen; --i)
    {
      p.position.y = i;
      p.depth = (_fogStartYBottom - i) * _floorDepthStep + floorCameraDiff;
      _pixelFunction(p);  
    }

    if (z1Screen < y)
      y = z1Screen;

    // draw ceiling until wall

    p.isFloor = 0;

    if (_ceilFunction != 0)
    {
      Unit ceilCameraDiff = absVal(worldZPrevCeil) * VERTICAL_DEPTH_MULTIPLY;

      for (int_maybe32_t i = y2; i < z1ScreenCeil; ++i)
      {
        p.position.y = i;
        p.depth = (i - _fogStartYTop) * _floorDepthStep + ceilCameraDiff;
        _pixelFunction(p);  
      }
    }

    if (z1ScreenCeil > y2)
      y2 = z1ScreenCeil;

    // draw floor wall

    p.isWall = 1;
    p.depth = dist;
    p.isFloor = 1;

    int_maybe32_t iTo = y2 < zTop ? zTop : y2;

    for (int_maybe32_t i = z1Screen < y ? z1Screen : y; i >= iTo; --i)
    {
      p.position.y = i;
      p.hit = hit;

      if (_computeTextureCoords)
        p.textureCoordY = UNITS_PER_SQUARE - 1 -((i - z1ScreenNoClamp) *
          UNITS_PER_SQUARE) / wallScreenHeightNoClamp;

      _pixelFunction(p);
    }

    // draw ceiling wall

    p.isFloor = 0;

    if (_ceilFunction != 0)
    {
      iTo = y > zBottomCeil ? zBottomCeil : y;

      for (int_maybe32_t i = z1ScreenCeil > y2 ? z1ScreenCeil : y2; i <= iTo;
        ++i)
      {
        p.position.y = i;
        p.hit = hit;

        if (_computeTextureCoords)
          p.textureCoordY = ((i - z1ScreenCeilNoClamp) *
            UNITS_PER_SQUARE) / wallScreenHeightCeilNoClamp;

        _pixelFunction(p);
      }
    }

    y = y > zTop ? zTop : y;
    worldZPrev = worldZ2;

    y2 = y2 < zBottomCeil ? zBottomCeil : y2;
    worldZPrevCeil = worldZ2Ceil;

    if (y <= y2)
      break; // walls on ceiling and floor met
  }

  // draw floor until horizon

  p.isWall = 0;
  p.isFloor = 1;

  Unit floorCameraDiff = absVal(worldZPrev) * VERTICAL_DEPTH_MULTIPLY;
  Unit horizon = (y2 < _middleRow || _ceilFunction == 0) ? _middleRow : y2;

  for (int_maybe32_t i = y; i >= horizon + (horizon > y2 ? 0 : 1); --i)
  {
    p.position.y = i;
    p.depth = (_fogStartYBottom - i) * _floorDepthStep + floorCameraDiff;
    _pixelFunction(p);
  }

  // draw ceiling until horizon

  p.isFloor = 0;

  Unit ceilCameraDiff = 
    _ceilFunction != 0 ?
    absVal(worldZPrevCeil) * VERTICAL_DEPTH_MULTIPLY : UNITS_PER_SQUARE;

  horizon = y > _middleRow ? _middleRow : y;

  for (int_maybe32_t i = y2; i < horizon; ++i)
  {
    p.position.y = i;
    p.depth = (i - _fogStartYTop) * _floorDepthStep + ceilCameraDiff;
    _pixelFunction(p);
  }

  #undef VERTICAL_DEPTH_MULTIPLY
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

  _fogStartYBottom = _middleRow + halfResY;
  _fogStartYTop = _middleRow - halfResY;

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
  _floorDepthStep = (12 * UNITS_PER_SQUARE) / cam.resolution.y; 

  castRaysMultiHit(cam,_floorCeilFunction,typeFunction,
    _columnFunction,constraints);
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
    (result.depth * sinInt(HORIZONTAL_FOV_HALF)) / (cos == 0 ? 1 : cos); // sin/cos = tan

  result.position.x = (a * middleColumn) / b;
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
  ArrayFunction ceilingHeightFunc, int8_t computeHeight)
{
  // TODO: have the cam coll parameters precomputed as macros? => faster

  int8_t movesInPlane = planeOffset.x != 0 || planeOffset.y != 0;
  int16_t xSquareNew, ySquareNew;

  if (movesInPlane)
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
        dir##Collides = true;\
      else if (ceilingHeightFunc != 0)\
      {\
        height = ceilingHeightFunc(s1,s2);\
        if (height < topLimit)\
          dir##Collides = true;\
      }\
    }\
    else\
      dir##Collides = floorHeightFunc(s1,s2) > CAMERA_COLL_STEP_HEIGHT;

    // check a collision against non-diagonal square
    #define collCheckOrtho(dir,dir2,s1,s2,x)\
    if (dir##SquareNew != dir##Square)\
      collCheck(dir,s1,s2)\
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

    int8_t xCollides = false;
    collCheckOrtho(x,y,xSquareNew,ySquare,1)

    int8_t yCollides = false;
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
        int8_t xyCollides = false;
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

  if (computeHeight && (movesInPlane || heightOffset != 0))
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

#endif
