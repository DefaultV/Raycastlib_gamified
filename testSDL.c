#include <SDL2/SDL.h>
#include "raycastlib.h"
#include <stdint.h>
#include <stdio.h>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

uint32_t *pixels[SCREEN_WIDTH * SCREEN_HEIGHT];

#define KEYS 6
#define KEY_UP 0
#define KEY_RIGHT 1
#define KEY_DOWN 2
#define KEY_LEFT 3
#define KEY_Q 4
#define KEY_W 5

int keys[KEYS];

Camera camera;

#define RGB(r,g,b) (0 | ((r << 16) | (g << 8) | (b)))

Unit heightAt(int16_t x, int16_t y)
{
  if (x > 12 || y > 12)
    return x * y * UNITS_PER_SQUARE;

  return UNITS_PER_SQUARE * ((x < 0 || y < 0 || x > 9 || y > 9) ? 1 : 0);
}

void drawPixel(PixelInfo *pixel)
{
  uint32_t color = RGB(0,255,255);

  if (pixel->isWall)
    color = 0;

  pixels[pixel->position.y * SCREEN_WIDTH / 2 + pixel->position.x] = color;
}

void renderFrame()
{
  RayConstraints constraints;

  constraints.maxHits = 10;
  constraints.maxSteps = 20;
 
  render(camera,heightAt,0,0,drawPixel,constraints);
}

int main()
{
camera.position.x = 2 * UNITS_PER_SQUARE;
camera.position.y = 4 * UNITS_PER_SQUARE;
camera.height = UNITS_PER_SQUARE;
camera.shear = 0;
camera.direction = 0;
camera.resolution.x = 200;//SCREEN_WIDTH;
camera.resolution.y = 150;//SCREEN_HEIGHT;

  for (int i = 0; i < KEYS; ++i)
    keys[i] = 0;

  SDL_Window *window = SDL_CreateWindow("raycasting", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN); 
  SDL_Renderer *renderer = SDL_CreateRenderer(window,-1,0);
  SDL_Texture *texture = SDL_CreateTexture(renderer,SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, SCREEN_WIDTH, SCREEN_HEIGHT);
  SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
  SDL_Event event;
  int running = 1;

  while (running)
  {
    renderFrame();

    SDL_UpdateTexture(texture,NULL,pixels,SCREEN_WIDTH * sizeof(uint32_t));

    while (SDL_PollEvent(&event))
    {
      int newState = 0;
      int keyIndex = -1;

      switch (event.type)
      {
        case SDL_KEYDOWN:
          newState = 1;
        case SDL_KEYUP:
          switch (event.key.keysym.scancode)
          {
            case SDL_SCANCODE_ESCAPE: running = 0; break;
            case SDL_SCANCODE_UP: keyIndex = KEY_UP; break;
            case SDL_SCANCODE_RIGHT: keyIndex = KEY_RIGHT; break;
            case SDL_SCANCODE_DOWN: keyIndex = KEY_DOWN; break;
            case SDL_SCANCODE_LEFT: keyIndex = KEY_LEFT; break;
            case SDL_SCANCODE_Q: keyIndex = KEY_Q; break;
            case SDL_SCANCODE_W: keyIndex = KEY_W; break;
            default: break;
          }
          break;

        case SDL_QUIT:
          running = 0;
          break;

        default:
          break;
      }

      if (keyIndex >= 0)
        keys[keyIndex] = newState;
    }

    int step = 1;
    int step2 = 1;

    Vector2D direction = angleToDirection(camera.direction);

    direction.x /= 10;
    direction.y /= 10;

    if (keys[KEY_UP])
    {
      camera.position.x += step * direction.x;
      camera.position.y += step * direction.y;
    }
    else if (keys[KEY_DOWN])
    {
      camera.position.x -= step * direction.x;
      camera.position.y -= step * direction.y;
    }

    if (keys[KEY_Q])
      camera.height += step * 10;
    else if (keys[KEY_W])
      camera.height -= step * 10;

    if (keys[KEY_RIGHT])
      camera.direction += step2;
    else if (keys[KEY_LEFT])
      camera.direction -= step2;

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer,texture,NULL,NULL);
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(render);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
