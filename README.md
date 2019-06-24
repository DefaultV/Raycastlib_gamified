# raycastlib

Ray casting library for (not only) limited-resource computers.

If you like this, you may also like my similar project: [small3dlib](https://gitlab.com/drummyfish/small3dlib).

eye-candy previews
------------------

Pokitto (32bit embedded console, 48 MHz, 36 kB RAM):

![](/media/pokitto1.gif)
![](/media/pokitto2.gif)
![](/media/pokitto3.gif)

SDL (PC):

![](/media/sdl.gif)

Arduboy (8bit Arduino console, 16 MHz, 2.5 kB RAM):

![](/media/arduboy.gif)
![](/media/arduboy2.gif)

terminal:

![](/media/terminal.gif)

Gamebuino META (Arduino 32bit console, 48 MHz, 32 kB RAM):

![](/media/gamebuino.gif)

features
--------

- Very fast, small and efficient.
- Uses only integer math (32bit).
- No dependencies (uses only stdint standard library), extremely portable.
- Single header, KISS.
- Advanced rendering of variable height floor and ceiling.
- Textured walls and floor.
- Depth information (e.g. for fog or z-buffer rendering).
- Camera shearing (looking up/down).
- Camera movement with collisions.
- Partial support for opening door.
- Pure C99, tested to run as C++ as well.
- Optional framework functions that handle the whole rendering.
- Still flexible -- pixels are left for you to draw in any way you want.
- Tested on multiple platforms (PC, Arduboy, Pokitto, Gamebuino META).
- Many compile-time options to tune the performance vs quality.
- Well commented and formatted code.
- Completely free of legal restrictions, do literally anything you want.

**NOTE**: Backwards compatibility isn't a goal of this libraray. It is meant to
be an as-is set of tools that the users is welcome to adjust for their
specific project. So new features will be preferred to keeping the same
interface.

how to use
----------

For start take a look at the [testTerminal.c](https://gitlab.com/drummyfish/raycastlib/blob/master/programs/testTerminal.c) program.
It is only a little bit more complex than a simple hello world.

For more examples see the other files, plus my [Pokitto demos](https://gitlab.com/drummyfish/Pokitto-Raycasting) repository,
which contains some better documented example code, including a [very simple hello world](https://gitlab.com/drummyfish/Pokitto-Raycasting/blob/master/helloRay.cpp).

The basic philosophy is:

- The library implements only a rendering back-end, it doesn't permorm any drawing to the actual screen,
  hence there is no dependency on any library such as OpenGL or SDL. It just calls your front-end function
  and tells you which pixels you should write. How you do it is up to you.
- Before including the header, define `RCL_PIXEL_FUNCTION` to the name of a function you will use to
  draw pixels. It is basically a fragment/pixel shader function that the library will call. You will
  be passed info about the pixel and can decide what to do with it, so you can process it, discard it,
  or simply write it to the screen.
- Call `RCL_renderSimple` or `RCL_renderComplex` to perform the frame rendering. This will cause the
  library to start calling the `RCL_PIXEL_FUNCTION` in order to draw the frame. You can optionally write
  a similar function of your own using the more low-level functions which are also provided.
- The library gets info about the world (such as floor or ceiling height) via *array* functions
  (`RCL_ArrayFunction` type) -- functions that take *x* and *y* coordinates of a square and return given
  information. This way you are free to not only fetch the map data from an array, but also generate
  the world procedurally if that is what you want.
- Fixed point arithmetics is used as a principle, but there is no abstraction above it, everything is simply
  an integer (`RCL_Unit` type). The space is considered to be a dense grid, where each world square
  has a side length of `RCL_UNITS_PER_SQUARE` units. Numbers are normalized by this constant, so e.g.
  the sin function returns a value from `-RCL_UNITS_PER_SQUARE` to `RCL_UNITS_PER_SQUARE`.

TODO
----

- Transparency (conditional ray passing through).
- Doors in the middle of squares.
- Rolling doors for `RCL_renderComplex`.
- Possibly merge all rendering functions into one.

license
-------

Everything is CC0 1.0 + a waiver of all other IP rights (including patents). Please share your own software as free and open-source.
