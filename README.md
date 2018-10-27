# raycastlib

Ray casting library for (not only) limited-resource computers.

Take a look at my [Pokitto demos](https://gitlab.com/drummyfish/Pokitto-Raycasting) repository,
it contains some better code for learning the usage, including a very simple hello world.

eye-candy previews
------------------

Pokitto:

![](/media/pokitto1.gif)
![](/media/pokitto2.gif)
![](/media/pokitto3.gif)

SDL:

![](/media/sdl.gif)

Arduboy:

![](/media/arduboy.gif)
![](/media/arduboy2.gif)

terminal:

![](/media/terminal.gif)

Gamebuino META:

![](/media/gamebuino.gif)

features
--------

- Very fast, small and efficient.
- Uses only integer math.
- No dependencies (uses only stdint standard library), extremely portable.
- Single header, KISS.
- Advanced rendering of variable height floor and ceiling.
- Textured walls and floor.
- Depth information (for fog).
- Camera shearing (looking up/down).
- Camera movement with collisions.
- Support for opening door.
- Pure C, tested to run as C++ as well.
- Optional framework functions that handle the whole rendering.
- Still flexible -- pixels are left for you to draw in any way you want.
- Tested on multiple platforms (PC, Arduboy, Pokitto, Gamebuino META).
- Many compile-time options to tune the performance vs quality.
- Well commented code.

**NOTE**: Backwards compatibility isn't a goal of this libraray. It is meant to
be an as-is set of tools that the users is welcome to adjust for their
specific project. So new features will be preferred to keeping the same
interface.

TODO
----

- Transparency (conditional ray passing through).
- Doors in the middle of squares.
- Rolling doors for `RCL_renderComplex`.
- Possibly merge all rendering functions into one.

license
-------

Everything is CC0 1.0. Please share your own software as free and open-source.