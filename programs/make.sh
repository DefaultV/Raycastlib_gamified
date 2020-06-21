#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "ERROR: expecting one argument, the name of program without extension (e.g. \"helloWorld\")"
  exit 0
fi

clear; clear; g++ -x c -g -fmax-errors=5 -pedantic -Wall -Wextra -o $1 $1.c -lSDL2 2>&1 >/dev/null && ./$1
