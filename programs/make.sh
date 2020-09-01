#!/bin/bash

# Make script for raycastlib programs.
# by drummyfish
# released under CC0 1.0, public domain

if [ "$#" -ne 1 ]; then
  echo "ERROR: expecting one argument, the name of program without extension (e.g. \"helloWorld\")"
  exit 0
fi

link=""

if [ "$1" = "testSDL" ] || [ "$1" = "gamified_SDL" ]; then
  link="-lSDL2"
fi

clear
clear

flags="-x c -g -O3 -std=c99 -pedantic -Wall -Wextra -o $1 $1.c ${link}"
compiler=gcc
#compiler=clang

echo "making:" ${compiler} ${flags}

${compiler} ${flags} && ./$1
