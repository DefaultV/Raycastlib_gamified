#!/bin/bash
clear; clear; g++ -x c -fmax-errors=5 -pedantic -Wall -Wextra -o test raycastlib.c 2>&1 >/dev/null && ./test
