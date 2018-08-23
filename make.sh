#!/bin/bash
clear; clear; g++ -x c -fmax-errors=5 -pedantic -Wall -Wextra -o test test.c 2>&1 >/dev/null && ./test
