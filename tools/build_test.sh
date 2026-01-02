#!/bin/bash
#bash script for CI to build all test functions

SCRPIT_DIR=$(dirname "$0")
TEST_FUNCTIONS=$(cat Src/test/test.c | grep -P '(?<=void )test_.+(?=\()' -o)
for test_function in $TEST_FUNCTIONS
do
    make TEST=$test_function
done