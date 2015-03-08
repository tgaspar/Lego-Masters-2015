#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cmath>

int int_round(int numberToRound);
int random(int multiplier);
int fixAngle(int inputAngle);
void particleFilterMain(int robotPosition[3], int robotMovements[3], int robotMoveCommand[2], int readColor);
