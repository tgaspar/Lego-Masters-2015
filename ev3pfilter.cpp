#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cmath>

#define NUMPARTICLES 300
#define ENCODERNOISE 5
#define ANGLENOISE 1
#define MAPSIZEX 18
#define MAPSIZEY 10

// Initialize particle arrray
int particlePositionArray[NUMPARTICLES][3] = {0};
int robotPositionArray[3] = {rand() % 1800,rand() % 1000,rand() % 360};
int particleQualityArray[NUMPARTICLES] = {0};
int sumOfParticlePositions[2] = {0};
int numberOfValidParticles = 0;
int avaragePosition[3] = {0};


int colorMap[10*18] = 
{//	00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 
	4, 1, 3, 4, 3, 2, 4, 3, 1, 5, 1, 3, 5, 2, 1, 4, 1, 3,// 00
	0, 3, 1, 1, 3, 0, 2, 2, 2, 0, 2, 2, 5, 2, 2, 2, 5, 1,// 01
	4, 1, 3, 0, 3, 1, 3, 5, 4, 2, 2, 3, 2, 2, 2, 0, 1, 0,// 02
	1, 2, 2, 3, 5, 3, 1, 3, 2, 0, 3, 5, 1, 0, 5, 4, 0, 2,// 03
	4, 0, 4, 3, 2, 3, 2, 5, 4, 0, 5, 3, 2, 4, 4, 0, 0, 4,// 04
	0, 3, 1, 3, 5, 2, 0, 1, 4, 5, 4, 4, 4, 3, 3, 3, 2, 2,// 05
	1, 3, 1, 5, 5, 4, 2, 3, 5, 1, 0, 3, 1, 5, 1, 3, 2, 0,// 06
	5, 4, 2, 0, 5, 3, 3, 1, 2, 4, 2, 3, 5, 4, 1, 1, 0, 4,// 07
	5, 3, 2, 0, 2, 2, 3, 4, 2, 0, 1, 1, 2, 4, 4, 0, 5, 1,// 08
	1, 2, 5, 1, 5, 0, 5, 0, 4, 4, 1, 4, 4, 0, 2, 2, 0, 2 // 09
};

int fixAngle(int inputAngle)
{
	int correctedAngle =inputAngle;
	while(correctedAngle > 360)
		correctedAngle = correctedAngle - 360;

	while(correctedAngle < 0)
		correctedAngle = correctedAngle + 360;
	
	return correctedAngle;
}

void particleFilterMain(int robotPosition[3], int readColor)
{
	
	int robotMovements[3] = {0}; // 0 -> x; 1 -> y; 2 -> angle
	
	// Calculate the difference in motion
	cout << "Copying data \n;
	robotMovements[0] = robotPositionArray[0] - robotPosition[0];
	robotMovements[1] = robotPositionArray[1] - robotPosition[1];
	if(robotMovements[1] == 0)
		robotMovements[1] += 1;
	
	robotMovements[2] = atan2(robotMovements[0],robotMovements[1]);
// 	robotMovements[2] = robotPositionArray[2] - robotPosition[2];
	
	robotMovements[2] = fixAngle(robotMovements[2]); //Put the angle between 0 and 360

	robotPositionArray[0] = robotPositionArray[0] + robotMovements[0]*(rand() % ENCODERNOISE);
	robotPositionArray[1] = robotPositionArray[1] + robotMovements[1]*(rand() % ENCODERNOISE);
	robotPositionArray[2] = robotPositionArray[2] + robotMovements[2]*(rand() % ANGLENOISE);
	
	cout << "Data copied \n";
	
	cout << "Simullating particle movement \n";
	//Simulate particle movement
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		particlePositionArray[i][0] = particlePositionArray[i][0] + robotMovements[0]*(rand() % ENCODERNOISE);
		particlePositionArray[i][1] = particlePositionArray[i][1] + robotMovements[1]*(rand() % ENCODERNOISE);
		particlePositionArray[i][2] = particlePositionArray[i][2] + robotMovements[2]*(rand() % ANGLENOISE);
	}
	
	cout << "Partil movemil simulail\n";
	
	
	cout << "Particle movemm";
	
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		int currentParticleColor = colorMap[MAPSIZEY*int(round(particlePositionArray[i][1])) +  particlePositionArray[i][0]];
		
		if(currentParticleColor == readColor)
		{
			numberOfValidParticles += 1;
			particleQualityArray[i] = 1;
			sumOfParticlePositions[0] = sumOfParticlePositions[0] + particlePositionArray[i][0];
			sumOfParticlePositions[1] = sumOfParticlePositions[1] + particlePositionArray[i][1];
		}
		else
		{
			particleQualityArray[i] = 0;
		}

	}
	
	avaragePosition[0] = round((sumOfParticlePositions[0]*1.0)/(numberOfValidParticles*1.0));
	avaragePosition[1] = round((sumOfParticlePositions[1]*1.0)/(numberOfValidParticles*1.0));
	
	
	int sumOfAbsoluteErrorNew = 0;
	int minimumAbsoluteError = 1000;
	int closesParticle = 0;
	
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		if(particleQualityArray[i] == 1)
		{
				sumOfAbsoluteErrorNew  += abs(avaragePosition[0] - particlePositionArray[i][0]);
				sumOfAbsoluteErrorNew  += abs(avaragePosition[1] - particlePositionArray[i][1]);
				if (sumOfAbsoluteErrorNew < minimumAbsoluteError)
				{
					minimumAbsoluteError = sumOfAbsoluteErrorNew;
					closesParticle = i;
				}
				
		}
	}
	avaragePosition[2] = particlePositionArray[closesParticle][2];
	
	
	
}