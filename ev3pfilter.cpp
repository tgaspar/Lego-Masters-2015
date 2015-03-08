#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

#define NUMPARTICLES 1000
#define ENCODERNOISE 5
#define ANGLENOISE 1
#define MAPSIZEX 18
#define MAPSIZEY 10

// Random generator
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.0,1.0);
// Initialize particle arrray
int particlePositionArray[NUMPARTICLES][3] = {0};

int particleQualityArray[NUMPARTICLES] = {0};
int sumOfParticlePositions[2] = {0};
int numberOfValidParticles = 0;
int avaragePosition[3] = {0};

int firstCall = 1;

int int_round(int numberToRound)
{
	return static_cast<int>(round(numberToRound));
}
int random(int multiplier)
{
	return static_cast<int>(round(((double)rand() / (RAND_MAX))*multiplier));
}

double GetMedian(double daArray[], int iSize) {
    // Allocate an array of the same size and sort it.
    double* dpSorted = new double[iSize];
    for (int i = 0; i < iSize; ++i) 
    {
        dpSorted[i] = daArray[i];
    }
    
    for (int i = iSize - 1; i > 0; --i) 
    {
        for (int j = 0; j < i; ++j) 
	{
            if (dpSorted[j] > dpSorted[j+1]) 
	    {
                double dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    double dMedian = 0.0;
    if ((iSize % 2) == 0) 
    {
        dMedian = (dpSorted[iSize/2] + dpSorted[(iSize/2) - 1])/2.0;
    } else 
    {
        dMedian = dpSorted[iSize/2];
    }
    delete [] dpSorted;
    return dMedian;
}

int robotEstimatePositionArrayNew[3] = {random(1800),random(1000),random(360)};
int robotEstimatePositionArrayOld[3] = {0};
int robotPositionArray[3] = {0};

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

void particleFilterMain(int robotPosition[3], int robotMovements[3], int robotMoveCommand[2], int readColor)
{
	srand(time(0));
	
// 	int robotMovements[3] = {0}; // 0 -> x; 1 -> y; 2 -> angle
	
	// Calculate the difference in motion
	printf("******************************************\n");
/*
// 	printf("Copying data ...\n");
// 	printf("robotPosition[0] = %d \n",robotPosition[0]);
// 	printf("robotPosition[1] = %d \n",robotPosition[1]);
// 	printf("robotPosition[2] = %d \n",robotPosition[2]);
// 	
// 	printf("robotEstimatePositionArrayNew[0] = %d \n",robotEstimatePositionArrayNew[0]);
// 	printf("robotEstimatePositionArrayNew[1] = %d \n",robotEstimatePositionArrayNew[1]);
// 	printf("robotEstimatePositionArrayNew[2] = %d \n",robotEstimatePositionArrayNew[2]);
// 	
	
// 	robotMovements[0] = robotEstimatePositionArrayOld[0] - robotEstimatePositionArrayNew[0];
// 	robotMovements[1] = robotEstimatePositionArrayOld[1] - robotPositionNew[1];
// 	if(robotMovements[1] == 0)
// 		robotMovements[1] += 1;
	
// 	robotMovements[2] = atan2(robotMovements[0],robotMovements[1]);
// 	robotMovements[2] = robotPositionArray[2] - robotPosition[2];
*/	
	robotMovements[2] = fixAngle(robotMovements[2]); //Put the angle between 0 and 360

	
	
	for(int i = 0; i < 3; i++)
	{
		printf("robotMovements[%d] = %d \n", i, robotMovements[i]);
		robotEstimatePositionArrayOld[i] = robotEstimatePositionArrayNew[i];
	}
	
	
	robotEstimatePositionArrayNew[0] = robotEstimatePositionArrayNew[0] + robotMoveCommand[1];
	robotEstimatePositionArrayNew[1] = robotEstimatePositionArrayNew[1] + int_round(robotMoveCommand[0]*cos(robotEstimatePositionArrayNew[2]*1.0));
	robotEstimatePositionArrayNew[2] = robotEstimatePositionArrayNew[2] + int_round(robotMoveCommand[0]*sin(robotEstimatePositionArrayNew[2]*1.0));
		
	//Simulate particle movement
	numberOfValidParticles = 0;
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		if(firstCall)
		{
			particlePositionArray[i][0] = random(1800);
			particlePositionArray[i][1] = random(1000);
			particlePositionArray[i][2] = random(360);
		}
		particlePositionArray[i][2] = particlePositionArray[i][2] + robotMoveCommand[1];
		particlePositionArray[i][0] = particlePositionArray[i][0] + int_round(robotMoveCommand[0]*cos(particlePositionArray[i][2]*1.0));
		particlePositionArray[i][1] = particlePositionArray[i][1] + int_round(robotMoveCommand[0]*sin(particlePositionArray[i][2]*1.0));
		/*
		particlePositionArray[i][0] = particlePositionArray[i][0] + robotMovements[0]*random(ENCODERNOISE);
		particlePositionArray[i][1] = particlePositionArray[i][1] + robotMovements[1]*random(ENCODERNOISE);
		particlePositionArray[i][2] = particlePositionArray[i][2] + robotMovements[2]*random(ANGLENOISE);
		*/
	}
	firstCall = 0;
/*	
	printf("Particle movement simulated\n");
	
	printf("particlePositionArray[1][0]/100.0) = %d \n", int_round(particlePositionArray[1][0]/100.0));
	printf("particlePositionArray[1][1]/100.0) = %d \n", int_round(particlePositionArray[1][1]/100.0));*/
	sumOfParticlePositions[0] = {0};
	sumOfParticlePositions[1] = {0};
	
	
	
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		
		int currentParticleColor =-2;
		
		currentParticleColor = colorMap[MAPSIZEX*int_round(particlePositionArray[i][1]/100.0) +  int_round(particlePositionArray[i][0]/100.0)];
			
// 		printf("currentParticleColor = %d \n",currentParticleColor);
		
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
	
// 	avaragePosition[0] = int_round((sumOfParticlePositions[0]*1.0)/(numberOfValidParticles*1.0));
// 	avaragePosition[1] = int_round((sumOfParticlePositions[1]*1.0)/(numberOfValidParticles*1.0));
	
	
	int sumOfAbsoluteErrorNew = 0;
	int minimumAbsoluteError = 1000;
	int closestParticle = 0;
	
	double validParticlesArray[2][numberOfValidParticles];
	int validParticleIndex = 0;
	
	double medianVal = 0;
	
	
	
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		if(particleQualityArray[i] == 1)
		{
			validParticlesArray[0][validParticleIndex] = particlePositionArray[i][0];
			validParticlesArray[1][validParticleIndex] = particlePositionArray[i][1];
			validParticleIndex += 1;

		}
	}
	medianVal = (int)GetMedian(&validParticlesArray[0][0], numberOfValidParticles);
	avaragePosition[0] = medianVal;
	medianVal = (int)GetMedian(&validParticlesArray[1][0], numberOfValidParticles);
	avaragePosition[1] = medianVal;
	
	for(int i = 0; i < NUMPARTICLES; i++)
	{
		sumOfAbsoluteErrorNew = 0;
		if(particleQualityArray[i] == 1)
		{
				sumOfAbsoluteErrorNew  += abs(avaragePosition[0] - particlePositionArray[i][0]);
				sumOfAbsoluteErrorNew  += abs(avaragePosition[1] - particlePositionArray[i][1]);
				
				if (sumOfAbsoluteErrorNew < minimumAbsoluteError)
				{
					minimumAbsoluteError = sumOfAbsoluteErrorNew;
					closestParticle = i;
				}
				
		}
		else
		{
			particlePositionArray[i][0] = random(1800);
			particlePositionArray[i][1] = random(1000);
			particlePositionArray[i][2] = random(360);
		}
	}
	
 	
	
	avaragePosition[2] = particlePositionArray[closestParticle][2];
	
	printf("numberOfValidParticles = %d \n",numberOfValidParticles);
	printf("closestParticle = %d \n",closestParticle);
// 	printf("particlePositionArray[%d][0] = %d \n", closestParticle, particlePositionArray[closestParticle][0]);
// 	printf("particlePositionArray[%d][1] = %d \n", closestParticle, particlePositionArray[closestParticle][1]);
// 	printf("particlePositionArray[%d][2] = %d \n", closestParticle, particlePositionArray[closestParticle][2]);
// 	
	
	robotPosition[0] = avaragePosition[0];
	robotPosition[1] = avaragePosition[1];
	robotPosition[2] = fixAngle(avaragePosition[2]);
	
	
	
}