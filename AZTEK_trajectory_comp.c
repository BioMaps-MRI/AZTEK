/**
 * Generate the 3D Radial Trajectory
 *
 * @param[out] *gxS, gradient scale array in x direction
 * @param[out] *gyS, gradient scale array in y direction
 * @param[out] *gzS, gradient scale array in z direction
 * @param[out] *max_iamp, maximum gradient instruction
 * @param[in]  totalSpoke, total number of spokes
 * @param[in]  scale_factor, used for low res scan
 *
 * @return  0: success,  1: failure
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
 /*#include "epic_hw_defs.h"*/ /*To uncomment for GE*/
#include "AZTEK_trajectory_comp.h"

/* VxWorks 5.5 does not define M_PI. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int AZTEK_trajectory_comp(const int totalSpoke, short* gxS, short* gyS, short* gzS, const int scale_factor, short* max_iamp, double AZTEK_Twist, double AZTEK_Shuffle, int AZTEK_Speed)
{

	int MAX_PG_IAMP = 32767; /*To suppress*/

	double goldenRatioInv = (double)2 / ((double)1 + sqrt((double)5));
	double tmp_scale, mean_N_theta, theta_pos, goldenRatioAcc, bestGoldenRatioAccDecim, goldenRatioCorr;
	double theta_pos_shift, thetaTmp, dPhi, twist, shufflePhiStep, i_phi_shuffle_double, goldenRatioSpeedStep, speedOrder_double;
	int N_theta, N_phi, N_phi_corrShift, N_theta_cumul, i_theta, i_phi, i_theta_shuffle, i_phi_shuffle, counter, increment, gradPos, angPos;
	int totSpkOverSpdFac, totSpkModSpdFac, n, m, p, nModSpdFac, nWithSpdFac;

	double* theta = NULL;
	double* phi = NULL;
	int* nbThetaTable = NULL;
	int* nbThetaTableCumul = NULL;
	double* goldenRatioAccDecim = NULL;
	int* phiSelected = NULL;
	short* gxSTmp = NULL;
	short* gySTmp = NULL;
	short* gzSTmp = NULL;
	int* speedOrder = NULL;
	int* speedOrderSelected = NULL;
	int* speedOrderCorr = NULL;

	theta = malloc(totalSpoke * sizeof(double));
	phi = malloc(totalSpoke * sizeof(double));

	if ((scale_factor < 1) || (scale_factor > 32))
	{
		return 1;
	}
	else
	{
		tmp_scale = (double)MAX_PG_IAMP / scale_factor;
	}

	*max_iamp = (short)MAX_PG_IAMP;

	N_phi = ceil(2 * sqrt((double)totalSpoke) / ((double)1 + AZTEK_Twist));

	N_phi_corrShift = 13;

	if (N_phi < (2 * N_phi_corrShift))
	{
		N_phi_corrShift = 1;
	}

	if (N_phi % 2 == 1)
	{
		N_phi_corrShift++;
	}

	goldenRatioAccDecim = malloc((2 * N_phi_corrShift) * sizeof(double));

	for (n = 0; n < N_phi_corrShift; n++)
	{
		goldenRatioAcc = goldenRatioInv * (N_phi + 2 * n - N_phi_corrShift + 1);
		goldenRatioAccDecim[n] = (double)floor(goldenRatioAcc) - goldenRatioAcc;
		goldenRatioAccDecim[N_phi_corrShift + n] = (double)ceil(goldenRatioAcc) - goldenRatioAcc;
	}

	m = 0;
	bestGoldenRatioAccDecim = goldenRatioAccDecim[m];

	for (n = 1; n < (2 * N_phi_corrShift); n++)
	{
		if (fabs(goldenRatioAccDecim[n]) < fabs(goldenRatioAccDecim[m]))
		{
			m = n;
			bestGoldenRatioAccDecim = goldenRatioAccDecim[m];
		}
	}

	N_phi += 2 * (m % N_phi_corrShift) - N_phi_corrShift + 1;
	goldenRatioCorr = bestGoldenRatioAccDecim / (double)N_phi;

	mean_N_theta = (double)totalSpoke / (double)N_phi;

	dPhi = 2 * M_PI / ((double)N_phi);
	twist = (AZTEK_Twist * N_phi) / (double)4;

	nbThetaTable = malloc(N_phi * sizeof(int));
	nbThetaTableCumul = malloc((N_phi + 1) * sizeof(int));

	nbThetaTableCumul[0] = 0;
	N_theta_cumul = 0;

	theta_pos_shift = 0.5;

	for (i_phi = 0; i_phi < N_phi; i_phi++)
	{
		N_theta = ceil((i_phi + 1) * mean_N_theta) - ceil(i_phi * mean_N_theta);

		for (i_theta = 0; i_theta < N_theta; i_theta++)
		{
			theta_pos = ((double)i_theta + theta_pos_shift) / (double)N_theta;
			thetaTmp = acos(1 - 2 * theta_pos);
			theta[N_theta_cumul + i_theta] = thetaTmp;
			phi[N_theta_cumul + i_theta] = ((double)i_phi + twist * cos(thetaTmp)) * dPhi;
		}

		theta_pos_shift += goldenRatioInv + goldenRatioCorr;
		theta_pos_shift -= floor(theta_pos_shift);

		nbThetaTable[i_phi] = N_theta;
		nbThetaTableCumul[i_phi + 1] = nbThetaTableCumul[i_phi] + N_theta;
		N_theta_cumul += N_theta;
	}

	shufflePhiStep = N_phi * (0.5 + AZTEK_Shuffle * (goldenRatioInv - 0.5));
	phiSelected = malloc(N_phi * sizeof(int));

	for (i_phi = 0; i_phi < N_phi; i_phi++)
	{
		phiSelected[i_phi] = 0;
	}

	i_phi_shuffle_double = -shufflePhiStep;
	i_phi_shuffle = 0;
	N_theta_cumul = 0;

	gxSTmp = malloc(totalSpoke * sizeof(short));
	gySTmp = malloc(totalSpoke * sizeof(short));
	gzSTmp = malloc(totalSpoke * sizeof(short));

	for (i_phi = 0; i_phi < N_phi; i_phi++)
	{
		i_phi_shuffle_double += shufflePhiStep;
		i_phi_shuffle = floor(i_phi_shuffle_double);
		i_phi_shuffle %= N_phi;
		counter = 0;
		increment = 0;

		while (phiSelected[i_phi_shuffle] == 1)
		{
			counter++;
			increment = counter * (1 - 2 * ((counter - 1) % 2));
			i_phi_shuffle += increment;

			if (i_phi_shuffle < 0)
			{
				i_phi_shuffle += N_phi;
			}

			i_phi_shuffle %= N_phi;
		}

		i_phi_shuffle_double += (double)increment;

		if (i_phi_shuffle_double < 0)
		{
			i_phi_shuffle_double += (double)N_phi;
		}

		for (i_theta_shuffle = 0; i_theta_shuffle < nbThetaTable[i_phi_shuffle]; i_theta_shuffle++)
		{
			gradPos = N_theta_cumul + i_theta_shuffle;

			if (i_phi % 2 == 0)
			{
				angPos = nbThetaTableCumul[i_phi_shuffle] + i_theta_shuffle;

				gxSTmp[gradPos] = (short)(tmp_scale * (sin(theta[angPos]) * cos(phi[angPos])));
				gySTmp[gradPos] = (short)(tmp_scale * (sin(theta[angPos]) * sin(phi[angPos])));
				gzSTmp[gradPos] = (short)(tmp_scale * cos(theta[angPos]));
			}
			else
			{
				angPos = nbThetaTableCumul[i_phi_shuffle + 1] - 1 - i_theta_shuffle;

				gxSTmp[gradPos] = (short)(tmp_scale * (sin(theta[angPos]) * cos(phi[angPos])));
				gySTmp[gradPos] = (short)(tmp_scale * (sin(theta[angPos]) * sin(phi[angPos])));
				gzSTmp[gradPos] = (short)(tmp_scale * cos(theta[angPos]));
			}
		}

		phiSelected[i_phi_shuffle] = 1;
		N_theta_cumul += nbThetaTable[i_phi_shuffle];
	}

	AZTEK_Speed++;

	totSpkOverSpdFac = totalSpoke / AZTEK_Speed;
	totSpkModSpdFac = totalSpoke % AZTEK_Speed;

	speedOrder = malloc(AZTEK_Speed * sizeof(int));
	speedOrderSelected = malloc(AZTEK_Speed * sizeof(int));

	for (n = 0; n < AZTEK_Speed; n++)
	{
		speedOrderSelected[n] = 0;
	}

	speedOrder_double = 0;
	goldenRatioSpeedStep = AZTEK_Speed * goldenRatioInv;

	for (n = 0; n < AZTEK_Speed; n++)
	{
		speedOrder_double += goldenRatioSpeedStep;
		speedOrder[n] = floor(speedOrder_double);
		speedOrder[n] %= AZTEK_Speed;
		counter = 0;
		increment = 0;

		while (speedOrderSelected[speedOrder[n]] == 1)
		{
			counter++;
			increment = counter * (1 - 2 * ((counter - 1) % 2));
			speedOrder[n] += increment;

			if (speedOrder[n] < 0)
			{
				speedOrder[n] += AZTEK_Speed;
			}

			speedOrder[n] %= AZTEK_Speed;
		}

		speedOrder_double += (double)increment;

		if (speedOrder_double < 0)
		{
			speedOrder_double += (double)AZTEK_Speed;
		}

		speedOrderSelected[speedOrder[n]] = 1;
	}

	speedOrderCorr = malloc(AZTEK_Speed * sizeof(int));

	for (n = 0; n < AZTEK_Speed; n++)
	{
		speedOrderCorr[n] = 0;

		if (speedOrder[n] > 0)
		{
			for (m = 0; m < speedOrder[n]; m++)
			{
				for (p = 0; p < totSpkModSpdFac; p++)
				{
					if (m == speedOrder[p])
					{
						speedOrderCorr[n]++;
					}
				}
			}
		}
	}

	for (n = 0; n < totalSpoke; n++)
	{
		nModSpdFac = n % AZTEK_Speed;

		nWithSpdFac = n / AZTEK_Speed + speedOrder[nModSpdFac] * totSpkOverSpdFac + speedOrderCorr[nModSpdFac];

		gxS[nWithSpdFac] = gxSTmp[n];
		gyS[nWithSpdFac] = gySTmp[n];
		gzS[nWithSpdFac] = gzSTmp[n];
	}

	free(theta);
	free(phi);
	free(nbThetaTable);
	free(nbThetaTableCumul);
	free(goldenRatioAccDecim);
	free(phiSelected);
	free(gxSTmp);
	free(gySTmp);
	free(gzSTmp);
	free(speedOrder);
	free(speedOrderSelected);
	free(speedOrderCorr);

	return 0;

}
