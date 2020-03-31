/*
 ============================================================================
 Name        : Start_trajectory_comp.c
 Author      : Tanguy Boucneau
 Version     : 1.1
 Copyright   : University Paris-Saclay, CEA, CNRS, Inserm, BioMaps
 Description : Computation of AZTEK trajectories for 3D radial UTE MR acquistions
               initially developed for GE TEP-MR system at SHFJ
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "AZTEK_trajectory_comp.h"

int main() {

    char argv[6][50];
    printf("Please enter separated by space :\n (1) the number of spokes\n (2) AZTEK_Twist\n (3) AZTEK_Shuffle\n (4) AZTEK_Speed: \n>>  ");
    scanf("%20s %20s %20s %20s", argv[1], argv[3], argv[4], argv[5]);
    int nbSpokes = atoi(argv[1]);
    short max_iamp = 1;
    const int scale_factor = 1;
    int n;

    short* gxScale = NULL;
    short* gyScale = NULL;
    short* gzScale = NULL;

	char gradListName[256];
    FILE* fileID = NULL;

    gxScale = malloc(nbSpokes * sizeof(short));
    gyScale = malloc(nbSpokes * sizeof(short));
    gzScale = malloc(nbSpokes * sizeof(short));

    AZTEK_trajectory_comp(nbSpokes, gxScale, gyScale, gzScale, scale_factor, &max_iamp, atof(argv[3]), atof(argv[4]), atoi(argv[5]));


    sprintf(gradListName, "AZTEK_trajectory_Spokes%s_Twist%s_Shuffle%s_Speed%s.txt", argv[1], argv[3], argv[4], argv[5]);

    fileID = fopen(gradListName, "w+");

    int rot_sign;

    if (fileID != 0)
    {
        fprintf(fileID, "      Gx      Gy      Gz     Rot. sign\n");
        for (n = 0; n < nbSpokes; n++)
        {
            if (n > 0)
            {
                if ((gxScale[n - 1] * gyScale[n] - gxScale[n] * gyScale[n - 1]) < 0)
                {
                    rot_sign = -1;
                }
                else
                {
                    rot_sign = 1;
                }
                fprintf(fileID, "%8d%8d%8d%12d\n", gxScale[n], gyScale[n], gzScale[n], rot_sign);
            }
            else
            {
                fprintf(fileID, "%8d%8d%8d         N/A\n", gxScale[n], gyScale[n], gzScale[n]);
            }
        }

        fclose(fileID);
    }

	printf("Gradient amplitudes recorded in %s\n", gradListName);
	return EXIT_SUCCESS;

}
