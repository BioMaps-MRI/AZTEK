#ifndef RADIAL3D_TRAJECTORY_AZTEK
#define RADIAL3D_TRAJECTORY_AZTEK

#ifdef __cplusplus
extern "C" {
#endif

	int AZTEK_trajectory_comp(const int totalSpoke, short* gxS, short* gyS, short* gzS, const int scale_factor, short* max_iamp, double AZTEK_Twist, double AZTEK_Shuffle, int AZTEK_Speed);

#ifdef __cplusplus
}
#endif

#endif
