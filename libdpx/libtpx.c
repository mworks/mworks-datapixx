#pragma once
#ifndef libtpx_h
#define lbitpx_h

#define COEFF_X_BASEADDR	0x200
#define COEFF_X_L_BASEADDR	0x214
#define COEFF_Y_BASEADDR	0x228
#define COEFF_Y_L_BASEADDR	0x23C
#define	CALIB_STATUS		0x24E
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_rng.h>
#include "libdpx.h"
#if defined(_MSC_VER)
#include <intrin.h>
#elif defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
#include <x86intrin.h>
#endif


void TPxGetEyePosition_validation(float x_i[2], float y_i[2], float coeff_x[9][2], float coeff_y[9][2], float* eyeReturn)
{ 
	// x[2]: x[0] is eye1_x, x[1] is eye2_x
	// coeff_x[i][0] are the parameters for the polynomial eye 1
	// coeff_x[i][1] are the parameters for the polynomial eye 2
	// eyeReturn[i]: x_eye1, y_eye1, x_eye2, y_eye2.
	
	float x = x_i[0];
	float y = y_i[0];
	float x2 = x_i[1];
	float y2 = y_i[1];
	eyeReturn[0] =  coeff_x[0][0]*1       +
					coeff_x[1][0]*x       +
					coeff_x[2][0]*y       +
					coeff_x[3][0]*x*x     +
					coeff_x[4][0]*y*y     +
					coeff_x[5][0]*x*x*x   +
					coeff_x[6][0]*x*y     +
					coeff_x[7][0]*x*x*y   +
					coeff_x[8][0]*x*x*y*y;
	

	eyeReturn[1] =  coeff_y[0][0]*1       +
					coeff_y[1][0]*x       +
					coeff_y[2][0]*y       +
					coeff_y[3][0]*x*x     +
					coeff_y[4][0]*y*y     +
					coeff_y[5][0]*x*x*x   +
					coeff_y[6][0]*x*y     +
					coeff_y[7][0]*x*x*y   +
					coeff_y[8][0]*x*x*y*y;

	x = x2;
	y = y2;
	eyeReturn[2] =  coeff_x[0][1]*1       +
					coeff_x[1][1]*x       +
					coeff_x[2][1]*y       +
					coeff_x[3][1]*x*x     +
					coeff_x[4][1]*y*y     +
					coeff_x[5][1]*x*x*x   +
					coeff_x[6][1]*x*y     +
					coeff_x[7][1]*x*x*y   +
					coeff_x[8][1]*x*x*y*y;
	

	eyeReturn[3] =  coeff_y[0][1]*1       +
					coeff_y[1][1]*x       +
					coeff_y[2][1]*y       +
					coeff_y[3][1]*x*x     +
					coeff_y[4][1]*y*y     +
					coeff_y[5][1]*x*x*x   +
					coeff_y[6][1]*x*y     +
					coeff_y[7][1]*x*x*y   +
					coeff_y[8][1]*x*x*y*y;
	return;
}

void TPxSaveCoefficient_test()
{
	int i,j, number;
	int addr_x = COEFF_X_BASEADDR;
	for (i = 0; i < 9; i++)
	{
		for (j = 0; j < 4; j++)
		{
			number = i*0x1100+j*0x11;
			DPxSetReg16(addr_x+2*i+j*0x14, number);
			DPxUpdateRegCache();
		}


	}
}

void TPxReadCoefficient_test()
{
	int i,j,number;
	int addr_x = COEFF_X_BASEADDR;
	for (i = 0; i < 9; i++)
	{
		for (j = 0; j < 4; j++)
		{
			DPxUpdateRegCache();
			number = DPxGetReg16(addr_x+2*i+j*0x14);
			printf("Value read: 0x%x (%d, %d)\n", number, i, j);

		}

	}
}

void TPxSaveCoefficientsInTracker(float coeff_x[9][2], float coeff_y[9][2])
{
	
	int i;

	float multiplier_coeff_high = 1 << 3;
	float multiplier_coeff_low = 1 << 11;
	int addr_x, addr_x_L, addr_y, addr_y_L, tracker_calibrated_addr;
	int number;
	DPxUpdateRegCache();
	addr_x = COEFF_X_BASEADDR;
	addr_x_L = COEFF_X_L_BASEADDR;
	addr_y = COEFF_Y_BASEADDR;
	addr_y_L = COEFF_Y_L_BASEADDR;
	tracker_calibrated_addr = CALIB_STATUS;
	for (i = 0; i < 9; i++)
	{
		// Bit 15: 1 for +, 0 for -
		// Bit 14: 1 for 8 < abs(x) < 2048, 1 otherwise.
		// Bit 13-0: (int) number*coeff
		number = 0;
		number = coeff_x[i][0] > 0 ? 0x0000 : 0x8000;
		number += fabs(coeff_x[i][0]) > 8 ? 0x4000 : 0x0000;
		if (fabs(coeff_x[i][0]) > 2048)
			return; //TOO BIG
		number += (int) fabs(coeff_x[i][0]) > 8 ? (int)(fabs(coeff_x[i][0])*multiplier_coeff_high) : (int)(fabs(coeff_x[i][0])*multiplier_coeff_low);
		DPxSetReg16(addr_x+2*i, number);

		number = 0;
		number = coeff_x[i][1] > 0 ? 0x0000 : 0x8000;
		number += fabs(coeff_x[i][1]) > 8 ? 0x4000 : 0x0000;
		if (fabs(coeff_x[i][1]) > 2048)
			return; //TOO BIG
		number += (int) fabs(coeff_x[i][1]) > 8 ? (int)( fabs(coeff_x[i][1])*multiplier_coeff_high) : (int)(fabs(coeff_x[i][1])*multiplier_coeff_low);
		DPxSetReg16(addr_x_L+2*i, number);

		number = 0;
		number = coeff_y[i][0] > 0 ? 0x0000 : 0x8000;
		number += fabs(coeff_y[i][0]) > 8 ? 0x4000 : 0x0000;
		if (fabs(coeff_y[i][0]) > 2048)
			return; //TOO BIG
		number += (int) fabs(coeff_y[i][0]) > 8 ? (int)(fabs(coeff_y[i][0])*multiplier_coeff_high) : (int)(fabs(coeff_y[i][0])*multiplier_coeff_low);
		DPxSetReg16(addr_y+2*i, number);
		
		number = 0;
		number = coeff_y[i][1] > 0 ? 0x0000 : 0x8000;
		number += fabs(coeff_y[i][1]) > 8 ? 0x4000 : 0x0000;
		if (fabs(coeff_y[i][1]) > 2048)
			return; //TOO BIG
		number += (int) fabs(coeff_y[i][1]) > 8 ? (int)(fabs(coeff_y[i][1])*multiplier_coeff_high) : (int)(fabs(coeff_y[i][1])*multiplier_coeff_low);
		DPxSetReg16(addr_y_L+2*i, number);

		DPxUpdateRegCache();
	}
	DPxSetReg16(tracker_calibrated_addr, 0xFFFF); // Device is calibrated
	DPxUpdateRegCache();

}
void TPxGetEyePosition_noCeoff(float* eyeReturn)
{
	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;
	float dividor, dividor_coeff;
	float x,y;
    float dividor_coeff_high = 1 << 3;
	float dividor_coeff_low = 1 << 11;
	float coeff_x[9][2];
	float coeff_y[9][2];
	int i;
	int addr_x, addr_x_L, addr_y, addr_y_L, tracker_calibrated_addr;
	addr_x = COEFF_X_BASEADDR;
	addr_x_L = COEFF_X_L_BASEADDR;
	addr_y = COEFF_Y_BASEADDR;
	addr_y_L = COEFF_Y_L_BASEADDR;
	tracker_calibrated_addr = CALIB_STATUS;
	// Check if device is calibrated

	// Get the coefficients
	dividor_coeff = 1 << 15;
	DPxUpdateRegCache();
	if (DPxGetReg16(tracker_calibrated_addr) != 0xFFFF)
	{
		return;
	}

	for (i = 0; i < 9; i++)
	{
		
		coeff_x[i][0] = ((DPxGetReg16(addr_x+2*i) & 0x8000 ? -1 : 1) * (DPxGetReg16(addr_x+2*i) & 0x3FFF)) /  (DPxGetReg16(addr_x+2*i) & 0x4000 ? dividor_coeff_high : dividor_coeff_low);
		coeff_x[i][1] = ((DPxGetReg16(addr_x_L+2*i) & 0x8000 ? -1 : 1) * (DPxGetReg16(addr_x_L+2*i) & 0x3FFF)) /  (DPxGetReg16(addr_x_L+2*i) & 0x4000 ? dividor_coeff_high : dividor_coeff_low);
		coeff_y[i][0] = ((DPxGetReg16(addr_y+2*i) & 0x8000 ? -1 : 1) * (DPxGetReg16(addr_y+2*i) & 0x3FFF)) /  (DPxGetReg16(addr_y+2*i) & 0x4000 ? dividor_coeff_high : dividor_coeff_low);
		coeff_y[i][1] = ((DPxGetReg16(addr_y_L+2*i) & 0x8000 ? -1 : 1) * (DPxGetReg16(addr_y_L+2*i) & 0x3FFF)) /  (DPxGetReg16(addr_y_L+2*i) & 0x4000 ? dividor_coeff_high : dividor_coeff_low);
	}


	
	dividor = 1 << 8;
	DPxUpdateRegCache();
	LUXA_RIGHT_X = ((short)DPxGetReg16(0x390)) / dividor;
	LUXA_RIGHT_Y = ((short)DPxGetReg16(0x392)) / dividor;
			
	LUXA_LEFT_X = ((short)DPxGetReg16(0x380)) / dividor;
	LUXA_LEFT_Y = ((short)DPxGetReg16(0x382)) / dividor; 

	x = LUXA_RIGHT_X;
	y = LUXA_RIGHT_Y;

	eyeReturn[0] =  coeff_x[0][0]*1       +
					coeff_x[1][0]*x       +
					coeff_x[2][0]*y       +
					coeff_x[3][0]*x*x     +
					coeff_x[4][0]*y*y     +
					coeff_x[5][0]*x*x*x   +
					coeff_x[6][0]*x*y     +
					coeff_x[7][0]*x*x*y   +
					coeff_x[8][0]*x*x*y*y;
	

	eyeReturn[1] =  coeff_y[0][0]*1       +
					coeff_y[1][0]*x       +
					coeff_y[2][0]*y       +
					coeff_y[3][0]*x*x     +
					coeff_y[4][0]*y*y     +
					coeff_y[5][0]*x*x*x   +
					coeff_y[6][0]*x*y     +
					coeff_y[7][0]*x*x*y   +
					coeff_y[8][0]*x*x*y*y;
	
	x = LUXA_LEFT_X;
	y = LUXA_LEFT_Y;

	eyeReturn[2] =  coeff_x[0][1]*1       +
					coeff_x[1][1]*x       +
					coeff_x[2][1]*y       +
					coeff_x[3][1]*x*x     +
					coeff_x[4][1]*y*y     +
					coeff_x[5][1]*x*x*x   +
					coeff_x[6][1]*x*y     +
					coeff_x[7][1]*x*x*y   +
					coeff_x[8][1]*x*x*y*y;
	

	eyeReturn[3] =  coeff_y[0][1]*1       +
					coeff_y[1][1]*x       +
					coeff_y[2][1]*y       +
					coeff_y[3][1]*x*x     +
					coeff_y[4][1]*y*y     +
					coeff_y[5][1]*x*x*x   +
					coeff_y[6][1]*x*y     +
					coeff_y[7][1]*x*x*y   +
					coeff_y[8][1]*x*x*y*y;
}

void TPxGetEyePosition_lib(float coeff_x[9][2], float coeff_y[9][2], float* eyeReturn)
{

	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;
	float dividor;
	float x,y;

	dividor = 1 << 8;
	DPxUpdateRegCache();
	LUXA_RIGHT_X = ((short)DPxGetReg16(0x390)) / dividor;
	LUXA_RIGHT_Y = ((short)DPxGetReg16(0x392)) / dividor;
			
	LUXA_LEFT_X = ((short)DPxGetReg16(0x380)) / dividor;
	LUXA_LEFT_Y = ((short)DPxGetReg16(0x382)) / dividor; 

	x = LUXA_RIGHT_X;
	y = LUXA_RIGHT_Y;

	eyeReturn[0] =  coeff_x[0][0]*1       +
					coeff_x[1][0]*x       +
					coeff_x[2][0]*y       +
					coeff_x[3][0]*x*x     +
					coeff_x[4][0]*y*y     +
					coeff_x[5][0]*x*x*x   +
					coeff_x[6][0]*x*y     +
					coeff_x[7][0]*x*x*y   +
					coeff_x[8][0]*x*x*y*y;
	

	eyeReturn[1] =  coeff_y[0][0]*1       +
					coeff_y[1][0]*x       +
					coeff_y[2][0]*y       +
					coeff_y[3][0]*x*x     +
					coeff_y[4][0]*y*y     +
					coeff_y[5][0]*x*x*x   +
					coeff_y[6][0]*x*y     +
					coeff_y[7][0]*x*x*y   +
					coeff_y[8][0]*x*x*y*y;
	
	x = LUXA_LEFT_X;
	y = LUXA_LEFT_Y;

	eyeReturn[2] =  coeff_x[0][1]*1       +
					coeff_x[1][1]*x       +
					coeff_x[2][1]*y       +
					coeff_x[3][1]*x*x     +
					coeff_x[4][1]*y*y     +
					coeff_x[5][1]*x*x*x   +
					coeff_x[6][1]*x*y     +
					coeff_x[7][1]*x*x*y   +
					coeff_x[8][1]*x*x*y*y;
	

	eyeReturn[3] =  coeff_y[0][1]*1       +
					coeff_y[1][1]*x       +
					coeff_y[2][1]*y       +
					coeff_y[3][1]*x*x     +
					coeff_y[4][1]*y*y     +
					coeff_y[5][1]*x*x*x   +
					coeff_y[6][1]*x*y     +
					coeff_y[7][1]*x*x*y   +
					coeff_y[8][1]*x*x*y*y;
	//printf("\n\n *** Final Polynomial In GetEye_LIB in X_L *** \n\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", coeff_x[0][1],\
	//			coeff_x[1][1], coeff_x[2][1], coeff_x[3][1], coeff_x[4][1], coeff_x[5][1], coeff_x[6][1], coeff_x[7][1], coeff_x[8][1]);
	return;
}

int TPxFinishCalibration_poly1(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2])
{
	// TO BE DONE
	/* In this case we use
	Sx = A + Bx + Cx^3 + Dy^2+ Exy
	Sy = A + Bx + Cx^2 + Dy + Ey^2 + Fxy + Gx^2y
	*/
#if USE_GSL

	
	gsl_vector *x_screen = gsl_vector_alloc(9);
	gsl_vector *y_screen = gsl_vector_alloc(9);
	gsl_vector *x_screen_L = gsl_vector_alloc(9);
	gsl_vector *y_screen_L = gsl_vector_alloc(9);

	

	// Right eye!
	gsl_matrix *Ax_matrix_track = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff = gsl_vector_alloc(5);
	gsl_vector *y_coeff = gsl_vector_alloc(7);

	gsl_permutation *permu_x = gsl_permutation_alloc(5); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(7); int v;

	// Left Eye too!

	gsl_matrix *Ax_matrix_track_L = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track_L = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track_L = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track_L = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff_L = gsl_vector_alloc(5);
	gsl_vector *y_coeff_L = gsl_vector_alloc(7);

	gsl_permutation *permu_x_L = gsl_permutation_alloc(5); int s_L;
	gsl_permutation *permu_y_L = gsl_permutation_alloc(7); int v_L;

	double det_x, det_y; // Determinants
	int i;
	float x,y;
	float x_tracker[2], y_tracker[2], error_x, error_y, error_x_L, error_y_L;
	float error_avg = 0;
	float error_avg_L = 0;
	float eyeReturn[4];
	FILE* fp;
	fp = fopen("NewCalib_debug.txt", "a+");

	for (i = 0; i < 9; i++)
	{
		fp = fopen("NewCalib_debug.txt", "a+");
		fprintf(fp, "Writing in the text file! Loop %d.\n", i);
		fclose(fp);
		gsl_vector_set(x_screen, i, screen_data[i][0]);
		gsl_vector_set(y_screen, i, screen_data[i][1]);
		gsl_vector_set(x_screen_L, i, screen_data[i][0]);
		gsl_vector_set(y_screen_L, i, screen_data[i][1]);

		x = eye_data[i][0];
		y = eye_data[i][1];
		// Set up A_x
		gsl_matrix_set(Ax_matrix_track, i, 0, 1);
		gsl_matrix_set(Ax_matrix_track, i, 1, x);
		gsl_matrix_set(Ax_matrix_track, i, 2, x*x*x);
		gsl_matrix_set(Ax_matrix_track, i, 3, y*y);	
		gsl_matrix_set(Ax_matrix_track, i, 4, x*y);
		// Set up A_y
		gsl_matrix_set(Ay_matrix_track, i, 0, 1);
		gsl_matrix_set(Ay_matrix_track, i, 1, x);
		gsl_matrix_set(Ay_matrix_track, i, 2, x*x);
		gsl_matrix_set(Ay_matrix_track, i, 3, y);	
		gsl_matrix_set(Ay_matrix_track, i, 4, y*y);
		gsl_matrix_set(Ay_matrix_track, i, 5, x*y);
		gsl_matrix_set(Ay_matrix_track, i, 6, x*x*y);

		x = eye_data[i][2];
		y = eye_data[i][3];

		// Set up A_x_L
		gsl_matrix_set(Ax_matrix_track_L, i, 0, 1);
		gsl_matrix_set(Ax_matrix_track_L, i, 1, x);
		gsl_matrix_set(Ax_matrix_track_L, i, 2, x*x*x);
		gsl_matrix_set(Ax_matrix_track_L, i, 3, y*y);	
		gsl_matrix_set(Ax_matrix_track_L, i, 4, x*y);
		// Set up A_y_L
		gsl_matrix_set(Ay_matrix_track_L, i, 0, 1);
		gsl_matrix_set(Ay_matrix_track_L, i, 1, x);
		gsl_matrix_set(Ay_matrix_track_L, i, 2, x*x);
		gsl_matrix_set(Ay_matrix_track_L, i, 3, y);	
		gsl_matrix_set(Ay_matrix_track_L, i, 4, y*y);
		gsl_matrix_set(Ay_matrix_track_L, i, 5, x*y);
		gsl_matrix_set(Ay_matrix_track_L, i, 6, x*x*y);
	}
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Matrix all initiated.\n");
	fclose(fp);
	// Need to make A_x, A_y, A_x_L and A_y_L square matrices. Multiply both side by A^T
	//RHS
	gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track, x_screen, 0.0, x_coeff); // x_coeff = A^T * x_screen; Store in coeff(result) because same size for now
	gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track, y_screen, 0.0, y_coeff); // y_coeff = A^T * y_screen;

	//LHS (Ax has been modified after that, it is in its new square form!)
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track, Ax_matrix_track, 0.0, AxS_matrix_track); // Ax_matrix_track = A^T* A
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track, Ay_matrix_track, 0.0, AyS_matrix_track); // Ay_matrix_track = A^T* A

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Right Matrix should all be squared.\n");
	fclose(fp);
	gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track_L, x_screen_L, 0.0, x_coeff_L); // x_screen = A^T * x_screen;
	gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track_L, y_screen_L, 0.0, y_coeff_L); // y_screen = A^T * y_screen;
	//LHS (Ax has been modified after that, it is in its new square form!)
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track_L, Ax_matrix_track_L, 0.0, AxS_matrix_track_L); // Ax_matrix_track = A^T* A
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track_L, Ay_matrix_track_L, 0.0, AyS_matrix_track_L); // Ay_matrix_track = A^T* A
	
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Left Matrix should all be squared\n");
	fclose(fp);
	// Calculations!
	gsl_linalg_LU_decomp(AxS_matrix_track, permu_x, &s);	
	gsl_linalg_LU_decomp(AyS_matrix_track, permu_y, &v);

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "LU decom done for right\n");
	fclose(fp);
	// Calculations!
	gsl_linalg_LU_decomp(AxS_matrix_track_L, permu_x_L, &s_L);	
	gsl_linalg_LU_decomp(AyS_matrix_track_L, permu_y_L, &v_L);
		
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "LU decom done for left\n");
	fclose(fp);
	det_x = gsl_linalg_LU_det(AxS_matrix_track, s);
	det_y = gsl_linalg_LU_det(AyS_matrix_track, v);

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Just calculated determinants: det_x:%f, det_y:%f\n", det_x, det_y);

	if (det_x == 0 || det_y == 0)
	{ // A matrix is singular, cannot solve.
		fprintf(fp, "Not solvable system");
	}
	else
	{
		fclose(fp);
		gsl_linalg_LU_solve(AxS_matrix_track, permu_x, x_coeff, x_coeff);
		gsl_linalg_LU_solve(AyS_matrix_track, permu_y, y_coeff, y_coeff);

		gsl_linalg_LU_solve(AxS_matrix_track_L, permu_x_L, x_coeff_L, x_coeff_L);
		gsl_linalg_LU_solve(AyS_matrix_track_L, permu_y_L, y_coeff_L, y_coeff_L);
		fp = fopen("NewCalib_debug.txt", "a+");
		fprintf(fp, "LU solved done for right\n");
		// 
		coeff_x[0][0] = (float)gsl_vector_get(x_coeff, 0);
		coeff_x[0][1] = (float)gsl_vector_get(x_coeff_L, 0);
		coeff_x[1][0] = (float)gsl_vector_get(x_coeff, 1);
		coeff_x[1][1] = (float)gsl_vector_get(x_coeff_L, 1);
		coeff_x[2][0] = 0; coeff_x[2][1] = 0;
		coeff_x[3][0] = (float)gsl_vector_get(x_coeff, 3);
		coeff_x[3][1] = (float)gsl_vector_get(x_coeff_L, 3);
		coeff_x[4][0] = (float)gsl_vector_get(x_coeff, 2);
		coeff_x[4][1] = (float)gsl_vector_get(x_coeff_L, 2);
		coeff_x[5][0] = 0;	coeff_x[5][1] = 0;
		coeff_x[6][0] = 0;	coeff_x[6][1] = 0;
		coeff_x[7][0] = 0;	coeff_x[7][1] = 0;
		coeff_x[8][0] = 0;	coeff_x[8][1] = 0;

		coeff_y[0][0] = (float)gsl_vector_get(y_coeff, 0);
		coeff_y[0][1] = (float)gsl_vector_get(y_coeff_L, 0);
		coeff_y[1][0] = (float)gsl_vector_get(y_coeff, 1);
		coeff_y[1][1] = (float)gsl_vector_get(y_coeff_L, 1);
		coeff_y[2][0] = (float)gsl_vector_get(y_coeff, 3); 
		coeff_y[2][1] = (float)gsl_vector_get(y_coeff_L, 3);
		coeff_y[3][0] = (float)gsl_vector_get(y_coeff, 2);
		coeff_y[3][1] = (float)gsl_vector_get(y_coeff_L, 2);
		coeff_y[4][0] = (float)gsl_vector_get(y_coeff, 4);
		coeff_y[4][1] = (float)gsl_vector_get(y_coeff_L, 4);
		coeff_y[5][0] = 0;	coeff_y[5][1] = 0;
		coeff_y[6][0] = (float)gsl_vector_get(y_coeff, 5);	
		coeff_y[6][1] = (float)gsl_vector_get(y_coeff_L, 5);
		coeff_y[7][0] = (float)gsl_vector_get(y_coeff, 6);	
		coeff_y[7][1] = (float)gsl_vector_get(y_coeff_L, 6);
		coeff_y[8][0] = 0;	coeff_y[8][1] = 0;

	} 		// STEP ONE DONE
	
	
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ay_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ay_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);


	// Now we need to calculate the distance between the calibration and the 5 validation points

	for (i = 9; i < 14; i++)
	{
		x_tracker[0] = eye_data[i][0];
		y_tracker[0] = eye_data[i][1];
		x_tracker[1] = eye_data[i][2];
		y_tracker[1] = eye_data[i][3];
		TPxGetEyePosition_validation(x_tracker, y_tracker, coeff_x, coeff_y, eyeReturn);
		error_x = screen_data[i][0] - eyeReturn[0];
		error_y = screen_data[i][1] - eyeReturn[1];
		error_x_L = screen_data[i][0] - eyeReturn[0];
		error_y_L = screen_data[i][1] - eyeReturn[1];
		error_avg += (float)sqrt(error_x*error_x + error_y*error_y);
		error_avg_L += (float)sqrt(error_x_L*error_x_L+ error_y_L*error_y_L);
	}
	error_avg /= 5;
	error_avg_L /= 5;

	fprintf(fp, "Matrix freed, work here is done.\n");
	fclose(fp);
	return 1;
#endif
}


int TPxFinishCalibration_poly1_returns_error(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x[2], float error_y[2])
{
	// TO BE DONE
	/* In this case we use
	Sx = A + Bx + Cx^3 + Dy^2+ Exy
	Sy = A + Bx + Cx^2 + Dy + Ey^2 + Fxy + Gx^2y
	*/
#if USE_GSL

	
	gsl_vector *x_screen = gsl_vector_alloc(9);
	gsl_vector *y_screen = gsl_vector_alloc(9);
	gsl_vector *x_screen_L = gsl_vector_alloc(9);
	gsl_vector *y_screen_L = gsl_vector_alloc(9);

	

	// Right eye!
	gsl_matrix *Ax_matrix_track = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff = gsl_vector_alloc(5);
	gsl_vector *y_coeff = gsl_vector_alloc(7);

	gsl_permutation *permu_x = gsl_permutation_alloc(5); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(7); int v;

	// Left Eye too!

	gsl_matrix *Ax_matrix_track_L = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track_L = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track_L = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track_L = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff_L = gsl_vector_alloc(5);
	gsl_vector *y_coeff_L = gsl_vector_alloc(7);

	gsl_permutation *permu_x_L = gsl_permutation_alloc(5); int s_L;
	gsl_permutation *permu_y_L = gsl_permutation_alloc(7); int v_L;

	double det_x, det_y; // Determinants
	int i;
	float x,y;
	float x_tracker[2], y_tracker[2], error_x_curr, error_y_curr, error_x_L, error_y_L;
	float error_avg = 0;
	float error_avg_L = 0;
	float eyeReturn[4];
	FILE* fp;
	fp = fopen("NewCalib_debug.txt", "a+");

	for (i = 0; i < 9; i++)
	{
		fp = fopen("NewCalib_debug.txt", "a+");
		fprintf(fp, "Writing in the text file! Loop %d.\n", i);
		fclose(fp);
		gsl_vector_set(x_screen, i, screen_data[i][0]);
		gsl_vector_set(y_screen, i, screen_data[i][1]);
		gsl_vector_set(x_screen_L, i, screen_data[i][0]);
		gsl_vector_set(y_screen_L, i, screen_data[i][1]);

		x = eye_data[i][0];
		y = eye_data[i][1];
		// Set up A_x
		gsl_matrix_set(Ax_matrix_track, i, 0, 1);
		gsl_matrix_set(Ax_matrix_track, i, 1, x);
		gsl_matrix_set(Ax_matrix_track, i, 2, x*x*x);
		gsl_matrix_set(Ax_matrix_track, i, 3, y*y);	
		gsl_matrix_set(Ax_matrix_track, i, 4, x*y);
		// Set up A_y
		gsl_matrix_set(Ay_matrix_track, i, 0, 1);
		gsl_matrix_set(Ay_matrix_track, i, 1, x);
		gsl_matrix_set(Ay_matrix_track, i, 2, x*x);
		gsl_matrix_set(Ay_matrix_track, i, 3, y);	
		gsl_matrix_set(Ay_matrix_track, i, 4, y*y);
		gsl_matrix_set(Ay_matrix_track, i, 5, x*y);
		gsl_matrix_set(Ay_matrix_track, i, 6, x*x*y);

		x = eye_data[i][2];
		y = eye_data[i][3];

		// Set up A_x_L
		gsl_matrix_set(Ax_matrix_track_L, i, 0, 1);
		gsl_matrix_set(Ax_matrix_track_L, i, 1, x);
		gsl_matrix_set(Ax_matrix_track_L, i, 2, x*x*x);
		gsl_matrix_set(Ax_matrix_track_L, i, 3, y*y);	
		gsl_matrix_set(Ax_matrix_track_L, i, 4, x*y);
		// Set up A_y_L
		gsl_matrix_set(Ay_matrix_track_L, i, 0, 1);
		gsl_matrix_set(Ay_matrix_track_L, i, 1, x);
		gsl_matrix_set(Ay_matrix_track_L, i, 2, x*x);
		gsl_matrix_set(Ay_matrix_track_L, i, 3, y);	
		gsl_matrix_set(Ay_matrix_track_L, i, 4, y*y);
		gsl_matrix_set(Ay_matrix_track_L, i, 5, x*y);
		gsl_matrix_set(Ay_matrix_track_L, i, 6, x*x*y);
	}
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Matrix all initiated.\n");
	fclose(fp);
	// Need to make A_x, A_y, A_x_L and A_y_L square matrices. Multiply both side by A^T
	//RHS
	gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track, x_screen, 0.0, x_coeff); // x_coeff = A^T * x_screen; Store in coeff(result) because same size for now
	gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track, y_screen, 0.0, y_coeff); // y_coeff = A^T * y_screen;

	//LHS (Ax has been modified after that, it is in its new square form!)
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track, Ax_matrix_track, 0.0, AxS_matrix_track); // Ax_matrix_track = A^T* A
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track, Ay_matrix_track, 0.0, AyS_matrix_track); // Ay_matrix_track = A^T* A

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Right Matrix should all be squared.\n");
	fclose(fp);
	gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track_L, x_screen_L, 0.0, x_coeff_L); // x_screen = A^T * x_screen;
	gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track_L, y_screen_L, 0.0, y_coeff_L); // y_screen = A^T * y_screen;
	//LHS (Ax has been modified after that, it is in its new square form!)
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track_L, Ax_matrix_track_L, 0.0, AxS_matrix_track_L); // Ax_matrix_track = A^T* A
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track_L, Ay_matrix_track_L, 0.0, AyS_matrix_track_L); // Ay_matrix_track = A^T* A
	
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Left Matrix should all be squared\n");
	fclose(fp);
	// Calculations!
	gsl_linalg_LU_decomp(AxS_matrix_track, permu_x, &s);	
	gsl_linalg_LU_decomp(AyS_matrix_track, permu_y, &v);

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "LU decom done for right\n");
	fclose(fp);
	// Calculations!
	gsl_linalg_LU_decomp(AxS_matrix_track_L, permu_x_L, &s_L);	
	gsl_linalg_LU_decomp(AyS_matrix_track_L, permu_y_L, &v_L);
		
	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "LU decom done for left\n");
	fclose(fp);
	det_x = gsl_linalg_LU_det(AxS_matrix_track, s);
	det_y = gsl_linalg_LU_det(AyS_matrix_track, v);

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Just calculated determinants: det_x:%f, det_y:%f\n", det_x, det_y);

	if (det_x == 0 || det_y == 0)
	{ // A matrix is singular, cannot solve.
		fprintf(fp, "Not solvable system");
	}
	else
	{
		fclose(fp);
		gsl_linalg_LU_solve(AxS_matrix_track, permu_x, x_coeff, x_coeff);
		gsl_linalg_LU_solve(AyS_matrix_track, permu_y, y_coeff, y_coeff);

		gsl_linalg_LU_solve(AxS_matrix_track_L, permu_x_L, x_coeff_L, x_coeff_L);
		gsl_linalg_LU_solve(AyS_matrix_track_L, permu_y_L, y_coeff_L, y_coeff_L);
		fp = fopen("NewCalib_debug.txt", "a+");
		fprintf(fp, "LU solved done for right\n");
		// 
		coeff_x[0][0] = (float)gsl_vector_get(x_coeff, 0);
		coeff_x[0][1] = (float)gsl_vector_get(x_coeff_L, 0);
		coeff_x[1][0] = (float)gsl_vector_get(x_coeff, 1);
		coeff_x[1][1] = (float)gsl_vector_get(x_coeff_L, 1);
		coeff_x[2][0] = 0; coeff_x[2][1] = 0;
		coeff_x[3][0] = (float)gsl_vector_get(x_coeff, 3);
		coeff_x[3][1] = (float)gsl_vector_get(x_coeff_L, 3);
		coeff_x[4][0] = (float)gsl_vector_get(x_coeff, 2);
		coeff_x[4][1] = (float)gsl_vector_get(x_coeff_L, 2);
		coeff_x[5][0] = 0;	coeff_x[5][1] = 0;
		coeff_x[6][0] = 0;	coeff_x[6][1] = 0;
		coeff_x[7][0] = 0;	coeff_x[7][1] = 0;
		coeff_x[8][0] = 0;	coeff_x[8][1] = 0;

		coeff_y[0][0] = (float)gsl_vector_get(y_coeff, 0);
		coeff_y[0][1] = (float)gsl_vector_get(y_coeff_L, 0);
		coeff_y[1][0] = (float)gsl_vector_get(y_coeff, 1);
		coeff_y[1][1] = (float)gsl_vector_get(y_coeff_L, 1);
		coeff_y[2][0] = (float)gsl_vector_get(y_coeff, 3); 
		coeff_y[2][1] = (float)gsl_vector_get(y_coeff_L, 3);
		coeff_y[3][0] = (float)gsl_vector_get(y_coeff, 2);
		coeff_y[3][1] = (float)gsl_vector_get(y_coeff_L, 2);
		coeff_y[4][0] = (float)gsl_vector_get(y_coeff, 4);
		coeff_y[4][1] = (float)gsl_vector_get(y_coeff_L, 4);
		coeff_y[5][0] = 0;	coeff_y[5][1] = 0;
		coeff_y[6][0] = (float)gsl_vector_get(y_coeff, 5);	
		coeff_y[6][1] = (float)gsl_vector_get(y_coeff_L, 5);
		coeff_y[7][0] = (float)gsl_vector_get(y_coeff, 6);	
		coeff_y[7][1] = (float)gsl_vector_get(y_coeff_L, 6);
		coeff_y[8][0] = 0;	coeff_y[8][1] = 0;

	} 		// STEP ONE DONE
	
	
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ay_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ay_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);


	// Now we need to calculate the distance between the calibration and the 5 validation points
	error_x_curr = 0;
	error_y_curr = 0;
	for (i = 9; i < 14; i++)
	{
		x_tracker[0] = eye_data[i][0];
		y_tracker[0] = eye_data[i][1];
		x_tracker[1] = eye_data[i][2];
		y_tracker[1] = eye_data[i][3];
		TPxGetEyePosition_validation(x_tracker, y_tracker, coeff_x, coeff_y, eyeReturn);
		error_x_curr += screen_data[i][0] - eyeReturn[0];
		error_y_curr += screen_data[i][1] - eyeReturn[1];
		error_x_L = screen_data[i][0] - eyeReturn[0];
		error_y_L = screen_data[i][1] - eyeReturn[1];
		error_avg += (float)sqrt(error_x_curr*error_x_curr + error_y_curr*error_y_curr);
		error_avg_L += (float)sqrt(error_x_L*error_x_L+ error_y_L*error_y_L);
	}
	error_x_curr /= 5;
	error_y_curr /= 5;
	error_avg /= 5;
	error_avg_L /= 5;
	error_x[0] = error_x_curr;
	error_y[0] = error_y_curr;
	fprintf(fp, "Matrix freed, work here is done.\n");
	fclose(fp);
	return error_avg < error_avg_L? (int)error_avg : (int)error_avg_L;
#endif
}
int TPxFinishCalibration_poly_general_3terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2])
{
	/* In this case we use
	Sx = A + Bx + Cx^3 + Dy^2+ Exy
	Sy = A + Bx + Cx^2 + Dy + Ey^2 + Fxy + Gx^2y
	*/
#if USE_GSL
	gsl_vector *x_screen = gsl_vector_alloc(9);
	gsl_vector *y_screen = gsl_vector_alloc(9);
	gsl_vector *x_screen_L = gsl_vector_alloc(9);
	gsl_vector *y_screen_L = gsl_vector_alloc(9);
		// Right eye!
	gsl_matrix *Ax_matrix_track = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track = gsl_matrix_alloc(9,5);
	gsl_matrix *AxS_matrix_track = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track = gsl_matrix_alloc(5,5);

	gsl_vector *x_coeff = gsl_vector_alloc(5);
	gsl_vector *y_coeff = gsl_vector_alloc(5);

	gsl_permutation *permu_x = gsl_permutation_alloc(5); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(5); int v;

	// Left Eye too!

	gsl_matrix *Ax_matrix_track_L = gsl_matrix_alloc(9,5);
	gsl_matrix *Ay_matrix_track_L = gsl_matrix_alloc(9,5);
	gsl_matrix *AxS_matrix_track_L = gsl_matrix_alloc(5,5);
	gsl_matrix *AyS_matrix_track_L = gsl_matrix_alloc(5,5);

	gsl_vector *x_coeff_L = gsl_vector_alloc(5);
	gsl_vector *y_coeff_L = gsl_vector_alloc(5);

	gsl_permutation *permu_x_L = gsl_permutation_alloc(5); int s_L;
	gsl_permutation *permu_y_L = gsl_permutation_alloc(5); int v_L;

	double det_x, det_y; // Determinants
	int i,j,k,l;
	int sanity_checker;
	float x,y,x_L, y_L;
	float x_tracker[2], y_tracker[2], error_x, error_y, error_x_L, error_y_L;
	float coeff_x_temp[9][2];
	float coeff_y_temp[9][2];
	float error_avg = 0;
	float error_avg_L = 0;
	float eyeReturn[4];
	sanity_checker = 0;
	for (j = 0; j < 125; j++)
	{ // doing 3 terms, so hamming weight must be 3.
		if (__popcnt(j) != 3)
			continue;
		sanity_checker++;
		for (i = 0; i < 9; i++)
		{
			gsl_vector_set(x_screen, i, screen_data[i][0]);
			gsl_vector_set(y_screen, i, screen_data[i][1]);
			gsl_vector_set(x_screen_L, i, screen_data[i][0]);
			gsl_vector_set(y_screen_L, i, screen_data[i][1]);

			x = eye_data[i][0];
			y = eye_data[i][1];
			x_L = eye_data[i][2];
			y_L = eye_data[i][3];

			gsl_matrix_set(Ax_matrix_track, i, 0, 1);
			gsl_matrix_set(Ax_matrix_track_L, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track_L, i, 0, 1);

			gsl_matrix_set(Ax_matrix_track, i, 1, x);
			gsl_matrix_set(Ax_matrix_track_L, i, 1, x);

			gsl_matrix_set(Ay_matrix_track, i, 1, y);
			gsl_matrix_set(Ay_matrix_track_L, i, 1, y);
			k = 2;
		// Do X first 1 x always included, then 3 from
		// y x^2 y^2 x^3 xy x^2y x^2y^2
			if (j & (1 << 0)) // y
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y);
				gsl_matrix_set(Ay_matrix_track, i, k, x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L);
			}
			if (j & (1 << 1))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L);
			}
			if (j & (1 << 2))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, y_L*y_L);
			}
			if (j & (1 << 3))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*x_L);
			}
			if (j & (1 << 4))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*y_L);
			}
			if (j & (1 << 5))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L);
			}
			if (j & (1 << 6))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L*y_L);
			}
		}
		// Need to calculate!
		//RHS
		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track, x_screen, 0.0, x_coeff); // x_coeff = A^T * x_screen; Store in coeff(result) because same size for now
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track, y_screen, 0.0, y_coeff); // y_coeff = A^T * y_screen;

		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track, Ax_matrix_track, 0.0, AxS_matrix_track); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track, Ay_matrix_track, 0.0, AyS_matrix_track); // Ay_matrix_track = A^T* A

		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track_L, x_screen_L, 0.0, x_coeff_L); // x_screen = A^T * x_screen;
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track_L, y_screen_L, 0.0, y_coeff_L); // y_screen = A^T * y_screen;
		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track_L, Ax_matrix_track_L, 0.0, AxS_matrix_track_L); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track_L, Ay_matrix_track_L, 0.0, AyS_matrix_track_L); // Ay_matrix_track = A^T* A
	
		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track, permu_x, &s);	
		gsl_linalg_LU_decomp(AyS_matrix_track, permu_y, &v);

		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track_L, permu_x_L, &s_L);	
		gsl_linalg_LU_decomp(AyS_matrix_track_L, permu_y_L, &v_L);
		
		det_x = gsl_linalg_LU_det(AxS_matrix_track, s);
		det_y = gsl_linalg_LU_det(AyS_matrix_track, v);

		if (det_x == 0 || det_y == 0)
		{ // A matrix is singular, cannot solve.
			return -1;
		}
		else
		{
			gsl_linalg_LU_solve(AxS_matrix_track, permu_x, x_coeff, x_coeff);
			gsl_linalg_LU_solve(AyS_matrix_track, permu_y, y_coeff, y_coeff);

			gsl_linalg_LU_solve(AxS_matrix_track_L, permu_x_L, x_coeff_L, x_coeff_L);
			gsl_linalg_LU_solve(AyS_matrix_track_L, permu_y_L, y_coeff_L, y_coeff_L);
			
			
			// Set Coeffs! Start by emptying previous ones
			for (l = 0; l < 9; l++)
			{
				coeff_x_temp[l][0] = 0;
				coeff_x_temp[l][1] = 0;
				coeff_y_temp[l][0] = 0;
				coeff_y_temp[l][1] = 0;

			}

			// Set the A, x and A, y
			//cst
			coeff_x_temp[0][0] = (float)gsl_vector_get(x_coeff, 0);
			coeff_x_temp[0][1] = (float)gsl_vector_get(x_coeff_L, 0);
			coeff_y_temp[0][0] = (float)gsl_vector_get(y_coeff, 0);
			coeff_y_temp[0][1] = (float)gsl_vector_get(y_coeff_L, 0);
			
			//x
			coeff_x_temp[1][0] = (float)gsl_vector_get(x_coeff, 1);
			coeff_x_temp[1][1] = (float)gsl_vector_get(x_coeff_L, 1);
			//y
			coeff_y_temp[2][0] = (float)gsl_vector_get(y_coeff, 1);
			coeff_y_temp[2][1] = (float)gsl_vector_get(y_coeff_L, 1);

			k = 2;
			if (j & (1 << 0)) // y, or x!
			{
				//y
				coeff_x_temp[2][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[2][1] = (float)gsl_vector_get(x_coeff_L, k);
				//x
				coeff_y_temp[1][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[1][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 1))
			{
				coeff_x_temp[3][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[3][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[3][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[3][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 2))
			{
				coeff_x_temp[4][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[4][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[4][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[4][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 3))
			{
				coeff_x_temp[5][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[5][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[5][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[5][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 4))
			{
				coeff_x_temp[6][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[6][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[6][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[6][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 5))
			{
				coeff_x_temp[7][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[7][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[7][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[7][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 6))
			{
				coeff_x_temp[8][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[8][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[8][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[8][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
		}
	// Now we need to calculate the error see if there is any improvement from this algorithm
		error_x = 0;
		error_x_L = 0;
		error_y = 0;
		error_y_L = 0;
		for (i = 9; i < 14; i++)
		{
			x_tracker[0] = eye_data[i][0];
			y_tracker[0] = eye_data[i][1];
			x_tracker[1] = eye_data[i][2];
			y_tracker[1] = eye_data[i][3];
			TPxGetEyePosition_validation(x_tracker, y_tracker, coeff_x_temp, coeff_y_temp, eyeReturn);
			error_x += (float)fabs(screen_data[i][0] - eyeReturn[0]);
			error_y += (float)fabs(screen_data[i][1] - eyeReturn[1]);
			error_x_L += (float)fabs(screen_data[i][0] - eyeReturn[2]);
			error_y_L += (float)fabs(screen_data[i][1] - eyeReturn[3]);
			error_avg += (float)sqrt(error_x*error_x + error_y*error_y);
			error_avg_L +=(float)sqrt(error_x_L*error_x_L+ error_y_L*error_y_L);
		}
		error_x /= 5;
		error_x_L /= 5;
		error_y /= 5;
		error_y_L /= 5;
		error_avg /= 5;
		error_avg_L /= 5;
		//printf("Sanity checker (7 choose 3):%d    Current errors: X:%f, Y:%f\n", sanity_checker, error_x, error_y);
		if (error_x < error_x_min[0]) // New winner on x, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][0] = coeff_x_temp[l][0];
			}
			//printf("Iteration %d: Error X improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x, error_x_min[0], coeff_x_temp[0][0], coeff_x_temp[1][0],\
			//	coeff_x_temp[2][0], coeff_x_temp[3][0], coeff_x_temp[4][0], coeff_x_temp[5][0], coeff_x_temp[6][0], coeff_x_temp[7][0], coeff_x_temp[8][0]);
			error_x_min[0] = error_x;
		}
		
		if (error_x_L < error_x_min[1]) // New winner on x_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][1] = coeff_x_temp[l][1];
			}
			//printf("Iteration %d: Error X_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x_L, error_x_min[1], coeff_x_temp[0][1], coeff_x_temp[1][1],\
			//	coeff_x_temp[2][1], coeff_x_temp[3][1], coeff_x_temp[4][1], coeff_x_temp[5][1], coeff_x_temp[6][1], coeff_x_temp[7][1], coeff_x_temp[8][1]);
			error_x_min[1] = error_x_L;
		}

		if (error_y < error_y_min[0]) // New winner on y, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][0] = coeff_y_temp[l][0];
			}
			//printf("Iteration %d: Error Y improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y, error_y_min[0], coeff_y_temp[0][0], coeff_y_temp[1][0],\
			//	coeff_y_temp[2][0], coeff_y_temp[3][0], coeff_y_temp[4][0], coeff_y_temp[5][0], coeff_y_temp[6][0], coeff_y_temp[7][0], coeff_y_temp[8][0]);
			error_y_min[0] = error_y;
		}

		if (error_y_L < error_y_min[1]) // New winner on y_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][1] = coeff_y_temp[l][1];
			}
			//printf("Iteration %d: Error Y_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y_L, error_y_min[1], coeff_y_temp[0][1], coeff_y_temp[1][1],\
			//	coeff_y_temp[2][1], coeff_y_temp[3][1], coeff_y_temp[4][1], coeff_y_temp[5][1], coeff_y_temp[6][1], coeff_y_temp[7][1], coeff_y_temp[8][1]);
			error_y_min[1] = error_y_L;
		}


	}//EXIT OF all poly loop
		
		// STEP ONE DONE
	
	
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ay_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ay_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);


	// Now we need to calculate the distance between the calibration and the 5 validation points

	return 3;
#endif
}

int TPxFinishCalibration_poly_general_4terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2])
{
	/* In this case we use
	Sx = A + Bx + Cx^3 + Dy^2+ Exy
	Sy = A + Bx + Cx^2 + Dy + Ey^2 + Fxy + Gx^2y
	*/
#if USE_GSL
	gsl_vector *x_screen = gsl_vector_alloc(9);
	gsl_vector *y_screen = gsl_vector_alloc(9);
	gsl_vector *x_screen_L = gsl_vector_alloc(9);
	gsl_vector *y_screen_L = gsl_vector_alloc(9);
		// Right eye!
	gsl_matrix *Ax_matrix_track = gsl_matrix_alloc(9,6);
	gsl_matrix *Ay_matrix_track = gsl_matrix_alloc(9,6);
	gsl_matrix *AxS_matrix_track = gsl_matrix_alloc(6,6);
	gsl_matrix *AyS_matrix_track = gsl_matrix_alloc(6,6);

	gsl_vector *x_coeff = gsl_vector_alloc(6);
	gsl_vector *y_coeff = gsl_vector_alloc(6);

	gsl_permutation *permu_x = gsl_permutation_alloc(6); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(6); int v;

	// Left Eye too!

	gsl_matrix *Ax_matrix_track_L = gsl_matrix_alloc(9,6);
	gsl_matrix *Ay_matrix_track_L = gsl_matrix_alloc(9,6);
	gsl_matrix *AxS_matrix_track_L = gsl_matrix_alloc(6,6);
	gsl_matrix *AyS_matrix_track_L = gsl_matrix_alloc(6,6);

	gsl_vector *x_coeff_L = gsl_vector_alloc(6);
	gsl_vector *y_coeff_L = gsl_vector_alloc(6);

	gsl_permutation *permu_x_L = gsl_permutation_alloc(6); int s_L;
	gsl_permutation *permu_y_L = gsl_permutation_alloc(6); int v_L;

	double det_x, det_y; // Determinants
	int i,j,k,l;
	int sanity_checker;
	float x,y,x_L, y_L;
	float x_tracker[2], y_tracker[2], error_x, error_y, error_x_L, error_y_L;
	float coeff_x_temp[9][2];
	float coeff_y_temp[9][2];
	float error_avg = 0;
	float error_avg_L = 0;
	float eyeReturn[4];
	sanity_checker = 0;
	for (j = 0; j < 125; j++)
	{ // doing 3 terms, so hamming weight must be 3.
		if (__popcnt(j) != 4)
			continue;
		sanity_checker++;
		for (i = 0; i < 9; i++)
		{
			gsl_vector_set(x_screen, i, screen_data[i][0]);
			gsl_vector_set(y_screen, i, screen_data[i][1]);
			gsl_vector_set(x_screen_L, i, screen_data[i][0]);
			gsl_vector_set(y_screen_L, i, screen_data[i][1]);

			x = eye_data[i][0];
			y = eye_data[i][1];
			x_L = eye_data[i][2];
			y_L = eye_data[i][3];

			gsl_matrix_set(Ax_matrix_track, i, 0, 1);
			gsl_matrix_set(Ax_matrix_track_L, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track_L, i, 0, 1);

			gsl_matrix_set(Ax_matrix_track, i, 1, x);
			gsl_matrix_set(Ax_matrix_track_L, i, 1, x);

			gsl_matrix_set(Ay_matrix_track, i, 1, y);
			gsl_matrix_set(Ay_matrix_track_L, i, 1, y);
			k = 2;
		// Do X first 1 x always included, then 3 from
		// y or x, x^2 y^2 x^3 xy x^2y x^2y^2
			if (j & (1 << 0)) // y
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y);
				gsl_matrix_set(Ay_matrix_track, i, k, x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L);
			}
			if (j & (1 << 1))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L);
			}
			if (j & (1 << 2))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, y_L*y_L);
			}
			if (j & (1 << 3))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*x_L);
			}
			if (j & (1 << 4))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*y_L);
			}
			if (j & (1 << 5))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L);
			}
			if (j & (1 << 6))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L*y_L);
			}
		}
		// Need to calculate!
		//RHS
		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track, x_screen, 0.0, x_coeff); // x_coeff = A^T * x_screen; Store in coeff(result) because same size for now
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track, y_screen, 0.0, y_coeff); // y_coeff = A^T * y_screen;

		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track, Ax_matrix_track, 0.0, AxS_matrix_track); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track, Ay_matrix_track, 0.0, AyS_matrix_track); // Ay_matrix_track = A^T* A

		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track_L, x_screen_L, 0.0, x_coeff_L); // x_screen = A^T * x_screen;
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track_L, y_screen_L, 0.0, y_coeff_L); // y_screen = A^T * y_screen;
		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track_L, Ax_matrix_track_L, 0.0, AxS_matrix_track_L); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track_L, Ay_matrix_track_L, 0.0, AyS_matrix_track_L); // Ay_matrix_track = A^T* A
	
		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track, permu_x, &s);	
		gsl_linalg_LU_decomp(AyS_matrix_track, permu_y, &v);

		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track_L, permu_x_L, &s_L);	
		gsl_linalg_LU_decomp(AyS_matrix_track_L, permu_y_L, &v_L);
		
		det_x = gsl_linalg_LU_det(AxS_matrix_track, s);
		det_y = gsl_linalg_LU_det(AyS_matrix_track, v);

		if (det_x == 0 || det_y == 0)
		{ // A matrix is singular, cannot solve.
			return -1;
		}
		else
		{
			gsl_linalg_LU_solve(AxS_matrix_track, permu_x, x_coeff, x_coeff);
			gsl_linalg_LU_solve(AyS_matrix_track, permu_y, y_coeff, y_coeff);

			gsl_linalg_LU_solve(AxS_matrix_track_L, permu_x_L, x_coeff_L, x_coeff_L);
			gsl_linalg_LU_solve(AyS_matrix_track_L, permu_y_L, y_coeff_L, y_coeff_L);
			
			
			// Set Coeffs! Start by emptying previous ones
			for (l = 0; l < 9; l++)
			{
				coeff_x_temp[l][0] = 0;
				coeff_x_temp[l][1] = 0;
				coeff_y_temp[l][0] = 0;
				coeff_y_temp[l][1] = 0;

			}

			// Set the A, x and A, y
			//cst
			coeff_x_temp[0][0] = (float)gsl_vector_get(x_coeff, 0);
			coeff_x_temp[0][1] = (float)gsl_vector_get(x_coeff_L, 0);
			coeff_y_temp[0][0] = (float)gsl_vector_get(y_coeff, 0);
			coeff_y_temp[0][1] = (float)gsl_vector_get(y_coeff_L, 0);
			
			//x
			coeff_x_temp[1][0] = (float)gsl_vector_get(x_coeff, 1);
			coeff_x_temp[1][1] = (float)gsl_vector_get(x_coeff_L, 1);
			//y
			coeff_y_temp[2][0] = (float)gsl_vector_get(y_coeff, 1);
			coeff_y_temp[2][1] = (float)gsl_vector_get(y_coeff_L, 1);

			k = 2;
			if (j & (1 << 0)) // y, or x!
			{
				//y
				coeff_x_temp[2][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[2][1] = (float)gsl_vector_get(x_coeff_L, k);
				//x
				coeff_y_temp[1][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[1][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 1))
			{
				coeff_x_temp[3][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[3][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[3][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[3][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 2))
			{
				coeff_x_temp[4][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[4][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[4][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[4][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 3))
			{
				coeff_x_temp[5][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[5][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[5][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[5][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 4))
			{
				coeff_x_temp[6][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[6][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[6][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[6][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 5))
			{
				coeff_x_temp[7][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[7][1] = (float)gsl_vector_get(x_coeff_L, k);

				coeff_y_temp[7][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[7][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 6))
			{
				coeff_x_temp[8][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[8][1] = (float)gsl_vector_get(x_coeff_L, k);

				coeff_y_temp[8][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[8][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
		}
	// Now we need to calculate the error see if there is any improvement from this algorithm
		error_x = 0;
		error_x_L = 0;
		error_y = 0;
		error_y_L = 0;
		for (i = 9; i < 14; i++)
		{
			x_tracker[0] = eye_data[i][0];
			y_tracker[0] = eye_data[i][1];
			x_tracker[1] = eye_data[i][2];
			y_tracker[1] = eye_data[i][3];
			TPxGetEyePosition_validation(x_tracker, y_tracker, coeff_x_temp, coeff_y_temp, eyeReturn);
			error_x += (float)fabs(screen_data[i][0] - eyeReturn[0]);
			error_y += (float)fabs(screen_data[i][1] - eyeReturn[1]);
			error_x_L += (float)fabs(screen_data[i][0] - eyeReturn[2]);
			error_y_L += (float)fabs(screen_data[i][1] - eyeReturn[3]);
			error_avg += (float)sqrt(error_x*error_x + error_y*error_y);
			error_avg_L +=(float)sqrt(error_x_L*error_x_L+ error_y_L*error_y_L);
		}
		error_x /= 5; // on average
		error_x_L /= 5;
		error_y /= 5;
		error_y_L /= 5;
		error_avg /= 5;
		error_avg_L /= 5;
		//printf("Sanity checker (7 choose 4):%d    Current errors: X:%f, Y:%f\n", sanity_checker, error_x, error_y);
		if (error_x < error_x_min[0]) // New winner on x, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][0] = coeff_x_temp[l][0];
			}
			//printf("Iteration %d: Error X improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x, error_x_min[0], coeff_x_temp[0][0], coeff_x_temp[1][0],\
			//	coeff_x_temp[2][0], coeff_x_temp[3][0], coeff_x_temp[4][0], coeff_x_temp[5][0], coeff_x_temp[6][0], coeff_x_temp[7][0], coeff_x_temp[8][0]);
			error_x_min[0] = error_x;
		}
		
		if (error_x_L < error_x_min[1]) // New winner on x_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][1] = coeff_x_temp[l][1];
			}
			//printf("Iteration %d: Error X_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x_L, error_x_min[1], coeff_x_temp[0][1], coeff_x_temp[1][1],\
			//	coeff_x_temp[2][1], coeff_x_temp[3][1], coeff_x_temp[4][1], coeff_x_temp[5][1], coeff_x_temp[6][1], coeff_x_temp[7][1], coeff_x_temp[8][1]);
			error_x_min[1] = error_x_L;
		}

		if (error_y < error_y_min[0]) // New winner on y, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][0] = coeff_y_temp[l][0];
			}
			//printf("Iteration %d: Error Y improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y, error_y_min[0], coeff_y_temp[0][0], coeff_y_temp[1][0],\
			//	coeff_y_temp[2][0], coeff_y_temp[3][0], coeff_y_temp[4][0], coeff_y_temp[5][0], coeff_y_temp[6][0], coeff_y_temp[7][0], coeff_y_temp[8][0]);
			error_y_min[0] = error_y;
		}

		if (error_y_L < error_y_min[1]) // New winner on y_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][1] = coeff_y_temp[l][1];
			}
			//printf("Iteration %d: Error Y_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y_L, error_y_min[1], coeff_y_temp[0][1], coeff_y_temp[1][1],\
			//	coeff_y_temp[2][1], coeff_y_temp[3][1], coeff_y_temp[4][1], coeff_y_temp[5][1], coeff_y_temp[6][1], coeff_y_temp[7][1], coeff_y_temp[8][1]);
			error_y_min[1] = error_y_L;
		}


	}//EXIT OF all poly loop
		
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ay_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ay_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);

	return 4;
#endif
}

int TPxFinishCalibration_poly_general_5terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2])
{
#if USE_GSL
	gsl_vector *x_screen = gsl_vector_alloc(9);
	gsl_vector *y_screen = gsl_vector_alloc(9);
	gsl_vector *x_screen_L = gsl_vector_alloc(9);
	gsl_vector *y_screen_L = gsl_vector_alloc(9);
		// Right eye!
	gsl_matrix *Ax_matrix_track = gsl_matrix_alloc(9,7);
	gsl_matrix *Ay_matrix_track = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track = gsl_matrix_alloc(7,7);
	gsl_matrix *AyS_matrix_track = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff = gsl_vector_alloc(7);
	gsl_vector *y_coeff = gsl_vector_alloc(7);

	gsl_permutation *permu_x = gsl_permutation_alloc(7); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(7); int v;

	// Left Eye too!

	gsl_matrix *Ax_matrix_track_L = gsl_matrix_alloc(9,7);
	gsl_matrix *Ay_matrix_track_L = gsl_matrix_alloc(9,7);
	gsl_matrix *AxS_matrix_track_L = gsl_matrix_alloc(7,7);
	gsl_matrix *AyS_matrix_track_L = gsl_matrix_alloc(7,7);

	gsl_vector *x_coeff_L = gsl_vector_alloc(7);
	gsl_vector *y_coeff_L = gsl_vector_alloc(7);

	gsl_permutation *permu_x_L = gsl_permutation_alloc(7); int s_L;
	gsl_permutation *permu_y_L = gsl_permutation_alloc(7); int v_L;

	double det_x, det_y; // Determinants
	int i,j,k,l;
	int sanity_checker;
	float x,y,x_L, y_L;
	float x_tracker[2], y_tracker[2], error_x, error_y, error_x_L, error_y_L;
	float coeff_x_temp[9][2];
	float coeff_y_temp[9][2];
	float error_avg = 0;
	float error_avg_L = 0;
	float eyeReturn[4];
	sanity_checker = 0;
	for (j = 0; j < 125; j++)
	{ // doing 3 terms, so hamming weight must be 3.
		if (__popcnt(j) != 5)
			continue;
		sanity_checker++;
		for (i = 0; i < 9; i++)
		{
			gsl_vector_set(x_screen, i, screen_data[i][0]);
			gsl_vector_set(y_screen, i, screen_data[i][1]);
			gsl_vector_set(x_screen_L, i, screen_data[i][0]);
			gsl_vector_set(y_screen_L, i, screen_data[i][1]);

			x = eye_data[i][0];
			y = eye_data[i][1];
			x_L = eye_data[i][2];
			y_L = eye_data[i][3];

			gsl_matrix_set(Ax_matrix_track, i, 0, 1);
			gsl_matrix_set(Ax_matrix_track_L, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track, i, 0, 1);
			gsl_matrix_set(Ay_matrix_track_L, i, 0, 1);

			gsl_matrix_set(Ax_matrix_track, i, 1, x);
			gsl_matrix_set(Ax_matrix_track_L, i, 1, x);

			gsl_matrix_set(Ay_matrix_track, i, 1, y);
			gsl_matrix_set(Ay_matrix_track_L, i, 1, y);
			k = 2;
		// Do X first 1 x always included, then 3 from
		// y x^2 y^2 x^3 xy x^2y x^2y^2
			if (j & (1 << 0)) // y
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y);
				gsl_matrix_set(Ay_matrix_track, i, k, x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L);
			}
			if (j & (1 << 1))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L);
			}
			if (j & (1 << 2))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, y_L*y_L);
			}
			if (j & (1 << 3))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*x);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*x_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*x_L);
			}
			if (j & (1 << 4))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*y_L);
			}
			if (j & (1 << 5))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L);
			}
			if (j & (1 << 6))
			{
				gsl_matrix_set(Ax_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ay_matrix_track, i, k, x*x*y*y);
				gsl_matrix_set(Ax_matrix_track_L, i, k, x_L*x_L*y_L*y_L);
				gsl_matrix_set(Ay_matrix_track_L, i, k++, x_L*x_L*y_L*y_L);
			}
		}
		// Need to calculate!
		//RHS
		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track, x_screen, 0.0, x_coeff); // x_coeff = A^T * x_screen; Store in coeff(result) because same size for now
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track, y_screen, 0.0, y_coeff); // y_coeff = A^T * y_screen;

		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track, Ax_matrix_track, 0.0, AxS_matrix_track); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track, Ay_matrix_track, 0.0, AyS_matrix_track); // Ay_matrix_track = A^T* A

		gsl_blas_dgemv(CblasTrans, 1.0, Ax_matrix_track_L, x_screen_L, 0.0, x_coeff_L); // x_screen = A^T * x_screen;
		gsl_blas_dgemv(CblasTrans, 1.0, Ay_matrix_track_L, y_screen_L, 0.0, y_coeff_L); // y_screen = A^T * y_screen;
		//LHS (Ax has been modified after that, it is in its new square form!)
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ax_matrix_track_L, Ax_matrix_track_L, 0.0, AxS_matrix_track_L); // Ax_matrix_track = A^T* A
		gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, Ay_matrix_track_L, Ay_matrix_track_L, 0.0, AyS_matrix_track_L); // Ay_matrix_track = A^T* A
	
		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track, permu_x, &s);	
		gsl_linalg_LU_decomp(AyS_matrix_track, permu_y, &v);

		// Calculations!
		gsl_linalg_LU_decomp(AxS_matrix_track_L, permu_x_L, &s_L);	
		gsl_linalg_LU_decomp(AyS_matrix_track_L, permu_y_L, &v_L);
		
		det_x = gsl_linalg_LU_det(AxS_matrix_track, s);
		det_y = gsl_linalg_LU_det(AyS_matrix_track, v);

		if (det_x == 0 || det_y == 0)
		{ // A matrix is singular, cannot solve.
			return -1;
		}
		else
		{
			gsl_linalg_LU_solve(AxS_matrix_track, permu_x, x_coeff, x_coeff);
			gsl_linalg_LU_solve(AyS_matrix_track, permu_y, y_coeff, y_coeff);

			gsl_linalg_LU_solve(AxS_matrix_track_L, permu_x_L, x_coeff_L, x_coeff_L);
			gsl_linalg_LU_solve(AyS_matrix_track_L, permu_y_L, y_coeff_L, y_coeff_L);
			
			
			// Set Coeffs! Start by emptying previous ones
			for (l = 0; l < 9; l++)
			{
				coeff_x_temp[l][0] = 0;
				coeff_x_temp[l][1] = 0;
				coeff_y_temp[l][0] = 0;
				coeff_y_temp[l][1] = 0;

			}

			// Set the A, x and A, y
			//cst
			coeff_x_temp[0][0] = (float)gsl_vector_get(x_coeff, 0);
			coeff_x_temp[0][1] = (float)gsl_vector_get(x_coeff_L, 0);
			coeff_y_temp[0][0] = (float)gsl_vector_get(y_coeff, 0);
			coeff_y_temp[0][1] = (float)gsl_vector_get(y_coeff_L, 0);
			
			//x
			coeff_x_temp[1][0] = (float)gsl_vector_get(x_coeff, 1);
			coeff_x_temp[1][1] = (float)gsl_vector_get(x_coeff_L, 1);
			//y
			coeff_y_temp[2][0] = (float)gsl_vector_get(y_coeff, 1);
			coeff_y_temp[2][1] = (float)gsl_vector_get(y_coeff_L, 1);

			k = 2;
			if (j & (1 << 0)) // y, or x!
			{
				//y
				coeff_x_temp[2][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[2][1] = (float)gsl_vector_get(x_coeff_L, k);
				//x
				coeff_y_temp[1][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[1][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 1))
			{
				coeff_x_temp[3][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[3][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[3][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[3][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 2))
			{
				coeff_x_temp[4][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[4][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[4][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[4][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 3))
			{
				coeff_x_temp[5][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[5][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[5][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[5][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 4))
			{
				coeff_x_temp[6][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[6][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[6][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[6][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 5))
			{
				coeff_x_temp[7][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[7][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[7][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[7][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
			if (j & (1 << 6))
			{
				coeff_x_temp[8][0] = (float)gsl_vector_get(x_coeff, k);
				coeff_x_temp[8][1] = (float)gsl_vector_get(x_coeff_L, k);
				coeff_y_temp[8][0] = (float)gsl_vector_get(y_coeff, k);
				coeff_y_temp[8][1] = (float)gsl_vector_get(y_coeff_L, k++);
			}
		}
	// Now we need to calculate the error see if there is any improvement from this algorithm
		error_x = 0;
		error_x_L = 0;
		error_y = 0;
		error_y_L = 0;
		for (i = 9; i < 14; i++)
		{
			x_tracker[0] = eye_data[i][0];
			y_tracker[0] = eye_data[i][1];
			x_tracker[1] = eye_data[i][2];
			y_tracker[1] = eye_data[i][3];
			TPxGetEyePosition_validation(x_tracker, y_tracker, coeff_x_temp, coeff_y_temp, eyeReturn);
			error_x += (float)fabs(screen_data[i][0] - eyeReturn[0]);
			error_y += (float)fabs(screen_data[i][1] - eyeReturn[1]);
			error_x_L += (float)fabs(screen_data[i][0] - eyeReturn[2]);
			error_y_L += (float)fabs(screen_data[i][1] - eyeReturn[3]);
			error_avg +=   (float)sqrt(error_x*error_x + error_y*error_y);
			error_avg_L += (float)sqrt(error_x_L*error_x_L+ error_y_L*error_y_L);
		}
		error_x /= 5;
		error_x_L /= 5;
		error_y /= 5;
		error_y_L /= 5;
		error_avg /= 5;
		error_avg_L /= 5;
		//printf("Sanity checker (7 choose 5):%d    Current errors: X:%f, Y:%f\n", sanity_checker, error_x, error_y);
		if (error_x < error_x_min[0]) // New winner on x, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][0] = coeff_x_temp[l][0];
			}
			printf("Iteration %d: Error X improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x, error_x_min[0], coeff_x_temp[0][0], coeff_x_temp[1][0],\
				coeff_x_temp[2][0], coeff_x_temp[3][0], coeff_x_temp[4][0], coeff_x_temp[5][0], coeff_x_temp[6][0], coeff_x_temp[7][0], coeff_x_temp[8][0]);
			error_x_min[0] = error_x;
		}
		
		if (error_x_L < error_x_min[1]) // New winner on x_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_x[l][1] = coeff_x_temp[l][1];
			}
			//printf("Iteration %d: Error X_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_x_L, error_x_min[1], coeff_x_temp[0][1], coeff_x_temp[1][1],\
			//	coeff_x_temp[2][1], coeff_x_temp[3][1], coeff_x_temp[4][1], coeff_x_temp[5][1], coeff_x_temp[6][1], coeff_x_temp[7][1], coeff_x_temp[8][1]);
			error_x_min[1] = error_x_L;
		}

		if (error_y < error_y_min[0]) // New winner on y, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][0] = coeff_y_temp[l][0];
			}
			//printf("Iteration %d: Error Y improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y, error_y_min[0], coeff_y_temp[0][0], coeff_y_temp[1][0],\
			//	coeff_y_temp[2][0], coeff_y_temp[3][0], coeff_y_temp[4][0], coeff_y_temp[5][0], coeff_y_temp[6][0], coeff_y_temp[7][0], coeff_y_temp[8][0]);
			error_y_min[0] = error_y;
		}

		if (error_y_L < error_y_min[1]) // New winner on y_L, set the coeff and new error_x_min
		{
			for (l = 0; l < 9; l++)
			{
				coeff_y[l][1] = coeff_y_temp[l][1];
			}
			//printf("Iteration %d: Error Y_L improved, %f from %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", j, error_y_L, error_y_min[1], coeff_y_temp[0][1], coeff_y_temp[1][1],\
			//	coeff_y_temp[2][1], coeff_y_temp[3][1], coeff_y_temp[4][1], coeff_y_temp[5][1], coeff_y_temp[6][1], coeff_y_temp[7][1], coeff_y_temp[8][1]);
			error_y_min[1] = error_y_L;
		}


	}//EXIT OF all poly loop
		
		// STEP ONE DONE
	
	
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ay_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ay_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);


	// Now we need to calculate the distance between the calibration and the 5 validation points

	return 3;
#endif
}


#endif