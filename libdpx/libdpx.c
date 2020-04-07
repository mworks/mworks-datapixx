/*
 *	DATAPixx cross-platform low-level C programming library
 *	Created by Peter April.
 *	Copyright (C) 2008-2017 Peter April, VPixx Technologies
 *	
 *	This library is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Library General Public
 *	License as published by the Free Software Foundation; either
 *	version 2 of the License, or (at your option) any later version.
 *	
 *	This library is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *	Library General Public License for more details.
 *	
 *	You should have received a copy of the GNU Library General Public
 *	License along with this library; if not, write to the
 *	Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 *	Boston, MA  02110-1301, USA.
 *
 */

// Maximum number of USB bulk I/O retries
#define MAX_RETRIES	10  // Important! This value should never be less than 5 else, SPI write will not work.

// Set to 1 to enable console debugging output from EZ to host.
// Must match setting in EZ firmware.
#define	ENABLE_CONSOLE	0

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "libdpx.h"		// user API.  Make sure to include libdpx src folder in user include path
//#include "libusb.h"



/************************************************************************************/
/*																					*/
/*	Here we start to define low-level interfaces not presented in lib_datapixx.h	*/
/*																					*/
/************************************************************************************/


int		dpxInitialized = 0;								// Has to be global so that libusb can be initialized without necessarily opening a DATAPixx; eg; for usb_scan command
int		dpxError = 0;									// A global function error code
int		dpxDebugLevel = 0;								// 0/1/2 controls level of debug output
int		dpxActivePSyncTimeout = -1;						// When not -1, gives the current psync register readback timeout.

int     dpxSysDevsel = DPX_DEVSEL_INVALID;              // DPX_DEVSEL_FIRST_DEVICE to DPX_DEVSEL_LAST_DEVICE, or DPX_DEVSEL_INVALID
int     dpxUsrDevsel = DPX_DEVSEL_AUTO;                 // DPX_DEVSEL_FIRST_DEVICE to DPX_DEVSEL_LAST_DEVICE, or DPX_DEVSEL_AUTO meaning API will decide

UInt16	dpxSavedRegisters[DPX_REG_SPACE_MAX/2] = { 0 };		// Local copy of DATAPixx register for save/restore

// I can't define this struct in libdpx_i.h, or C++ compiles complain about redefinition of structure
static struct stDeviceTable {
#ifdef USE_LIB01
    struct usb_device   *dpxDev;
    usb_dev_handle		*dpxHdl;
#else
	struct libusb_device   *dpxDev;
	libusb_device_handle   *dpxHdl;
#endif

    int					dpxRawUsb;                              // Non-0 if a detected DP has no EZ-USB firmware
    int					dpxGoodFpga;                            // Non-0 if system has a well-configured FPGA
    int                 dpxIsDatapixx;                          // Non-0 if a detected peripheral is actually a DATAPixx
	int                 dpxIsDatapixx2;                         // Non-0 if a detected peripheral is actually a DATAPixx2
	int                 dpxIsDatapixx3;                         // Non-0 if a detected peripheral is actually a DATAPixx3
    int                 dpxIsViewpixx;                          // Non-0 if a detected peripheral is actually a VIEWPixx
    int                 dpxIsPropixx;                           // Non-0 if a detected peripheral is actually a PROPixx
	int                 dpxIsPropixxCtrl;                       // Non-0 if a detected peripheral is actually a PROPixx Controller
    int                 DPxIsTrackpixx;							// Non-0 if a detected peripheral is actually a TRACKPixx
	int                 DPxIsTrackpixxCtrl;                     // Non-0 if a detected peripheral is actually a TRACKPixx Controller
	int                 DPxIsTrackpixxBridge;                   // Non-0 if a detected peripheral is actually a TRACKPixx Bridge
	char				dpxDeviceName[64];
	char				dpxSerialNum[64];
    unsigned short      dpxRegisterCache[DPX_REG_SPACE_MAX/2];		// Local copy of DATAPixx register set read over USB
    int                 dpxRegisterModified[DPX_REG_SPACE_MAX/2];	// When set, means that value in dpxRegisterCache[] must be written back to DATAPixx.
    int                 nEP1Writes;
    int                 nEP1Reads;
} dpxDeviceTable[DPX_DEVSEL_TABLE_SIZE] = { {0} };


static int  devselCnt[DPX_DEVSEL_CNT_SIZE] = {0};


// Keep track of the total number of USB bulk I/O retries/fails for each endpoint and direction
int		dpxEp1WrRetries = 0;
int		dpxEp1RdRetries = 0;
int		dpxEp2WrRetries = 0;
int		dpxEp6RdRetries = 0;
int		dpxEp1WrFails = 0;
int		dpxEp1RdFails = 0;
int		dpxEp2WrFails = 0;
int		dpxEp6RdFails = 0;

// Largest EP1 trams are 265 bytes for SPI page R/W:
//	   4 bytes for tram header
//	+  4 bytes for SPI cmd/addr1/addr2/addr3
//	+256 bytes for SPI page data
//  +  1 byte delay in SPI fast readback command
// That's a payload of 261 bytes
//unsigned char ep1in_Tram[265];
//unsigned char ep1out_Tram[265];
// Redefined below

// FX3 FW can support up to 1024 bytes EP
// Largest EP1 trams are 265 bytes for SPI page R/W:
//	   4 bytes for tram header
//	+  6 bytes for SPI cmd0/cmd1/addr1/addr2/addr3/addr4
//	+768 bytes for SPI page data
//  +  1 byte delay in SPI fast readback command
// That's a payload of 779 bytes
// We are using the full USB3 EP length
unsigned char ep1in_Tram[1024];
unsigned char ep1out_Tram[1024];

// We will limit other endpoint trams to 64k bytes long.
// This means that the maximum payload size is 65536 - 4-byte header = 65532 bytes.
unsigned char ep2out_Tram[65536];
unsigned char ep6in_Tram[65536];

// FPGA configuration data buffers
unsigned char configBuffer[CONFIG_BUFFER_SIZE];
unsigned char configBuffer2[CONFIG_BUFFER_SIZE];

// We'll cache CODEC I2C registers for (optional) faster readback.
// Try to initialize cache to the actual reset values.
unsigned char cachedCodecRegs[128] = {
	0x00, 0x00, 0x22, 0x20, 0x04, 0x00, 0x00, 0x6A, 0x00, 0x4E, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x50,
	0x50, 0xFF, 0xFF, 0x04, 0x78, 0x78, 0x04, 0x78, 0x78, 0x44, 0x00, 0xFE, 0x00, 0x00, 0xFE, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xCC, 0xE0, 0x1C, 0x00, 0x80, 0x00, 0x8C, 0x00, 0x00, 0x00, 0x00, 0xA8,
	0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xA8, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x0C,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int gSpifEnable = 1;                    // Enable fast FPGA SPI access

/* GLOBAL VARIABLE TO STORE THE EYE DATA AND PARAMETER */
float eye_data[25000][8]; // 36 000 000 can contain 1 hour of 10khz eye data.

int NUMBER_OF_SAMPLES;
int EYE_PER_SAMPLE = 25;
int CALIBRATION_POINTS_NUMBER = 14;
float screen_data[25000][2];

float parameter_X_eye1[6];
float parameter_Y_eye1[6];
float parameter_X_eye2[6];
float parameter_Y_eye2[6];
float parameter_X_eye1_cam_B[6];
float parameter_Y_eye1_cam_B[6];
float parameter_X_eye2_cam_B[6];
float parameter_Y_eye2_cam_B[6];
int IS_DEVICE_CALIBRATED = 0;

float eye_data_calib_avg[25][8];
float screen_data_calib_avg[25][2];
float coeff_x_eye1[5]; // A, B, C, D, E
float coeff_y_eye1[7]; // F, G, H, I, J
float coeff_x_eye2[5]; // A, B, C, D, E
float coeff_y_eye2[7]; // F, G, H, I, J

float coeff_x[9][2];
float coeff_y[9][2];
float m[4]; // for X depending on quadrants. 
float n[4]; // for Y depending on quadrants.
float x_drift_corr;
float y_drift_corr;


int tpx_enable_acquisition = 0;

#if USE_GSL

#include <gsl\gsl_matrix.h>
#include <gsl\gsl_blas.h>
#include <gsl\gsl_linalg.h>
#include <gsl\gsl_rng.h>
#include "libtpx.h"
#include <intrin.h>
#endif

#ifndef USE_LIB01
char *usb_strerror(void)
{
	return strerror(errno);
}
#endif

// Returns non-zero if we found a real device with a configured FPGA
int DPxSelectDevice(int devsel)
{
    if (devsel == DPX_DEVSEL_AUTO || (devsel >= DPX_DEVSEL_FIRST_DEVICE && devsel <= DPX_DEVSEL_LAST_DEVICE)) {
        dpxUsrDevsel = devsel;
        return DPxSelectSysDevice(DPX_DEVSEL_ANY); // Initialize dpxSysDevsel to a reasonable value
    }
    else {
		fprintf(stderr, "ERROR: DPxSelectDevice() passed illegal device %d\n", devsel);
		DPxSetError(DPX_ERR_USB_DEVSEL_INDEX);
        return 0;
    }
}


// Returns non-zero if we found a real device with a configured FPGA
int DPxSelectDeviceSubName(int devtype, char* devname)
{
	int iDev, devFound = 0;
	int devIndex = 0;
	
    if (devtype == DPX_DEVSEL_DPX || devtype == DPX_DEVSEL_VPX || devtype == DPX_DEVSEL_PPC || devtype == DPX_DEVSEL_PPX || devtype == DPX_DEVSEL_DP2 || devtype == DPX_DEVSEL_TPX || devtype == DPX_DEVSEL_TPC || devtype == DPX_DEVSEL_TPB) {
		dpxUsrDevsel = devtype;

		for (iDev = dpxUsrDevsel; iDev < dpxUsrDevsel+DPX_DEVSEL_MULTI && !devFound; iDev++) {
			dpxSysDevsel = iDev;
			if (strcmp(dpxDeviceTable[dpxSysDevsel].dpxDeviceName, devname) == 0) {
				devIndex = iDev - dpxUsrDevsel + 1;
				devFound = 1;
			}
		}
		if (!devFound) {
			fprintf(stderr, "ERROR: DPxSelectDeviceSubName() device name %s not found\n", devname);
			DPxSetError(DPX_ERR_USB_DEVSEL_INDEX);
			return 0;
		}
		
		dpxUsrDevsel = dpxUsrDevsel + devIndex - 1;
        return DPxSelectSysDevice(dpxUsrDevsel); // Initialize dpxSysDevsel to a reasonable value
    }
    else {
		fprintf(stderr, "ERROR: DPxSelectDeviceSubName() passed illegal device type %d\n", devtype);
		DPxSetError(DPX_ERR_USB_DEVSEL_INDEX);
        return 0;
    }
}


int DPxDetectDevice(int devsel)
{
    if (devsel >= DPX_DEVSEL_FIRST_DEVICE && devsel <= DPX_DEVSEL_LAST_DEVICE)
        return dpxDeviceTable[devsel].dpxHdl != 0;
    else
        return 0;
}

unsigned char* createImageChar()
{
	unsigned char *ptr = (unsigned char*)malloc(1280*1024*sizeof(char));
	int x,y;
	for (y = 0; y < 1024; y++)
	{
		for (x = 0; x < 1280; x++)
		{
			ptr[x+y*1280] = (char) (x+y*3) % 256;
		}
	}
	return ptr;
}
	


void PPxDownloadTestPattern(TestStruct* testPattern, int loadAddr, int page)
{

	unsigned short megaBuff[512*1024];	
	int nDisplayLines, iPlane, iRgb, iBitPos, megaIndex, megaBit, x, y, patDatum, patBit;
    int demuxThresh;
    loadAddr = DPxGetVidSwtpAddr() == 0x08000000 ? 0x0A000000 : 0x08000000;
    nDisplayLines = 1080;

    // OK, we're on a PROPixx, and we have to split up the bit planes
    for (iPlane = 0; iPlane < 24; iPlane++) {
        iRgb = iPlane % 3;
        iBitPos = 7 - iPlane / 3;
        memset(megaBuff, 0, sizeof(megaBuff));
        megaIndex = 0;
        megaBit = 1;
        for (y = 0; y < nDisplayLines; y++) {
            for (x = 0; x < 1920; x++) {
                patDatum = testPattern->array[y][x][iRgb];
                patBit = (patDatum & (1 << iBitPos)) ? 1 : 0;
                if (patBit)
                    megaBuff[megaIndex] |= megaBit;
                if (megaBit < 0x8000)
                    megaBit <<= 1;
                else {
                    megaBit = 1;
                    megaIndex++;
                }
            }
        }
        DPxWriteRam(loadAddr + page*0x01000000 + iPlane*0x40000, megaIndex*2, megaBuff);
    }
    
    // Since PPX VHDL rev 2, we place demuxed bit planes 6/5 into planes A/B/C
    for ( ; iPlane < 33; iPlane++) {
        iRgb = iPlane % 3;
        demuxThresh = ((iPlane-24) / 3 + 1) << 5;
        memset(megaBuff, 0, sizeof(megaBuff));
        megaIndex = 0;
        megaBit = 1;
        for (y = 0; y < nDisplayLines; y++) {
            for (x = 0; x < 1920; x++) {
                patDatum = testPattern->array[y][x][iRgb];
                patBit = (patDatum & 0x60) >= demuxThresh ? 1 : 0;
                if (patBit)
                    megaBuff[megaIndex] |= megaBit;
                if (megaBit < 0x8000)
                    megaBit <<= 1;
                else {
                    megaBit = 1;
                    megaIndex++;
                }
            }
        }
        DPxWriteRam(loadAddr + page*0x01000000 + iPlane*0x40000, megaIndex*2, megaBuff);
    }
	DPxSetVidMode(DPXREG_VID_CTRL_MODE_C24);
    DPxSetVidSwtp(0x08000000);
	DPxUpdateRegCache();
	return;
}

float TPxFilterFunctionLevelOne(float* x2, float* x1, float* x)
{
	if (((*x2 > *x1) && (*x1 < *x)) || ((*x2 < *x1) && (*x1 > *x)))	{
		
		if ((fabsf(*x2-*x1) > fabsf(*x1 - *x))) {
			*x1 = *x;
		}
		else {
			*x1 = *x2;
		}
	}
	*x2 = *x1;
	*x1 = *x;
	return *x2;
}

void TPxGetEyePositionDuringCalib(float screen_x, float screen_y) 
{
#if USE_GSL
	int i;
	int iteration_number = 0;
	double initial_time;
	float dividor, average, middle;
	float averages[8];
	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
			
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;

	// WE IGNORE LUXB FOR NOW

	float LUXB_LEFT_X; 
	float LUXB_LEFT_Y;
			
	float LUXB_RIGHT_X; 
	float LUXB_RIGHT_Y;
	int point_number;
	FILE* fp; 
	FILE* fp_raw;
	fp = fopen("00a.txt", "a+");
	fp_raw = fopen("raw_data.csv", "a+");
	fprintf(fp, "NUMBER OF SAMPLES: %d!\n", NUMBER_OF_SAMPLES);
	//fprintf(fp_raw, "Point number #%d\n", NUMBER_OF_SAMPLES / EYE_PER_SAMPLE);
	DPxUpdateRegCache();
	initial_time = DPxGetTime();
	for (i = 0; i < 8; i++)
	{
		averages[i] = 0;
	}
	for (i = 0; i < EYE_PER_SAMPLE; i++)
	{
		// PP = PUPIL
		// CR = CORNEA REFLEX
		while (DPxGetTime() - initial_time < 0.01*i )
		{
			DPxUpdateRegCache();
			i = i - 1 + 1;
		}
		screen_data[NUMBER_OF_SAMPLES][0] = screen_x;
		screen_data[NUMBER_OF_SAMPLES][1] = screen_y;
		DPxUpdateRegCache();

		// We read this as 8.8 ints, we must convert to  float by deviding by 2^8
		dividor = 1 << 8;


		LUXA_RIGHT_X = ((short)DPxGetReg16(0x390)) / dividor;
		LUXA_RIGHT_Y = ((short)DPxGetReg16(0x392)) / dividor;
			
		LUXA_LEFT_X = ((short)DPxGetReg16(0x380)) / dividor;
		LUXA_LEFT_Y = ((short)DPxGetReg16(0x382)) / dividor; 

		// WE IGNORE LUXB FOR NOW

		LUXB_LEFT_X = ((short)DPxGetReg16(0x3A0)) / dividor;
		LUXB_LEFT_Y = ((short)DPxGetReg16(0x3A2)) / dividor;
			
		LUXB_RIGHT_X = ((short)DPxGetReg16(0x3B0)) / dividor;
		LUXB_RIGHT_Y = ((short)DPxGetReg16(0x3B2)) / dividor;

		fprintf(fp, "Number of sample : %d, right_x:%f, right_y: %f, average[0]: %f, screen_x: %f\n", NUMBER_OF_SAMPLES, LUXA_RIGHT_X, LUXA_RIGHT_Y, averages[0], screen_x);
		fprintf(fp_raw, "%d,%f,%f,%f,%f,%f,%f\n", NUMBER_OF_SAMPLES / EYE_PER_SAMPLE, LUXA_RIGHT_X, LUXA_RIGHT_Y, LUXA_LEFT_X, LUXA_LEFT_Y, screen_x, screen_y);
		eye_data[NUMBER_OF_SAMPLES][0] = LUXA_RIGHT_X; // eye one, cam one x
		averages[0] += LUXA_RIGHT_X;
		eye_data[NUMBER_OF_SAMPLES][1] = LUXA_RIGHT_Y; // eye one, cam one y
		averages[1] += LUXA_RIGHT_Y;

		eye_data[NUMBER_OF_SAMPLES][2] = LUXA_LEFT_X; // eye two, cam one x
		averages[2] += LUXA_LEFT_X;
		eye_data[NUMBER_OF_SAMPLES][3] = LUXA_LEFT_Y; // eye two, cam one y
		averages[3] += LUXA_LEFT_Y;

		eye_data[NUMBER_OF_SAMPLES][4] = LUXB_LEFT_X; // eye one, cam two x
		averages[4] += LUXB_LEFT_X;
		eye_data[NUMBER_OF_SAMPLES][5] = LUXB_LEFT_Y; // eye one, cam two y
		averages[5] += LUXB_LEFT_Y;

		eye_data[NUMBER_OF_SAMPLES][6] = LUXB_RIGHT_X; // eye two, cam two x
		averages[6] += LUXB_RIGHT_X;
		eye_data[NUMBER_OF_SAMPLES][7] = LUXB_RIGHT_Y; // eye two, cam two y
		averages[7] += LUXB_RIGHT_Y;

		NUMBER_OF_SAMPLES++;	
	}
	// Check the STD of [NUMBER_OF_SAMPLES-25, NUMBER_OF_SAMPLES]
	average = 0;
	middle = 0;
	for(i = 0; i < 8; i++)
	{
		averages[i] = averages[i]/EYE_PER_SAMPLE;
	}
	average = averages[0];
	fprintf(fp, "THE AVERAGE IS %f\n", average);
	for(i = NUMBER_OF_SAMPLES-25; i < NUMBER_OF_SAMPLES; i++)
	{
		middle += (eye_data[i][0]-average)*(eye_data[i][0]-average);
	}
	middle = middle / 25;
	point_number = NUMBER_OF_SAMPLES / EYE_PER_SAMPLE -1;
	fprintf(fp, "THE STD^2 IS: %f, point_number is %d\n", middle, point_number);
	// Set-up values for post calibration


	for(i = 0; i < 8; i++)
	{
		eye_data_calib_avg[point_number][i] = averages[i];
	}
	fprintf(fp, "Average for right eye x:%f (in global: %f), right eye y:%f (in global: %f)\n", averages[0], eye_data_calib_avg[point_number][0], averages[1], eye_data_calib_avg[point_number][0]);
	screen_data_calib_avg[point_number][0] = screen_x;
	screen_data_calib_avg[point_number][1] = screen_y;
	fprintf(fp, "Screen data x:%f (in global: %f),  y:%f (in global: %f)\n", screen_x, screen_data_calib_avg[point_number][0], screen_y, screen_data_calib_avg[point_number][1]);

	// Checking if we have bad values
	if (middle > 0.30 && IS_DEVICE_CALIBRATED < 2 )
	{ // bad sample, resample!

		NUMBER_OF_SAMPLES = NUMBER_OF_SAMPLES - 25;
		IS_DEVICE_CALIBRATED = IS_DEVICE_CALIBRATED+ 1; // Using IS_DEVICE_CALIBRATED as a cheat 
		fprintf(fp, "RESAMPLING BECAUSE BAD SAMPLE! Number of samples = %d, Device calib : %d *-_\n", NUMBER_OF_SAMPLES, IS_DEVICE_CALIBRATED);
		fclose(fp);
		TPxGetEyePositionDuringCalib(screen_x, screen_y);
	}	


	IS_DEVICE_CALIBRATED = 0;
	fclose(fp);
#endif
}
void TPxGetEyePositionDuringCalib_returnsRaw(float screen_x, float screen_y, float* rawReturn) 
{
	int current_point_number, i;
	//TPxGetEyePositionDuringCalib(screen_x, screen_y);
	current_point_number = NUMBER_OF_SAMPLES / EYE_PER_SAMPLE - 1;
	for (i = 0; i < 4; i++)
		rawReturn[i] = i*16;//eye_data_calib_avg[current_point_number][i];
	printf("My current point is: %d, I am returning raw: RIGHT:(%f, %f), LEFTP(%f, %f)\n", current_point_number,  rawReturn[0], rawReturn[1], rawReturn[2], rawReturn[3]);
	return;
}
/*
void TPxGetEyePositionDuringCalib_old(float screen_x, float screen_y) 
{
	int i;
	int iteration_number = 0;
	double initial_time;
	float dividor, average, middle;
	float LUXA_PP_LEFT_X;
	float LUXA_PP_LEFT_Y;
	float LUXA_CR_LEFT_X;
	float LUXA_CR_LEFT_Y;

	float LUXA_PP_RIGHT_X;
	float LUXA_PP_RIGHT_Y;
	float LUXA_CR_RIGHT_X;
	float LUXA_CR_RIGHT_Y;

	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
			
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;

	// WE IGNORE LUXB FOR NOW
	float LUXB_PP_LEFT_X;
	float LUXB_PP_LEFT_Y;
	float LUXB_CR_LEFT_X;
	float LUXB_CR_LEFT_Y;

	float LUXB_PP_RIGHT_X;
	float LUXB_PP_RIGHT_Y;
	float LUXB_CR_RIGHT_X;
	float LUXB_CR_RIGHT_Y;

	float LUXB_LEFT_X; 
	float LUXB_LEFT_Y;
			
	float LUXB_RIGHT_X; 
	float LUXB_RIGHT_Y;
	FILE* fp;
	fp = fopen("00a.txt", "a+");
	fprintf(fp, "NUMBER OF SAMPLES: %d!\n", NUMBER_OF_SAMPLES);
	DPxUpdateRegCache();
	initial_time = DPxGetTime();
	for (i = 0; i < EYE_PER_SAMPLE; i++)
	{
		// PP = PUPIL
		// CR = CORNEA REFLEX
		while (DPxGetTime() - initial_time < 0.01*i )
		{
			DPxUpdateRegCache();
			i = i - 1 + 1;
		}
		screen_data[NUMBER_OF_SAMPLES][0] = screen_x;
		screen_data[NUMBER_OF_SAMPLES][1] = screen_y;
		DPxUpdateRegCache();

		// We read this as 16.16 ints, we must convert to  float by deviding by 2^16
		dividor = 1 << 16;
		LUXA_PP_LEFT_X =	DPxGetReg32(0x310) / dividor;
		LUXA_PP_LEFT_Y =	DPxGetReg32(0x314) / dividor;
		LUXA_CR_LEFT_X =	DPxGetReg32(0x318) / dividor;
		LUXA_CR_LEFT_Y =	DPxGetReg32(0x31C) / dividor;

		LUXA_PP_RIGHT_X =	DPxGetReg32(0x320) / dividor;
		LUXA_PP_RIGHT_Y =	DPxGetReg32(0x324) / dividor;
		LUXA_CR_RIGHT_X =	DPxGetReg32(0x328) / dividor;
		LUXA_CR_RIGHT_Y =	DPxGetReg32(0x32C) / dividor;

		LUXA_LEFT_X = LUXA_CR_LEFT_X - LUXA_PP_LEFT_X; 
		LUXA_LEFT_Y = LUXA_CR_LEFT_Y - LUXA_PP_LEFT_Y;
			
		LUXA_RIGHT_X = LUXA_CR_RIGHT_X - LUXA_PP_RIGHT_X; 
		LUXA_RIGHT_Y = LUXA_CR_RIGHT_Y - LUXA_PP_RIGHT_Y;

		// WE IGNORE LUXB FOR NOW
		LUXB_PP_LEFT_X =	DPxGetReg32(0x330) / dividor;
		LUXB_PP_LEFT_Y =	DPxGetReg32(0x334) / dividor;
		LUXB_CR_LEFT_X =	DPxGetReg32(0x338) / dividor;
		LUXB_CR_LEFT_Y =	DPxGetReg32(0x33C) / dividor;

		LUXB_PP_RIGHT_X =	DPxGetReg32(0x340) / dividor;
		LUXB_PP_RIGHT_Y =	DPxGetReg32(0x344) / dividor;
		LUXB_CR_RIGHT_X =	DPxGetReg32(0x348) / dividor;
		LUXB_CR_RIGHT_Y =	DPxGetReg32(0x34C) / dividor;

		LUXB_LEFT_X = LUXB_CR_LEFT_X - LUXB_PP_LEFT_X; 
		LUXB_LEFT_Y = LUXB_CR_LEFT_Y - LUXB_PP_LEFT_Y;
			
		LUXB_RIGHT_X = LUXB_CR_RIGHT_X - LUXB_PP_RIGHT_X; 
		LUXB_RIGHT_Y = LUXB_CR_RIGHT_Y - LUXB_PP_RIGHT_Y;

		fprintf(fp, "Number of sample : %d, right_x:%f, right_y: %f, left_x: %f, left_y: %f\n", NUMBER_OF_SAMPLES, LUXA_RIGHT_X, LUXA_RIGHT_Y, LUXA_LEFT_X, LUXA_RIGHT_Y);

		eye_data[NUMBER_OF_SAMPLES][0] = LUXA_LEFT_X; // eye one, cam one x
		eye_data[NUMBER_OF_SAMPLES][1] = LUXA_LEFT_Y; // eye one, cam one y

		eye_data[NUMBER_OF_SAMPLES][2] = LUXA_RIGHT_X; // eye two, cam one x
		eye_data[NUMBER_OF_SAMPLES][3] = LUXA_RIGHT_Y; // eye two, cam one y

		eye_data[NUMBER_OF_SAMPLES][4] = LUXB_LEFT_X; // eye one, cam two x
		eye_data[NUMBER_OF_SAMPLES][5] = LUXB_LEFT_Y; // eye one, cam two y

		eye_data[NUMBER_OF_SAMPLES][6] = LUXB_RIGHT_X; // eye two, cam two x
		eye_data[NUMBER_OF_SAMPLES][7] = LUXB_RIGHT_Y; // eye two, cam two y

		NUMBER_OF_SAMPLES++;	
	}
	// Check the STD of [NUMBER_OF_SAMPLES-25, NUMBER_OF_SAMPLES]
	average = 0;
	middle = 0;
	for (i = NUMBER_OF_SAMPLES-25; i < NUMBER_OF_SAMPLES; i++)
	{
		//fprintf(fp, "SO FAR AVERAGE IS AFTER (%d) ITERATION: %f, adding EYE_DATA: %f\n", i, average, eye_data[i][0]);
		average = average + eye_data[i][0];
	}
	average = average / 25;
	fprintf(fp, "THE AVERAGE IS %f\n", average);
	for (i = NUMBER_OF_SAMPLES-25; i < NUMBER_OF_SAMPLES; i++)
	{
		middle += (eye_data[i][0]-average)*(eye_data[i][0]-average);
	}
	middle = middle / 25;
	fprintf(fp, "THE STD^2 IS: %f, iteration_number is %d\n", middle, iteration_number);
	if (middle > 0.30 && IS_DEVICE_CALIBRATED < 2 )
	{ // bad sample, resample!

		NUMBER_OF_SAMPLES = NUMBER_OF_SAMPLES - 25;
		IS_DEVICE_CALIBRATED = IS_DEVICE_CALIBRATED+ 1; // Using IS_DEVICE_CALIBRATED as a cheat 
		fprintf(fp, "RESAMPLING BECAUSE BAD SAMPLE! Number of samples = %d, Device calib : %d *-_\n", NUMBER_OF_SAMPLES, IS_DEVICE_CALIBRATED);
		fclose(fp);
		TPxGetEyePositionDuringCalib(screen_x, screen_y);
	}
	IS_DEVICE_CALIBRATED = 0;
	fclose(fp);
}
*/
void TPxFinishCalibration_new()
{ // TO BE DONE
	/* Calibration is to be done in 4 steps

	Step 1: Calculate A,B,C,D,E and F,G,H,I,J coefficients using the first first points.

	A x = b
	Solve for x;
	A = Averaged tracker values (5x5)
	| xtrack0 ytrack0 xtrack0^2 ytrack0^2 1 |
	| xtrack1 ytrack1 xtrack1^2 ytrack1^2 1 |
	...
	| xtrack4 ytrack4 xtrack4^2 ytrack4^2 1 |

	b = screen coordinates (5x1)
	| xscreen0 |
	| xscreen1 |
	...
	| xscreen4 |
	x = Coefficients matrix
	| A |
	| B |
	...
	| E |

	Step 2, Calculate (x5, y5), (x6, y6), (x7, y7), (x8, y8), using the above coefficients and (xtrack5, ytrack5), ..., (xtrack8, ytrack8)

	x6 = A*xtrack6 + B*ytrack6 + C*xtrack6^2 + D*xtrack6^2 + E
	y6 = F*xtrack6 + G*ytrack6 + H*xtrack6^2 + I*xtrack6^2 + J

	Step 3, Calculate m[i] and n[i] by using x5,y5,... and xscreen6, yscreen6 and their quadrant information

	xscreen6 = x6 + m[1]x6y6
	yscreen6 = y6 + n[1]x6y6 // This forces xscreen6,yscreen6 to be quadrant 1

	Step 4, Calculate the drift factor, DFx and DFy

	xtrack0 - xtrack9 = DFx
	ytrack0 - ytrack9 = DFy
	*/
#if USE_GSL

	// Only RIGHT EYE now
	gsl_vector *x_screen_step1 = gsl_vector_alloc(5);
	gsl_vector *y_screen_step1 = gsl_vector_alloc(5);

	gsl_matrix *Ax_matrix_step1 = gsl_matrix_alloc(5,5);
	gsl_matrix *Ay_matrix_step1 = gsl_matrix_alloc(5,5);

	gsl_vector *x_coeff_step1 = gsl_vector_alloc(5);
	gsl_vector *y_coeff_step1 = gsl_vector_alloc(5);

	// Permutation for LU solving
	gsl_permutation *permu_x = gsl_permutation_alloc(5); int s;
	gsl_permutation *permu_y = gsl_permutation_alloc(5); int v;

//	double* x_screen, *y_screen; 
	double det_x, det_y; // Determinants
	int i;
	float x5, x6, x7, x8;
	float y5, y6, y7, y8;
	FILE* fp;
	fp = fopen("00c.txt", "a+");

	// Data is located in eye_data_calib_avg (has 10x4 values (left, right eye), x_track, y_track, )
	// screen_data_calib_avg has 10x2 values( x_screen, y_screen)
	for (i = 0; i < 5; i++)
	{
		gsl_vector_set(x_screen_step1, i, screen_data_calib_avg[i][0]);
		gsl_vector_set(y_screen_step1, i, screen_data_calib_avg[i][1]);

		gsl_matrix_set(Ax_matrix_step1, i, 0, eye_data_calib_avg[i][0]); // x_track_i
		gsl_matrix_set(Ax_matrix_step1, i, 1, eye_data_calib_avg[i][1]); // y_track_i
		gsl_matrix_set(Ax_matrix_step1, i, 2, eye_data_calib_avg[i][0]*eye_data_calib_avg[i][0]); // x_track_i^2
		gsl_matrix_set(Ax_matrix_step1, i, 3, eye_data_calib_avg[i][1]*eye_data_calib_avg[i][1]);		// y_track_i^2
		gsl_matrix_set(Ax_matrix_step1, i, 4, 1);		// 1

		fprintf(fp, "Using the following averages (point number: %d): \n ", i);
		fprintf(fp, "Screen data: (%f,%f)\n", screen_data_calib_avg[i][0], screen_data_calib_avg[i][1]);
		fprintf(fp, "Eye Data: (%f,%f)\n", eye_data_calib_avg[i][0], eye_data_calib_avg[i][1]);
	}
	gsl_matrix_memcpy(Ay_matrix_step1, Ax_matrix_step1);
	
	// Calculations!
	gsl_linalg_LU_decomp(Ax_matrix_step1, permu_x, &s);	
	gsl_linalg_LU_decomp(Ay_matrix_step1, permu_y, &v);

		
	det_x = gsl_linalg_LU_det(Ax_matrix_step1, s);
	det_y = gsl_linalg_LU_det(Ay_matrix_step1, v);

	fprintf(fp, "Just calculated determinants: det_x:%f, det_y:%f\n", det_x, det_y);
	if (det_x == 0 || det_y == 0) // || det_left_cam_B == 0 || det_right_cam_B == 0)
	{ // A matrix is singular, cannot solve.
		IS_DEVICE_CALIBRATED = 0;
	}
	else
	{
		gsl_linalg_LU_solve(Ax_matrix_step1, permu_x, x_screen_step1, x_coeff_step1);
		gsl_linalg_LU_solve(Ay_matrix_step1, permu_y, y_screen_step1, y_coeff_step1);

		for (i = 0; i < 5; i++)
		{
		coeff_x_eye1[i] = (float)gsl_vector_get(x_coeff_step1, i);
		coeff_y_eye1[i] = (float)gsl_vector_get(y_coeff_step1, i);
		}
		IS_DEVICE_CALIBRATED = 1;
		fprintf(fp, "Just calculated x coefficients: A:%f, B:%f, C:%f, D:%f, E:%f\n Just calculated y coefficients: F:%f, G:%f, H:%f, I:%f, J:%f\n ", coeff_x_eye1[0], coeff_x_eye1[1], coeff_x_eye1[2], coeff_x_eye1[3], coeff_x_eye1[4], coeff_y_eye1[0], coeff_y_eye1[1], coeff_y_eye1[2], coeff_y_eye1[3], coeff_y_eye1[4]);
	} 		// STEP ONE DONE
	
	
	// STEP 2, we must find x5, x6, x7, x8 using x_tracker5, x_tracker6,...
	// x6 = A*xtrack6 + B*ytrack6 + C*xtrack6^2 + D*xtrack6^2 + E
	x5 = coeff_x_eye1[0]*eye_data_calib_avg[5][0] +
		 coeff_x_eye1[1]*eye_data_calib_avg[5][1] +
		 coeff_x_eye1[2]*eye_data_calib_avg[5][0]*eye_data_calib_avg[5][0] +
		 coeff_x_eye1[3]*eye_data_calib_avg[5][1]*eye_data_calib_avg[5][1] +
		 coeff_x_eye1[4];

	y5 = coeff_y_eye1[0]*eye_data_calib_avg[5][0] +
		 coeff_y_eye1[1]*eye_data_calib_avg[5][1] +
		 coeff_y_eye1[2]*eye_data_calib_avg[5][0]*eye_data_calib_avg[5][0] +
		 coeff_y_eye1[3]*eye_data_calib_avg[5][1]*eye_data_calib_avg[5][1] +
		 coeff_y_eye1[4];

	x6 = coeff_x_eye1[0]*eye_data_calib_avg[6][0] +
		 coeff_x_eye1[1]*eye_data_calib_avg[6][1] +
		 coeff_x_eye1[2]*eye_data_calib_avg[6][0]*eye_data_calib_avg[6][0] +
		 coeff_x_eye1[3]*eye_data_calib_avg[6][1]*eye_data_calib_avg[6][1] +
		 coeff_x_eye1[4];

	y6 = coeff_y_eye1[0]*eye_data_calib_avg[6][0] +
		 coeff_y_eye1[1]*eye_data_calib_avg[6][1] +
		 coeff_y_eye1[2]*eye_data_calib_avg[6][0]*eye_data_calib_avg[6][0] +
		 coeff_y_eye1[3]*eye_data_calib_avg[6][1]*eye_data_calib_avg[6][1] +
		 coeff_y_eye1[4];

	x7 = coeff_x_eye1[0]*eye_data_calib_avg[7][0] +
		 coeff_x_eye1[1]*eye_data_calib_avg[7][1] +
		 coeff_x_eye1[2]*eye_data_calib_avg[7][0]*eye_data_calib_avg[7][0] +
		 coeff_x_eye1[3]*eye_data_calib_avg[7][1]*eye_data_calib_avg[7][1] +
		 coeff_x_eye1[4];

	y7 = coeff_y_eye1[0]*eye_data_calib_avg[7][0] +
		 coeff_y_eye1[1]*eye_data_calib_avg[7][1] +
		 coeff_y_eye1[2]*eye_data_calib_avg[7][0]*eye_data_calib_avg[7][0] +
		 coeff_y_eye1[3]*eye_data_calib_avg[7][1]*eye_data_calib_avg[7][1] +
		 coeff_y_eye1[4];

	x8 = coeff_x_eye1[0]*eye_data_calib_avg[8][0] +
		 coeff_x_eye1[1]*eye_data_calib_avg[8][1] +
		 coeff_x_eye1[2]*eye_data_calib_avg[8][0]*eye_data_calib_avg[8][0] +
		 coeff_x_eye1[3]*eye_data_calib_avg[8][1]*eye_data_calib_avg[8][1] +
		 coeff_x_eye1[4]; 

	y8 = coeff_y_eye1[0]*eye_data_calib_avg[8][0] +
		 coeff_y_eye1[1]*eye_data_calib_avg[8][1] +
		 coeff_y_eye1[2]*eye_data_calib_avg[8][0]*eye_data_calib_avg[8][0] +
		 coeff_y_eye1[3]*eye_data_calib_avg[8][1]*eye_data_calib_avg[8][1] +
		 coeff_y_eye1[4];
	fprintf(fp, "Step 2: x5:%f, y5:%f\n", x5, y5);
	m[0] = (screen_data_calib_avg[5][0] - x5) / (x5*y5);
	m[1] = (screen_data_calib_avg[6][0] - x6) / (x6*y6);
	m[2] = (screen_data_calib_avg[7][0] - x7) / (x7*y7);
	m[3] = (screen_data_calib_avg[8][0] - x8) / (x8*y8);

	n[0] = (screen_data_calib_avg[5][1] - y5) / (x5*y5);
	n[1] = (screen_data_calib_avg[6][1] - y6) / (x6*y6);
	n[2] = (screen_data_calib_avg[7][1] - y7) / (x7*y7);
	n[3] = (screen_data_calib_avg[8][1] - y8) / (x8*y8);
	
	fprintf(fp, "Step 2: m[0]:%f, m[1]:%f, m[2]:%f, m[3]:%f, n[0]:%f, n[1]:%f, n[2]:%f, n[3]:%f \n", m[0], m[1], m[2], m[3], n[0], n[1], n[2], n[3]);
	// Step 2 done

	// Step 3 x_drift_corr = x_track0 - x_track9

	x_drift_corr = eye_data_calib_avg[9][0] - eye_data_calib_avg[0][0];
	y_drift_corr = eye_data_calib_avg[9][1] - eye_data_calib_avg[0][1];
	

	fprintf(fp, "Step 3: x_drift: %f, y_drift: %f \n", x_drift_corr, y_drift_corr);
	gsl_matrix_free(Ax_matrix_step1);
	gsl_matrix_free(Ay_matrix_step1);

	gsl_vector_free(x_screen_step1);
	gsl_vector_free(x_screen_step1);
	gsl_vector_free(x_coeff_step1);
	gsl_vector_free(y_coeff_step1);

	gsl_permutation_free(permu_x);
	gsl_permutation_free(permu_y);
	fprintf(fp, "Matrix freed, work here is done.\n");
	fclose(fp);
	NUMBER_OF_SAMPLES = 0;
	return;
#endif
}

void TPxBestPolyGetEyePosition(float* eyeReturn)
{
#if USE_GSL
  //printf("\n\n *** Polynomial being used Get Eye in X_L *** \n\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", coeff_x[0][1],\
  //				coeff_x[1][1], coeff_x[2][1], coeff_x[3][1], coeff_x[4][1], coeff_x[5][1], coeff_x[6][1], coeff_x[7][1], coeff_x[8][1]);
  TPxGetEyePosition_lib(coeff_x, coeff_y, eyeReturn);
  return;
#endif
}

void TPxBestPolyFinishCalibration()
{
#if USE_GSL
	float coeff_x_1[9][2];
	float coeff_y_1[9][2];
	float error_min_x[2];
	float error_min_y[2];	
	int calib_1_dist;
	int i;
	//float eyePosition[4];

	error_min_x[0] = 1000;
	error_min_x[1] = 1000;
	error_min_y[0] = 1000;
	error_min_y[1] = 1000;
	calib_1_dist = TPxFinishCalibration_poly_general_3terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);
	calib_1_dist = TPxFinishCalibration_poly_general_4terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);
	calib_1_dist = TPxFinishCalibration_poly_general_5terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);
	printf("Final BestPoly X (error: %f):  %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", error_min_x[0], coeff_x_1[0][0],\
				coeff_x_1[1][0], coeff_x_1[2][0], coeff_x_1[3][0], coeff_x_1[4][0], coeff_x_1[5][0], coeff_x_1[6][0], coeff_x_1[7][0], coeff_x_1[8][0]);
	printf("Final BestPoly X_L (error: %f):  %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", error_min_x[1], coeff_x_1[0][1],\
				coeff_x_1[1][1], coeff_x_1[2][1], coeff_x_1[3][1], coeff_x_1[4][1], coeff_x_1[5][1], coeff_x_1[6][1], coeff_x_1[7][1], coeff_x_1[8][1]);
	printf("Final BestPoly Y (error: %f):  %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", error_min_y[0], coeff_y_1[0][0],\
				coeff_y_1[1][0], coeff_y_1[2][0], coeff_y_1[3][0], coeff_y_1[4][0], coeff_y_1[5][0], coeff_y_1[6][0], coeff_y_1[7][0], coeff_y_1[8][0]);
	printf("Final BestPoly Y_L (error: %f):  %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", error_min_y[1], coeff_y_1[0][1],\
				coeff_y_1[1][1], coeff_y_1[2][1], coeff_y_1[3][1], coeff_y_1[4][1], coeff_y_1[5][1], coeff_y_1[6][1], coeff_y_1[7][1], coeff_y_1[8][1]);
	printf("To verify the above polynomail, use value: (%d,%d) (%d,%d)_L, which should give (%d,%d)\n", eye_data_calib_avg[11][0], eye_data_calib_avg[11][1],\
		eye_data_calib_avg[11][2],eye_data_calib_avg[11][3], screen_data_calib_avg[11][0], screen_data_calib_avg[11][1]);
	// Set the coefficient as the global variables.
	for (i = 0; i < 9; i++)
	{
		coeff_x[i][0] = coeff_x_1[i][0];
		coeff_x[i][1] = coeff_x_1[i][1];
		coeff_y[i][0] = coeff_y_1[i][0];
		coeff_y[i][1] = coeff_y_1[i][1];
	}
	IS_DEVICE_CALIBRATED = 1;
	//printf("\n\n *** Final Polynomial Finish Calibration in X_L *** \n\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", coeff_x[0][1],\
	//			coeff_x[1][1], coeff_x[2][1], coeff_x[3][1], coeff_x[4][1], coeff_x[5][1], coeff_x[6][1], coeff_x[7][1], coeff_x[8][1]);
	TPxSaveCoefficientsInTracker(coeff_x, coeff_y);
	//TPxGetEyePosition_noCeoff(eyePosition);
#endif
}

void TPxTestlibtpxFunctions()
{
#if USE_GSL
	float coeff_x_1[9][2];
	float coeff_y_1[9][2];
	float error_min_x[2];
	float error_min_y[2];	
	int calib_1_dist;
	int i;
	float eyePosition[4];
	float some_x[2];
	float some_y[2];


	error_min_x[0] = 1000;
	error_min_x[1] = 1000;
	error_min_y[0] = 1000;
	error_min_y[1] = 1000;
	eye_data_calib_avg[0][0] =  (float) 5.5044;
	eye_data_calib_avg[1][0] =  (float) 5.9482;
	eye_data_calib_avg[2][0] =  (float)19.5448;
	eye_data_calib_avg[3][0] =  (float) 5.4103;
	eye_data_calib_avg[4][0] =  (float)-6.5941;
	eye_data_calib_avg[5][0] =  (float)20.8025;
	eye_data_calib_avg[6][0] =  (float)-7.3572;
	eye_data_calib_avg[7][0] =  (float)-6.4638;
	eye_data_calib_avg[8][0] =  (float)19.0020;
	eye_data_calib_avg[9][0] =  (float) 5.8761;
	eye_data_calib_avg[10][0] =	(float) 9.4821;
	eye_data_calib_avg[11][0] =	(float) 9.8771;
	eye_data_calib_avg[12][0] =	(float) 0.9433;
	eye_data_calib_avg[13][0] =	(float)-3.7530;
	
	eye_data_calib_avg[0][1] =  (float)14.5334;
	eye_data_calib_avg[1][1] =  (float)23.2017;
	eye_data_calib_avg[2][1] =  (float)14.4498;
	eye_data_calib_avg[3][1] =  (float) 7.8006;
	eye_data_calib_avg[4][1] =  (float)14.5263;
	eye_data_calib_avg[5][1] =  (float)23.0511;
	eye_data_calib_avg[6][1] =  (float)22.9434;
	eye_data_calib_avg[7][1] =  (float) 7.7373;
	eye_data_calib_avg[8][1] =  (float) 7.0605;
	eye_data_calib_avg[9][1] =  (float)14.7058;
	eye_data_calib_avg[10][1] = (float)19.7207;
	eye_data_calib_avg[11][1] = (float) 9.7855;
	eye_data_calib_avg[12][1] = (float)20.6838;
	eye_data_calib_avg[13][1] = (float)10.3002;

	eye_data_calib_avg[0][2] =  (float) 5.5044;
	eye_data_calib_avg[1][2] =  (float) 5.9482;
	eye_data_calib_avg[2][2] =  (float)19.5448;
	eye_data_calib_avg[3][2] =  (float) 5.4103;
	eye_data_calib_avg[4][2] =  (float)-6.5941;
	eye_data_calib_avg[5][2] =  (float)20.8025;
	eye_data_calib_avg[6][2] =  (float)-7.3572;
	eye_data_calib_avg[7][2] =  (float)-6.4638;
	eye_data_calib_avg[8][2] =  (float)19.0020;
	eye_data_calib_avg[9][2] =  (float) 5.8761;
	eye_data_calib_avg[10][2] = (float) 9.4821;
	eye_data_calib_avg[11][2] = (float) 9.8771;
	eye_data_calib_avg[12][2] = (float) 0.9433;
	eye_data_calib_avg[13][2] = (float)-3.7530;
	
	eye_data_calib_avg[0][3] =  (float)14.5334;
	eye_data_calib_avg[1][3] =  (float)23.2017;
	eye_data_calib_avg[2][3] =  (float)14.4498;
	eye_data_calib_avg[3][3] =  (float) 7.8006;
	eye_data_calib_avg[4][3] =  (float)14.5263;
	eye_data_calib_avg[5][3] =  (float)23.0511;
	eye_data_calib_avg[6][3] =  (float)22.9434;
	eye_data_calib_avg[7][3] =  (float) 7.7373;
	eye_data_calib_avg[8][3] =  (float) 7.0605;
	eye_data_calib_avg[9][3] =  (float)14.7058;
	eye_data_calib_avg[10][3] = (float)19.7207;
	eye_data_calib_avg[11][3] = (float) 9.7855;
	eye_data_calib_avg[12][3] = (float)20.6838;
	eye_data_calib_avg[13][3] = (float)10.3002;

	screen_data_calib_avg[0][0] =  (float)960 ;	
	screen_data_calib_avg[1][0] =  (float)960 ;	
	screen_data_calib_avg[2][0] =  (float)1810;
	screen_data_calib_avg[3][0] =  (float)960 ;	
	screen_data_calib_avg[4][0] =  (float)110 ;	
	screen_data_calib_avg[5][0] =  (float)1810;
	screen_data_calib_avg[6][0] =  (float)110 ;	
	screen_data_calib_avg[7][0] =  (float)110 ;	
	screen_data_calib_avg[8][0] =  (float)1810;
	screen_data_calib_avg[9][0] =  (float)960 ;
	screen_data_calib_avg[10][0] = (float)1190;
	screen_data_calib_avg[11][0] = (float)1244;
	screen_data_calib_avg[12][0] = (float)657 ;
	screen_data_calib_avg[13][0] = (float)327 ;
	
	screen_data_calib_avg[0][1] =  (float)540 ;
	screen_data_calib_avg[1][1] =  (float)1020;
	screen_data_calib_avg[2][1] =  (float)540 ;
	screen_data_calib_avg[3][1] =  (float)60  ;
	screen_data_calib_avg[4][1] =  (float)540 ;
	screen_data_calib_avg[5][1] =  (float)1020;
	screen_data_calib_avg[6][1] =  (float)1020;
	screen_data_calib_avg[7][1] =  (float)60  ;
	screen_data_calib_avg[8][1] =  (float)60  ;
	screen_data_calib_avg[9][1] =  (float)540 ;
	screen_data_calib_avg[10][1] = (float)850 ;
	screen_data_calib_avg[11][1] = (float)219 ;
	screen_data_calib_avg[12][1] = (float)895 ;
	screen_data_calib_avg[13][1] = (float)235 ;
	/*
	for (i = 0; i < 124; i++)
	{
		if (__popcnt(i) > 3 && __popcnt(i) < 6)
		{
		printf("Polynomical is: Sx = A + Bx +");
		if (i & (1 << 0))
		{
			printf("C*y +");
		}
		if (i & (1 << 1))
		{
			printf("D*x^2 +");
		}
		if (i & (1 << 2))
		{
			printf("F*y^2 +");
		}
		if (i & (1 << 3))
		{
			printf("G*x^3 +");
		}
		if (i & (1 << 4))
		{
			printf("H*x*y +");
		}
		if (i & (1 << 5))
		{
			printf("I*x^2*y +");
		}
		if (i & (1 << 6))
		{
			printf("J*x^2*y^2 +");
		}
		printf(" Iteartion (%d)\n", i);
		}
	}*/
	calib_1_dist = TPxFinishCalibration_poly_general_3terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);

	printf("After Iteration %d terms: Error Y_L: %f.\n %f + %f*x + %f*y + %f*x^2 + %f*y^2 + %f*x^3 + %f*xy + %f*xxy + %f*xxyy\n", calib_1_dist, error_min_y[0], coeff_y_1[0][1],\
				coeff_y_1[1][1], coeff_y_1[2][1], coeff_y_1[3][1], coeff_y_1[4][1], coeff_y_1[5][1], coeff_y_1[6][1], coeff_y_1[7][1], coeff_y_1[8][1]);
	calib_1_dist = TPxFinishCalibration_poly_general_4terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);
	calib_1_dist = TPxFinishCalibration_poly_general_5terms(eye_data_calib_avg, screen_data_calib_avg, coeff_x_1, coeff_y_1, error_min_x, error_min_y);
	
	// Set the coefficient as the global variables.
	for (i = 0; i < 9; i++)
	{
		coeff_x[i][0] = coeff_x_1[i][0];
		coeff_x[i][1] = coeff_x_1[i][1];
		coeff_y[i][0] = coeff_y_1[i][0];
		coeff_y[i][1] = coeff_y_1[i][1];
	}
	printf("Done with tests\n");

	IS_DEVICE_CALIBRATED = 1;
	some_x[0] = (float)9.4821;
	some_x[1] = (float)9.4821;
	some_y[0] = (float)19.7207;
	some_y[1] = (float)19.7207;
	TPxGetEyePosition_validation(some_x, some_y, coeff_x, coeff_y, eyePosition);

	some_x[0] = (float)9.8771;
	some_x[1] = (float)9.8771;
	some_y[0] = (float)9.7855;
	some_y[1] = (float)9.7855;
	TPxGetEyePosition_validation(some_x, some_y, coeff_x, coeff_y, eyePosition);


	TPxSaveCoefficientsInTracker(coeff_x, coeff_y);
	TPxGetEyePosition_noCeoff(eyePosition);
	return;
#endif
}
void TPxFinishCalibration_newPoly()
{ // TO BE DONE
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
	float x_tracker[2], y_tracker[2], error_x, error_y, error_x_L, error_y_L, eyeReturn[4];
	FILE* fp;
	fp = fopen("NewCalib_debug.txt", "a+");

	for (i = 0; i < 9; i++)
	{
		fp = fopen("NewCalib_debug.txt", "a+");
		fprintf(fp, "Writing in the text file! Loop %d.\n", i);
		fclose(fp);
		gsl_vector_set(x_screen, i, screen_data_calib_avg[i][0]);
		gsl_vector_set(y_screen, i, screen_data_calib_avg[i][1]);
		gsl_vector_set(x_screen_L, i, screen_data_calib_avg[i][0]);
		gsl_vector_set(y_screen_L, i, screen_data_calib_avg[i][1]);

		x = eye_data_calib_avg[i][0];
		y = eye_data_calib_avg[i][1];
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

		x = eye_data_calib_avg[i][2];
		y = eye_data_calib_avg[i][3];

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
	det_x = gsl_linalg_LU_det(AxS_matrix_track_L, s_L);
	det_y = gsl_linalg_LU_det(AyS_matrix_track_L, v_L);

	fp = fopen("NewCalib_debug.txt", "a+");
	fprintf(fp, "Just calculated determinants: det_x:%f, det_y:%f\n", det_x, det_y);

	if (det_x == 0 || det_y == 0)
	{ // A matrix is singular, cannot solve.
		IS_DEVICE_CALIBRATED = 0;
		fprintf(fp, "Not solvable system");
		return;
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
		for (i = 0; i < 5; i++)
		{
		coeff_x_eye1[i] = (float)gsl_vector_get(x_coeff, i);
		coeff_x_eye2[i] = (float)gsl_vector_get(x_coeff_L, i);
		}
		for (i = 0; i < 7; i++)
		{
		coeff_y_eye1[i] = (float)gsl_vector_get(y_coeff, i);
		coeff_y_eye2[i] = (float)gsl_vector_get(y_coeff_L, i);
		}
		IS_DEVICE_CALIBRATED = 1;
	} 		// STEP ONE DONE
	
	
	gsl_matrix_free(Ax_matrix_track); gsl_matrix_free(Ax_matrix_track);
	gsl_matrix_free(Ax_matrix_track_L); gsl_matrix_free(Ax_matrix_track_L);

	gsl_vector_free(x_screen); gsl_vector_free(y_screen);
	gsl_vector_free(x_coeff); gsl_vector_free(y_coeff);
	gsl_vector_free(x_coeff_L); gsl_vector_free(y_coeff_L);

	gsl_permutation_free(permu_x); gsl_permutation_free(permu_y);
	gsl_permutation_free(permu_x_L); gsl_permutation_free(permu_y_L);

	fprintf(fp, "Matrix freed, work here is done.\n");
	fclose(fp);
	NUMBER_OF_SAMPLES = 0;

	error_x = 0;
	error_x_L = 0;
	error_y = 0;
	error_y_L = 0;
	for (i = 9; i < 14; i++)
		{
			x_tracker[0] = eye_data_calib_avg[i][0];
			y_tracker[0] = eye_data_calib_avg[i][1];
			x_tracker[1] = eye_data_calib_avg[i][2];
			y_tracker[1] = eye_data_calib_avg[i][3];
			TPxGetEyePosition_newPoly_valid(x_tracker, y_tracker, coeff_x_eye1, coeff_y_eye1, coeff_x_eye2, coeff_y_eye2, eyeReturn);
			error_x += (float)fabs(screen_data[i][0] - eyeReturn[0]);
			error_y += (float)fabs(screen_data[i][1] - eyeReturn[1]);
			error_x_L += (float)fabs(screen_data[i][0] - eyeReturn[2]);
			error_y_L += (float)fabs(screen_data[i][1] - eyeReturn[3]);
		}
		error_x /= 5;
		error_x_L /= 5;
		error_y /= 5;
		error_y_L /= 5;

		
		printf("Final newPoly X (error: %f):  %f + %f*x + %f*x^3 + %f*yy + %f*xy\n", error_x, coeff_x_eye1[0],\
				coeff_x_eye1[1], coeff_x_eye1[2], coeff_x_eye1[3], coeff_x_eye1[4]);
		
		printf("Final newPoly X_L (error: %f):  %f + %f*x + %f*x^3 + %f*yy + %f*xy\n", error_x_L, coeff_x_eye2[0],\
				coeff_x_eye2[1], coeff_x_eye2[2], coeff_x_eye2[3], coeff_x_eye2[4]);
		
		printf("Final newPoly Y (error: %f):  %f + %f*x + %f*x^2 + %f*y + %f*yy + %f*xy + %f*xxy\n", error_y, coeff_y_eye1[0],\
				coeff_y_eye1[1], coeff_y_eye1[2], coeff_y_eye1[3], coeff_y_eye1[4], coeff_y_eye1[5], coeff_y_eye1[6]);
		
		printf("Final newPoly Y_L (error: %f): %f + %f*x + %f*x^2 + %f*y + %f*yy + %f*xy + %f*xxy\n", error_y_L, coeff_y_eye2[0],\
				coeff_y_eye2[1], coeff_y_eye2[2], coeff_y_eye2[3], coeff_y_eye2[4], coeff_y_eye2[5], coeff_y_eye2[6]);
		
		return;
#endif
}

void TPxFinishCalibration()
{
#if USE_GSL
	// A x = b
	// Solve for x;
	// A = tracker (225,6)
	// b = screen (225, 1)
	// x = results (6,1)
	// We need A to be squared for LU solve, so we get
	// (A_tranpose * A) * x = (A_transpose * b)
	// A = tracker_transpose * tracker = tracker_square (6,6)
	// b = tracker_tranpose * screen (6,1)

	gsl_matrix *tracker_left = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);
	gsl_matrix *tracker_right = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);
	gsl_matrix *tracker_left_cam_B = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);
	gsl_matrix *tracker_right_cam_B = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);
	
	gsl_matrix *tracker_left_square = gsl_matrix_alloc(6, 6);
	gsl_matrix *tracker_right_square = gsl_matrix_alloc(6, 6);
	gsl_matrix *tracker_left_square_cam_B = gsl_matrix_alloc(6, 6);
	gsl_matrix *tracker_right_square_cam_B = gsl_matrix_alloc(6, 6);

	gsl_vector *screen_x = gsl_vector_alloc(25*CALIBRATION_POINTS_NUMBER);
	gsl_vector *screen_y = gsl_vector_alloc(25*CALIBRATION_POINTS_NUMBER);

	gsl_vector *results_x_left = gsl_vector_alloc(6);
	gsl_vector *results_y_left = gsl_vector_alloc(6);
	gsl_vector *results_x_right = gsl_vector_alloc(6);
	gsl_vector *results_y_right = gsl_vector_alloc(6);
	gsl_vector *results_x_left_cam_B = gsl_vector_alloc(6);
	gsl_vector *results_y_left_cam_B = gsl_vector_alloc(6);
	gsl_vector *results_x_right_cam_B = gsl_vector_alloc(6);
	gsl_vector *results_y_right_cam_B = gsl_vector_alloc(6);

	gsl_permutation *perm_left = gsl_permutation_alloc(6); int s;
	gsl_permutation *perm_right = gsl_permutation_alloc(6); int v;
	gsl_permutation *perm_left_cam_B = gsl_permutation_alloc(6); //int s_b;
	gsl_permutation *perm_right_cam_B = gsl_permutation_alloc(6); //int v_b;

	// Fill in the matrices of known value:
	int i;
	double x_left, y_left, x_right, y_right, det_left, det_right;
	double x_left_cam_B, y_left_cam_B, x_right_cam_B, y_right_cam_B;
	//double det_left_cam_B, det_right_cam_B

	for (i = 0; i < NUMBER_OF_SAMPLES; i++)
	{
		gsl_vector_set(screen_x, i, screen_data[i][0]);
		gsl_vector_set(screen_y, i, screen_data[i][1]);
		
		x_left =  eye_data[i][2];
		y_left =  eye_data[i][3];

		x_right = eye_data[i][0];
		y_right = eye_data[i][1];
		
		x_left_cam_B = eye_data[i][6];
		y_left_cam_B = eye_data[i][7];

		x_right_cam_B = eye_data[i][4];
		y_right_cam_B = eye_data[i][5];


		gsl_matrix_set(tracker_left, i, 0, x_left*x_left); // x^2
		gsl_matrix_set(tracker_left, i, 1, y_left*y_left); // y^2
		gsl_matrix_set(tracker_left, i, 2, x_left*y_left); // xy
		gsl_matrix_set(tracker_left, i, 3, x_left);		// x
		gsl_matrix_set(tracker_left, i, 4, y_left);		// y
		gsl_matrix_set(tracker_left, i, 5, 1);		// C

		gsl_matrix_set(tracker_right, i, 0, x_right*x_right);
		gsl_matrix_set(tracker_right, i, 1, y_right*y_right);
		gsl_matrix_set(tracker_right, i, 2, x_right*y_right);
		gsl_matrix_set(tracker_right, i, 3, x_right);
		gsl_matrix_set(tracker_right, i, 4, y_right);
		gsl_matrix_set(tracker_right, i, 5, 1);

		gsl_matrix_set(tracker_left_cam_B, i, 0, x_left_cam_B*x_left_cam_B); // x^2
		gsl_matrix_set(tracker_left_cam_B, i, 1, y_left_cam_B*y_left_cam_B); // y^2
		gsl_matrix_set(tracker_left_cam_B, i, 2, x_left_cam_B*y_left_cam_B); // xy
		gsl_matrix_set(tracker_left_cam_B, i, 3, x_left_cam_B);		// x
		gsl_matrix_set(tracker_left_cam_B, i, 4, y_left_cam_B);		// y
		gsl_matrix_set(tracker_left_cam_B, i, 5, 1);		// C

		gsl_matrix_set(tracker_right_cam_B, i, 0, x_right_cam_B*x_right_cam_B);
		gsl_matrix_set(tracker_right_cam_B, i, 1, y_right_cam_B*y_right_cam_B);
		gsl_matrix_set(tracker_right_cam_B, i, 2, x_right_cam_B*y_right_cam_B);
		gsl_matrix_set(tracker_right_cam_B, i, 3, x_right_cam_B);
		gsl_matrix_set(tracker_right_cam_B, i, 4, y_right_cam_B);
		gsl_matrix_set(tracker_right_cam_B, i, 5, 1);
	}

	// Calculations!

	// We must make A square, multiply it (and the RHS) by its transpose
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, tracker_left, tracker_left, 0.0, tracker_left_square);
	gsl_blas_dgemv(CblasTrans, 1.0, tracker_left, screen_x, 0.0, results_x_left);
	gsl_blas_dgemv(CblasTrans, 1.0, tracker_left, screen_y, 0.0, results_y_left);
	
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, tracker_right, tracker_right, 0.0, tracker_right_square);
	gsl_blas_dgemv(CblasTrans, 1.0, tracker_right, screen_x, 0.0, results_x_right);
	gsl_blas_dgemv(CblasTrans, 1.0, tracker_right, screen_y, 0.0, results_y_right);

	//gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, tracker_left_cam_B, tracker_left_cam_B, 0.0, tracker_left_square_cam_B);
	//gsl_blas_dgemv(CblasTrans, 1.0, tracker_left_cam_B, screen_x, 0.0, results_x_left_cam_B);
	//gsl_blas_dgemv(CblasTrans, 1.0, tracker_left_cam_B, screen_y, 0.0, results_y_left_cam_B);

	//gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, tracker_right_cam_B, tracker_right, 0.0, tracker_right_square_cam_B);
	//gsl_blas_dgemv(CblasTrans, 1.0, tracker_right_cam_B, screen_x, 0.0, results_x_right_cam_B);
	//gsl_blas_dgemv(CblasTrans, 1.0, tracker_right_cam_B, screen_y, 0.0, results_y_right_cam_B);

	gsl_linalg_LU_decomp(tracker_left_square, perm_left, &s);	
	gsl_linalg_LU_decomp(tracker_right_square, perm_right, &v);

	//gsl_linalg_LU_decomp(tracker_left_square_cam_B, perm_left_cam_B, &s_b);	
	//gsl_linalg_LU_decomp(tracker_right_square_cam_B, perm_right_cam_B, &v_b);
	
	
	det_left = gsl_linalg_LU_det(tracker_left_square, s);
	det_right = gsl_linalg_LU_det(tracker_right_square, v);

	//det_left_cam_B = gsl_linalg_LU_det(tracker_left_square_cam_B, s_b);
	//det_right_cam_B = gsl_linalg_LU_det(tracker_right_square_cam_B, v_b_;

	//if (det_left == 0 || det_right == 0) // || det_left_cam_B == 0 || det_right_cam_B == 0)
	if (det_right == 0 || det_left == 0)
	{ // A matrix is singular, cannot solve.
		IS_DEVICE_CALIBRATED = 0;
	}
	else
	{
		gsl_linalg_LU_solve(tracker_left_square, perm_left, results_x_left, results_x_left);
		gsl_linalg_LU_solve(tracker_left_square, perm_left, results_y_left, results_y_left);

		gsl_linalg_LU_solve(tracker_right_square, perm_right, results_x_right, results_x_right);
		gsl_linalg_LU_solve(tracker_right_square, perm_right, results_y_right, results_y_right);

		//gsl_linalg_LU_solve(tracker_left_square_cam_B, perm_left_cam_B, results_x_left_cam_B, results_x_left_cam_B);
		//gsl_linalg_LU_solve(tracker_left_square_cam_B, perm_left_cam_B, results_y_left_cam_B, results_y_left_cam_B);

		//gsl_linalg_LU_solve(tracker_right_square_cam_B, perm_right_cam_B, results_x_right_cam_B, results_x_right_cam_B);
		//gsl_linalg_LU_solve(tracker_right_square_cam_B, perm_right_cam_B, results_y_right_cam_B, results_y_right_cam_B);

		// Results
		for (i = 0; i < 6; i++)
		{
			parameter_X_eye1[i] = (float)gsl_vector_get(results_x_left, i);
			parameter_Y_eye1[i] = (float)gsl_vector_get(results_y_left, i);
			parameter_X_eye2[i] = (float)gsl_vector_get(results_x_right, i);
			parameter_Y_eye2[i] = (float)gsl_vector_get(results_y_right, i);
			//parameter_X_eye1_cam_B[i] = gsl_vector_get(results_x_left_cam_B, i);
			//parameter_Y_eye1_cam_B[i] = gsl_vector_get(results_y_left_cam_B, i);
			//parameter_X_eye2_cam_B[i] = gsl_vector_get(results_x_right_cam_B, i);
			//parameter_Y_eye2_cam_B[i] = gsl_vector_get(results_y_right_cam_B, i);

		}
		IS_DEVICE_CALIBRATED = 1;
	}
	// Freeing the matrices!
	gsl_matrix_free(tracker_left);
	gsl_matrix_free(tracker_right);
	gsl_matrix_free(tracker_left_cam_B);
	gsl_matrix_free(tracker_right_cam_B);

	gsl_matrix_free(tracker_left_square);
	gsl_matrix_free(tracker_right_square);
	gsl_matrix_free(tracker_left_square_cam_B);
	gsl_matrix_free(tracker_right_square_cam_B);

	gsl_vector_free(screen_x);
	gsl_vector_free(screen_y);

	gsl_vector_free(results_x_left);
	gsl_vector_free(results_y_left);
	gsl_vector_free(results_x_right);
	gsl_vector_free(results_y_right);
	gsl_vector_free(results_x_left_cam_B);
	gsl_vector_free(results_y_left_cam_B);
	gsl_vector_free(results_x_right_cam_B);
	gsl_vector_free(results_y_right_cam_B);

	gsl_permutation_free(perm_left);
	gsl_permutation_free(perm_right);
	gsl_permutation_free(perm_left_cam_B);
	gsl_permutation_free(perm_right_cam_B);

	NUMBER_OF_SAMPLES = 0;
	return;
#endif
}

int TPxIsDeviceCalibrated()
{
//#if USE_GSL
	return IS_DEVICE_CALIBRATED;
//#endif
}
/*
void TPxFinishCalibration_OLD()
{
#if USE_GSL
	// T represent the tracker given (x,y)
	// S represents the screen given (x,y)
	// Screen = C * Tracker
	// The math is as follow:
	// C = S * transpose(T) * (T*transpose(T))^(-1)
	
	// Contruct the matricies
	// Screen
	gsl_matrix *S_x = gsl_matrix_alloc(1, 25*CALIBRATION_POINTS_NUMBER);
	gsl_matrix *S_y = gsl_matrix_alloc(1, 25*CALIBRATION_POINTS_NUMBER);
	// Tracker
	gsl_matrix *T_L = gsl_matrix_alloc(6, 25*CALIBRATION_POINTS_NUMBER);
	gsl_matrix *T_R = gsl_matrix_alloc(6, 25*CALIBRATION_POINTS_NUMBER);
	// Results
	gsl_matrix *C_x_L = gsl_matrix_alloc(1, 6);
	gsl_matrix *C_y_L = gsl_matrix_alloc(1, 6);
	gsl_matrix *C_x_R = gsl_matrix_alloc(1, 6);
	gsl_matrix *C_y_R = gsl_matrix_alloc(1, 6);
	
	gsl_matrix* T_L_trans = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);
	gsl_matrix *T_R_trans = gsl_matrix_alloc(25*CALIBRATION_POINTS_NUMBER, 6);

	gsl_matrix *T_L_mult = gsl_matrix_alloc(6,6);
	gsl_matrix *T_R_mult = gsl_matrix_alloc(6,6);

	gsl_matrix *S_x_times_tt_L = gsl_matrix_alloc(1,6);
	gsl_matrix *S_y_times_tt_L = gsl_matrix_alloc(1,6);
	gsl_matrix *S_x_times_tt_R = gsl_matrix_alloc(1,6);
	gsl_matrix *S_y_times_tt_R = gsl_matrix_alloc(1,6);

	double x_L;
	double y_L;
		
	double x_R;
	double y_R;

	int i;

	gsl_matrix *T_L_mult_inv = gsl_matrix_alloc(6,6);
	gsl_matrix *T_R_mult_inv = gsl_matrix_alloc(6,6);
	gsl_permutation * perm = gsl_permutation_alloc (6); int s;

	// Initiate the matrices:
	for (i = 0; i < NUMBER_OF_SAMPLES; i++)
	{
		gsl_matrix_set(S_x, 0, i, screen_data[i][0]);
		gsl_matrix_set(S_y, 0, i, screen_data[i][1]);
		
		x_L = eye_data[i][0];
		y_L = eye_data[i][1];
		
		x_R = eye_data[i][2];
		y_R = eye_data[i][3];
		
		gsl_matrix_set(T_L, 0, i, x_L*x_L); // x^2
		gsl_matrix_set(T_L, 1, i, y_L*y_L); // y^2
		gsl_matrix_set(T_L, 2, i, x_L*y_L); // xy
		gsl_matrix_set(T_L, 3, i, x_L);		// x
		gsl_matrix_set(T_L, 4, i, y_L);		// y
		gsl_matrix_set(T_L, 5, i, 1);		// C

		gsl_matrix_set(T_R, 0, i, x_R*x_R);
		gsl_matrix_set(T_R, 1, i, y_R*y_R);
		gsl_matrix_set(T_R, 2, i, x_R*y_R);
		gsl_matrix_set(T_R, 3, i, x_R);
		gsl_matrix_set(T_R, 4, i, y_R);
		gsl_matrix_set(T_R, 5, i, 1);
	}
	// Calculate transpose(T)
	gsl_matrix_transpose_memcpy(T_L_trans, T_L);
	gsl_matrix_transpose_memcpy(T_R_trans, T_R);

	//Calculate T*(transpose(T))
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T_L, T_L_trans, 0.0, T_L_mult);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T_R, T_R_trans, 0.0, T_R_mult);

	// invert this matrix, and make sure its determinant is not zero so it can be inverted.
	gsl_linalg_LU_decomp (T_L_mult, perm, &s);
	gsl_linalg_LU_invert(T_L_mult, perm, T_L_mult_inv);
	gsl_linalg_LU_decomp (T_R_mult, perm, &s);
	gsl_linalg_LU_invert(T_R_mult, perm, T_R_mult_inv);
	
	// Calculate S* T^t
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_x, T_L_trans, 0.0, S_x_times_tt_L);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_y, T_L_trans, 0.0, S_y_times_tt_L);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_x, T_R_trans, 0.0, S_x_times_tt_R);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_y, T_R_trans, 0.0, S_y_times_tt_R);

	// Calculate C, S*T^t * (T*T^t)^(-1)
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_x_times_tt_L, T_L_mult_inv, 0.0, C_x_L);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_y_times_tt_L, T_L_mult_inv, 0.0, C_y_L);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_x_times_tt_R, T_R_mult_inv, 0.0, C_x_R);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_y_times_tt_R, T_R_mult_inv, 0.0, C_y_R);
	
	// Prinit final results
	//print_matrix(stdout, C_x_L);
	//print_matrix(stdout, C_x_R);
	//print_matrix(stdout, C_y_L);
	//print_matrix(stdout, C_y_R);

	// Set our global results
	for (i = 0; i<6; i++)
	{
		parameter_X_eye1[i] = gsl_matrix_get(C_x_L, 0, i);
		parameter_Y_eye1[i] = gsl_matrix_get(C_y_L, 0, i); 
		parameter_X_eye2[i] = gsl_matrix_get(C_x_R, 0, i);
		parameter_Y_eye2[i] = gsl_matrix_get(C_y_R, 0, i); 	
	}

	// Free the matrices
	gsl_matrix_free(C_x_L);
	gsl_matrix_free(C_y_L);
	gsl_matrix_free(C_x_R);
	gsl_matrix_free(C_y_R);
	gsl_matrix_free(S_x);
	gsl_matrix_free(S_y);
	gsl_matrix_free(T_L);
	gsl_matrix_free(T_R);
	gsl_matrix_free(T_L_trans);
	gsl_matrix_free(T_R_trans);
	gsl_matrix_free(T_L_mult);
	gsl_matrix_free(T_R_mult);
	gsl_matrix_free(T_L_mult_inv);
	gsl_matrix_free(T_R_mult_inv);
	gsl_matrix_free(S_x_times_tt_L);
	gsl_matrix_free(S_y_times_tt_L);
	gsl_matrix_free(S_x_times_tt_R);
	gsl_matrix_free(S_y_times_tt_R);
	NUMBER_OF_SAMPLES = 0;
	return;
#endif
}
*/

void TPxGetEyePosition_newPoly_valid(float vx[2], float vy[2], float vcoeff_x_eye1[5], float vcoeff_y_eye1[7], float vcoeff_x_eye2[5],  float vcoeff_y_eye2[7], float* eyeReturn)
{
#if USE_GSL
	float x,y;

	x = vx[0];
	y = vy[0];

	eyeReturn[0] =  coeff_x_eye1[0]       +
					coeff_x_eye1[1]*x     +
					coeff_x_eye1[2]*x*x*x +
					coeff_x_eye1[3]*y*y   +
					coeff_x_eye1[4]*x*y;

	eyeReturn[1] = 	coeff_y_eye1[0]     +
					coeff_y_eye1[1]*x   +
					coeff_y_eye1[2]*x*x +
					coeff_y_eye1[3]*y   +
					coeff_y_eye1[4]*y*y +
					coeff_y_eye1[5]*x*y +
					coeff_y_eye1[6]*x*x*y;

	x = vx[1];
	y = vy[1];
	eyeReturn[2] =  coeff_x_eye2[0]       +
					coeff_x_eye2[1]*x     +
					coeff_x_eye2[2]*x*x*x +
					coeff_x_eye2[3]*y*y   +
					coeff_x_eye2[4]*x*y;

	eyeReturn[3] = 	coeff_y_eye2[0]     +
					coeff_y_eye2[1]*x   +
					coeff_y_eye2[2]*x*x +
					coeff_y_eye2[3]*y   +
					coeff_y_eye2[4]*y*y +
					coeff_y_eye2[5]*x*y +
					coeff_y_eye2[6]*x*x*y;
	return;
#endif
}

void TPxGetEyePosition_newPoly(float* eyeReturn)
{

#if USE_GSL

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
	eyeReturn[0] =  coeff_x_eye1[0]       +
					coeff_x_eye1[1]*x     +
					coeff_x_eye1[2]*x*x*x +
					coeff_x_eye1[3]*y*y   +
					coeff_x_eye1[4]*x*y;

	eyeReturn[1] = 	coeff_y_eye1[0]     +
					coeff_y_eye1[1]*x   +
					coeff_y_eye1[2]*x*x +
					coeff_y_eye1[3]*y   +
					coeff_y_eye1[4]*y*y +
					coeff_y_eye1[5]*x*y +
					coeff_y_eye1[6]*x*x*y;

	x = LUXA_LEFT_X;
	y = LUXA_LEFT_Y;
	eyeReturn[2] =  coeff_x_eye2[0]       +
					coeff_x_eye2[1]*x     +
					coeff_x_eye2[2]*x*x*x +
					coeff_x_eye2[3]*y*y   +
					coeff_x_eye2[4]*x*y;

	eyeReturn[3] = 	coeff_y_eye2[0]     +
					coeff_y_eye2[1]*x   +
					coeff_y_eye2[2]*x*x +
					coeff_y_eye2[3]*y   +
					coeff_y_eye2[4]*y*y +
					coeff_y_eye2[5]*x*y +
					coeff_y_eye2[6]*x*x*y;
#endif
}

void TPxGetEyePosition_new(float* eyeReturn)
{
	// Step 1: Correct for drift  (x_drift_cor = x_track - DF)

	// Step 2: Calculate biquad value x_biquad = A*x_drift_cor+B*y_drift_cor+C*x_drift_cor^2+D*y_drift_cor^2+E

	// Step 3: Calculate quadrant correction x_screen = x_biquad + m[i]*x_biquad*y_biquad,
	// where i is the quadrant number of x_track_biquad
#if USE_GSL
	FILE* fp;
//	float result[4];
	float dividor;

	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
		
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;

	float LUXB_LEFT_X; 
	float LUXB_LEFT_Y;
			
	float LUXB_RIGHT_X; 
	float LUXB_RIGHT_Y;

	float x_biquad;
	float y_biquad;
	int distance_quad1, distance_quad2, distance_quad3, distance_quad4, distance_min;
	float quad_factor_x;
	float quad_factor_y;


	// We read this as 16.16 ints, we must convert to  float by deviding by 2^16
	fp = fopen("00b.txt", "a+");
	DPxUpdateRegCache();

	// We read this as 8.8 ints, we must convert to  float by deviding by 2^8
	dividor = 1 << 8;
	LUXA_RIGHT_X = ((short)DPxGetReg16(0x390)) / dividor;
	LUXA_RIGHT_Y = ((short)DPxGetReg16(0x392)) / dividor;
			
	LUXA_LEFT_X = ((short)DPxGetReg16(0x380)) / dividor;
	LUXA_LEFT_Y = ((short)DPxGetReg16(0x382)) / dividor; 
	// WE IGNORE LUXB FOR NOW
	LUXB_LEFT_X = ((short)DPxGetReg16(0x3A0)) / dividor;
	LUXB_LEFT_Y = ((short)DPxGetReg16(0x3A2)) / dividor;
			
	LUXB_RIGHT_X = ((short)DPxGetReg16(0x3B0)) / dividor;
	LUXB_RIGHT_Y = ((short)DPxGetReg16(0x3B2)) / dividor;


	//Step 1
	LUXA_RIGHT_X = LUXA_RIGHT_X - x_drift_corr;
	LUXA_RIGHT_Y = LUXA_RIGHT_Y - y_drift_corr;


	//Step 2


	x_biquad =	coeff_x_eye1[0]*LUXA_RIGHT_X +
				coeff_x_eye1[1]*LUXA_RIGHT_Y +
				coeff_x_eye1[2]*LUXA_RIGHT_X*LUXA_RIGHT_X +
				coeff_x_eye1[3]*LUXA_RIGHT_Y*LUXA_RIGHT_Y +
				coeff_x_eye1[4];
	
	y_biquad =	coeff_y_eye1[0]*LUXA_RIGHT_X +
				coeff_y_eye1[1]*LUXA_RIGHT_Y +
				coeff_y_eye1[2]*LUXA_RIGHT_X*LUXA_RIGHT_X +
				coeff_y_eye1[3]*LUXA_RIGHT_Y*LUXA_RIGHT_Y +
				coeff_y_eye1[4];

	fprintf(fp, "x_biquad:%f, y_biquad:%f\n", x_biquad, y_biquad);
	// Step 3, need to figure out the quadrant of x_biquad and y_biquad, Find the closest point.

	distance_quad1 = (int)sqrt(pow((x_biquad - screen_data_calib_avg[5][0]), 2) + pow((y_biquad - screen_data_calib_avg[5][1]), 2));
	distance_quad2 = (int)sqrt(pow((x_biquad - screen_data_calib_avg[6][0]), 2) + pow((y_biquad - screen_data_calib_avg[6][1]), 2));
	distance_quad3 = (int)sqrt(pow((x_biquad - screen_data_calib_avg[7][0]), 2) + pow((y_biquad - screen_data_calib_avg[7][1]), 2));
	distance_quad4 = (int)sqrt(pow((x_biquad - screen_data_calib_avg[8][0]), 2) + pow((y_biquad - screen_data_calib_avg[8][1]), 2));

#if __APPLE_CC__ || __linux__
	distance_min = fmin(fmin(fmin(distance_quad1, distance_quad2), distance_quad3), distance_quad4);
#else
	distance_min = min(min(min(distance_quad1, distance_quad2), distance_quad3), distance_quad4);
#endif

	fprintf(fp, "dist1:%d, dist2:%d, dist3:%d, dist4:%d\n", distance_quad1, distance_quad2, distance_quad3, distance_quad4);

	if (distance_min == distance_quad1)
	{
		quad_factor_x = m[0];
		quad_factor_y = n[0];
	}
	else if (distance_min == distance_quad2)
	{
		quad_factor_x = m[1];
		quad_factor_y = n[1];
	}
	else if (distance_min == distance_quad3)
	{
		quad_factor_x = m[2];
		quad_factor_y = n[2];
	}
	else if (distance_min == distance_quad4)
	{
		quad_factor_x = m[3];
		quad_factor_y = n[3];
	}
	else
	{
		quad_factor_x = m[0];
		quad_factor_y = n[0];
	}
	

	// final values
	eyeReturn[0] = x_biquad + quad_factor_x*x_biquad*y_biquad;
	eyeReturn[1] = y_biquad + quad_factor_y*x_biquad*y_biquad;
	eyeReturn[2] = x_biquad + quad_factor_x*x_biquad*y_biquad;
	eyeReturn[3] = y_biquad + quad_factor_y*x_biquad*y_biquad;

	fprintf(fp, "%f, %f, %f, %f\n", LUXA_RIGHT_X, LUXA_RIGHT_Y, eyeReturn[0], eyeReturn[1]);
	fclose(fp);
	return;
#endif
}
void TPxGetEyePosition(float* eyeReturn)
{
#if USE_GSL
	FILE* fp;
//	float result[4];
	float dividor;

	float LUXA_LEFT_X; 
	float LUXA_LEFT_Y;
		
	float LUXA_RIGHT_X; 
	float LUXA_RIGHT_Y;

	float LUXB_LEFT_X; 
	float LUXB_LEFT_Y;
			
	float LUXB_RIGHT_X; 
	float LUXB_RIGHT_Y;

	// We read this as 16.16 ints, we must convert to  float by deviding by 2^16
	fp = fopen("00b.txt", "a+");
	DPxUpdateRegCache();

	// We read this as 8.8 ints, we must convert to  float by deviding by 2^8
	dividor = 1 << 8;


	LUXA_RIGHT_X = ((short)DPxGetReg16(0x390)) / dividor;
	LUXA_RIGHT_Y = ((short)DPxGetReg16(0x392)) / dividor;
			
	LUXA_LEFT_X = ((short)DPxGetReg16(0x380)) / dividor;
	LUXA_LEFT_Y = ((short)DPxGetReg16(0x382)) / dividor; 

	// WE IGNORE LUXB FOR NOW

	LUXB_LEFT_X = ((short)DPxGetReg16(0x3A0)) / dividor;
	LUXB_LEFT_Y = ((short)DPxGetReg16(0x3A2)) / dividor;
			
	LUXB_RIGHT_X = ((short)DPxGetReg16(0x3B0)) / dividor;
	LUXB_RIGHT_Y = ((short)DPxGetReg16(0x3B2)) / dividor;

	eyeReturn[0] = parameter_X_eye1[0]*LUXA_LEFT_X*LUXA_LEFT_X+
				parameter_X_eye1[1]*LUXA_LEFT_Y*LUXA_LEFT_Y+
				parameter_X_eye1[2]*LUXA_LEFT_X*LUXA_LEFT_Y+
				parameter_X_eye1[3]*LUXA_LEFT_X+
				parameter_X_eye1[4]*LUXA_LEFT_Y+
				parameter_X_eye1[5];
	
	eyeReturn[1] = parameter_Y_eye1[0]*LUXA_LEFT_X*LUXA_LEFT_X+
				parameter_Y_eye1[1]*LUXA_LEFT_Y*LUXA_LEFT_Y+
				parameter_Y_eye1[2]*LUXA_LEFT_X*LUXA_LEFT_Y+
				parameter_Y_eye1[3]*LUXA_LEFT_X+
				parameter_Y_eye1[4]*LUXA_LEFT_Y+
				parameter_Y_eye1[5];
	
	eyeReturn[2] = parameter_X_eye2[0]*LUXA_RIGHT_X*LUXA_RIGHT_X+
				parameter_X_eye2[1]*LUXA_RIGHT_Y*LUXA_RIGHT_Y+
				parameter_X_eye2[2]*LUXA_RIGHT_X*LUXA_RIGHT_Y+
				parameter_X_eye2[3]*LUXA_RIGHT_X+
				parameter_X_eye2[4]*LUXA_RIGHT_Y+
				parameter_X_eye2[5];

	eyeReturn[3] = parameter_Y_eye2[0]*LUXA_RIGHT_X*LUXA_RIGHT_X+
				parameter_Y_eye2[1]*LUXA_RIGHT_Y*LUXA_RIGHT_Y+
				parameter_Y_eye2[2]*LUXA_RIGHT_X*LUXA_RIGHT_Y+
				parameter_Y_eye2[3]*LUXA_RIGHT_X+
				parameter_Y_eye2[4]*LUXA_RIGHT_Y+
				parameter_Y_eye2[5];
	/*
	result[0] = parameter_X_eye1_cam_B[0]*LUXB_LEFT_X*LUXB_LEFT_X+
				parameter_X_eye1_cam_B[1]*LUXB_LEFT_Y*LUXB_LEFT_Y+
				parameter_X_eye1_cam_B[2]*LUXB_LEFT_X*LUXB_LEFT_Y+
				parameter_X_eye1_cam_B[3]*LUXB_LEFT_X+
				parameter_X_eye1_cam_B[4]*LUXB_LEFT_Y+
				parameter_X_eye1_cam_B[5];
	
	result[1] = parameter_Y_eye1_cam_B[0]*LUXB_LEFT_X*LUXB_LEFT_X+
				parameter_Y_eye1_cam_B[1]*LUXB_LEFT_Y*LUXB_LEFT_Y+
				parameter_Y_eye1_cam_B[2]*LUXB_LEFT_X*LUXB_LEFT_Y+
				parameter_Y_eye1_cam_B[3]*LUXB_LEFT_X+
				parameter_Y_eye1_cam_B[4]*LUXB_LEFT_Y+
				parameter_Y_eye1_cam_B[5];

	result[2] = parameter_X_eye2_cam_B[0]*LUXB_RIGHT_X*LUXB_RIGHT_X+
				parameter_X_eye2_cam_B[1]*LUXB_RIGHT_Y*LUXB_RIGHT_Y+
				parameter_X_eye2_cam_B[2]*LUXB_RIGHT_X*LUXB_RIGHT_Y+
				parameter_X_eye2_cam_B[3]*LUXB_RIGHT_X+
				parameter_X_eye2_cam_B[4]*LUXB_RIGHT_Y+
				parameter_X_eye2_cam_B[5];

	result[3] = parameter_Y_eye2_cam_B[0]*LUXB_RIGHT_X*LUXB_RIGHT_X+
				parameter_Y_eye2_cam_B[1]*LUXB_RIGHT_Y*LUXB_RIGHT_Y+
				parameter_Y_eye2_cam_B[2]*LUXB_RIGHT_X*LUXB_RIGHT_Y+
				parameter_Y_eye2_cam_B[3]*LUXB_RIGHT_X+
				parameter_Y_eye2_cam_B[4]*LUXB_RIGHT_Y+
				parameter_Y_eye2_cam_B[5];
	// If we want to average both cameras:
	eyeReturn[0] = (eyeReturn[0] + result[0]) / 2;
	eyeReturn[1] = (eyeReturn[1] + result[1]) / 2;
	eyeReturn[2] = (eyeReturn[2] + result[2]) / 2;
	eyeReturn[3] = (eyeReturn[3] + result[3]) / 2;*/

	fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f\n", LUXA_LEFT_X, LUXA_LEFT_Y, eyeReturn[0], eyeReturn[1], LUXA_RIGHT_X, LUXA_RIGHT_Y, eyeReturn[2], eyeReturn[3]);
	fclose(fp);
	return;
#endif
}


// Set TRACKPixx RAM buffer start address.  Must be an even value.
void TPxSetBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: TPxSetBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_TRK_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= 0xF800000) {
	//if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: TPxSetBuffBaseAddr(0x%x) exceeds TRACKPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_TRK_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	if (buffBaseAddr <= 0x140000) {
		DPxDebugPrint1("ERROR: TPxSetBuffBaseAddr(0x%x) is too low.\n", buffBaseAddr);
		DPxSetError(DPX_ERR_TRK_BUFF_BASEADDR_TOO_LOW);
		return;
	}
	DPxSetReg32(DPXREG_TRK_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get TRACKPixx RAM buffer start address
unsigned TPxGetBuffBaseAddr()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_TRK_BUFF_BASEADDR_L);
}


// Set RAM address to which next TRACKPixx datum will be written.  Must be an even value.
void TPxSetBuffWriteAddr(unsigned buffWriteAddr)
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	if (buffWriteAddr & 1) {
		DPxDebugPrint1("ERROR: TPxSetBuffWriteAddr(0x%x) illegal odd address\n", buffWriteAddr);
		DPxSetError(DPX_ERR_TRK_BUFF_ODD_WRITEADDR);
		return;
	}
	if (buffWriteAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: TPxSetBuffWriteAddr(0x%x) exceeds TRACKPixx RAM\n", buffWriteAddr);
		DPxSetError(DPX_ERR_TRK_BUFF_WRITEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_TRK_BUFF_WRITEADDR_L, buffWriteAddr);
}


// Get RAM address to which next TRACKPixx datum will be written
unsigned TPxGetBuffWriteAddr()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_TRK_BUFF_WRITEADDR_L);
}


// Set TRACKPixx RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffWriteAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies continuous spooled acquisition.
void TPxSetBuffSize(unsigned buffSize)
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: TPxSetBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_TRK_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: TPxSetBuffSize(0x%x) exceeds TRACKPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_TRK_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_TRK_BUFF_SIZE_L, buffSize);
}


// Get TRACKPixx RAM buffer size in bytes
unsigned TPxGetBuffSize()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_TRK_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void TPxSetBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	TPxSetBuffBaseAddr(buffAddr);
	TPxSetBuffWriteAddr(buffAddr);
	TPxSetBuffSize(buffSize);
}


// Each buffered TRACKPixx sample is preceeded with a 64-bit nanosecond timetag
void TPxEnableLogTimetags()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	DPxSetReg32(DPXREG_TRK_SCHED_CTRL_L, DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Buffered data has no timetags
void TPxDisableLogTimetags()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	DPxSetReg32(DPXREG_TRK_SCHED_CTRL_L, DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
int TPxIsLogTimetags()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_TIMETAG;
}

void TPxEnableFreeRun()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    TPxSetWantedData();
	DPxSetReg32(DPXREG_TRK_SCHED_CTRL_L, DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_EVENTS);
}

void TPxDisableFreeRun()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;
    
	DPxSetReg32(DPXREG_TRK_SCHED_CTRL_L, DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_EVENTS);
}


int TPxIsFreeRun()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_TRK_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_EVENTS;
}


void TPxSetWantedData()
{
    if (!DPxSelectDevice(DPX_DEVSEL_TPC))
        return;

	DPxSetReg16(DPXREG_TRK_CHANSEL, 0xFFF);
};


int DisableTpxAcquisition(int enable)
{
	tpx_enable_acquisition = enable;
	return tpx_enable_acquisition;
}


 unsigned TPxSaveToCsv(unsigned address, const char* fileName)
 {
#if USE_GSL
	unsigned i= 0;
	unsigned int header_written = 0;
	unsigned int timetag_low, timetag_high, new_address, total_nbr_addr;
	unsigned char ram_buffer;
	unsigned char* p_ram_buffer;
	char comma_str[2] = ",";
	float luxA_right_x, luxA_right_y, screen_right_x, screen_right_y;
	float luxA_left_x, luxA_left_y, screen_left_x, screen_left_y;
	float x,y;

	FILE *fp, *fp2;
	fp2 = fopen("00debug.txt", "a+");
	// we do not want to write the header everytime the function is called. So we will try to read the file, if we can
	// it means that we already wrote a header. In which case we will close the file and open it in append mode.
	fp = fopen(fileName, "r");
	if (!fp)
	{
		fp = fopen(fileName, "a+");
		if (!fp)
		{
			printf("Can't open file %s \nPlease verify that the file is not currently opened and try again.\n", fileName);
			return address;
		}
		else 
		{
			if (IS_DEVICE_CALIBRATED)
			{
				fprintf(fp, "Timestamp, Left Eye Screen X, Left Eye Screen Y, Left A Size, Left B Size, Left Angle, Right Eye Screen X, Right Eye Screen Y, Right A Size, Right B Size, Right Angle, Digital In, Left Blink, Right Blink\n"); 
			}
			else
			{
				fprintf(fp, "Timestamp, Left X Vector, Left Y Vector, Left A Size, Left B Size, Left Angle, Right X Vector, Right Y Vector, Right A Size, Right B Size, Right Angle, Digital In, Left Blink, Right Blink\n"); 
			}
		}
	}
	else
	{
		fclose(fp);
		header_written = 1; 
		fp = fopen(fileName, "a+");
		if (!fp)
		{
			printf("Can't open file TRACKPixx_output_data.csv\nPlease verify that the file is not currently opened and try again.\n");
			return address;
		}
	}

	// Get total nbr of bytes to read
	DPxUpdateRegCache();
	new_address = TPxGetBuffWriteAddr();
	//fprintf(fp2, "NEW ADDRESS:%x, OLD ADDRESS:%x", new_address, address);
	if (new_address < address) { 
		total_nbr_addr = TPxGetBuffSize()-new_address;
	} //Memory just wrapped. The rest of the buffer will be porcess on next call.
	else { total_nbr_addr = new_address - address; }

	// Get the data from Tracker's RAM
	//Array used for RAM data
	p_ram_buffer = (unsigned char*) malloc(total_nbr_addr * sizeof(ram_buffer));
	if(!p_ram_buffer) {return address;}

	memset(p_ram_buffer, 0, total_nbr_addr*sizeof(ram_buffer)); //initialize the array
	DPxReadRam(address, total_nbr_addr, p_ram_buffer); // 8 bytes for timetag, 24 bytes for the data = 32
	
	// Seperate the data received from the RAM and
	// Export data to CSV file	
	//fprintf(fp, "\naddress:,0x%x\nnew_address:,0x%x\ntotal_nbr_addr:,%d \n", address, new_address, total_nbr_addr);
	//printf( "\naddress:,0x%x\nnew_address:,0x%x\ntotal_nbr_addr:,%d \n", address, new_address, total_nbr_addr);
	for(i=0; i<total_nbr_addr; i+=INDEX_INCEMENT)
	{
		//printf("next_address: %x \n", next_address);
		timetag_low  = (p_ram_buffer[i+3] << (8*3) ) | (p_ram_buffer[i+2] << (8*2) ) | (p_ram_buffer[i+1] << (8*1) ) | (p_ram_buffer[i+0] );
		timetag_high = (p_ram_buffer[i+7] << (8*3) ) | (p_ram_buffer[i+6] << (8*2) ) | (p_ram_buffer[i+5] << (8*1) ) | (p_ram_buffer[i+4] );
	
		fprintf(fp, "%0.9f%s", (4294967296.0 * timetag_high + timetag_low)/1000000000, comma_str);
		luxA_left_x = (float) ((signed short)((p_ram_buffer[ i+LEFT_X_VECTOR_HIGH ] << 8) | (p_ram_buffer[ i+LEFT_X_VECTOR_LOW ]) )) /256;
		luxA_left_y = (float) ((signed short)((p_ram_buffer[ i+LEFT_Y_VECTOR_HIGH ] << 8) | (p_ram_buffer[ i+LEFT_Y_VECTOR_LOW ]) )) /256;
		if (IS_DEVICE_CALIBRATED)
		{
			x = luxA_left_x;
			y = luxA_left_y;
			screen_left_x = coeff_x_eye1[0]       +
							coeff_x_eye1[1]*x     +
							coeff_x_eye1[2]*x*x*x +
							coeff_x_eye1[3]*y*y   +
							coeff_x_eye1[4]*x*y;

			screen_left_y = coeff_y_eye1[0]     +
							coeff_y_eye1[1]*x   +
							coeff_y_eye1[2]*x*x +
							coeff_y_eye1[3]*y   +
							coeff_y_eye1[4]*y*y +
							coeff_y_eye1[5]*x*y +
							coeff_y_eye1[6]*x*x*y;
			/* OLD
			screen_left_x = parameter_X_eye1[0]*luxA_left_x*luxA_left_x+
			parameter_X_eye1[1]*luxA_left_y*luxA_left_y+
			parameter_X_eye1[2]*luxA_left_x*luxA_left_y+
			parameter_X_eye1[3]*luxA_left_x+
			parameter_X_eye1[4]*luxA_left_y+
			parameter_X_eye1[5];

			screen_left_y = parameter_Y_eye1[0]*luxA_right_x*luxA_left_x+
			parameter_Y_eye1[1]*luxA_left_y*luxA_left_y+
			parameter_Y_eye1[2]*luxA_left_x*luxA_left_y+
			parameter_Y_eye1[3]*luxA_left_x+
			parameter_Y_eye1[4]*luxA_left_y+
			parameter_Y_eye1[5];
			*/
				
			fprintf(fp, "%f%s",   screen_left_x, comma_str);
			fprintf(fp, "%f%s",   screen_left_y, comma_str);
			fprintf(fp2, "%f, %f, %f, %f\n", luxA_left_x, luxA_left_y, screen_left_x, screen_left_y);
		}
		else
		{
			fprintf(fp, "%f%s",   luxA_left_x, comma_str);
			fprintf(fp, "%f%s",   luxA_left_y, comma_str);
		}
		
		fprintf(fp, "%f%s",   (float) ((p_ram_buffer[ i+LEFT_A_SIZE_HIGH ]    << 8) | (p_ram_buffer[ i+LEFT_A_SIZE_LOW ]) )    /256, comma_str);
		fprintf(fp, "%f%s",   (float) ((p_ram_buffer[ i+LEFT_B_SIZE_HIGH ]    << 8) | (p_ram_buffer[ i+LEFT_B_SIZE_LOW ]) )    /256, comma_str);
		fprintf(fp, "%f%s",   (float) ((p_ram_buffer[ i+LEFT_ANGLE_HIGH ]     << 8) | (p_ram_buffer[ i+LEFT_ANGLE_LOW ]) )     /256, comma_str);
		

		luxA_right_x = (float) ((signed short) ((p_ram_buffer[ i+RIGHT_X_VECTOR_HIGH ] << 8) | (p_ram_buffer[ i+RIGHT_X_VECTOR_LOW ]) )) /256;
		luxA_right_y = (float) ((signed short) ((p_ram_buffer[ i+RIGHT_Y_VECTOR_HIGH ] << 8) | (p_ram_buffer[ i+RIGHT_Y_VECTOR_LOW ]) )) /256;
		if (IS_DEVICE_CALIBRATED)
		{
			x = luxA_right_x;
			y = luxA_right_y;
			screen_right_x = coeff_x_eye1[0]       +
							coeff_x_eye1[1]*x     +
							coeff_x_eye1[2]*x*x*x +
							coeff_x_eye1[3]*y*y   +
							coeff_x_eye1[4]*x*y;

			screen_right_y = coeff_y_eye1[0]     +
							coeff_y_eye1[1]*x   +
							coeff_y_eye1[2]*x*x +
							coeff_y_eye1[3]*y   +
							coeff_y_eye1[4]*y*y +
							coeff_y_eye1[5]*x*y +
							coeff_y_eye1[6]*x*x*y;

			/* OLD
			screen_right_x = parameter_X_eye2[0]*luxA_right_x*luxA_right_x+
			parameter_X_eye2[1]*luxA_right_y*luxA_right_y+
			parameter_X_eye2[2]*luxA_right_x*luxA_right_y+
			parameter_X_eye2[3]*luxA_right_x+
			parameter_X_eye2[4]*luxA_right_y+
			parameter_X_eye2[5];

			screen_right_y = parameter_Y_eye2[0]*luxA_right_x*luxA_right_x+
			parameter_Y_eye2[1]*luxA_right_y*luxA_right_y+
			parameter_Y_eye2[2]*luxA_right_x*luxA_right_y+
			parameter_Y_eye2[3]*luxA_right_x+
			parameter_Y_eye2[4]*luxA_right_y+
			parameter_Y_eye2[5];
			*/
				
			fprintf(fp, "%f%s",   screen_right_x, comma_str);
			fprintf(fp, "%f%s",   screen_right_y, comma_str);
			fprintf(fp2, "%f, %f, %f, %f\n", luxA_right_x, luxA_right_y, screen_right_x, screen_right_y);
		}
		else 
		{
			fprintf(fp, "%f%s",   luxA_right_x, comma_str);
			fprintf(fp, "%f%s",   luxA_right_y, comma_str);
		}
		fprintf(fp, "%f%s",   (float) ((p_ram_buffer[ i+RIGHT_A_SIZE_HIGH ]   << 8) | (p_ram_buffer[ i+RIGHT_A_SIZE_LOW ]) )   /256, comma_str);
		fprintf(fp, "%f%s",   (float) ((p_ram_buffer[ i+RIGHT_B_SIZE_HIGH ]   << 8) | (p_ram_buffer[ i+RIGHT_B_SIZE_LOW ]) )   /256, comma_str);
		fprintf(fp, "%f%s",	  (float) ((p_ram_buffer[ i+RIGHT_ANGLE_HIGH ]    << 8) | (p_ram_buffer[ i+RIGHT_ANGLE_LOW ]) )    /256, comma_str);
		fprintf(fp, "%d%s",   (int)   ((p_ram_buffer[ i+DIN_VALUE_HIGH ]	  << 8) | (p_ram_buffer[ i+DIN_VALUE_LOW ]) ), comma_str);
		fprintf(fp, "%d%s",   (int)   ((p_ram_buffer[ i+EYE_BLINK ] & 0x01) / 1), comma_str);
		fprintf(fp, "%d%s\n", (int)   ((p_ram_buffer[ i+EYE_BLINK ] & 0x02) / 2), comma_str);
		
		//lets not forget the DIN data please...
	}
	fclose(fp);
	fclose(fp2);
	free(p_ram_buffer);

	return new_address;//Return the new address for next function call
#else
	return address;
#endif
 }


// Returns non-zero if we found a real device with a configured FPGA
int DPxSelectSysDevice(int devsel)
{
    int tmpDevsel;

    // Assume the worst
    dpxSysDevsel = DPX_DEVSEL_INVALID;

    // System context could require a specific device
    if (devsel >= DPX_DEVSEL_FIRST_DEVICE && devsel <= DPX_DEVSEL_LAST_DEVICE) {
        if (dpxDeviceTable[devsel].dpxHdl)
            dpxSysDevsel = devsel;
    }

    // System context has no preference
    else if (devsel == DPX_DEVSEL_ANY) {
        if (dpxUsrDevsel >= DPX_DEVSEL_FIRST_DEVICE && dpxUsrDevsel <= DPX_DEVSEL_LAST_DEVICE) {    // User has requested a specific device
            if (dpxDeviceTable[dpxUsrDevsel].dpxHdl)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the first device
            for (tmpDevsel = DPX_DEVSEL_FIRST_DEVICE; tmpDevsel <= DPX_DEVSEL_LAST_DEVICE; tmpDevsel++) {
                if (dpxDeviceTable[tmpDevsel].dpxHdl) {
                    dpxSysDevsel = tmpDevsel;                   // Found first device
                    break;
                }
            }
        }
    }
    
    // Probably looking for an I/O subsystem
    else if (devsel == DPX_DEVSEL_PPC_VPX_DPX_TPC) {
		// User has requested a specific device
		if ((dpxUsrDevsel >= DPX_DEVSEL_PPC && dpxUsrDevsel <= DPX_DEVSEL_PPC_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DPX && dpxUsrDevsel <= DPX_DEVSEL_DPX_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_VPX && dpxUsrDevsel <= DPX_DEVSEL_VPX_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DP2 && dpxUsrDevsel <= DPX_DEVSEL_DP2_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_TPC && dpxUsrDevsel <= DPX_DEVSEL_TPC_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DP3 && dpxUsrDevsel <= DPX_DEVSEL_DP3_LAST)) {
            if (dpxDeviceTable[dpxUsrDevsel].dpxGoodFpga)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the highest priority match
            if (dpxDeviceTable[DPX_DEVSEL_PPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPC;
            else if (dpxDeviceTable[DPX_DEVSEL_DP2].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP2;
			else if (dpxDeviceTable[DPX_DEVSEL_DP3].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP3;
			else if (dpxDeviceTable[DPX_DEVSEL_VPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_VPX;
            else if (dpxDeviceTable[DPX_DEVSEL_DPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DPX;
			else if (dpxDeviceTable[DPX_DEVSEL_TPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_TPC;
        }
    }

    // Some VIEWPixx PCB stuff like power supply monitoring
    else if (devsel == DPX_DEVSEL_PPC_VPX_TPC) {
		// User has requested a specific device
		if ((dpxUsrDevsel >= DPX_DEVSEL_PPC && dpxUsrDevsel <= DPX_DEVSEL_PPC_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DP2 && dpxUsrDevsel <= DPX_DEVSEL_DP2_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_VPX && dpxUsrDevsel <= DPX_DEVSEL_VPX_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_TPC && dpxUsrDevsel <= DPX_DEVSEL_TPC_LAST)) {
            if (dpxDeviceTable[dpxUsrDevsel].dpxGoodFpga)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the highest priority match
            if (dpxDeviceTable[DPX_DEVSEL_PPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPC;
            else if (dpxDeviceTable[DPX_DEVSEL_DP2].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP2;
            else if (dpxDeviceTable[DPX_DEVSEL_VPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_VPX;
			else if (dpxDeviceTable[DPX_DEVSEL_TPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_TPC;
        }
    }
    
    // SWTP, HWTP
    else if (devsel == DPX_DEVSEL_PPX_PPC_VPX) {
		// User has requested a specific device
		if ((dpxUsrDevsel >= DPX_DEVSEL_PPX && dpxUsrDevsel <= DPX_DEVSEL_PPX_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_PPC && dpxUsrDevsel <= DPX_DEVSEL_PPC_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_DP2 && dpxUsrDevsel <= DPX_DEVSEL_DP2_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_VPX && dpxUsrDevsel <= DPX_DEVSEL_VPX_LAST) ||
			(dpxUsrDevsel >= DPX_DEVSEL_DP3 && dpxUsrDevsel <= DPX_DEVSEL_DP3_LAST) ) {
            if (dpxDeviceTable[dpxUsrDevsel].dpxGoodFpga)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the highest priority match
            if (dpxDeviceTable[DPX_DEVSEL_PPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPX;
            else if (dpxDeviceTable[DPX_DEVSEL_PPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPC;
            else if (dpxDeviceTable[DPX_DEVSEL_DP2].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP2;
			else if (dpxDeviceTable[DPX_DEVSEL_DP3].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP3;
            else if (dpxDeviceTable[DPX_DEVSEL_VPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_VPX;
        }
    }
    
    // VESA, CLUT
    else if (devsel == DPX_DEVSEL_PPX_PPC_VPX_DPX) {
		// User has requested a specific device
		if ((dpxUsrDevsel >= DPX_DEVSEL_PPX && dpxUsrDevsel <= DPX_DEVSEL_PPX_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_PPC && dpxUsrDevsel <= DPX_DEVSEL_PPC_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_VPX && dpxUsrDevsel <= DPX_DEVSEL_VPX_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DPX && dpxUsrDevsel <= DPX_DEVSEL_DPX_LAST) || 
			(dpxUsrDevsel >= DPX_DEVSEL_DP2 && dpxUsrDevsel <= DPX_DEVSEL_DP2_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DP3 && dpxUsrDevsel <= DPX_DEVSEL_DP3_LAST)) {
            if (dpxDeviceTable[dpxUsrDevsel].dpxGoodFpga)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the highest priority match
            if (dpxDeviceTable[DPX_DEVSEL_PPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPX;
            else if (dpxDeviceTable[DPX_DEVSEL_PPC].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_PPC;
            else if (dpxDeviceTable[DPX_DEVSEL_DP2].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP2;
			else if (dpxDeviceTable[DPX_DEVSEL_DP3].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP3;
            else if (dpxDeviceTable[DPX_DEVSEL_VPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_VPX;
            else if (dpxDeviceTable[DPX_DEVSEL_DPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DPX;
        }
    }
    
    // HSPLIT
    else if (devsel == DPX_DEVSEL_DP2_DPX) {
		// User has requested a specific device
        if ((dpxUsrDevsel >= DPX_DEVSEL_DP2 && dpxUsrDevsel <= DPX_DEVSEL_DP2_LAST) || (dpxUsrDevsel >= DPX_DEVSEL_DPX && dpxUsrDevsel <= DPX_DEVSEL_DPX_LAST)) {    
			if (dpxDeviceTable[dpxUsrDevsel].dpxGoodFpga)
                dpxSysDevsel = dpxUsrDevsel;                    // User's request is valid
        }
        else if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {             // User has no device preference, so we'll just return the highest priority match
            if (dpxDeviceTable[DPX_DEVSEL_DP2].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DP2;
            else if (dpxDeviceTable[DPX_DEVSEL_DPX].dpxGoodFpga)
                dpxSysDevsel = DPX_DEVSEL_DPX;
        }
    }

    if (dpxSysDevsel == DPX_DEVSEL_INVALID) {
//		DPxDebugPrint1("ERROR: DPxSelectSysDevice() received invalid device %d for current operation\n", devsel);
		fprintf(stderr, "ERROR: Invalid device for current operation\n");
		DPxSetError(DPX_ERR_USB_SYSDEVSEL_INDEX);        
    }

    // Caller just wants to know if we found a real programmable device, otherwise it can abort
    return dpxSysDevsel >= DPX_DEVSEL_FIRST_DEVICE && dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE && dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED;
}


// Note that this string will only be valid until the next call to DPxGetDevselName();
char tmpDevName[256];

const char* DPxGetDevselName()
{
    int devsel;
    char *tag = NULL;

	devsel = dpxSysDevsel;

	if (devsel >= DPX_DEVSEL_UNCONFIGURED && devsel < DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED])
		tag = "RawEZ-USB";
	else if (devsel >= DPX_DEVSEL_DPX && devsel < DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX])
		tag = "DATAPixx";
	else if (devsel >= DPX_DEVSEL_VPX && devsel < DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX])
		tag = "VIEWPixx";
	else if (devsel >= DPX_DEVSEL_PPC && devsel < DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC])
		tag = "PROPixxCtrl";
	else if (devsel >= DPX_DEVSEL_PPX && devsel < DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX])
		tag = "PROPixx";
	else if (devsel >= DPX_DEVSEL_DP2 && devsel < DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2])
		tag = "DATAPixx2";
	else if (devsel >= DPX_DEVSEL_TPC && devsel < DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC])
		tag = "TRACKPixx Controller";
	else if (devsel >= DPX_DEVSEL_TPB && devsel < DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB])
		tag = "TRACKPixx Bridge";
	else if (devsel >= DPX_DEVSEL_TPX && devsel < DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX])
		tag = "TRACKPixx";
	else if (devsel >= DPX_DEVSEL_DP3 && devsel < DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3])
		tag = "DATAPixx3";

	// If dpxSysDevsel doesn't have a valid device, let's at least refer back to what the user was requesting
	if (!tag) {
        devsel = dpxUsrDevsel;

		if (devsel >= DPX_DEVSEL_UNCONFIGURED && devsel <= DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED])
			tag = "Missing RawEZ-USB";
		else if (devsel >= DPX_DEVSEL_DPX && devsel <= DPX_DEVSEL_DPX_LAST)
			tag = "Missing DATAPixx";
		else if (devsel >= DPX_DEVSEL_VPX && devsel <= DPX_DEVSEL_VPX_LAST)
			tag = "Missing VIEWPixx";
		else if (devsel >= DPX_DEVSEL_PPC && devsel <= DPX_DEVSEL_PPC_LAST)
			tag = "Missing PROPixxCtrl";
		else if (devsel >= DPX_DEVSEL_PPX && devsel <= DPX_DEVSEL_PPX_LAST)
			tag = "Missing PROPixx";
		else if (devsel >= DPX_DEVSEL_DP2 && devsel <= DPX_DEVSEL_DP2_LAST)
			tag = "Missing DATAPixx2";
		else if (devsel >= DPX_DEVSEL_TPC && devsel <= DPX_DEVSEL_TPC_LAST)
			tag = "Missing TRACKPixx Controller";
		else if (devsel >= DPX_DEVSEL_TPB && devsel <= DPX_DEVSEL_TPB_LAST)
			tag = "Missing TRACKPixx Bridge";
		else if (devsel >= DPX_DEVSEL_TPX && devsel <= DPX_DEVSEL_TPX_LAST)
			tag = "Missing TRACKPixx";
		else if (devsel >= DPX_DEVSEL_DP3 && devsel <= DPX_DEVSEL_DP3_LAST)
			tag = "Missing DATAPixx3";
		else if (devsel == DPX_DEVSEL_AUTO)
			tag = "ANY DEVICE";
		else
			tag = "UNKNOWN DEVICE";
    }

	sprintf(tmpDevName, "[%d.%d] > %s #%d", devsel/DPX_DEVSEL_MULTI, devsel%DPX_DEVSEL_MULTI+1 ,tag, (devsel%DPX_DEVSEL_MULTI) + 1); 
    return tmpDevName;
}

int DPxGetDevselCount()
{
	int devsel, devselCount;
	devsel = dpxSysDevsel;
	devselCount = 0;
	
	if (devsel >= DPX_DEVSEL_UNCONFIGURED && devsel < DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED])
		devselCount = devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED];
	else if (devsel >= DPX_DEVSEL_DPX && devsel < DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX])
		devselCount = devselCnt[DPX_DEVSEL_CNT_DPX];
	else if (devsel >= DPX_DEVSEL_VPX && devsel < DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX])
		devselCount = devselCnt[DPX_DEVSEL_CNT_VPX];
	else if (devsel >= DPX_DEVSEL_PPC && devsel < DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC])
		devselCount = devselCnt[DPX_DEVSEL_CNT_PPC];
	else if (devsel >= DPX_DEVSEL_PPX && devsel < DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX])
		devselCount = devselCnt[DPX_DEVSEL_CNT_PPX];
	else if (devsel >= DPX_DEVSEL_DP2 && devsel < DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2])
		devselCount = devselCnt[DPX_DEVSEL_CNT_DP2];
	else if (devsel >= DPX_DEVSEL_TPC && devsel < DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC])
		devselCount = devselCnt[DPX_DEVSEL_CNT_TPC];
	else if (devsel >= DPX_DEVSEL_TPB && devsel < DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB])
		devselCount = devselCnt[DPX_DEVSEL_CNT_TPB];
	else if (devsel >= DPX_DEVSEL_TPX && devsel < DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX])
		devselCount = devselCnt[DPX_DEVSEL_CNT_TPX];
	else if (devsel >= DPX_DEVSEL_DP3 && devsel < DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3])
		devselCount = devselCnt[DPX_DEVSEL_CNT_DP3];
	
	return devselCount;
}


// Get user-specified device name, ignoring whether or not it has been assigned to dpxSysDevsel
const char* DPxGetUsrDevselName()
{
    char *tag = NULL;
	int  isUnknown = 0;
	int  devsel;

	devsel = dpxUsrDevsel;

	if (devsel >= DPX_DEVSEL_UNCONFIGURED && devsel < DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED])
		tag = "RawEZ-USB";
	else if (devsel >= DPX_DEVSEL_DPX && devsel < DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX])
		tag = "DATAPixx";
	else if (devsel >= DPX_DEVSEL_VPX && devsel < DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX])
		tag = "VIEWPixx";
	else if (devsel >= DPX_DEVSEL_PPC && devsel < DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC])
		tag = "PROPixxCtrl";
	else if (devsel >= DPX_DEVSEL_PPX && devsel < DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX])
		tag = "PROPixx";
	else if (devsel >= DPX_DEVSEL_DP2 && devsel < DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2])
		tag = "DATAPixx2";
	else if (devsel >= DPX_DEVSEL_TPC && devsel < DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC])
		tag = "TRACKPixx Controller";
	else if (devsel >= DPX_DEVSEL_TPB && devsel < DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB])
		tag = "TRACKPixx Bridge";
	else if (devsel >= DPX_DEVSEL_TPX && devsel < DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX])
		tag = "TRACKPixx";
	else if (devsel >= DPX_DEVSEL_DP3 && devsel < DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3])
		tag = "DATAPixx3";

	// If dpxSysDevsel doesn't have a valid device, let's at least refer back to what the user was requesting
	if (!tag) {

		if (devsel >= DPX_DEVSEL_UNCONFIGURED && devsel <= DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED])
			tag = "Missing RawEZ-USB";
		else if (devsel >= DPX_DEVSEL_DPX && devsel <= DPX_DEVSEL_DPX_LAST)
			tag = "Missing DATAPixx";
		else if (devsel >= DPX_DEVSEL_VPX && devsel <= DPX_DEVSEL_VPX_LAST)
			tag = "Missing VIEWPixx";
		else if (devsel >= DPX_DEVSEL_PPC && devsel <= DPX_DEVSEL_PPC_LAST)
			tag = "Missing PROPixxCtrl";
		else if (devsel >= DPX_DEVSEL_PPX && devsel <= DPX_DEVSEL_PPX_LAST)
			tag = "Missing PROPixx";
		else if (devsel >= DPX_DEVSEL_DP2 && devsel <= DPX_DEVSEL_DP2_LAST)
			tag = "Missing DATAPixx2";
		else if (devsel >= DPX_DEVSEL_TPC && devsel <= DPX_DEVSEL_TPC_LAST)
			tag = "Missing TRACKPixx Controller";
		else if (devsel >= DPX_DEVSEL_TPB && devsel <= DPX_DEVSEL_TPB_LAST)
			tag = "Missing TRACKPixx Bridge";
		else if (devsel >= DPX_DEVSEL_TPX && devsel <= DPX_DEVSEL_TPX_LAST)
			tag = "Missing TRACKPixx";
		else if (devsel >= DPX_DEVSEL_DP3 && devsel <= DPX_DEVSEL_DP3_LAST)
			tag = "Missing DATAPixx3";
		else if (devsel == DPX_DEVSEL_AUTO)
			tag = "ANY DEVICE";
		else
			tag = "UNKNOWN DEVICE";
    }

	if (dpxUsrDevsel == DPX_DEVSEL_AUTO || isUnknown)
		sprintf(tmpDevName, "%d:(%s)", dpxUsrDevsel, tag);
	else
		sprintf(tmpDevName, "[%d.%d] > %s #%d", dpxUsrDevsel/DPX_DEVSEL_MULTI, dpxUsrDevsel%DPX_DEVSEL_MULTI+1 ,tag, (dpxUsrDevsel%DPX_DEVSEL_MULTI) + 1); 
    
	return tmpDevName;
}

// Get user-specified device name, ignoring whether or not it has been assigned to dpxSysDevsel
const char* DPxGetCustomDevName()
{
	return dpxDeviceTable[dpxSysDevsel].dpxDeviceName;
}

void EZUploadRam(unsigned char *buf, int start, int len)
{
	int i;
	int tlen;
	int quanta=16;
	int a;

    // Caller is responsible for ensuring valid dpxSysDevsel
	for (i=start;i<start+len;i+=quanta) {
		tlen = len+start-i;
		if (tlen > quanta)
			tlen = quanta;

#ifdef USE_LIB01
		a = usb_control_msg(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x40, 0xa0, i, 0, (char*)(buf+(i-start)), tlen, 1000);
#else
		a = libusb_control_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x40, 0xa0 & 0xff, i, 0, (unsigned char*)(buf+(i-start)), tlen, 1000);
#endif

		if (a < 0)
			return;
	}
}


// Can't write the SFR's, but CAN write the CPU RESET bit.
// Uses EP0, so might not maintain order wrt EZWriteByte, which uses EP1.
void EZUploadByte(int addr, unsigned char val)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	EZUploadRam(&val, addr, 1);
}


int EZWriteByte(unsigned short addr, unsigned char val)
{
	unsigned char buffer[7];

    // Caller is responsible for ensuring valid dpxSysDevsel
	buffer[0] = '^';
	buffer[1] = EP1OUT_WRITEBYTE;
	buffer[2] = 3;
	buffer[3] = 0;
	buffer[4] = LSB(addr);
	buffer[5] = MSB(addr);
	buffer[6] = val;
	if (EZWriteEP1Tram(buffer, 0, 0)) {
		DPxDebugPrint0("ERROR: EZWriteByte() call to EZWriteEP1Tram() failed\n");
		return -1;
	}
	return 0;
}


// Read a single byte from EZ memory.
// Return the byte value (0-255), or -1 if an error.
int EZReadByte(unsigned short addr)
{
	unsigned char buffer[6];

    // Caller is responsible for ensuring valid dpxSysDevsel
	buffer[0] = '^';
	buffer[1] = EP1OUT_READBYTE;
	buffer[2] = 2;
	buffer[3] = 0;
	buffer[4] = LSB(addr);
	buffer[5] = MSB(addr);
	if (EZWriteEP1Tram(buffer, EP1IN_READBYTE, 1)) {
		DPxDebugPrint0("ERROR: EZReadByte() call to EZWriteEP1Tram() failed\n");
		return -1;
	}
	return ep1in_Tram[4];
}


// Write a byte to an EZ-USB Special Function Register.
// Return -1 if there was an error.
int EZWriteSFR(unsigned char addr, unsigned char val)
{
	unsigned char buffer[6];

    // Caller is responsible for ensuring valid dpxSysDevsel
	buffer[0] = '^';
	buffer[1] = EP1OUT_WRITEBYTE;
	buffer[2] = 2;
	buffer[3] = 0;
	buffer[4] = addr;
	buffer[5] = val;
	if (EZWriteEP1Tram(buffer, 0, 0)) {
		DPxDebugPrint0("ERROR: EZWriteSFR() call to EZWriteEP1Tram() failed\n");
		return -1;
	}
	return 0;
}


// Read a byte from an EZ-USB Special Function Register.
// Return the byte value (0-255), or -1 if an error.
int EZReadSFR(unsigned char addr)
{
	unsigned char buffer[5];

    // Caller is responsible for ensuring valid dpxSysDevsel
	buffer[0] = '^';
	buffer[1] = EP1OUT_READBYTE;
	buffer[2] = 1;
	buffer[3] = 0;
	buffer[4] = addr;
	if (EZWriteEP1Tram(buffer, EP1IN_READBYTE, 1)) {
		DPxDebugPrint0("ERROR: EZReadSFR() call to EZWriteEP1Tram() failed\n");
		return -1;
	}
	return ep1in_Tram[4];
}


// Write a tram to EP1OUT, and read EP1IN at least once to see if there's any console data.
// Optionally wait for a response tram whose code is passed in rxTramCode.
// Returns 0 for success, or -1 error if:
//	-EP1OUT write failed.
//	-EP1IN read failed.
//	-expectedRxTram was non-0 (indicating an expected Rx message), but the Rx tram had the wrong code.
int EZWriteEP1Tram(unsigned char* txTram, unsigned char expectedRxTram, int expectedRxLen)
{
	int packetSize;
	int nTxBytes = 4 + txTram[2] + (txTram[3] << 8);									// Number of bytes to transmit
	int iRetry;
    int readEP1;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif

    // Caller is responsible for ensuring valid dpxSysDevsel
#if ENABLE_CONSOLE
	readEP1 = (txTram[1] != EP1OUT_RESET);
#else
	readEP1 = expectedRxTram;
#endif

	while (nTxBytes) {
		packetSize = nTxBytes >= 64 ? 64 : nTxBytes;									// EZ EP1 only supports 64 byte packets
		for (iRetry = 0; ; iRetry++) {
            dpxDeviceTable[dpxSysDevsel].nEP1Writes++;
#ifdef USE_LIB01
			if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 1, (char*)txTram, packetSize, 1000) == packetSize)
#else
			if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 1, (char*)txTram, packetSize, &packetSizeWrite, 1000) == 0) &&
				(packetSize == packetSizeWrite))
#endif
				break;
			else if (iRetry < MAX_RETRIES) {
				DPxDebugPrint1("ERROR: EZWriteEP1Tram() bulk write retried: %s\n", usb_strerror());
				dpxEp1WrRetries++;
			}
			else {
				DPxDebugPrint1("ERROR: EZWriteEP1Tram() bulk write failed: %s\n", usb_strerror());
				dpxEp1WrFails++;
				return -1;
			}
		}
		txTram += packetSize;
		nTxBytes -= packetSize;
	}

	// Do at least one EP1IN read to catch EZ console output, unless the tram we just sent is resetting the EZ.
	if (readEP1 && EZReadEP1Tram(expectedRxTram, expectedRxLen) < 0) {
		DPxDebugPrint0("ERROR: EZWriteEP1Tram() call to EZReadEP1Tram() failed\n");
		return -1;
	}
	return 0;
}


// EZReadEP1Tram() reads a tram from the EZUSB EP1IN endpoint.
// There are 2 modes of operation, depending on the value of the "expectedTram" argument:
// 1) If expectedTram = 0, then EZReadEP1Tram() operates in a look-ahead mode.
// EZReadEP1Tram() is called often in this mode (even when no trams are expected) and the function should return as quicky as possible.
// Individual USB reads should typically be pretty snappy because EZ FW keeps stuffing empty EP1IN pipe with flush trams.
// This mode does a maximum of 1 read from EZ, printing any returned console trams.
// If a data tram is received, it is cached and its tram code is returned.
// The cached tram will be returned again the next time EZReadEP1Tram() is called.
// Otherwise EZReadEP1Tram() returns 0 if no data trams are available, or a negative error code.
// 2) If expectedTram != 0, then EZReadEP1Tram() will wait until a data tram is received, or a timeout occurs.
// If a data tram is received, and its tram code equals expectedTram, EZReadEP1Tram() returns 0.
// All other cases return an error code.
int EZReadEP1Tram(unsigned char expectedTram, int expectedLen)
{
	static char packet[64];				// Largest possible EP1 USB packet
	static int	packetLength	= 0;
	static int	packetRdIndex	= 0;
	int	tramWrIndex		= 0;
	static int	tramLen			= 0;	// The length of the payload
	static int	cached			= 0;
	int iRetry;

    // Caller is responsible for ensuring valid dpxSysDevsel

	// Do we already have a tram cached from a previous call to EZReadEP1Tram(0) ?
	if (cached) {
		if (expectedTram == 0)
			return ep1in_Tram[1];								// Next caller will get same tram
		cached = 0;
		if (ep1in_Tram[1] != expectedTram) {
			DPxDebugPrint2("ERROR: EZReadEP1Tram() received tram code [%d] instead of [%d]\n", (int)ep1in_Tram[1], (int)expectedTram);
			return -1;
		}
		if (tramLen != expectedLen) {
			DPxDebugPrint2("ERROR: EZReadEP1Tram() received tram length [%d] instead of [%d]\n", tramLen, expectedLen);
			return -1;
		}
		return 0;
	}

	// Each iteration either reads a new 64 byte USB packet, or starts a new tram
	while (1) {
		// If we're out of data, or we had an error, read another packet.
		// If the EZ FW is still alive, it should always return pretty quickly with at least a flush packet;
		// otherwise, FW is toast, or breakdown in USB communications.
		if (packetLength <= 0) {
			for (iRetry = 0; ; iRetry++) {
                dpxDeviceTable[dpxSysDevsel].nEP1Reads++;
#ifdef USE_LIB01
				packetLength = usb_bulk_read(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x81, packet, 64, 1000);
#else
				libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x81, packet, 64, &packetLength, 1000);
#endif
				if (packetLength > 0)
					break;
				else if (iRetry < MAX_RETRIES) {
					DPxDebugPrint1("ERROR: EZReadEP1Tram() bulk read failed with [%d], retrying...\n", packetLength);
					dpxEp1RdRetries++;
				}
				else {
					DPxDebugPrint1("ERROR: EZReadEP1Tram() bulk read failed with [%d]\n", packetLength);
					dpxEp1RdFails++;
					return packetLength;
				}
			}

			packetRdIndex = 0;			// We start reading the new packet from index 0
		}

		// Each iteration copies 1 byte from the USB packet to the tram.
		while (packetLength) {
			ep1in_Tram[tramWrIndex++] = packet[packetRdIndex++];	// Copy the byte from the USB packet to the tram
			packetLength--;

			// If there's a framing error, flush byte and keep scanning for hat
			if (tramWrIndex == 1 && ep1in_Tram[0] != '^') {
				DPxDebugPrint1("ERROR: EZReadEP1Tram() framing error [%d]\n", (int)ep1in_Tram[0]);
				while (packetLength && packet[packetRdIndex] != '^') {	// Try to only print 1 error message per framing error
					packetLength--;
					packetRdIndex++;
				}
				tramWrIndex = 0;
				return -1;
			}

			if (tramWrIndex == 4)
				tramLen = ep1in_Tram[2] + (ep1in_Tram[3] << 8);

			// The tram ends as soon as we've received the payload
			if (tramWrIndex >= 4 && tramWrIndex == tramLen + 4) {
				tramWrIndex = 0;										// Next tram will start writing at start of buffer
				if (ep1in_Tram[1] == EP1IN_CONSOLE)						// Filter out and print console trams
					EZPrintConsoleTram(ep1in_Tram);
				else if (ep1in_Tram[1] == EP1OUT_FLUSH)					// Ignore flush trams
					(void)0;
				else if (expectedTram) {								// We're looking for a specific tram
					if (ep1in_Tram[1] != expectedTram) {
						DPxDebugPrint2("ERROR: EZReadEP1Tram() received tram code [%d] instead of [%d]\n", (int)ep1in_Tram[1], (int)expectedTram);
						return -1;
					}
					if (tramLen != expectedLen) {
						DPxDebugPrint2("ERROR: EZReadEP1Tram() received tram length [%d] instead of [%d]\n", tramLen, expectedLen);
						return -1;
					}
					return 0;
				}
				else {
					cached = 1;
					return ep1in_Tram[1];								// Next caller will get same tram
				}
			}
		}	// while (packetLength)

		// If we get here, we've used up the current packet, but have no data tram assembled yet.
		// Under some circumstances, usb_bulk_read can stick for the entire timeout time.
		// (I'm not sure where I saw this.  Might have been due to a bug of mine).
		// In any case, if we're not actually looking for a command to process (expectedTram == 0),
		// then get back to caller ASAP.
		if (expectedTram == 0)
			return 0;
	}
	return 0;	// Make compiler happy
}


// Write a tram to EP2OUT, and optionally wait for a response tram whose code is passed in rxTramCode.
// Returns 0 for success, or -1 error if:
//	-EP2OUT write failed.
//	-EP6IN read failed.
//	-expectedRxTram was non-0 (indicating an expected Rx message), but the Rx tram had the wrong code.
int EZWriteEP2Tram(unsigned char* txTram, unsigned char expectedRxTram, int expectedRxLen)
{
	int packetSize;
	int nTxBytes = 4 + txTram[2] + (txTram[3] << 8);									// Number of bytes to transmit
	int iRetry;

	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif

    // Caller is responsible for ensuring valid dpxSysDevsel

	// There seems to be a bug when requesting a memory read.
	// If the resulting returned message's length is a multiple of 512 bytes,
	// there's a mismatch in someone's handshaking.
	// I've also seen references to this on the web.
	// The EZ should follow the full packet with a 0-length packet indicating that the msg is over.
	// It does do this (at least it's programmed to do to), but OS X still mixes it up.
	// I will get over this by simply detecting requests which would result in a x512 result,
	// and bumping up the request length by 2 bytes.
	if (txTram[1] == EP2OUT_READRAM && txTram[8] == 0xfc && (txTram[9] & 1) && !DPxIsA10Arch()) {
		txTram[8] += 2;
		expectedRxLen += 2;
	}

	while (nTxBytes) {
	
		// Allowing total size gives write bandwidth approaching 30 MBps.
		// FYI, limiting maximum size to 512B reduces write bandwidth to about 4 MBps.
		// Limiting maximum size to 100B reduces write bandwidth to about 0.2 MBps.
		packetSize = nTxBytes;

		for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
			if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)txTram, packetSize, 1000) == packetSize)
#else
			if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)txTram, packetSize, &packetSizeWrite, 1000) == 0) &&
				(packetSize == packetSizeWrite))
#endif
				break;
			else if (iRetry < MAX_RETRIES) {
				DPxDebugPrint1("ERROR: EZWriteEP2Tram() bulk write retried: %s\n", usb_strerror());
				dpxEp2WrRetries++;
			}
			else {
				DPxDebugPrint1("ERROR: EZWriteEP2Tram() bulk write failed: %s\n", usb_strerror());
				dpxEp2WrFails++;
				return -1;
			}
		}
		txTram += packetSize;
		nTxBytes -= packetSize;
	}

	// Read from EP6IN if requested
	if (expectedRxTram && EZReadEP6Tram(expectedRxTram, expectedRxLen) < 0) {
		DPxDebugPrint0("ERROR: EZWriteEP2Tram() call to EZReadEP6Tram() failed\n");
		return -1;
	}
	return 0;
}

#if 1
// EZReadEP6Tram() reads a tram from the EZUSB EP6IN endpoint.
// If a data tram is received, and its tram code equals expectedTram, EZReadEP6Tram() returns 0.
// All other cases return an error code.
int EZReadEP6Tram(unsigned char expectedTram, int expectedLen)
{
	int	reqLength, tramLen, packetLength;
	int iRetry;
	int timeout;
	
    // Caller is responsible for ensuring valid dpxSysDevsel

	// Default USB read timeout will be 1 second.
	// Watch out though.  If this read is behind a pixel sync, the timeout could be much larger.
	// We'll estimate the maximum psync timeout consertively, assuming a 60 Hz refresh rate.
	timeout = 5000;
	if (dpxActivePSyncTimeout != -1)
		timeout = (int)(dpxActivePSyncTimeout / 60.0 * 1000);

	reqLength = expectedLen + 4;
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		packetLength = usb_bulk_read(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x86, (char*)ep6in_Tram, reqLength, timeout);
#else
		libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x86, (char*)ep6in_Tram, reqLength, &packetLength, timeout);
#endif
		if (packetLength == reqLength)
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint2("ERROR: EZReadEP6Tram() bulk read returned [%d] instead of [%d] bytes, retrying...\n", packetLength, reqLength);
			dpxEp6RdRetries++;
		}
		else {
			DPxDebugPrint2("ERROR: EZReadEP6Tram() bulk read returned [%d] instead of [%d] bytes, failed\n", packetLength, reqLength);
			dpxEp6RdFails++;
			return -1;
		}
	}

	if (ep6in_Tram[0] != '^') {
		DPxDebugPrint1("ERROR: EZReadEP6Tram() framing error [%d]\n", (int)ep6in_Tram[0]);
		return -1;
	}
	if (ep6in_Tram[1] != expectedTram) {
		DPxDebugPrint2("ERROR: EZReadEP6Tram() received tram code [%d] instead of [%d]\n", (int)ep6in_Tram[1], (int)expectedTram);
		return -1;
	}
	tramLen = ep6in_Tram[2] + (ep6in_Tram[3] << 8);
	if (tramLen != expectedLen) {
		DPxDebugPrint2("ERROR: EZReadEP6Tram() received tram length [%d] instead of [%d]\n", tramLen, expectedLen);
		return -1;
	}
	
	return 0;
}

#else

// EZReadEP6Tram() reads a tram from the EZUSB EP6IN endpoint.
// If a data tram is received, and its tram code equals expectedTram, EZReadEP6Tram() returns 0.
// All other cases return an error code.
// Keep this strategy of buffering returned trams, just in case the DP is returning multiple trams for some reason.
// I could go back to using this routine if I wanted to.  It has a more of an async interface to trams.
int EZReadEP6Tram(unsigned char expectedTram, int expectedLen)
{
	static char packet[65536];				// Largest possible EP6 USB packet
	static int	packetLength	= 0;
	static int	packetRdIndex	= 0;
	static int	tramWrIndex		= 0;
	static int	tramLen			= 0;		// The length of the payload

    // Caller is responsible for ensuring valid dpxSysDevsel

	// Each iteration either reads a new 512 byte USB packet, or starts a new tram
	while (1) {

		// If we're out of data, or we had an error, read another packet.
		if (packetLength <= 0) {
			packetLength = usb_bulk_read(dpxHdl, 0x86, packet, expectedLen+4, 1000);
			if (packetLength <= 0) {
				DPxDebugPrint1("ERROR: EZReadEP6Tram() bulk read returned [%d]\n", packetLength);
				return packetLength;
			}
			packetRdIndex = 0;			// We start reading the new packet from index 0
		}

		// Each iteration copies 1 byte from the USB packet to the tram.
		while (packetLength) {
			ep6in_Tram[tramWrIndex++] = packet[packetRdIndex++];	// Copy the byte from the USB packet to the tram
			packetLength--;

			// If there's a framing error, flush byte and keep scanning for hat
			if (tramWrIndex == 1 && ep6in_Tram[0] != '^') {
				DPxDebugPrint1("ERROR: EZReadEP6Tram() framing error [%d]\n", (int)ep6in_Tram[0]);
				while (packetLength && packet[packetRdIndex] != '^') {	// Try to only print 1 error message per framing error
					packetLength--;
					packetRdIndex++;
				}
				tramWrIndex = 0;
				return -1;
			}

			if (tramWrIndex == 4)
				tramLen = ep6in_Tram[2] + (ep6in_Tram[3] << 8);

			// The tram ends as soon as we've received the payload
			if (tramWrIndex >= 4 && tramWrIndex == tramLen + 4) {
				tramWrIndex = 0;										// Next tram will start writing at start of buffer
				if (ep6in_Tram[1] == EP1IN_CONSOLE)						// Filter out and print console trams
					EZPrintConsoleTram(ep6in_Tram);
				else if (ep6in_Tram[1] == EP1OUT_FLUSH)					// Ignore flush trams
					(void)0;
				else if (expectedTram) {								// We're looking for a specific tram
					if (ep6in_Tram[1] != expectedTram) {
						DPxDebugPrint1("ERROR: EZReadEP6Tram() received tram code [%d] instead of [%d]\n", (int)ep6in_Tram[1], (int)expectedTram);
						return -1;
					}
					if (tramLen != expectedLen) {
						DPxDebugPrint1("ERROR: EZReadEP6Tram() received tram length [%d] instead of [%d]\n", tramLen, expectedLen);
						return -1;
					}
					return 0;
				}
			}
		}	// while (packetLength)

		// If we get here, we've used up the current packet, but have no data tram assembled yet.
		// Under some circumstances, usb_bulk_read can stick for the entire timeout time.
		// (I'm not sure where I saw this.  Might have been due to a bug of mine).
		// In any case, if we're not actually looking for a command to process (expectedTram == 0),
		// then get back to caller ASAP.
		if (expectedTram == 0)
			return 0;
	}
	return 0;	// Make compiler happy
}
#endif


void EZPrintConsoleTram(unsigned char* tram)
{
	static int newLine = 1;

	int i, theChar, nChars;

    // Caller is responsible for ensuring valid dpxSysDevsel
	nChars = tram[2] + (tram[3] << 8);
	for (i = 0; i < nChars; i++) {
		if (newLine) {
			printf(" EZ_CONSOLE> ");
			newLine = 0;
		}
		theChar = tram[i+4];
		switch (theChar) {
			case EP1IN_ERR_HAT:
				printf("Tram: Framing error\n");
				newLine = 1;
				break;
			case EP1IN_ERR_NOP:
				printf("Tram: Null command code\n");
				newLine = 1;
				break;
			case EP1IN_ERR_LEN:
				printf("Tram: Illegal payload length\n");
				newLine = 1;
				break;
			case EP1IN_ERR_CMD:
				printf("Tram: Unrecognized command code\n");
				newLine = 1;
				break;
			default:
				putchar(theChar);
				newLine = (theChar == 10);
		}
	}
	fflush(stdout);
}


// Returns non-zero if we can access the SPI flash through the VIEWPixx/PROPixx high-speed FPGA interface,
// or returns 0 if we must use the slower software-based EZ-USB interface.
// Currently, the FPGA interface only works for an open, configured VIEWPixx/PROPixx/PROPixxCtrl.
// DATAPixx only supports the EZ-USB access to SPI.
// A rev1 PCB VIEWPixx doesn't support _any_ access to SPI if FPGA is unconfigured!  Need to use byte-blaster to configure FPGAs for those first PCBs.
int DPxSpiHasVpxFpgaCtrl()
{
	int isReady, savedSysDevsel;

	// Here we save and restore because DPxIsReady function can modify SysDevsel value.
	savedSysDevsel= dpxSysDevsel;
	isReady = DPxIsReady();
	DPxSelectSysDevice(savedSysDevsel);
	
    // Caller is responsible for ensuring valid dpxSysDevsel
    return DPxIsOpen() && (DPxIsS3Arch() || DPxIsA5Arch() || DPxIsA10Arch()) && isReady && gSpifEnable;

}


#define FAIL_IF_SPI_CONFIG_NEG(x) do { if (x < 0) { fprintf(stderr, "ERROR: DPxSpiStart() error = %d\n", x); goto fail; }} while (0)
#define FAIL_IF_SPI_CONFIG_ERR do { if (DPxGetError()) { fprintf(stderr, "ERROR: DPxSpiConfig() error = %d\n", DPxGetError()); goto fail; }} while (0)

int DPxSpiConfig(int writeMode)
{
	int rc;

    // Caller is responsible for ensuring valid dpxSysDevsel	

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// TRACKPIXX ONLY for now.
	if (DPxIsA5Arch()) {

		// Micron N25Q128A
		// https://www.micron.com/~/media/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_128mb_3v_65nm.pdf

		// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on TPX Bridge
		rc = EZReadSFR(EZ_SFR_IOD); FAIL_IF_SPI_CONFIG_NEG(rc);
		rc = EZWriteSFR(EZ_SFR_IOD, ((unsigned char)rc | 0x14) & ~0x08); FAIL_IF_SPI_CONFIG_NEG(rc);

		// Drive output onto SPI_CSn, SPI_SCK, SPI_DI, and FPGA_SPI_CTRL on TPX Bridge
		rc = EZReadSFR(EZ_SFR_OED); FAIL_IF_SPI_CONFIG_NEG(rc);
		rc = EZWriteSFR(EZ_SFR_OED, (unsigned char)rc | 0x1D ); FAIL_IF_SPI_CONFIG_NEG(rc);

		//////////////////////////////////////////////////////////////////////////////////////////////////
		// SPI ID Check
		if (EZWriteEP1Tram((unsigned char*)"^S\x04\x00\x9F\x00\x00\x00", EP1IN_SPI, 4))
			{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

		printf("\nSPI ID                   => 0x%hhX%hhX%hhX", ep1in_Tram[7], ep1in_Tram[6], ep1in_Tram[5]);

		// Micron N25Q128A
		if (memcmp(ep1in_Tram+5, "\x20\xBA\x18", 3))
		{
			fprintf(stderr, "ERROR: DPxSpiConfig() does not recognize SPI ID device:");
			goto fail;
		}				
		// Byte           |#1 & #2|    #3     |    #4     |#5 |#6 #7 #8
		// EZWriteEP1Tram |  ^S   |PAYLOAD LSB|PAYLOAD MSB|CMD|


		//////////////////////////////////////////////////////////////////////////////////////////////////
		// Read volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x85\x00", EP1IN_SPI, 2);
		printf("\nVolatile Config Reg      => 0x%hhX", ep1in_Tram[5]);

		// Read non-volatile configuration register (NVCR)
		rc = EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB5\x00\x00", EP1IN_SPI, 3); 
		printf("\nNon-Volatile Config Reg  => 0x%hhX%hhX", ep1in_Tram[6], ep1in_Tram[5]);

		// Read enhanced volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x65\x00", EP1IN_SPI, 2); 
		printf("\nEnh. Volatile Config Reg => 0x%hhX", ep1in_Tram[5]);	

		// WRITE MODE!
		if (writeMode == 1) { 
			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Unlock the SPI flash.  We never lock it, but maybe some other software...
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))                     // WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }
	
			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x01\x00", EP1IN_SPI, 2))                 // WRSR clear status reg to disable write protection
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

			printf("\nWRSR clear status reg to disable write protection.");

			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Write 0xCFEF to non-volatile configuration register (NVCR)
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #1\n"); goto fail; }

			if (EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB1\xEF\xCF", EP1IN_SPI, 3)) // try #1
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #2\n"); goto fail; }

			printf("\nWrite 0xCFEF to non-volatile configuration register (NVCR)");
		}

		printf("\n");
		printf("\n");
		
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////
	// A10	
	// Micron N25Q256A
	else if (DPxIsA10Arch()) {
		
		// This board use a EZ-FX3. Command format are different

		// Byte           | #0-1|     #2-3      |         #4           |  #5  |#6 #7 #8
		// EZWriteEP1Tram |  ^P |PAYLOAD LSB-MSB| B7=WR + CMD byte CNT | CMD


		// Set FX3 iobufs to their working states
		rc = EZReadSFR(FX3_REG_TRIS); FAIL_IF_SPI_CONFIG_NEG(rc);
		rc = EZWriteSFR(FX3_REG_TRIS, ((unsigned char)rc | 0x07)); FAIL_IF_SPI_CONFIG_NEG(rc);

		// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on DP3
		rc = EZReadSFR(FX3_REG_IO); FAIL_IF_SPI_CONFIG_NEG(rc);
		rc = EZWriteSFR(FX3_REG_IO, ((unsigned char)rc | 0x05) & ~0x02); FAIL_IF_SPI_CONFIG_NEG(rc);
		

		// First of all, enter Quad mode
		rc = EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", 's', 0); // WR Enable command
		// Write enhanced volatile configuration register command
		// 0x4F > BIT#7 = 0 >> Quad mode Enable!
		//      > BIT#4 = 0 >> Reset/Hold disabled
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x61\x4F", EP1IN_SPI, 0);
		printf("\n >>> FX3 put SPI Flash to Quad mode");

		ep1in_Tram[6] = 0;
		ep1in_Tram[5] = 0;
		ep1in_Tram[4] = 0;

		//////////////////////////////////////////////////////////////////////////////////////////////////
		// SPI ID Check (x4)
		if (EZWriteEP1Tram((unsigned char*)"^P\x05\x00\x01\xAF\x00\x00\x00", 'p', 3))
			{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n");  }

		printf("\nSPI ID                   => 0x%hhX%hhX%hhX", ep1in_Tram[6], ep1in_Tram[5], ep1in_Tram[4]);

		// Micron N25Q256A
		if (memcmp(ep1in_Tram+4, "\x20\xBB\x19", 3))
		{
			fprintf(stderr, "ERROR: DPxSpiConfig() does not recognize SPI ID device:");
			goto fail;
		}
		else
			printf(" >>> Micron N25Q256A");
		
		printf("\n\n");

		// rc = EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x05\x00", 'p', 1))             // RDSR Read Status Register

		// Read volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x85\x00", 'p', 1);
		printf("\nVolatile Config Reg (x85)      => 0x%hhX", ep1in_Tram[4]);

		// Read non-volatile configuration register (NVCR)
		rc = EZWriteEP1Tram((unsigned char*)"^P\x04\x00\x01\xB5\x00\x00", 'p', 2);
		printf("\nNon-Volatile Config Reg  (xB5) => 0x%hhX%hhX", ep1in_Tram[5], ep1in_Tram[4]);

		// Read enhanced volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x65\x00", 'p', 1);
		printf("\nEnh. Volatile Config Reg (x65) => 0x%hhX", ep1in_Tram[4]);
		printf("\n\n");


		// Write!
		if (writeMode == 1) { 
			// WR Enable command
			rc = EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0);

			// Write 0xAFEE to non-volatile configuration register (NVCR)
			rc = EZWriteEP1Tram((unsigned char*)"^P\x04\x00\x83\xB1\xEE\xAF", 'p', 0);
		}

		// First of all, go back to x1 mode
		rc = EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0);
		// Write enhanced volatile configuration register command
		// 0x4F > BIT#7 = 1 >> Quad mode Disable!
		//      > BIT#4 = 0 >> Reset/Hold disabled
		
		rc = EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x82\x61\xDF", 'p', 0);
		printf("\n >>> FX3 put SPI Flash to x1 mode\n");
	}
	
	// Stratix III using a MT25QL128 SPI	
	// Nouveau chip 17 Mai 2019 -> Micron N25Q064A13ESFH0F [3V]
	else {

		// Micron MT25QL128ABA8ESF-0SIT
		// https://www.micron.com/~/media/documents/products/data-sheet/nor-flash/serial-nor/mt25q/die-rev-a/mt25q_qlhs_l_128_aba_0.pdf

		// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on VIEWPixx
    	rc = EZReadSFR(EZ_SFR_IOC); FAIL_IF_SPI_CONFIG_NEG(rc);
    	rc = EZWriteSFR(EZ_SFR_IOC, ((unsigned char)rc | 0x04 | 0x20) & ~0x08); FAIL_IF_SPI_CONFIG_NEG(rc);

    	// Drive output onto SPI_CSn, SPI_SCK, SPI_DI, and FPGA_SPI_CTRL on VIEWPixx
    	rc = EZReadSFR(EZ_SFR_OEC); FAIL_IF_SPI_CONFIG_NEG(rc);
    	rc = EZWriteSFR(EZ_SFR_OEC, (unsigned char)rc | 0x0D | 0x20); FAIL_IF_SPI_CONFIG_NEG(rc);

		//////////////////////////////////////////////////////////////////////////////////////////////////
		// SPI ID Check
		if (EZWriteEP1Tram((unsigned char*)"^S\x06\x00\x9F\x00\x00\x00\x00\x00", EP1IN_SPI, 6))
			{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

		printf("\nManufacturer ID          => 0x%hhX", ep1in_Tram[5]);
		
		if (memcmp(ep1in_Tram+5, "\x20", 1) == 0)
		{
			printf("   [Micron]");
		}

		printf("\nDevice ID                => 0x%hhX%hhX", ep1in_Tram[7], ep1in_Tram[6]);
		
		if (memcmp(ep1in_Tram+7, "\x17", 1) == 0)
		{
			printf(" [64Mb]");
		}
		else if (memcmp(ep1in_Tram+7, "\x18", 1) == 0)
		{
			printf(" [128Mb]");
		}

		printf("\nExtended device ID       => 0x%hhX", ep1in_Tram[9]);
		printf("\n\n");

		// Micron N25Q128A
		//if (memcmp(ep1in_Tram+5, "\x20\xBA\x17", 3))
		if (memcmp(ep1in_Tram+5, "\x20\xBA\x18", 3))
		{
			//fprintf(stderr, "\nERROR: DPxSpiConfig() does not need to be runned on this SPI device.\n");
			fprintf(stderr, "\nERROR: DPxSpiConfig() does not recognize SPI ID device.\n");
			goto fail;
		}				
		// Byte           |#1 & #2|    #3     |    #4     |#5 |#6 #7 #8
		// EZWriteEP1Tram |  ^S   |PAYLOAD LSB|PAYLOAD MSB|CMD|


		//////////////////////////////////////////////////////////////////////////////////////////////////
		// Read volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x85\x00", EP1IN_SPI, 2); 
		printf("\nVolatile Config Reg      => 0x%hhX", ep1in_Tram[5]);

		// Read non-volatile configuration register (NVCR)
		rc = EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB5\x00\x00", EP1IN_SPI, 3); 
		printf("\nNon-Volatile Config Reg  => 0x%hhX%hhX", ep1in_Tram[6], ep1in_Tram[5]);

		// Read enhanced volatile configuration register
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x65\x00", EP1IN_SPI, 2); 
		printf("\nEnh. Volatile Config Reg => 0x%hhX", ep1in_Tram[5]);	
		
#if 0
		// Read Flag Status register
		rc = EZWriteEP1Tram((unsiscwned char*)"^S\x02\x00\x70\x00", EP1IN_SPI, 2); 
		printf("\nFlag Status Reg          => 0x%hhX", ep1in_Tram[5]);

		// Read Status register
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x05\x00", EP1IN_SPI, 2); 
		printf("\nStatus Reg               => 0x%hhX", ep1in_Tram[5]);

		// Protection Management Register Command 
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x2B\x00", EP1IN_SPI, 2); 
		printf("\nProtect. Management Reg  => 0x%hhX", ep1in_Tram[5]);
#endif

		// WRITE MODE!
		if (writeMode == 1) { 
#if 0
			printf("\n");
			printf("\n");		

			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Write 0x8B to volatile configuration register (VCR)

			// WRITE ENABLE command needed before writing status reg
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }
	
			// WRITE STATUS REGISTER command clear status reg to disable write protection (has no effect on bit #1 and #0)
			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x01\x00", EP1IN_SPI, 2))
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

			printf("\nWRITE STATUS REGISTER clear status reg to disable write protection.");			
			
			//  WRITE ENABLE commandneeded before writing status reg
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #1\n"); goto fail; }

			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x81\x8B", EP1IN_SPI, 2))		// 0x8B >>> Dummy cycle = 8
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #2\n"); goto fail; }

			printf("\nWrite 0x8B (Dummy cycle = 8) to volatile configuration register (VCR)");

			
			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Write 0xCF to enhanced volatile configuration register (EVCR)			
			
			// WRITE ENABLE command needed before writing status reg
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #3\n"); goto fail; }

			// WRSR clear status reg to disable write protection
			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x01\x00", EP1IN_SPI, 2))     
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

			printf("\nWRSR clear status reg to disable write protection.");

			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #3\n"); goto fail; }

			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x61\xCF", EP1IN_SPI, 2))		// Reset/Hold disabled
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #4\n"); goto fail; }

			printf("\nWrite 0xCF (Reset/Hold disabled) to enhanced volatile configuration register (EVCR)");
#endif
		
			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Write 0x8FEF to non-volatile configuration register (NVCR)

			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #5\n"); goto fail; }

			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x01\x00", EP1IN_SPI, 2))     // WRSR clear status reg to disable write protection
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed\n"); goto fail; }

			printf("\nWRSR clear status reg to disable write protection.");

			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #5\n"); goto fail; }

			// The WRITE NONVOLATILE CONFIGURATION REGISTER operation must have input data starting from the least significant byte.
			// The datasheet sucks, documentation does not mention that you have to write "11" into Bit #1 and #0. If you do this, the NVCR write will not work... (You will see a "Failure protection error" in "Flag Status Register")
			if (EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB1\xEF\x8F", EP1IN_SPI, 3)) // 0x8FEF >>> Dummy cycle = 8 + Reset/Hold disabled + DTR disabled 
				{ fprintf(stderr,"ERROR: DPxSpiConfig() call to EZWriteEP1Tram() failed #6\n"); goto fail; }
			printf("\nWrite 0x8FEF to non-volatile configuration register (NVCR)");

#if 0
			// Read Flag Status register
			rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x70\x00", EP1IN_SPI, 2); 
			if (ep1in_Tram[5]&0x02 == 0x02)
				printf("\nFlag Status Reg          => 0x%hhX >>> Failure protection error", ep1in_Tram[5]);

			printf("\n");

			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Read volatile configuration register
			rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x85\x00", EP1IN_SPI, 2); 
			printf("\nVolatile Config Reg      => 0x%hhX", ep1in_Tram[5]);

			// Read non-volatile configuration register (NVCR)
			rc = EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB5\x00\x00", EP1IN_SPI, 3); 
			printf("\nNon-Volatile Config Reg  => 0x%hhX%hhX", ep1in_Tram[6], ep1in_Tram[5]);

			// Read enhanced volatile configuration register
			rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x65\x00", EP1IN_SPI, 2); 
			printf("\nEnh. Volatile Config Reg => 0x%hhX", ep1in_Tram[5]);
			
			// Read General purpose register
			rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x96\x00", EP1IN_SPI, 2); 
			printf("\nGeneral purpose Reg      => 0x%hhX", ep1in_Tram[5]);
#endif
		}
		printf("\n");
	}

    return 0;

fail:
	DPxSetError(DPX_ERR_SPI_FIRST_CONFIG);
    return DPX_ERR_SPI_FIRST_CONFIG;
}

#define FAIL_IF_SPI_START_NEG(x) do { if (x < 0) { fprintf(stderr, "ERROR: DPxSpiStart() error = %d\n", x); goto fail; }} while (0)

// Prepare SPI interface for I/O.
// return 0 for success.
int DPxSpiStart()
{
	int rc, i, qsBitMask;

    // Caller is responsible for ensuring valid dpxSysDevsel

    // VIEWPixx uses Stratix 3, which never seems to tristate its damned SPI interface!
    // We had to put a Quickswitch on the PCB between the SPI and FPGA, in order for the EZ-USB to have access to SPI.usb_fwd 
    // If we want to use the FPGA interface to the SPI, make sure that this QS is enabled, and EZ SPI access is tristate.
    // Nope.  Programming the SPI using ASMI is very unreliable.
    // We'll use the FPGA's home-grown SPI interface.
	if (DPxSpiHasVpxFpgaCtrl()) {

        // Tell FPGA to not use ASMI
		DPxSetReg16(DPXREG_CTRL, DPxGetReg16(DPXREG_CTRL) | 0x8000);
		DPxUpdateRegCache();
		
		// TRACKPixx Bridge use a smaller package EZ-USB chip. It uses the Port D as User IO instead of Port C
		if (DPxIsA5Arch()) {
			// Drive output onto FPGA_SPI_CTRL signal, but tristate rest of EZ access to SPI device
			rc = EZWriteSFR(EZ_SFR_OED, (unsigned char)0x10); FAIL_IF_SPI_START_NEG(rc);

			// Open QuickSwitch to block FPGA ASMI from accessing SPI
			rc = EZReadSFR(EZ_SFR_IOD); FAIL_IF_SPI_START_NEG(rc);
			rc = EZWriteSFR(EZ_SFR_IOD, (unsigned char)rc | 0x10); FAIL_IF_SPI_START_NEG(rc);
		}
		if (DPxIsA10Arch()) {
			// Drive output onto FPGA_SPI_CTRL signal, but tristate rest of EZ access to SPI device
			rc = EZWriteSFR(FX3_REG_TRIS, (unsigned char)0x04); FAIL_IF_SPI_START_NEG(rc);

			// Open QuickSwitch to block FPGA ASMI from accessing SPI
			rc = EZReadSFR(FX3_REG_IO); FAIL_IF_SPI_START_NEG(rc);
			rc = EZWriteSFR(FX3_REG_IO, (unsigned char)rc | 0x04); FAIL_IF_SPI_START_NEG(rc);
		}
		else {
			// Drive output onto FPGA_SPI_CTRL signal, but tristate rest of EZ access to SPI device
			rc = EZWriteSFR(EZ_SFR_OEC, (unsigned char)0x20); FAIL_IF_SPI_START_NEG(rc);

			// Open QuickSwitch to block FPGA ASMI from accessing SPI
			rc = EZReadSFR(EZ_SFR_IOC); FAIL_IF_SPI_START_NEG(rc);
			rc = EZWriteSFR(EZ_SFR_IOC, (unsigned char)rc | 0x20); FAIL_IF_SPI_START_NEG(rc);
		}
        return 0;
    }

    // We will be using the EZ-USB to access the SPI device.
    // For VIEWPixx, we'll drive FPGA_SPI_CTRL high to stop Stratix 3 from hogging the SPI bus.
    // For DATAPixx, once I had this great idea of forcing FPGA_PGMn low before accessing the SPI over USB.
    // This would deconfigure the FPGA, but that's the only way we can be absolutely sure it's not driving SPI bus.
    // Otherwise, a buggy FPGA load could inhibit us from reprogramming the SPI with a correct load.
    // That's the plan anyways.  In practice, doing this was causing one production DP to give "ERROR: does not recognize SPI device" from below.
    // Seems that bringing down PGMn was clobbering EZ-USB access to SPI.  No idea why, but I'll leave the FPGA configured for now.
    // This isn't too bad.  The FPGA only accesses the SPI for a could of ms at startup to read calibration data.
    // By forcing reconfig off, the FPGA won't reconfig during the SPI reprogramming, so the user won't loose their display.
    // Only on the next powerup will the real reconfiguration occur.  Might be a good thing.
    // Leave it up to a higher-level API to decide if/when FPGA PGMn should be strobed.
    // Kinda makes sense.  The configuration logic might be wanting to control the SPI bus about this time,
    // although the datasheet seems to suggest it is floating the SPI bus.
    // Probably best to not try to drive the SPI bus when I drop PGMn if I can help it.
    // Best to strobe PGMn _after_ I've programmed the SPI.
    qsBitMask = DPxIsS3Arch() ? 0x20 : 0; // Note: qsBitMask is not used in Arria V project

	if (DPxIsA5Arch()) {

		// Micron N25Q128A
		// https://www.micron.com/~/media/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_128mb_1_8v_65nm.pdf

		// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on TPX Bridge
		rc = EZReadSFR(EZ_SFR_IOD); FAIL_IF_SPI_START_NEG(rc);
		rc = EZWriteSFR(EZ_SFR_IOD, ((unsigned char)rc | 0x14) & ~0x08); FAIL_IF_SPI_START_NEG(rc);

		// Drive output onto SPI_CSn, SPI_SCK, SPI_DI, and FPGA_SPI_CTRL on TPX Bridge
		rc = EZReadSFR(EZ_SFR_OED); FAIL_IF_SPI_START_NEG(rc);
		rc = EZWriteSFR(EZ_SFR_OED, (unsigned char)rc | 0x1D ); FAIL_IF_SPI_START_NEG(rc);
	}
	else if (DPxIsA10Arch()){
		// Set FX3 iobufs to their working states
		rc = EZReadSFR(FX3_REG_TRIS); FAIL_IF_SPI_START_NEG(rc);
		rc = EZWriteSFR(FX3_REG_TRIS, ((unsigned char)rc | 0x07)); FAIL_IF_SPI_START_NEG(rc);

		// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on DP3
		rc = EZReadSFR(FX3_REG_IO); FAIL_IF_SPI_START_NEG(rc);
		rc = EZWriteSFR(FX3_REG_IO, ((unsigned char)rc | 0x05) & ~0x02); FAIL_IF_SPI_START_NEG(rc);

		// Put SPI in quad mode
		rc = EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1); // WR Enable command
		//EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x65\x00", EP1IN_SPI, 2);
		// Write enhanced volatile configuration register command
		// 0x5F > BIT#7 = 0 >> Quad mode enabled
		rc = EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x61\x5F", EP1IN_SPI, 2); 
	}
	else {

    	// Make sure SPI_CSn is high, and SPI_SCK is low, and FPGA_SPI_CTRL is high on VIEWPixx
    	rc = EZReadSFR(EZ_SFR_IOC); FAIL_IF_SPI_START_NEG(rc);
    	rc = EZWriteSFR(EZ_SFR_IOC, ((unsigned char)rc | 0x04 | qsBitMask) & ~0x08); FAIL_IF_SPI_START_NEG(rc);

    	// Drive output onto SPI_CSn, SPI_SCK, SPI_DI, and FPGA_SPI_CTRL on VIEWPixx
    	rc = EZReadSFR(EZ_SFR_OEC); FAIL_IF_SPI_START_NEG(rc);
    	rc = EZWriteSFR(EZ_SFR_OEC, (unsigned char)rc | 0x0D | qsBitMask); FAIL_IF_SPI_START_NEG(rc);
	}

    // Execute SPI RDID command to ensure we recognize device.
    // Also a sanity check that device is powered up, SPI interface works, etc.
	//  For an A10 architecture like DP3, we assume that the SPI interface is x4.
	//   At this point, the bus is no longer full duplex. The FX3 FW needs to know after how many bytes
	//   the bus needs to be turned around and read. In this case, this is indicated after the 4B header.
	//   Here we have a write byte content of only 1. For a read memory transaction, we will have 5B or more.
	if (DPxIsA10Arch())
	{
		if (EZWriteEP1Tram((unsigned char*)"^P\x05\x00\x01\xAF\x00\x00\x00", 'p', 3))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
	}
	else
	{
		if (EZWriteEP1Tram((unsigned char*)"^S\x04\x00\x9F\x00\x00\x00", EP1IN_SPI, 4))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
	}
    if (memcmp(ep1in_Tram+5, "\x20\x20\x16", 3) &&								// STMicroelectronics M25P32
        memcmp(ep1in_Tram+5, "\x20\x20\x17", 3) &&								// STMicroelectronics M25P64
		memcmp(ep1in_Tram+5, "\x20\xBA\x17", 3) && 								// VIEWPixx/PROPixx since May 2017 test
		memcmp(ep1in_Tram+5, "\x20\xBA\x18", 3) && 								// Micron N25Q128A used in Trackpixx Bridge and VIEWPixx/PROPixx since May 2017
		memcmp(ep1in_Tram+4, "\x20\xBB\x19", 3)  								// Micron N25Q256A used in Datapixx3
		)
	{
        fprintf(stderr, "ERROR: DPxSpiStart() does not recognize SPI device:");
        for (i = 5; i < 8; i++)
            fprintf(stderr, " %02X", ep1in_Tram[i]);										// So we know just what SPI returned
        putchar('\n');
        goto fail;
    }

	if (DPxIsA10Arch()) {
		// We use the program quad instruction which does not return any data
		if (EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0))			// WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #1\n"); goto fail; }
	
		if (EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x82\x81\xAB", 'p', 0))		// Write Volatile configuration register >>> 10 Dummy bits
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #2\n"); goto fail; }

		// Unlock the SPI flash.  We never lock it, but maybe some other software...
		if (EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0))			// WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }    
	
		if (EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x82\x01\x00", 'p', 0))		// WRSR clear status reg to disable write protection
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
				
		// WREN for next SPI transaction
		if (EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0))			// WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }

		// For SPI flash greater than 128Mbit, address size is 4 Bytes. Thus we need to enable this addressing mode
		if (EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\xB7", 'p', 0))			// WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
	}
	else {

		// Unlock the SPI flash.  We never lock it, but maybe some other software...
		if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))         // WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
	
		if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x01\x00", EP1IN_SPI, 2))     // WRSR clear status reg to disable write protection
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }

		if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #1\n"); goto fail; }
	
		if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x81\x8B", EP1IN_SPI, 2))		// Write Volatile configuration register >>> 8 Dummy bit!
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #2\n"); goto fail; }

		if (EZWriteEP1Tram((unsigned char*)"^S\x04\x00\x9F\x00\x00\x00", EP1IN_SPI, 4)) // SPI ID Check
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
		
		// VIEWPixx/PROPixx since May 2017
		// Micron MT25QL128ABA8ESF  > New Gen Micron (128Mb).
		if (!memcmp(ep1in_Tram+5, "\x20\xBA\x18", 3) && DPxIsS3Arch())
		{			
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))			// WREN command needed before writing status reg
				{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
	
			// Write non-volatile configuration register (NVCR) operation must have input data starting from the least significant byte.
			if (EZWriteEP1Tram((unsigned char*)"^S\x03\x00\xB1\xEF\x8F", EP1IN_SPI, 3)) // 0x8FEF >>> Dummy cycle = 8 + Reset/Hold disabled + DTR disabled 
				{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
		}

	}
    do {                                                                                // Have to wait until WRSR has completed
		if (DPxIsA10Arch()){
			if (EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x05\x00", 'p', 1))       // RDSR Read Status Register
				{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
			ep1in_Tram[5] = ep1in_Tram[4];
		}
		else {
			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x05\x00", EP1IN_SPI, 2))             // RDSR Read Status Register
				{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed\n"); goto fail; }
		}
    } while (ep1in_Tram[5] & 1);		// Stay here while the Write In Progress bit is set.

    return 0;

fail:
	DPxSetError(DPX_ERR_SPI_START);
    return DPX_ERR_SPI_START;
}


int DPxSpiStop()
{
	int rc;

    // Caller is responsible for ensuring valid dpxSysDevsel

	if (DPxIsA5Arch()) {

		// Stop EZ from driving SPI_CSn, SPI_SCK, SPI_DI nets. 
		// The FPGA might be driving this interface soon!
		rc = EZReadSFR(EZ_SFR_OED);
		if (rc < 0)
		    { fprintf(stderr, "ERROR: DPxSpiStop() error 1\n"); goto fail; }
		if (EZWriteSFR(EZ_SFR_OED, (unsigned char)rc & ~0x0D) < 0)
		{ fprintf(stderr, "ERROR: DPxSpiStop() error 2\n"); goto fail; }
	}
	else if (DPxIsA10Arch()){
		
		// Dont need to change SPI setting back to normal, need to confirm
		//if (DPxSpiHasVpxFpgaCtrl())
		//	return 0;

		// Stop EZ from driving SPI_CSn, SPI_SCK, SPI_DI nets. 
		// The FPGA might be driving this interface soon!

		// WR enable command
		if (EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #1\n");}
	
		// Put SPI in single mode by disabling quad IO
		if (EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x82\x61\xDF", 'p', 0))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #2\n");}

		if (EZWriteSFR(FX3_REG_TRIS, 0x00) < 0)
		    { fprintf(stderr, "ERROR: DPxSpiStop() error 2\n"); goto fail; }
	}
	else {
		// Stop EZ from driving SPI_CSn, SPI_SCK, SPI_DI nets.
		// The FPGA might be driving this interface soon!
		rc = EZReadSFR(EZ_SFR_OEC);
		if (rc < 0)
		    { fprintf(stderr, "ERROR: DPxSpiStop() error 1\n"); goto fail; }
		if (EZWriteSFR(EZ_SFR_OEC, (unsigned char)rc & ~0x0D) < 0)
		    { fprintf(stderr, "ERROR: DPxSpiStop() error 2\n"); goto fail; }
	}
    // On VIEWPixx FPGA, we'll return to ASMI access.  ASMI reads seem to be reliable.
    // NOPE.  Now we are no longer using ASMI at all.  Leave the quickswitch disabled.
#if 0
    if (DPxSpiHasVpxFpgaCtrl()) {
        DPxSetReg16(DPXREG_CTRL, DPxGetReg16(DPXREG_CTRL) & ~0x8000);
        DPxUpdateRegCache();
        rc = EZReadSFR(EZ_SFR_IOC);
        if (rc < 0)
            { fprintf(stderr, "ERROR: DPxSpiStop() error 3\n"); goto fail; }
        rc = EZWriteSFR(EZ_SFR_IOC, (unsigned char)rc & ~0x20);
    }
#endif

    return 0;

fail:
	DPxSetError(DPX_ERR_SPI_STOP);
    return DPX_ERR_SPI_STOP;
}


// Read a SPI block of any size.
// Assumes that spiAddr starts on a 256-byte boundary.
int DPxSpiRead(int spiAddr, int nReadBytes, char* readBuffer, PercentCompletionCallback percentCompletionCallback)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
    if (DPxSpiStart() == DPX_SUCCESS) {
        if (DPxSpiReadNoStartStop(spiAddr,  nReadBytes, readBuffer, percentCompletionCallback) == DPX_SUCCESS) {
            DPxSpiStop();
            return 0;
        }
    }
    
    // Failure
    DPxSpiStop();
    return DPX_ERR_SPI_READ;
}


#define FAIL_IF_SPI_READ_ERR do { if (DPxGetError()) { fprintf(stderr, "ERROR: DPxSpiRead() error = %d\n", DPxGetError()); goto fail; }} while (0)

// We have a NoStartStop version so that users of other SPI commands can do a read w/o indirectly calling DPxSpiStop().
int DPxSpiReadNoStartStop(int spiAddr, int nReadBytes, char* readBuffer, PercentCompletionCallback percentCompletionCallback)
{
    unsigned int nBytesRead, nBytesRemaining, nSpiBytes, nTramPayloadBytes, oldPercentDone, newPercentDone;

    // Caller is responsible for ensuring valid dpxSysDevsel
    nBytesRead = 0;
    oldPercentDone = 0;
    if (percentCompletionCallback)
        percentCompletionCallback(0);

    while (nBytesRead < (unsigned)nReadBytes) {
        nBytesRemaining = nReadBytes - nBytesRead;
        
		if (DPxIsA10Arch()){
			nSpiBytes = nBytesRemaining < 768 ? nBytesRemaining : 768;
			if (DPxSpiHasVpxFpgaCtrl()) {
				ep2out_Tram[0] = '^';		// 0x5E
				ep2out_Tram[1] = 'S';		// 0x53
				ep2out_Tram[2] = 12;
				ep2out_Tram[3] = 0;
				ep2out_Tram[4] = 10;		// Command length (is it used in VHDL???) JOEJOEJOE
				//ep2out_Tram[5] = 0xEC;		// Fast Read 4-Byte Quad input/output command (use this line when SPI_QUAD will work)
				ep2out_Tram[5] = 0x0C;		// Fast Read 4-Byte command
				ep2out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep2out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep2out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep2out_Tram[9] = (unsigned char)(spiAddr);
				ep2out_Tram[10] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 1st
				ep2out_Tram[11] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 2nd
				// JOE: Should be remove but need some VHDL modif
				ep2out_Tram[12] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 3rd
				ep2out_Tram[13] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 4th
				ep2out_Tram[14] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 5th
				ep2out_Tram[15] = 0x00;		// Zero stuffing
				EZWriteEP2Tram(ep2out_Tram, EP6IN_SPI, 768); FAIL_IF_SPI_READ_ERR;
				memcpy(readBuffer, ep6in_Tram+4, nSpiBytes);
			}
			else {
				// 1 Quad internal command + 1 SPI command byte + 4 SPI address bytes + 10 dummy as 5 byte
				nTramPayloadBytes = nSpiBytes + 11;
				ep1out_Tram[0] = '^';
				ep1out_Tram[1] = 'P';
				ep1out_Tram[2] = LSB(nTramPayloadBytes);
				ep1out_Tram[3] = MSB(nTramPayloadBytes);
				ep1out_Tram[4] = 10;		// Command length
				ep1out_Tram[5] = 0x0B;		// Fast Read command
				ep1out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep1out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep1out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep1out_Tram[9] = (unsigned char)spiAddr;
				ep1out_Tram[10] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 1st
				ep1out_Tram[11] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 2nd
				ep1out_Tram[12] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 3rd
				ep1out_Tram[13] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 4th
				ep1out_Tram[14] = 0x00;		// Dummy two clock cycles for ten dummy cycles, 5th
				//EZWriteEP1Tram(ep1out_Tram, EP1IN_SPI_QUAD, nTramPayloadBytes);// FAIL_IF_SPI_READ_ERR;
				EZWriteEP1Tram(ep1out_Tram, EP1IN_SPI_QUAD, nSpiBytes); FAIL_IF_SPI_READ_ERR;
				memcpy(readBuffer, ep1in_Tram+4, nSpiBytes);
			}
		}
        else {
			nSpiBytes = nBytesRemaining < 256 ? nBytesRemaining : 256;

			if (DPxSpiHasVpxFpgaCtrl()) {
				ep2out_Tram[0] = '^';
				ep2out_Tram[1] = 'S';
				ep2out_Tram[2] = 4;
				ep2out_Tram[3] = 0;
				ep2out_Tram[4] = 0x0B;		// Fast Read command
				ep2out_Tram[5] = (unsigned char)(spiAddr >> 16);
				ep2out_Tram[6] = (unsigned char)(spiAddr >>  8);
				ep2out_Tram[7] = (unsigned char)spiAddr;
				EZWriteEP2Tram(ep2out_Tram, EP6IN_SPI, 256); FAIL_IF_SPI_READ_ERR;
				memcpy(readBuffer, ep6in_Tram+4, nSpiBytes);
			}
			else {
				nTramPayloadBytes = nSpiBytes + 5;  // 1 SPI command byte + 3 SPI address bytes + 1 dummy readback byte
				ep1out_Tram[0] = '^';
				ep1out_Tram[1] = 'S';
				ep1out_Tram[2] = LSB(nTramPayloadBytes);
				ep1out_Tram[3] = MSB(nTramPayloadBytes);
				ep1out_Tram[4] = 0x0B;		// Fast Read command
				ep1out_Tram[5] = (unsigned char)(spiAddr >> 16);
				ep1out_Tram[6] = (unsigned char)(spiAddr >>  8);
				ep1out_Tram[7] = (unsigned char)spiAddr;
				EZWriteEP1Tram(ep1out_Tram, EP1IN_SPI, nTramPayloadBytes); FAIL_IF_SPI_READ_ERR;
				memcpy(readBuffer, ep1in_Tram+9, nSpiBytes);
			}
		}
        spiAddr += nSpiBytes;
        readBuffer += nSpiBytes;
        nBytesRead += nSpiBytes;
        newPercentDone = nBytesRead * 100 / nReadBytes;
        if (percentCompletionCallback && newPercentDone != oldPercentDone)
            percentCompletionCallback(newPercentDone);
        oldPercentDone = newPercentDone;
    }

    return 0;

fail:
	DPxSetError(DPX_ERR_SPI_READ);
    return DPX_ERR_SPI_READ;
}


#define FAIL_IF_SPI_WRITE_ERR do { if (DPxGetError()) { fprintf(stderr, "ERROR: DPxSpiWrite() error = %d\n", DPxGetError()); goto fail; }} while (0)

// Write a SPI block of any size.
// Assumes that spiAddr starts on a 256-byte boundary.
// If nWriteBytes is not a multiple of 256 bytes, the last page is padded with 0.
int DPxSpiWrite(int spiAddr, int nWriteBytes, char* writeBuffer, PercentCompletionCallback percentCompletionCallback)
{
    unsigned int nBytesWritten, nBytesRemaining, nSpiBytes, oldPercentDone, newPercentDone;
    char dummyBuff;

    // Caller is responsible for ensuring valid dpxSysDevsel
    DPxSpiStart(); FAIL_IF_SPI_WRITE_ERR;

    nBytesWritten = 0;
    oldPercentDone = 0;
    if (percentCompletionCallback)
        percentCompletionCallback(0);

    while (nBytesWritten < (unsigned)nWriteBytes) {
        nBytesRemaining = nWriteBytes - nBytesWritten;
        nSpiBytes = nBytesRemaining < 256 ? nBytesRemaining : 256;

		if (DPxIsA10Arch()) {
			if (DPxSpiHasVpxFpgaCtrl()) {
				ep2out_Tram[0] = '^';
				ep2out_Tram[1] = 'S';
				ep2out_Tram[2] = 8;	 // We will always program SPI in chunks of 256 bytes
				ep2out_Tram[3] = 1;    
				ep2out_Tram[4] = 0x85;
				ep2out_Tram[5] = 0x12;  //4B SPI Page Program command
				ep2out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep2out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep2out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep2out_Tram[9] = (unsigned char)spiAddr;
				ep2out_Tram[10] = 0;
				ep2out_Tram[11] = 0;
				memcpy(ep2out_Tram+12, writeBuffer, nSpiBytes);
				memset(ep2out_Tram+12+nSpiBytes, 0, 256-nSpiBytes);  // Pad last page with 0's
				EZWriteEP2Tram(ep2out_Tram, 0, 0); FAIL_IF_SPI_WRITE_ERR;
			}
			else {
				EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0); FAIL_IF_SPI_WRITE_ERR;		// WREN command needed before programming each page
				ep1out_Tram[0] = '^';
				ep1out_Tram[1] = 'P';
				ep1out_Tram[2] = 6;
				ep1out_Tram[3] = 1;     // We will always program SPI in chunks of 256 bytes
				ep1out_Tram[4] = 0x85;  // SPI Page Program command
				ep1out_Tram[5] = 0x02;
				ep1out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep1out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep1out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep1out_Tram[9] = (unsigned char)spiAddr;
				memcpy(ep1out_Tram+10, writeBuffer, nSpiBytes);
				memset(ep1out_Tram+10+nSpiBytes, 0, 256-nSpiBytes);  // Pad last page with 0's
				EZWriteEP1Tram(ep1out_Tram, 'p', 0); FAIL_IF_SPI_WRITE_ERR;

				// Wait until the page write completes.
				// The FPGA SPI interface is robust, and automatically unlocks sectors, waits if SPI is busy, etc;
				// but our EZ-USB software SPI interface is not so robust, so we'll hang here until we know SPI is available for next operation.
				do {
					EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x05\x00", 'p', 1); FAIL_IF_SPI_WRITE_ERR;	// RDSR Read Status Register
				} while (ep1in_Tram[4] & 1);		// Stay here while the Write In Progress bit is set.
			}
		}
        else 
			if (DPxSpiHasVpxFpgaCtrl()) {
				ep2out_Tram[0] = '^';
				ep2out_Tram[1] = 'S';
				ep2out_Tram[2] = 4;
				ep2out_Tram[3] = 1;     // We will always program SPI in chunks of 256 bytes
				ep2out_Tram[4] = 0x02;  // SPI Page Program command
				ep2out_Tram[5] = (unsigned char)(spiAddr >> 16);
				ep2out_Tram[6] = (unsigned char)(spiAddr >>  8);
				ep2out_Tram[7] = (unsigned char)spiAddr;
				memcpy(ep2out_Tram+8, writeBuffer, nSpiBytes);
				memset(ep2out_Tram+8+nSpiBytes, 0, 256-nSpiBytes);  // Pad last page with 0's
				EZWriteEP2Tram(ep2out_Tram, 0, 0); FAIL_IF_SPI_WRITE_ERR;
			}
			else {
			    EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1); FAIL_IF_SPI_WRITE_ERR;		// WREN command needed before programming each page
				ep1out_Tram[0] = '^';
				ep1out_Tram[1] = 'S';
				ep1out_Tram[2] = 4;
				ep1out_Tram[3] = 1;     // We will always program SPI in chunks of 256 bytes
				ep1out_Tram[4] = 0x02;  // SPI Page Program command
				ep1out_Tram[5] = (unsigned char)(spiAddr >> 16);
				ep1out_Tram[6] = (unsigned char)(spiAddr >>  8);
				ep1out_Tram[7] = (unsigned char)spiAddr;
				memcpy(ep1out_Tram+8, writeBuffer, nSpiBytes);
				memset(ep1out_Tram+8+nSpiBytes, 0, 256-nSpiBytes);  // Pad last page with 0's
				EZWriteEP1Tram(ep1out_Tram, EP1IN_SPI, 260); FAIL_IF_SPI_WRITE_ERR;

				// Wait until the page write completes.
				// The FPGA SPI interface is robust, and automatically unlocks sectors, waits if SPI is busy, etc;
				// but our EZ-USB software SPI interface is not so robust, so we'll hang here until we know SPI is available for next operation.
				do {
					EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x05\x00", EP1IN_SPI, 2); FAIL_IF_SPI_WRITE_ERR;	// RDSR Read Status Register
				} while (ep1in_Tram[5] & 1);		// Stay here while the Write In Progress bit is set.
			}
        spiAddr += nSpiBytes;
        writeBuffer += nSpiBytes;
        nBytesWritten += nSpiBytes;
        newPercentDone = nBytesWritten * 100 / nWriteBytes;
        if (percentCompletionCallback && newPercentDone != oldPercentDone)
            percentCompletionCallback(newPercentDone);
        oldPercentDone = newPercentDone;
    }

    // We'll wait around until last write has actually completed;
    // otherwise, someone like DPxSpiStop() can steal SPI bus while SP2 is still polling the SPI status register.
    if (DPxSpiHasVpxFpgaCtrl())
        DPxSpiReadNoStartStop(0, 1, &dummyBuff, NULL);

    DPxSpiStop();
    return 0;

fail:
    DPxSpiStop();
	DPxSetError(DPX_ERR_SPI_WRITE);
    return DPX_ERR_SPI_WRITE;
}


#define FAIL_IF_SPI_ERASE_ERR do { if (DPxGetError()) { fprintf(stderr, "ERROR: DPxSpiErase() error = %d\n", DPxGetError()); goto fail; }} while (0)

// Erase (sets to 1) a SPI block of any size.
// Assumes that spiAddr starts on a 64kB boundary.
// The SPI is erased in 64kB chunks; if nEraseBytes is not a 64k multiple, it is rounded up to the next 64k multiple.
int DPxSpiErase(int spiAddr, int nEraseBytes, PercentCompletionCallback percentCompletionCallback)
{
    unsigned int nBytesErased, nBytesRemaining, nSpiBytes, oldPercentDone, newPercentDone;
    char dummyBuff;

    // Caller is responsible for ensuring valid dpxSysDevsel
    DPxSpiStart(); FAIL_IF_SPI_ERASE_ERR;

    nBytesErased = 0;
    oldPercentDone = 0;
    if (percentCompletionCallback)
        percentCompletionCallback(0);

    while (nBytesErased < (unsigned)nEraseBytes) {
        nBytesRemaining = nEraseBytes - nBytesErased;
        nSpiBytes = nBytesRemaining < 65536 ? nBytesRemaining : 65536;
		if (DPxIsA10Arch()){
			if (DPxSpiHasVpxFpgaCtrl()) {
				ep2out_Tram[0] = '^';
				ep2out_Tram[1] = 'S';
				ep2out_Tram[2] = 8;
				ep2out_Tram[3] = 0;
				ep2out_Tram[4] = 0x85;
				ep2out_Tram[5] = 0xDC;										// Sector Erase command 4B
				ep2out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep2out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep2out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep2out_Tram[9] = (unsigned char)spiAddr;
				ep2out_Tram[10] = 0;
				ep2out_Tram[11] = 0;

				EZWriteEP2Tram(ep2out_Tram, 0, 0); FAIL_IF_SPI_ERASE_ERR;

				// We'll wait around until erase is actually completed;
				// otherwise the FPGA SPI controller will just instantly buffer all the erase commands,
				// and the next SPI read/write command will block until the erases complete and the USB transaction will timeout.
				// Also, the % complete status would instantly jump to 100%.
				DPxSpiReadNoStartStop(0, 1, &dummyBuff, NULL); FAIL_IF_SPI_ERASE_ERR;
			}
			else {
				EZWriteEP1Tram((unsigned char*)"^P\x02\x00\x81\x06", 'p', 0); FAIL_IF_SPI_ERASE_ERR;      // WREN command needed before erasing a sector
				ep1out_Tram[0] = '^';
				ep1out_Tram[1] = 'P';
				ep1out_Tram[2] = 6;
				ep1out_Tram[3] = 0;
				ep1out_Tram[4] = 0x85;
				ep1out_Tram[5] = 0xD8;										// Sector Erase command
				ep1out_Tram[6] = (unsigned char)(spiAddr >> 24) & 0x7f;
				ep1out_Tram[7] = (unsigned char)(spiAddr >> 16);
				ep1out_Tram[8] = (unsigned char)(spiAddr >>  8);
				ep1out_Tram[9] = (unsigned char)spiAddr;
				EZWriteEP1Tram(ep1out_Tram, 'p', 0); FAIL_IF_SPI_ERASE_ERR;

				// We'll wait around until erase is complete, so next operation can start without checking SPI status
				do {                                                                                            // Have to wait until WRSR has completed
					EZWriteEP1Tram((unsigned char*)"^P\x03\x00\x01\x05\x00", 'p', 1); FAIL_IF_SPI_ERASE_ERR;	// RDSR Read Status Register
				} while (ep1in_Tram[4] & 1);		// Stay here while the Write In Progress bit is set.
			}
		}
        else if (DPxSpiHasVpxFpgaCtrl()) {
            ep2out_Tram[0] = '^';
            ep2out_Tram[1] = 'S';
            ep2out_Tram[2] = 4;
            ep2out_Tram[3] = 0;
            ep2out_Tram[4] = 0xD8;										// Sector Erase command
            ep2out_Tram[5] = (unsigned char)(spiAddr >> 16);
            ep2out_Tram[6] = (unsigned char)(spiAddr >>  8);
            ep2out_Tram[7] = (unsigned char)spiAddr;
            EZWriteEP2Tram(ep2out_Tram, 0, 0); FAIL_IF_SPI_ERASE_ERR;

            // We'll wait around until erase is actually completed;
            // otherwise the FPGA SPI controller will just instantly buffer all the erase commands,
            // and the next SPI read/write command will block until the erases complete and the USB transaction will timeout.
            // Also, the % complete status would instantly jump to 100%.
            DPxSpiReadNoStartStop(0, 1, &dummyBuff, NULL); FAIL_IF_SPI_ERASE_ERR;
        }
        else {
            EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1); FAIL_IF_SPI_ERASE_ERR;      // WREN command needed before erasing a sector
            ep1out_Tram[0] = '^';
            ep1out_Tram[1] = 'S';
            ep1out_Tram[2] = 4;
            ep1out_Tram[3] = 0;
            ep1out_Tram[4] = 0xD8;										// Sector Erase command
            ep1out_Tram[5] = (unsigned char)(spiAddr >> 16);
            ep1out_Tram[6] = (unsigned char)(spiAddr >>  8);
            ep1out_Tram[7] = (unsigned char)spiAddr;
            EZWriteEP1Tram(ep1out_Tram, EP1IN_SPI, 4); FAIL_IF_SPI_ERASE_ERR;

            // We'll wait around until erase is complete, so next operation can start without checking SPI status
            do {                                                                                            // Have to wait until WRSR has completed
                EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x05\x00", EP1IN_SPI, 2); FAIL_IF_SPI_ERASE_ERR;	// RDSR Read Status Register
            } while (ep1in_Tram[5] & 1);		// Stay here while the Write In Progress bit is set.
        }
        spiAddr += nSpiBytes;
        nBytesErased += nSpiBytes;
        newPercentDone = nBytesErased * 100 / nEraseBytes;
        if (percentCompletionCallback && newPercentDone != oldPercentDone)
            percentCompletionCallback(newPercentDone);
        oldPercentDone = newPercentDone;
    }

    // We'll wait around until last erase has actually completed;
    // otherwise, someone like DPxSpiStop() can steal SPI bus while SP2 is still polling the SPI status register.
    if (DPxSpiHasVpxFpgaCtrl())
        DPxSpiRead(0, 1, &dummyBuff, NULL);

    DPxSpiStop();
    return 0;

fail:
    DPxSpiStop();
	DPxSetError(DPX_ERR_SPI_ERASE);
    return DPX_ERR_SPI_ERASE;
}



char spiModifyBuff[65536];

// Modify a region of SPI within a single 64kB page
void DPxSpiModify(int spiAddr, int nWriteBytes, unsigned char* writeBuffer)
{
    int page = spiAddr & 0xFFFF0000;
    int offset = spiAddr & 0x0000FFFF;

    // Caller is responsible for ensuring valid dpxSysDevsel
    DPxSpiRead(page, 65536, spiModifyBuff, NULL);
    memcpy(spiModifyBuff+offset, writeBuffer, nWriteBytes);   
    DPxSpiErase(page, 65536, NULL);
    DPxSpiWrite(page, 65536, spiModifyBuff, NULL);
}


// Non-0 if DPxOpen found a DP
int DPxIsOpen()
{
	return dpxSysDevsel >= DPX_DEVSEL_FIRST_DEVICE && dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE && dpxDeviceTable[dpxSysDevsel].dpxHdl != 0;
}


// Non-0 if a detected DP has no EZ-USB firmware
int DPxHasRawUsb()
{
	return dpxDeviceTable[dpxSysDevsel].dpxRawUsb;
}


// DPxReset causes the following sequence:
//	1) DATAPixx hardware reset for 200 microseconds
//	2) DATAPixx disconnects from USB
//	3) 1.5 second delay
//	4) DATAPixx reconnects to USB
// We need gDoingHardwareReset backdoor to tell DPxClose() to not try to do any USB access
int gDoingHardwareReset = 0;

void DPxReset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;

	if (DPxIsDatapixx())
	{
		// On the DATAPixx, we must force a manual reset. Simply using "EZWriteEP1Tram" does not work.
		// When using "EZWriteEP1Tram", MRESETn and RESETn are pulled down and never go up by their self. 
		// Forcing MRESETn down and then up is the only software way to be able to reset the DATAPixx properly. 
		EZWriteSFR(EZ_SFR_IOE, 0);
		EZWriteSFR(EZ_SFR_OEE, 1);
		EZWriteSFR(EZ_SFR_IOE, 1);

	}
	else
	{
		if (EZWriteEP1Tram((unsigned char*)"^B\x00\x00", 0, 0))
			{ fprintf(stderr,"ERROR: Sending reset tram\n"); }
	}

    gDoingHardwareReset = 1;
	DPxClose();
    gDoingHardwareReset = 0;
}

void DPxResetAll()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++)
	{
		if (dpxDeviceTable[dpxSysDevsel].dpxDev)
		{
			if (dpxSysDevsel == DPX_DEVSEL_DPX)
			{
				// On the DATAPixx, we must force a manual reset. Simply using "EZWriteEP1Tram" does not work.
				// When using "EZWriteEP1Tram", MRESETn and RESETn are pulled down and never go up by their self. 
				// Forcing MRESETn down and then up is the only software way to be able to reset the DATAPixx properly. 
				EZWriteSFR(EZ_SFR_IOE, 0);
				EZWriteSFR(EZ_SFR_OEE, 1);
				EZWriteSFR(EZ_SFR_IOE, 1);
			}
			else
			{
				if (EZWriteEP1Tram((unsigned char*)"^B\x00\x00", 0, 0))
					{ fprintf(stderr,"ERROR: Sending reset tram\n"); }
			}
		}
    }

    gDoingHardwareReset = 1;
	DPxClose();
    gDoingHardwareReset = 0;
}

StringCallback gStatusCallback;
static char statusMsg[256];

void FPGAErasePercentCompletionCallback(int percentCompletion);
void FPGAWritePercentCompletionCallback(int percentCompletion);
void FPGAVerifyPercentCompletionCallback(int percentCompletion);

void FPGAErasePercentCompletionCallback(int percentCompletion)
{
    sprintf(statusMsg, "\rFlash Erase  %3d%% completed", percentCompletion);
    if (gStatusCallback)
        gStatusCallback(statusMsg);
    else {
        printf("%s", statusMsg);
        fflush(stdout);
    }
}

void FPGAWritePercentCompletionCallback(int percentCompletion)
{
    sprintf(statusMsg, "\rFlash Write  %3d%% completed", percentCompletion);
    if (gStatusCallback)
        gStatusCallback(statusMsg);
    else {
        printf("%s", statusMsg);
        fflush(stdout);
    }
}


void FPGAVerifyPercentCompletionCallback(int percentCompletion)
{
    sprintf(statusMsg, "\rFlash Verify %3d%% completed", percentCompletion);
    if (gStatusCallback)
        gStatusCallback(statusMsg);
    else {
        printf("%s", statusMsg);
        fflush(stdout);
    }
}


int DPxProgramFPGA(unsigned char* configBuff, int configFileSize, int doProgram, int doVerify, int reconfigFpga, StringCallback statusCallback)
{
	int	spiAddr;
	int	sfr_iod, sfr_ioe, i, nErrors;
    char dummyBuff;
    int rdCount;

    // Enable this if an FPGA has a bad load, and we need to deconfigure it before we can access SPI.  Holds nConfig low.
#if 0
    if (EZWriteSFR(EZ_SFR_IOE,    0) < 0) { fprintf(stderr, "ERROR: Cfg 1 IOE EZWriteSFR() failed\n"); }
    if (EZWriteSFR(EZ_SFR_OEE, 0x20) < 0) { fprintf(stderr, "ERROR: Cfg 1 OEE EZWriteSFR() failed\n"); }
#endif

	if (DPxIsS3Arch() || DPxIsA5Arch() || DPxIsA10Arch())
		spiAddr = SPI_ADDR_VPX_FPGA;
	else
		spiAddr = SPI_ADDR_DPX_FPGA;

    // Need to flip bits in configuration file if programming VIEWPixx/PROPixx/TRACKPixx Bridge
    if (DPxIsS3Arch() || DPxIsA5Arch()) {
		for (i = 0; i < configFileSize; i++)
            configBuff[i] = (configBuff[i] & 0x01 ? 0x80 : 0)
                          + (configBuff[i] & 0x02 ? 0x40 : 0)
                          + (configBuff[i] & 0x04 ? 0x20 : 0)
                          + (configBuff[i] & 0x08 ? 0x10 : 0)
                          + (configBuff[i] & 0x10 ? 0x08 : 0)
                          + (configBuff[i] & 0x20 ? 0x04 : 0)
                          + (configBuff[i] & 0x40 ? 0x02 : 0)
                          + (configBuff[i] & 0x80 ? 0x01 : 0);
    }
	
	if (doProgram) {

		// Erase the SPI flash.  DATAPixx takes about 12 seconds, VIEWPixx/PROPixx takes about 26 seconds.
		if (!statusCallback)
			printf("\nReflashing %s\n*** Do not turn off system until flash programming complete! ***\n\n", DPxIsViewpixx()			? "VIEWPixx" :
                                                                                                            DPxIsPropixx()			? "PROPixx" :
                                                                                                            DPxIsPropixxCtrl()		? "PROPixx Ctrl" :
																											DPxIsTrackpixxCtrl()	? "TRACKPixx Ctrl" :
																											DPxIsTrackpixxBridge()	? "TRACKPixx Bridge" :
																											DPxIsTrackpixx()		? "TRACKPixx" :
																											DPxIsDatapixx2()		? "DATAPixx2" :
																											DPxIsDatapixx3()		? "DATAPixx3" :
																																	  "DATAPixx");
        gStatusCallback = statusCallback;
        if (DPxSpiErase(spiAddr, configFileSize, FPGAErasePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');

		// Write the SPI flash.  DATAPixx takes about 24 seconds, VIEWPixx takes about 10 seconds when using fast FPGA SPI interface.
        if (DPxSpiWrite(spiAddr, configFileSize, (char*)configBuff, FPGAWritePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
    }

	// Do readback to confirm that we successfully programmed the SPI.  DATAPixx takes about 19 seconds, VIEWPixx takes about 3 seconds when using fast FPGA SPI interface.
    // I was getting some verify errors on VIEWPixx, but I seem to have corrected the problem in VHDL.
    if (doVerify) {
        if (DPxSpiRead(spiAddr, configFileSize, (char*)configBuffer2, FPGAVerifyPercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
        if (memcmp(configBuff, configBuffer2, configFileSize)) {
            fprintf(stderr, "ERROR: flash verify failed\n");
            nErrors = 0;
            for (i = 0; i < configFileSize; i++)
                if (configBuff[i] != configBuffer2[i] && ++nErrors <= 10)
                    fprintf(stderr, "byte %d is %d instead of %d\n", i, configBuffer2[i], configBuff[i]);
            printf("%d total verify errors\n", nErrors);
        }
    }

    // If we're _not_ doing a verify, we'll still do 1 small SPI readback, just to ensure that the SPI programming has completed.
    else if (DPxSpiRead(0, 1, &dummyBuff, NULL))
        goto abort;

	// Strobe VIEWPixx nConfig, or DATAPixx PGMn _after_ I've finished programming the SPI device.
	// If the user is using our video output as their primary display, they don't loose the display during SPI programming.
	// Also, if the SPI programming is stopped, or fails, they haven't lost their FPGA unless they powerdown.
	// They still have a chance to retry the FPGA programming.
	if (reconfigFpga) {

        // Strategy is slightly different for DATAPixx (ie: Lattice ECP2), versus VIEWPixx/PROPixx which use Stratix 3.
		if (DPxIsS3Arch()) {

            // Drive low Stratix 3 nConfig net.
            if (EZWriteSFR(EZ_SFR_IOE,    0) < 0) { fprintf(stderr, "ERROR: Cfg 1 IOE EZWriteSFR() failed\n"); }
            if (EZWriteSFR(EZ_SFR_OEE, 0x20) < 0) { fprintf(stderr, "ERROR: Cfg 1 OEE EZWriteSFR() failed\n"); }

            // Enable quickswitch so FPGA can reach its configuration SPI.
            if (EZWriteSFR(EZ_SFR_IOC,    0) < 0) { fprintf(stderr, "ERROR: Cfg 2 IOC EZWriteSFR() failed\n"); };
            if (EZWriteSFR(EZ_SFR_OEC, 0x20) < 0) { fprintf(stderr, "ERROR: Cfg 2 OEC EZWriteSFR() failed\n"); };

            // Bring nConfig back up, starting reconfiguration
            if (EZWriteSFR(EZ_SFR_IOE, 0x20) < 0) { fprintf(stderr, "ERROR: Cfg 3 IOE EZWriteSFR() failed\n"); }

            // Wait until configuration complete before resetting board.
            // I don't think this should be necessary, since the EZ should wait until FPGA releases RESETn,
            // but if I go directly to DPxReset(), I'm still getting occasional EZ with no firmware and have to power cycle.
            // On MBP, requires between 2400-2600 loops for CONF_DONE to come back up.
            // If I only loop max 50 times, I get no firmware.  100 times I get firmware.  usleep 100ms is enough, but 10ms isn't.
            // Oh well.  I don't know who needs this time, but I'll just do 3000 loops.
            // It's possible that the reset supervisor's cap doesn't get fully discharged, so RESETn doesn't stay low long enough.
            for (rdCount = 0; rdCount < 3000; rdCount++) {
                sfr_ioe = EZReadSFR(EZ_SFR_IOE);
                if (sfr_ioe < 0) {
                    fprintf(stderr, "ERROR: Cfg 4 IOE EZReadSFR() failed\n");
                    break;
                }
                if (sfr_ioe & 0x04) // CONF_DONE
                    break;
            }
            
            // Hard board reset so EZ will re-read its firmware from FPGA, and all components start at known state.
            DPxReset();

		}
        else if (DPxIsA5Arch()) {

			// WREN command needed before writing status reg
			if (EZWriteEP1Tram((unsigned char*)"^S\x01\x00\x06", EP1IN_SPI, 1))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #3\n");}
	
			// Write Volatile configuration register >>> set back to 12 Dummy clock cycle! 
			if (EZWriteEP1Tram((unsigned char*)"^S\x02\x00\x81\xCB", EP1IN_SPI, 2))
			{ fprintf(stderr,"ERROR: DPxSpiStart() call to EZWriteEP1Tram() failed #4\n");}  // >>> JOE it doesn't work...

            // Drive low Arria V nConfig net.
            if (EZWriteSFR(EZ_SFR_IOD, 0x00) < 0) { fprintf(stderr, "ERROR: Cfg 1 IOD EZWriteSFR() failed\n"); }
            if (EZWriteSFR(EZ_SFR_OED, 0x40) < 0) { fprintf(stderr, "ERROR: Cfg 1 OED EZWriteSFR() failed\n"); }

            // Enable quickswitch so FPGA can reach its configuration SPI.
            if (EZWriteSFR(EZ_SFR_IOD, 0x00) < 0) { fprintf(stderr, "ERROR: Cfg 2 IOD EZWriteSFR() failed\n"); };
            if (EZWriteSFR(EZ_SFR_OED, 0x50) < 0) { fprintf(stderr, "ERROR: Cfg 2 OED EZWriteSFR() failed\n"); };

            // Bring nConfig back up, starting reconfiguration
            if (EZWriteSFR(EZ_SFR_IOD, 0x40) < 0) { fprintf(stderr, "ERROR: Cfg 3 IOD EZWriteSFR() failed\n"); }

			// Wait until configuration complete before resetting board.
			// WARNING: 12000 loop is totally arbitrary... You getting occasional EZ with no firmware and have to power cycle, Contact Joe or Peter!!!
            for (rdCount = 0; rdCount < 12000; rdCount++) {
                sfr_iod = EZReadSFR(EZ_SFR_IOD);
                if (sfr_iod < 0) {
                    fprintf(stderr, "ERROR: Cfg 4 IOD EZReadSFR() failed\n");
                    break;
                }
                if (sfr_iod & 0x80) // CONF_DONE
                    break;
            }
            
            // Hard board reset so EZ will re-read its firmware from FPGA, and all components start at known state.
            DPxReset();
        }
		else if (DPxIsA10Arch()) {
			// Drive low Arria 10 nConfig net, SPI is kept deselected by pull-up to Vccio
            // Enable quickswitch so FPGA can reach its configuration SPI. Quickswitch has pull-down, so tristate the ctrl
			
			if (EZWriteSFR(FX3_REG_TRIS, 0x84) < 0) { fprintf(stderr, "ERROR: Cfg 1 OED EZWriteSFR() failed\n"); }
			if (EZWriteSFR(FX3_REG_IO, 0x80) < 0) { fprintf(stderr, "ERROR: Cfg 1 IOD EZWriteSFR() failed\n"); }
			if (EZWriteSFR(FX3_REG_IO, 0x00) < 0) { fprintf(stderr, "ERROR: Cfg 1 IOD EZWriteSFR() failed\n"); }
            // Bring nConfig back up, starting reconfiguration
            if (EZWriteSFR(FX3_REG_IO, 0x80) < 0) { fprintf(stderr, "ERROR: Cfg 3 IOD EZWriteSFR() failed\n"); }

			// Wait until configuration complete before resetting board.
			// WARNING: 30000 loop is totally arbitrary... You getting occasional EZ with no firmware and have to power cycle, Contact Joe, Peter or Jacques!!!
			// This is an estimate obtained from uncompressed file size of Arria 10.
			//   It has a max size of ~178Mbit which is about x2.5 more data than an A5GXA1 (~72Mbit) so 12000 * 2.5 = 30K
            for (rdCount = 0; rdCount < 30000; rdCount++) {
				sfr_iod = EZReadSFR(FX3_REG_CONF);
                if (sfr_iod < 0) {
                    fprintf(stderr, "ERROR: Cfg 4 IOD EZReadSFR() failed\n");
                    break;
                }
                if (sfr_iod & 0x80) // CONF_DONE
                    break;
            }
            
            // Hard board reset so EZ will re-read its firmware from FPGA, and all components start at known state.
            DPxReset();
        }
        // DATAPixx
        else {
            
            // Drive low ECP2 FPGA_PGMn net.
            if (EZWriteSFR(EZ_SFR_IOE, 0) < 0) { fprintf(stderr, "ERROR: PGMn start IOE EZWriteSFR() failed\n"); }
            if (EZWriteSFR(EZ_SFR_OEE, 0x20) < 0) { fprintf(stderr, "ERROR: PGMn start OEE EZWriteSFR() failed\n"); }
            
            // ***Here's a good one.  I'm doing a powered reconfig here, so HRESET is currently inactive.
            // Unfortunately, on DATAPixx HRESET is driving the FPGA's SPIFASTN pin, so FPGA will use the slower SPI read command.
            // My SPI only has an Fmax of 20 MHz for the slow commands, as opposed to 50 MHz for the fast commands.
            // The FPGA tries to reconfigure at high speed using the slow commands, and crashes miserably (at least with the 64 Mbit parts).
            // The only solution is to immediately bring down HRESET!
            // When the EZ goes into reset, it releases FPGA_PGMn, and the FPGA starts to configure.
            // The FPGA sees low on SPIFASTN, so configures at high speed.
            DPxReset();
        }
	}

	return 0;

abort:
	return -1;
}


int DPxProgramDDD(unsigned char* configBuff, int configFileSize, int doProgram, int doVerify, int reconfigFpga, StringCallback statusCallback)
{
	int	spiAddr;
	int	i, nErrors;
    char dummyBuff;
	
	spiAddr = SPI_ADDR_PPX_DDD;

	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX)) // for PROPixx only
		return 0;

	for (i = 0; i < configFileSize; i++){
		if (i%2 == 0)
            configBuffer2[i] = configBuff[i+1];
		else
			configBuffer2[i] = configBuff[i-1];
	}

	configBuff = configBuffer2;

    if (doProgram) {

		// Erase the SPI flash. Takes about 10 seconds.
		if (!statusCallback)
			printf("\nReflashing DDD\n*** Do not turn off system until flash programming complete! ***\n\n");

        gStatusCallback = statusCallback;
        if (DPxSpiErase(spiAddr, configFileSize, FPGAErasePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');

        if (DPxSpiWrite(spiAddr, configFileSize, (char*)configBuff, FPGAWritePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
    }

    if (doVerify) {
        if (DPxSpiRead(spiAddr, configFileSize, (char*)configBuffer2, FPGAVerifyPercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
        if (memcmp(configBuff, configBuffer2, configFileSize)) {
            fprintf(stderr, "ERROR: flash verify failed\n");
            nErrors = 0;
            for (i = 0; i < configFileSize; i++)
                if (configBuff[i] != configBuffer2[i] && ++nErrors <= 10)
                    fprintf(stderr, "byte %d is %d instead of %d\n", i, configBuffer2[i], configBuff[i]);
            printf("%d total verify errors\n", nErrors);
        }
    }

    // If we're _not_ doing a verify, we'll still do 1 small SPI readback, just to ensure that the SPI programming has completed.
    else if (DPxSpiRead(0, 1, &dummyBuff, NULL))
        goto abort;
	
	return 0;

abort:
	return -1;
}


int DPxProgramSplashScreen(unsigned char* configBuff, int configFileSize, int doProgram, int doVerify, int secondSS, StringCallback statusCallback)
{
	int	spiAddr;
	int	i, nErrors;
    char dummyBuff;
	
	if (DPxIsPropixx())
		spiAddr = SPI_ADDR_PPX_SS;
	else if (secondSS == 0)
		spiAddr = SPI_ADDR_FULL_SS;
	else
		spiAddr = SPI_ADDR_LITE_SS;
	
	for (i = 0; i < configFileSize; i++){
		if (i%2 == 0)
            configBuffer2[i] = configBuff[i+1];
		else
			configBuffer2[i] = configBuff[i-1];
	}

	configBuff = configBuffer2;

    if (doProgram) {

		// Erase the SPI flash. Takes about 10 seconds.
		if (!statusCallback)
			printf("\nReflashing Splash Screen #%d\n*** Do not turn off system until flash programming complete! ***\n\n", secondSS+1);

        gStatusCallback = statusCallback;
        if (DPxSpiErase(spiAddr, configFileSize, FPGAErasePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');

        if (DPxSpiWrite(spiAddr, configFileSize, (char*)configBuff, FPGAWritePercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
    }

    if (doVerify) {
        if (DPxSpiRead(spiAddr, configFileSize, (char*)configBuffer2, FPGAVerifyPercentCompletionCallback))
            goto abort;
		if (!statusCallback)
			putchar('\n');
        if (memcmp(configBuff, configBuffer2, configFileSize)) {
            fprintf(stderr, "ERROR: flash verify failed\n");
            nErrors = 0;
            for (i = 0; i < configFileSize; i++)
                if (configBuff[i] != configBuffer2[i] && ++nErrors <= 10)
                    fprintf(stderr, "byte %d is %d instead of %d\n", i, configBuffer2[i], configBuff[i]);
            printf("%d total verify errors\n", nErrors);
        }
    }

    // If we're _not_ doing a verify, we'll still do 1 small SPI readback, just to ensure that the SPI programming has completed.
    else if (DPxSpiRead(0, 1, &dummyBuff, NULL))
        goto abort;
	
	return 0;

abort:
	return -1;
 }


#define HIGH_CAL_DAC_VALUE	0x6000	// Gives +7.5V on +-10V DACs
#define LOW_CAL_DAC_VALUE	0xA000	// Gives -7.5V on +-10V DACs

void DPxCalibRead()
{
	int spiAddr;
	unsigned char dataBuff[88], *dataPtr;
	int iChan;
	unsigned short param;
	double m, b;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;

    spiAddr = (DPxIsViewpixx() || DPxIsPropixxCtrl() || DPxIsDatapixx2()) ? SPI_ADDR_VPX_ANALOG : SPI_ADDR_DPX_ANALOG;
	spiAddr = (DPxIsDatapixx3() ? SPI_ADDR_DP3_CALIB : spiAddr);

    if (DPxSpiRead(spiAddr, 88, (char*)dataBuff, NULL)) {
        fprintf(stderr, "ERROR: Could not read SPI\n");
        return;
    }

    dataPtr = dataBuff;
	for (iChan = 0; iChan < 22; iChan++) {
		param = *dataPtr++ << 8;
		param += *dataPtr++;
		m = (param + 32768.0) / 65536.0;
		param = *dataPtr++ << 8;
		param += *dataPtr++;
		b = (signed short)param;
		if (iChan < 4)
			printf("DAC[%d]", iChan);
		else if (iChan < 20)
			printf("ADC[%d]", iChan - 4);
		else
			printf("REF[%d]", iChan - 20);
		printf(" m = %.5f, b = %.1f\n", m, b);
		if (iChan == 3)
			printf("\n");
    }
}


void DPxCalibWrite()
{
	double adcFDatum;
	double dacHighV[4];
	double dacLowV[4];
	double sx[18], sx2[18];
	double adcHighMeanDatum[18];
	double adcLowMeanDatum[18];
	double adcMean, adcSD;
	double rangeMin, rangeMax;
	double highCalDatum, lowCalDatum;
	double adcHighV, adcLowV;
	double m, b;
	char str[256];
	int i, iChan;
	int lsbRange;
	signed short adcDatum, adcMin[18], adcMax[18];
	int nSamples;
	signed short dacHighRawDatum, dacLowRawDatum;
	unsigned short dacVhdlm[4];
	signed short dacVhdlb[4];
	unsigned short adcVhdlm[18];
	signed short adcVhdlb[18];
	int spiAddr;
	unsigned char dataBuff[88], *dataPtr;
    
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;

	// Set the DACs and ADCs to bypass calibration
	DPxEnableDacCalibRaw();
	DPxEnableAdcCalibRaw();

	// Write the DAC values for the first calibration voltage
	printf("Enter first calibration DAC datum (hit enter for 0x%0X): ", HIGH_CAL_DAC_VALUE);
	fgets(str, 256, stdin);
	dacHighRawDatum = DPxStringToInt(str);
	if (dacHighRawDatum == 0)
		dacHighRawDatum = HIGH_CAL_DAC_VALUE;
	printf("Using DAC datum 0x%0X\n", (int)dacHighRawDatum & 0x0000FFFF);
	DPxSetDacValue(dacHighRawDatum, 0);
	DPxSetDacValue(dacHighRawDatum, 1);
	DPxSetDacValue(dacHighRawDatum, 2);
	DPxSetDacValue(dacHighRawDatum, 3);
	DPxEnableAdcFreeRun();
	DPxUpdateRegCache();
	if (DPxGetError())
		{ fprintf(stderr, "ERROR: Could not set initial DAC values\n"); return; }

	// Get the precision measured voltages from user
	printf("Enter measured voltages for DAC0 - DAC3: ");
	fgets(str, 256, stdin);
	if (sscanf(str, "%lf%lf%lf%lf", dacHighV, dacHighV+1, dacHighV+2, dacHighV+3) != 4) {
		printf("Couldn't read voltages\n");
		return;
	}

	// Read the ADCs 1000 times, and calculate the mean voltages
	for (i = 0; i < 18; i++) {
		sx[i] = 0;
		sx2[i] = 0;
		adcMin[i] = 0x7fff;	// Largest signed short
		adcMax[i] = 0x8000;	// Smallest signed short
	}

	nSamples = 1000;
	
	for (i = 0; i < nSamples; i++) {
		DPxUpdateRegCache();
		for (iChan = 0; iChan < 18; iChan++) {
			adcDatum = DPxGetAdcValue(iChan);
			adcFDatum = adcDatum;	// Convert from signed short to double.
			sx[iChan] += adcFDatum;
			sx2[iChan] += adcFDatum * adcFDatum;
			if (adcMin[iChan] > adcDatum)
				adcMin[iChan] = adcDatum;
			if (adcMax[iChan] < adcDatum)
				adcMax[iChan] = adcDatum;
		}
	}

	for (iChan = 0; iChan < 18; iChan++) {
		adcMean = sx[iChan] / nSamples;
		adcSD = sqrt(nSamples*sx2[iChan] - sx[iChan]*sx[iChan]) / nSamples;
		lsbRange = adcMax[iChan] - adcMin[iChan];
		printf("ch%02d: mean = %7.4fV, sd = %7.4fV, +-LSB = %d\n", iChan, adcMean / 32768.0 * 10.0, adcSD / 32768.0 * 10.0, lsbRange/2);
		adcHighMeanDatum[iChan] = adcMean;
	}

	// Write the DAC values for the second calibration voltage
	printf("Enter second calibration DAC datum (hit enter for 0x%0X): ", LOW_CAL_DAC_VALUE);
	fgets(str, 256, stdin);
	dacLowRawDatum = DPxStringToInt(str);
	if (dacLowRawDatum == 0)
		dacLowRawDatum = LOW_CAL_DAC_VALUE;
	printf("Using DAC datum 0x%0X\n", (int)dacLowRawDatum & 0x0000FFFF);
	DPxSetDacValue(dacLowRawDatum, 0);
	DPxSetDacValue(dacLowRawDatum, 1);
	DPxSetDacValue(dacLowRawDatum, 2);
	DPxSetDacValue(dacLowRawDatum, 3);
	DPxUpdateRegCache();
	if (DPxGetError())
		fprintf(stderr,"ERROR: call to EZWriteEP2Tram() failed\n");

	// Get the precision measured voltages from user
	printf("Enter measured voltages for DAC0 - DAC3: ");
	fgets(str, 256, stdin);
	if (sscanf(str, "%lf%lf%lf%lf", dacLowV, dacLowV+1, dacLowV+2, dacLowV+3) != 4) {
		printf("Couldn't read voltages\n");
		return;
	}

	// Read the ADCs 1000 times, and calculate the mean voltages
	for (i = 0; i < 18; i++) {
		sx[i] = 0;
		sx2[i] = 0;
		adcMin[i] = 0x7fff;	// Largest signed short
		adcMax[i] = 0x8000;	// Smallest signed short
	}

	for (i = 0; i < nSamples; i++) {
		DPxUpdateRegCache();
		for (iChan = 0; iChan < 18; iChan++) {
			adcDatum = DPxGetAdcValue(iChan);
			adcFDatum = adcDatum;	// Convert from signed short to double.
			sx[iChan] += adcFDatum;
			sx2[iChan] += adcFDatum * adcFDatum;
			if (adcMin[iChan] > adcDatum)
				adcMin[iChan] = adcDatum;
			if (adcMax[iChan] < adcDatum)
				adcMax[iChan] = adcDatum;
		}
	}

	for (iChan = 0; iChan < 18; iChan++) {
		adcMean = sx[iChan] / nSamples;
		adcSD = sqrt(nSamples*sx2[iChan] - sx[iChan]*sx[iChan]) / nSamples;
		lsbRange = adcMax[iChan] - adcMin[iChan];
		printf("ch%02d: mean = %7.4fV, sd = %7.4fV, +-LSB = %d\n", iChan, adcMean / 32768.0 * 10.0, adcSD / 32768.0 * 10.0, lsbRange/2);
		adcLowMeanDatum[iChan] = adcMean;
	}

	// Calculate mx+b terms to map DAC calib datum to raw datum.
	printf("\n");
	for (iChan = 0; iChan < 4; iChan++) {
        DPxGetDacRange(iChan, &rangeMin, &rangeMax);
		highCalDatum = dacHighV[iChan] / rangeMax * 32768.0;
		lowCalDatum = dacLowV[iChan] / rangeMax * 32768.0;
		m = ((double)dacHighRawDatum - (double)dacLowRawDatum) / (highCalDatum - lowCalDatum);
		b = dacLowRawDatum - m * lowCalDatum;
		printf("DAC[%d] m = %.5f, b = %.1f\n", iChan, m, b);
		if (m < 0.75 || m > 1.25 || b < -8192 || b > 8192) {	// Sanity check
			fprintf(stderr,"ERROR: DAC calibration factors out of range\n");
			return;
		}
		dacVhdlm[iChan] = (unsigned short)floor(m * 65536 - 32768 + 0.5);	// scale to 16-bit unsigned cal factor, and offset to give range of 0.5-1.5x
		dacVhdlb[iChan] = (short)floor(b + 0.5);							// Round to integer offset which will be used in VHDL calibration

		// See if extreme values will clamp in VHDL; ie: is a DAC physically capable of outputting its full +-10V (or 5V) range?
		// Need to do in double for enough sig digits.
		if (((dacVhdlm[iChan] + 32768.0) *  0x7FFF + dacVhdlb[iChan] > ( 0x7FFF * 65536.0)) ||
			((dacVhdlm[iChan] + 32768.0) * -0x8000 + dacVhdlb[iChan] < (-0x8000 * 65536.0)))
			printf("                            WARNING: DAC cannot drive full +-%dV range\n", (int)rangeMax);
	}

	// Calculate mx+b terms to map ADC raw datum to calib datum.
	printf("\n");
	for (iChan = 0; iChan < 18; iChan++) {
		if (iChan == 17) {
			adcHighV = dacHighV[3];
			adcLowV = dacLowV[3];
		}
		else if (iChan == 16) {
			adcHighV = dacHighV[2];
			adcLowV = dacLowV[2];
		}
		else if (iChan & 1) {
			adcHighV = dacHighV[1];
			adcLowV = dacLowV[1];
		}
		else {
			adcHighV = dacHighV[0];
			adcLowV = dacLowV[0];
		}

		highCalDatum = adcHighV / 10.0 * 32768.0;
		lowCalDatum = adcLowV / 10.0 * 32768.0;
		m = (highCalDatum - lowCalDatum) / (adcHighMeanDatum[iChan] - adcLowMeanDatum[iChan]);
		b = lowCalDatum - m * adcLowMeanDatum[iChan];
		printf("ADC[%d] m = %.5f, b = %.1f\n", iChan, m, b);
		if (m < 0.75 || m > 1.25 || b < -8192 || b > 8192) {	// Sanity check
			fprintf(stderr,"ERROR: ADC calibration factors out of range\n");
			return;
		}
		adcVhdlm[iChan] = (unsigned short)floor(m * 65536 - 32768 + 0.5);	// scale to 16-bit unsigned cal factor, and offset to give range of 0.5-1.5x
		adcVhdlb[iChan] = (short)floor(b + 0.5);							// Round to integer offset which will be used in VHDL calibration

		// See if extreme values will clamp in VHDL; ie: is the full +-10V range within the ADC's output codes?
		// Need to do in double for enough sig digits.
		if (((adcVhdlm[iChan] + 32768.0) *  0x7FFF + adcVhdlb[iChan] < ( 0x7FFF * 65536.0)) ||
			((adcVhdlm[iChan] + 32768.0) * -0x8000 + adcVhdlb[iChan] > (-0x8000 * 65536.0)))
			printf("                            WARNING: ADC cannot decode full +-10V range.  VHDL will clamp.\n");
	}

    // Put the data into a buffer
    dataPtr = dataBuff;
    for (iChan = 0; iChan < 4; iChan++) {
        *dataPtr++ = MSB(dacVhdlm[iChan]);
        *dataPtr++ = LSB(dacVhdlm[iChan]);
        *dataPtr++ = MSB(dacVhdlb[iChan]);
        *dataPtr++ = LSB(dacVhdlb[iChan]);
    }
    for (iChan = 0; iChan < 18; iChan++) {
        *dataPtr++ = MSB(adcVhdlm[iChan]);
        *dataPtr++ = LSB(adcVhdlm[iChan]);
        *dataPtr++ = MSB(adcVhdlb[iChan]);
        *dataPtr++ = LSB(adcVhdlb[iChan]);
    }

	// Write the calibration info to the SPI.
	// I'll assume that the FPGA is not accessing the SPI device.
	// It should be OK.  FPGA only accesses SPI for less than a millisecond, right after reset.
	// This way, I don't get screen flash, EZ Slave interface problems, etc.
	printf("Writing calibration to SPI\n");
    spiAddr = (DPxIsViewpixx() || DPxIsPropixxCtrl() || DPxIsDatapixx2()) ? SPI_ADDR_VPX_ANALOG : SPI_ADDR_DPX_ANALOG;
	spiAddr = (DPxIsDatapixx3() ? SPI_ADDR_DP3_CALIB : spiAddr);

    DPxSpiErase(spiAddr, 88, NULL);
    DPxSpiWrite(spiAddr, 88, (char*)dataBuff, NULL);

	// Put DAC/ADC back into calibration mode,
	// and tell CALIB_CTRL to reload its calibration table from SPI.
	DPxDisableDacCalibRaw();
	DPxDisableAdcCalibRaw();
	DPxEnableCalibReload();
	DPxUpdateRegCache();
}


// Reload LED, DAC and ADC hardware calibration tables
void DPxEnableCalibReload()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;

	DPxSetReg16(DPXREG_CTRL, DPxGetReg16(DPXREG_CTRL) | DPXREG_CTRL_CALIB_RELOAD);
}


int DPxStringToInt(char* string)
{
	int i;
	int retVal;
	
	// Convert to lower case.  Helps with hex "0x", hex digits, and maybe exponential?
	for (i = 0; string[i]; i++)
		string[i] = tolower(string[i]);

	// We accept hex, and decimal strings
	if (!strchr(string, 'x'))
		return atoi(string);
	sscanf(string, "%x", &retVal);
	return retVal;
}



int DPxIsUsbTreeChanged()
{
	// Repopulate libusb's USB hierarchy
#ifdef USE_LIB01
	usb_find_busses();
	return usb_find_devices();
#else
	return 0;
#endif
}

// Scan USB device tree in search of a VPixx devices
void DPxUsbScan(int doPrint)
{
	struct stDeviceTable	tempDeviceTable[DPX_DEVSEL_MULTI] = { {0} };
	struct stDeviceTable	tempDeviceTable2 = { 0 };

#ifdef USE_LIB01
	struct usb_bus*			bus = NULL;
	struct usb_device*		dev = NULL;
#endif

	char customDevName[64];

	int	rc;
	int	ID;
	int	devcnt;
    int saveSysDevsel, saveUsrDevsel;
	int	iDev, iDevIndex;
	int	m, n, p;
	int spiAddr;

#ifndef USE_LIB01
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt;
	int i = 0;
	struct libusb_device *dev;
	struct libusb_device_descriptor libusb_desc;
	libusb_device_handle *device = NULL;
#endif

    // We are going to be accessing the USB device.
    // Some of these calls require dpxSysDevsel to be set, but we don't want to permanently modify this global.
    // We'll save/restore it.
    // Same goes for dpxUsrDevsel.
    saveSysDevsel = dpxSysDevsel;
    saveUsrDevsel = dpxUsrDevsel;

	// One-time initialization of libusb, important for linux.
	if (!dpxInitialized) {
#ifdef USE_LIB01
		usb_init();
#else
		libusb_init(NULL);
#endif
		dpxInitialized = 1;
	}

	//libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_DEBUG);

	// We'll always rescan from scratch, in order to catch recent connects/disconnects.
	// If we detect a DATAPixx during this scan, we'll use the newfound DATAPixx;
	// so we'll start this scan by closing any existing DATAPixx which we might already be using.
    // This also completely clears dpxDeviceTable.
	DPxClose();

	// Repopulate libusb's USB hierarchy
#ifdef USE_LIB01
	usb_find_busses();
	usb_find_devices();
#else
	// Issue for libusb1.0 where multiple instances of same device are enumerated incorrectly
	//  First get device list sometime generates an extra item
	libusb_get_device_list(NULL, &devs);
	cnt = libusb_get_device_list(NULL, &devs);
#endif

	//Reset the device counter
	for (iDev = DPX_DEVSEL_CNT_UNCONFIGURED; iDev <= DPX_DEVSEL_CNT_LAST; iDev++)
		devselCnt[iDev] = 0;

#ifdef USE_LIB01
	// and look for our devices
	for (bus = usb_busses; bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			if (dev->descriptor.idVendor == 0x04b4 && dev->descriptor.idProduct == 0x8613) {
                if (dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxRawUsb = 1;
					devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == DPX_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxIsDatapixx = 1;
					devselCnt[DPX_DEVSEL_CNT_DPX]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == VPX_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxIsViewpixx = 1;
					devselCnt[DPX_DEVSEL_CNT_VPX]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == PCX_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxIsPropixxCtrl = 1;
					devselCnt[DPX_DEVSEL_CNT_PPC]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == PPX_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxIsPropixx = 1;
					devselCnt[DPX_DEVSEL_CNT_PPX]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == DP2_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxIsDatapixx2 = 1;
					devselCnt[DPX_DEVSEL_CNT_DP2]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == TPC_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].DPxIsTrackpixxCtrl = 1;
					devselCnt[DPX_DEVSEL_CNT_TPC]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == TPB_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].DPxIsTrackpixxBridge = 1;
					devselCnt[DPX_DEVSEL_CNT_TPB]++;
                }
			}
			else if (dev->descriptor.idVendor == DPX_VID && dev->descriptor.idProduct == TPX_PID) {
                if (dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].dpxDev == 0) {
                    dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].dpxDev = dev;
                    dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].DPxIsTrackpixx = 1;
					devselCnt[DPX_DEVSEL_CNT_TPX]++;
                }
			}
		}
	}   
#else
	while ((dev = devs[i++]) != NULL)
	{
		struct libusb_device_descriptor desc;
		libusb_get_device_descriptor(dev, &desc);

		if (desc.idVendor == 0x04b4 && desc.idProduct == 0x00f3) {
            if (dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxRawUsb = 1;
				devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]++;
            }
		}
		else if (desc.idVendor == 0x04b4 && desc.idProduct == 0x8613) {
            if (dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_UNCONFIGURED + devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]].dpxRawUsb = 1;
				devselCnt[DPX_DEVSEL_CNT_UNCONFIGURED]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == DPX_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_DPX + devselCnt[DPX_DEVSEL_CNT_DPX]].dpxIsDatapixx = 1;
				devselCnt[DPX_DEVSEL_CNT_DPX]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == VPX_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_VPX + devselCnt[DPX_DEVSEL_CNT_VPX]].dpxIsViewpixx = 1;
				devselCnt[DPX_DEVSEL_CNT_VPX]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == PCX_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_PPC + devselCnt[DPX_DEVSEL_CNT_PPC]].dpxIsPropixxCtrl = 1;
				devselCnt[DPX_DEVSEL_CNT_PPC]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == PPX_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_PPX + devselCnt[DPX_DEVSEL_CNT_PPX]].dpxIsPropixx = 1;
				devselCnt[DPX_DEVSEL_CNT_PPX]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == DP2_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_DP2 + devselCnt[DPX_DEVSEL_CNT_DP2]].dpxIsDatapixx2 = 1;
				devselCnt[DPX_DEVSEL_CNT_DP2]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == TPC_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_TPC + devselCnt[DPX_DEVSEL_CNT_TPC]].DPxIsTrackpixxCtrl = 1;
				devselCnt[DPX_DEVSEL_CNT_TPC]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == TPB_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_TPB + devselCnt[DPX_DEVSEL_CNT_TPB]].DPxIsTrackpixxBridge = 1;
				devselCnt[DPX_DEVSEL_CNT_TPB]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == TPX_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_TPX + devselCnt[DPX_DEVSEL_CNT_TPX]].DPxIsTrackpixx = 1;
				devselCnt[DPX_DEVSEL_CNT_TPX]++;
            }
		}
		else if (desc.idVendor == DPX_VID && desc.idProduct == DP3_PID) {
            if (dpxDeviceTable[DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3]].dpxDev == 0) {
                dpxDeviceTable[DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3]].dpxDev = dev;
                dpxDeviceTable[DPX_DEVSEL_DP3 + devselCnt[DPX_DEVSEL_CNT_DP3]].dpxIsDatapixx3 = 1;
				devselCnt[DPX_DEVSEL_CNT_DP3]++;
            }
		}
	}
#endif

    // Open all of our USB devices
    for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {

        // No device of this class found
        if (!dpxDeviceTable[dpxSysDevsel].dpxDev)
            continue;

        // Setup an interface handle
#ifdef USE_LIB01
        dpxDeviceTable[dpxSysDevsel].dpxHdl = usb_open(dpxDeviceTable[dpxSysDevsel].dpxDev);
#else
		rc = libusb_open(dpxDeviceTable[dpxSysDevsel].dpxDev, &dpxDeviceTable[dpxSysDevsel].dpxHdl);
#endif
        if (!dpxDeviceTable[dpxSysDevsel].dpxHdl) {
            DPxDebugPrint1("ERROR: Could not open %s!\n", DPxGetDevselName());
            DPxSetError(DPX_ERR_USB_OPEN);
            memset(&dpxDeviceTable[dpxSysDevsel], 0, sizeof(dpxDeviceTable[dpxSysDevsel]));
            continue;
        }

        // Set USB configuration
#ifdef USE_LIB01
        rc = usb_set_configuration(dpxDeviceTable[dpxSysDevsel].dpxHdl, 1);
#else
		rc = libusb_set_configuration(dpxDeviceTable[dpxSysDevsel].dpxHdl, 1);
#endif
        if (rc < 0) {
            DPxDebugPrint2("ERROR: Could not set %s configuration [%d]!\n", DPxGetDevselName(), rc);
            DPxSetError(DPX_ERR_USB_SET_CONFIG);
#ifdef USE_LIB01
            usb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#else
			libusb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#endif
            memset(&dpxDeviceTable[dpxSysDevsel], 0, sizeof(dpxDeviceTable[dpxSysDevsel]));
            continue;
        }

        // Claim USB interface
#ifdef USE_LIB01
        rc = usb_claim_interface(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0);
#else
		rc = libusb_claim_interface(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0);
#endif
        if (rc < 0) {
            DPxDebugPrint2("ERROR: Could not claim %s interface [%d]!\n", DPxGetDevselName(), rc);
            DPxSetError(DPX_ERR_USB_CLAIM_INTERFACE);
#ifdef USE_LIB01
			usb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#else
			libusb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#endif
            memset(&dpxDeviceTable[dpxSysDevsel], 0, sizeof(dpxDeviceTable[dpxSysDevsel]));
            continue;
        }
        
        // Claim USB alternate interface
#ifdef USE_LIB01
		rc = usb_set_altinterface(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0);
#else
		rc = libusb_set_interface_alt_setting(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0, 0);
#endif
        if (rc < 0) {
            DPxDebugPrint2("ERROR: Could not set %s alternate interface [%d]!\n", DPxGetDevselName(), rc);
            DPxSetError(DPX_ERR_USB_ALT_INTERFACE);
#ifdef USE_LIB01
            usb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#else
            libusb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#endif
            memset(&dpxDeviceTable[dpxSysDevsel], 0, sizeof(dpxDeviceTable[dpxSysDevsel]));
            continue;
        }

        // Reset host controller toggle bit on Windows machines.
        // Seems this is not needed under OS X, but Windows doesn't seem to reset the toggle bit.
        // Each time vputil starts, one of the above routines (claim interface?) probably clears the toggle on the EZ,
        // but the toggle on the host side never gets cleared, except by the code below.
        // Without this, vputil has a 50% chance (for each EP) of loading with the correct toggle value.
        // Note that EP0 doesn't have a toggle (I think).
        // Sometimes vputil works when I don't include this code, but sometimes it doesn't.
        // Lately, testing with JF, multiple rr work, but then after entering a rrr command, then rr, FAIL.
        // So keep this code here.
#ifdef USE_LIB01
        usb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x01);
        usb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x81);
        usb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x02);
        usb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x86);       
#else
        libusb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x01);

		// There is an issue with libusb1.0 where the clear_halt will hang and cause a soft-error
		// It seems that this clear_halt is not required for the above toggle bit issue.
		//  libusb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x81);

        libusb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x02);
        libusb_clear_halt(dpxDeviceTable[dpxSysDevsel].dpxHdl, 0x86);       
#endif
        // If we found a raw USB device (no firmware), we have done as much as we can with it.
        if (dpxDeviceTable[dpxSysDevsel].dpxRawUsb)
            continue;

        // In the code below, we start making calls into API, like DPxUpdateRegCache().
        // For their calls to DPxSelectSysDevsel() to work, we have to set dpxUsrDevsel.
        dpxUsrDevsel = dpxSysDevsel;
        
        // We found VPixx hardware, and its EZ-USB has a valid firmware.
        // If this firmware was downloaded from host, it's still possible that the FPGA has not been configured.
        // We'll just ask the EZ-USB if the FPGA has pulled down INT1 to indicate its healthy presence.
        // *** Windows chokes on this.  JF's machine returns errors.  Maybe Mario had this error as well.
        // Use conditional compile to remove this under Windows.
        // Note that my Windows Matlab mex build defines _MSC_VER, but the Octave build doesn't,
        // so I'll also define my own person WIN_BUILD when making mex files.
#if !TARGET_WINDOWS
        rc = EZReadSFR(EZ_SFR_IOA);
        if (rc < 0) {
            fprintf(stderr, "ERROR: DPxUsbScan() EZReadSFR() failed with error 0x%X\n", rc);
            DPxSetError(DPX_ERR_USB_OPEN_FPGA);
            continue;
        }

        // FPGA pulls down INT1 when it is running OK.  Otherwise, pullup means raw FPGA
        if (rc & 2) {
            DPxSetError(DPX_ERR_USB_RAW_FPGA);
//zzz            printf("Found raw FPGA\n");//zzz
            continue;
        }
//        else
//            printf("Found cooked FPGA\n");//zzz
#endif

        // Now that we know the FPGA is there, we'll try a register read.
        // We also absolutely have to initialize our cache with the real register values.
        // Remember that the registers could have been written by some other app.
        // We don't want our caller doing even one register write until our cache is valid.
        DPxUpdateRegCache();
        if (DPxGetError() != DPX_SUCCESS) {
            DPxSetError(DPX_ERR_USB_RAW_FPGA);
            continue;
        }

		ID = DPxGetID();

        // And finally, make sure we recognize the ID register.
        if (ID != DPXREG_DPID_DP && ID != DPXREG_DPID_VP && ID != DPXREG_DPID_PP && ID != DPXREG_DPID_PC && 
			ID != DPXREG_DPID_PC && ID != DPXREG_DPID_D2 && ID != DPXREG_DPID_TP && ID != DPXREG_DPID_TC && ID != DPXREG_DPID_TB &&
			ID != DPXREG_DPID_D3)
            DPxSetError(DPX_ERR_USB_UNKNOWN_DPID);

        dpxDeviceTable[dpxSysDevsel].dpxGoodFpga = 1;

		if (DPxIsS3Arch())
			spiAddr = SPI_ADDR_VPX_NAME;
		else if (DPxIsA5Arch())
			spiAddr = SPI_ADDR_TPB_NAME;
		else if (DPxIsA10Arch())
			spiAddr = SPI_ADDR_DP3_NAME;
		else
			spiAddr = SPI_ADDR_DPX_NAME;

		DPxSpiRead(spiAddr, 64, customDevName, NULL);
		//printf("\n%s", customDevName);
		customDevName[63] = 0;
		strcpy(dpxDeviceTable[dpxSysDevsel].dpxDeviceName, customDevName);
		dpxDeviceTable[dpxSysDevsel].dpxDeviceName[63] = 0;	// Make sure string returned from SPI is 0 terminated.
    }

	//reorder dev here
	for (iDev= DPX_DEVSEL_CNT_UNCONFIGURED; iDev <= DPX_DEVSEL_CNT_LAST; iDev++) {
		
		// reorder only when it's necessary
		if (devselCnt[iDev] > 1) {

			for (iDevIndex = 0; iDevIndex < devselCnt[iDev]; iDevIndex++) {
				tempDeviceTable[iDevIndex] = dpxDeviceTable[iDev*DPX_DEVSEL_MULTI+iDevIndex];
			}		

			for (m = 1; m < devselCnt[iDev]; m++) {
				for (n = 0; n < devselCnt[iDev]-m; n++) {
				
					p = strcmp(tempDeviceTable[n].dpxDeviceName, tempDeviceTable[n+1].dpxDeviceName);
				
					if (p > 0) {
						tempDeviceTable2     = tempDeviceTable[n];
						tempDeviceTable[n]   = tempDeviceTable[n+1];
						tempDeviceTable[n+1] = tempDeviceTable2;
					}
				}
			}

			for (iDevIndex = 0; iDevIndex < devselCnt[iDev]; iDevIndex++) {
			   dpxDeviceTable[iDev*DPX_DEVSEL_MULTI+iDevIndex] = tempDeviceTable[iDevIndex];
			}
		}
	}	    


	// Tell the user which devices we found.
    // It's nice that we do this in device order, instead of random USB order.
	devcnt = 0;
	if (doPrint) {
		printf("\n");
		printf("Scan of VPixx USB devices:\n");
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++){
            if (dpxDeviceTable[dpxSysDevsel].dpxDev) {
#ifdef USE_LIB01
                printf("Vendor ID = 0x%04X, Product ID = 0x%04X     %s", dpxDeviceTable[dpxSysDevsel].dpxDev->descriptor.idVendor,
                                                                         dpxDeviceTable[dpxSysDevsel].dpxDev->descriptor.idProduct,
                                                                         DPxGetDevselName());
#else
				struct libusb_device_descriptor desc;
				libusb_get_device_descriptor(dpxDeviceTable[dpxSysDevsel].dpxDev, &desc);
                printf("Vendor ID = 0x%04X, Product ID = 0x%04X     %s", desc.idVendor,
																		 desc.idProduct,
                                                                         DPxGetDevselName());
#endif
				strcpy(customDevName, DPxGetCustomDevName());
				if (customDevName[0] != '\0' && customDevName[0] != -1) // -1 = 0xFF means no name
					printf(" \"%s\"", customDevName);

				printf("\n");
				devcnt++;
			}
			fflush(stdout);
        }
		
		if (devcnt == 0) {
			printf("*** No VPixx USB device found! ***\n");
		}
	}

	dpxSysDevsel = saveSysDevsel;
    dpxUsrDevsel = saveUsrDevsel;

#ifndef USE_LIB01
	libusb_free_device_list(devs, 1);
#endif
}


/********************************************************************************/
/*																				*/
/*	Here we start to define the interface which is presented in libdpx.h	*/
/*																				*/
/********************************************************************************/


void DPxSetDebug(int level)
{
	// Don't look for ARG errors here, since DPxSetDebug() is used in macros that test for other ARG errors.
	// For the same reason, don't call DPxSetError() and change dpxError.
	dpxDebugLevel = level;

	// Don't print info here.  I'm changing the debug level many times as I test DPX_ARG_ERROR for API functions
//	if (dpDebugLevel || level)
//		DPxDebugPrint1("DPxSetDebug: Setting debugging level to %d\n", level);

	// Set libusb level to one less than ours, so that dp_api messages kick in before libusb messages.
#ifdef USE_LIB01
	usb_set_debug(level > 0 ? level - 1 : 0);
#else
	libusb_set_debug(NULL, level > 0 ? level - 1 : 0);
#endif
}


int DPxGetDebug()
{
	// DPxGetDebug() is used in error macros, so don't call DPxSetError() and change dpxError.
	return dpxDebugLevel;
}


void DPxSetError(int error)
{
	dpxError = error;
}


void DPxClearError()
{
	dpxError = DPX_SUCCESS;
}


// DPxGetError() does not clear dpError.
// User might want to implement an exception-type mechanism, in which error propagates.
int DPxGetError()
{
	return dpxError;
}


const char* DPxGetErrorString()
{
    switch (dpxError) {
        case DPX_SUCCESS						 : return "Function executed successfully";
        case DPX_FAIL							 : return "Generic failure code";
        case DPX_ERR_USB_NO_DATAPIXX			 : return "No VPixx Device was found";
        case DPX_ERR_USB_RAW_EZUSB				 : return "EZ-USB appears to have no firmware";
        case DPX_ERR_USB_RAW_FPGA				 : return "FPGA appears to be unconfigured";
        case DPX_ERR_USB_OPEN					 : return "An error occurred while opening a USB channel";
        case DPX_ERR_USB_OPEN_FPGA				 : return "An FPGA detection error occurred while opening DATAPixx";
        case DPX_ERR_USB_SET_CONFIG				 : return "Could not set the USB configuration.  Device already open in another process?";
        case DPX_ERR_USB_CLAIM_INTERFACE		 : return "Could not claim the USB interface";
        case DPX_ERR_USB_ALT_INTERFACE			 : return "Could not set the USB alternate interface";
        case DPX_ERR_USB_UNKNOWN_DPID			 : return "Unrecognized DATAPixx ID register value";
        case DPX_ERR_USB_REG_BULK_WRITE			 : return "USB error while writing register set";
        case DPX_ERR_USB_REG_BULK_READ			 : return "USB error while reading register set";
        case DPX_ERR_USB_DEVSEL_INDEX			 : return "Illegal device index";
        case DPX_ERR_USB_SYSDEVSEL_INDEX		 : return "Illegal system device index";
        case DPX_ERR_SPI_START					 : return "SPI communication startup error";
        case DPX_ERR_SPI_STOP					 : return "SPI communication termination error";
        case DPX_ERR_SPI_READ					 : return "SPI communication read error";
        case DPX_ERR_SPI_WRITE					 : return "SPI communication write error";
        case DPX_ERR_SPI_ERASE					 : return "SPI communication erase error";
        case DPX_ERR_SPI_WAIT_DONE				 : return "SPI communication error while waiting for SPI write to complete";
        case DPX_ERR_SETREG16_ADDR_ODD			 : return "DPxSetReg16 passed an odd address";
        case DPX_ERR_SETREG16_ADDR_RANGE		 : return "DPxSetReg16 passed an address which was out of range";
        case DPX_ERR_SETREG16_DATA_RANGE		 : return "DPxSetReg16 passed a datum which was out of range";
        case DPX_ERR_GETREG16_ADDR_ODD			 : return "DPxGetReg16 passed an odd address";
        case DPX_ERR_GETREG16_ADDR_RANGE		 : return "DPxGetReg16 passed an address which was out of range";
        case DPX_ERR_SETREG32_ADDR_ALIGN		 : return "DPxSetReg32 passed an address which was not 32-bit aligned";
        case DPX_ERR_SETREG32_ADDR_RANGE		 : return "DPxSetReg32 passed an address which was out of range";
        case DPX_ERR_GETREG32_ADDR_ALIGN		 : return "DPxGetReg32 passed an address which was not 32-bit aligned";
        case DPX_ERR_GETREG32_ADDR_RANGE		 : return "DPxGetReg32 passed an address which was out of range";
        case DPX_ERR_NANO_TIME_NULL_PTR			 : return "A pointer argument was null";
        case DPX_ERR_NANO_MARK_NULL_PTR			 : return "A pointer argument was null";
        case DPX_ERR_UNKNOWN_PART_NUMBER		 : return "Unrecognized part number";
        case DPX_ERR_RAM_UNKNOWN_SIZE			 : return "Unrecognized RAM configuration";
        case DPX_ERR_RAM_WRITEREAD_FAIL			 : return "RAM read did not return same value written";
        case DPX_ERR_RAM_WRITE_ADDR_ODD			 : return "RAM write buffer address must be even";
        case DPX_ERR_RAM_WRITE_LEN_ODD			 : return "RAM write buffer length must be even";
        case DPX_ERR_RAM_WRITE_TOO_HIGH			 : return "RAM write block exceeds end of DATAPixx memory";
        case DPX_ERR_RAM_WRITE_BUFFER_NULL		 : return "RAM write source buffer pointer is null";
        case DPX_ERR_RAM_WRITE_USB_ERROR		 : return "A USB error occurred while writing the RAM buffer";
        case DPX_ERR_RAM_READ_ADDR_ODD			 : return "RAM read buffer address must be even";
        case DPX_ERR_RAM_READ_LEN_ODD			 : return "RAM read buffer length must be even";
        case DPX_ERR_RAM_READ_TOO_HIGH			 : return "RAM read block exceeds end of DATAPixx memory";
        case DPX_ERR_RAM_READ_BUFFER_NULL		 : return "RAM read destination buffer pointer is null";
        case DPX_ERR_RAM_READ_USB_ERROR			 : return "A USB error occurred while reading the RAM buffer";
        case DPX_ERR_DAC_SET_BAD_CHANNEL		 : return "Valid channels are 0-3";
        case DPX_ERR_DAC_SET_BAD_VALUE			 : return "Value falls outside DAC's output range";
        case DPX_ERR_DAC_GET_BAD_CHANNEL		 : return "Valid channels are 0-3";
        case DPX_ERR_DAC_RANGE_NULL_PTR			 : return "A pointer argument was null";
        case DPX_ERR_DAC_RANGE_BAD_CHANNEL		 : return "Valid channels are 0-3";
        case DPX_ERR_DAC_BUFF_BAD_CHANNEL		 : return "Valid channels are 0-3";
        case DPX_ERR_DAC_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_DAC_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DAC_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_DAC_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_DAC_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_DAC_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_DAC_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_DAC_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DAC_SCHED_TOO_FAST			 : return "The requested schedule rate is too fast";
        case DPX_ERR_DAC_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_ADC_GET_BAD_CHANNEL		 : return "Valid channels are 0-17";
        case DPX_ERR_ADC_RANGE_NULL_PTR			 : return "A pointer argument was null";
        case DPX_ERR_ADC_RANGE_BAD_CHANNEL		 : return "Valid channels are 0-17";
        case DPX_ERR_ADC_REF_BAD_CHANNEL		 : return "Valid channels are 0-15";
        case DPX_ERR_ADC_BAD_CHAN_REF			 : return "Unrecognized channel reference parameter";
        case DPX_ERR_ADC_BUFF_BAD_CHANNEL		 : return "Valid channels are 0-15";
        case DPX_ERR_ADC_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_ADC_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_ADC_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_ADC_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_ADC_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_ADC_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_ADC_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_ADC_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_ADC_SCHED_TOO_FAST			 : return "The requested schedule rate is too fast";
        case DPX_ERR_ADC_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_DOUT_SET_BAD_MASK			 : return "Valid masks set bits 23 downto 0";
        case DPX_ERR_DOUT_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_DOUT_BUFF_BASEADDR_TOO_HIGH : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DOUT_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_DOUT_BUFF_READADDR_TOO_HIGH : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_DOUT_BUFF_ODD_WRITEADDR	 : return "An odd buffer write address was requested";
        case DPX_ERR_DOUT_BUFF_WRITEADDR_TOO_HIGH: return ":The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_DOUT_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_DOUT_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DOUT_SCHED_TOO_FAST		 : return "The requested schedule rate is too fast";
        case DPX_ERR_DOUT_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_DIN_SET_BAD_MASK			 : return "Valid masks set bits 23 downto 0";
        case DPX_ERR_DIN_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_DIN_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DIN_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_DIN_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_DIN_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_DIN_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_DIN_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_DIN_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_DIN_SCHED_TOO_FAST			 : return "The requested schedule rate is too fast";
        case DPX_ERR_DIN_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_DIN_BAD_STRENGTH			 : return "Strength is in the range 0-1";
        case DPX_ERR_AUD_SET_BAD_VALUE			 : return "Value falls outside AUD's output range";
        case DPX_ERR_AUD_SET_BAD_VOLUME			 : return "Valid volumes are in the range 0-1";
        case DPX_ERR_AUD_SET_BAD_LRMODE			 : return "See DPxSetAudLRMode() for valid values";
        case DPX_ERR_AUD_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_AUD_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_AUD_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_AUD_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_AUD_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_AUD_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_AUD_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_AUD_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_AUX_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_AUX_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_AUX_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_AUX_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_AUX_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_AUX_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_AUX_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_AUX_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_AUD_SCHED_TOO_FAST			 : return "The requested schedule rate is too fast";
        case DPX_ERR_AUD_SCHED_TOO_SLOW			 : return "The requested schedule rate is too slow";
        case DPX_ERR_AUD_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_AUD_CODEC_POWERUP			 : return "The CODEC didn't set its internal powerup bits.";
        case DPX_ERR_MIC_SET_GAIN_TOO_LOW		 : return "See DPxSetMicSource() for valid values";
        case DPX_ERR_MIC_SET_GAIN_TOO_HIGH		 : return "See DPxSetMicSource() for valid values";
        case DPX_ERR_MIC_SET_BAD_SOURCE			 : return "See DPxSetMicSource() for valid values";
        case DPX_ERR_MIC_SET_BAD_LRMODE			 : return "See DPxSetMicLRMode() for valid values";
        case DPX_ERR_MIC_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
        case DPX_ERR_MIC_BUFF_BASEADDR_TOO_HIGH	 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_MIC_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
        case DPX_ERR_MIC_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the DATAPixx RAM";
        case DPX_ERR_MIC_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
        case DPX_ERR_MIC_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the DATAPixx RAM";
        case DPX_ERR_MIC_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
        case DPX_ERR_MIC_BUFF_TOO_BIG			 : return "The requested buffer is larger than the DATAPixx RAM";
        case DPX_ERR_MIC_SCHED_TOO_FAST			 : return "The requested schedule rate is too fast";
        case DPX_ERR_MIC_SCHED_BAD_RATE_UNITS	 : return "Unnrecognized schedule rate units parameter";
        case DPX_ERR_VID_SET_BAD_MODE			 : return "See DPxSetVidMode() for valid values";
        case DPX_ERR_VID_CLUT_WRITE_USB_ERROR	 : return "A USB error occurred while writing a video CLUT";
        case DPX_ERR_VID_VSYNC_USB_ERROR		 : return "A USB error occurred while waiting for vertical sync";
        case DPX_ERR_VID_EDID_WRITE_USB_ERROR	 : return "A USB error occurred while writing EDID data";
        case DPX_ERR_VID_LINE_READ_USB_ERROR	 : return "A USB error occurred while reading the video line buffer";
        case DPX_ERR_VID_PSYNC_NPIXELS_ARG_ERROR : return "Pixel sync nPixels argument must be in the range 1-8";
        case DPX_ERR_VID_PSYNC_TIMEOUT_ARG_ERROR : return "Pixel sync timeout argument must be in the range 0-65535";
        case DPX_ERR_VID_PSYNC_LINE_ARG_ERROR	 : return "Pixel sync raster line argument must be in the range 0-4095";
        case DPX_ERR_VID_ALPHA_WRITE_USB_ERROR	 : return "A USB error occurred while writing video horizontal overlay alpha data";
        case DPX_ERR_VID_BASEADDR_ALIGN_ERROR	 : return "The requested base address was not aligned on a 64kB boundary";
        case DPX_ERR_VID_BASEADDR_TOO_HIGH       : return "The requested base address exceeds the DATAPixx RAM";
        case DPX_ERR_VID_VSYNC_WITHOUT_VIDEO     : return "The API was told to block until VSYNC; but DATAPixx is not receiving any video";
        case DPX_ERR_VID_BL_INTENSITY_ARG_ERROR  : return "Backlight intensity argument must be in the range 0 to 255";
        case DPX_ERR_PPX_BAD_VOLTAGE             : return "Unnrecognized voltage monitor. Argument is not in the range 0 to 6";
        case DPX_ERR_PPX_BAD_TEMP                : return "Unnrecognized temperature monitor. Argument is not in the range 0 to 9";
        case DPX_ERR_PPX_BAD_LED                 : return "Unnrecognized LED. Argument is not in the range 0 to 7";
        case DPX_ERR_PPX_BAD_FAN                 : return "Unnrecognized fan. Argument is not in the range 0 to 5";
        case DPX_ERR_PPX_BAD_LED_CURRENT         : return "Invalid LED current. Argument is not in the range 0 to 50";
        case DPX_ERR_PPX_SEQ_WRITE_USB_ERROR     : return "A USB error occurred while writing a video sequence";
		case DPX_ERR_TRK_BUFF_ODD_BASEADDR		 : return "An odd buffer base was requested";
		case DPX_ERR_TRK_BUFF_BASEADDR_TOO_LOW   : return "The requested buffer address is out of range";
		case DPX_ERR_TRK_BUFF_BASEADDR_TOO_HIGH  : return "The requested buffer is larger than the TRACKPixx RAM";
		case DPX_ERR_TRK_BUFF_ODD_READADDR		 : return "An odd buffer read address was requested";
		case DPX_ERR_TRK_BUFF_READADDR_TOO_HIGH	 : return "The requested read address exceeds the TRACKPixx RAM";
		case DPX_ERR_TRK_BUFF_ODD_WRITEADDR		 : return "An odd buffer write address was requested";
		case DPX_ERR_TRK_BUFF_WRITEADDR_TOO_HIGH : return "The requested write address exceeds the TRACKPixx RAM";
		case DPX_ERR_TRK_BUFF_ODD_SIZE			 : return "An odd buffer size was requested";
		case DPX_ERR_TRK_BUFF_TOO_BIG			 : return "The requested buffer is larger than the TRACKPixx RAM";
    }
    
    return "";
}


// Get number of USB retries/fails for each endpoint and direction
int DPxGetEp1WrRetries()
{
	return dpxEp1WrRetries;
}
int DPxGetEp1RdRetries()
{
	return dpxEp1RdRetries;
}
int DPxGetEp2WrRetries()
{
	return dpxEp2WrRetries;
}
int DPxGetEp6RdRetries()
{
	return dpxEp6RdRetries;
}
int DPxGetEp1WrFails()
{
	return dpxEp1WrFails;
}
int DPxGetEp1RdFails()
{
	return dpxEp1RdFails;
}
int DPxGetEp2WrFails()
{
	return dpxEp2WrFails;
}
int DPxGetEp6RdFails()
{
	return dpxEp6RdFails;
}


// Concatenate two unsigned 32-bit numbers and return as a 64-bit floating point number.
// Note that there may be some loss of precision, as an IEEE 64-bit float only has a 53 bit mantissa.
double DPxMakeFloat64FromTwoUInt32(UInt32 highUInt32, UInt32 lowUInt32)
{
	return (4294967296.0 * highUInt32 + lowUInt32);		// Use 2^32 constant.  Accuracy of pow(2,32) might be platform-dependant.
}


// Call before any other DPx*() functions
void DPxOpen()
{
	int err;

	// One-time initialization of libusb, important for linux.
	if (!dpxInitialized) {
#ifdef USE_LIB01
		usb_init();
#else
		libusb_init(NULL);
#endif
		dpxInitialized = 1;
	}

	// Open connections with any VPixx USB devices
	DPxUsbScan(0);

    // Try to set a reasonable initial device
    DPxSelectSysDevice(DPX_DEVSEL_ANY);

	// Set errors if there are no USB devices, or if selected device is raw
    if (!(dpxSysDevsel >= DPX_DEVSEL_FIRST_DEVICE && dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE))
		DPxSetError(DPX_ERR_USB_NO_DATAPIXX);
	else if (dpxDeviceTable[dpxSysDevsel].dpxRawUsb)
		DPxSetError(DPX_ERR_USB_RAW_EZUSB);
	
    // We can stop here if we don't even have a device to work with
    err = DPxGetError();
	if (err != DPX_SUCCESS) {
		if (err != DPX_ERR_USB_RAW_EZUSB)
			DPxDebugPrint1("Fail: [DPxOpen] failed with error %d\n", err);
		return;
	}
}


// Call when finished with VPixx devices.
// Should be OK to call DPxOpen() / DPxClose() multiple times within application.
void DPxClose()
{
    for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
        
        // Just skip if device was never successfully opened
        if (!dpxDeviceTable[dpxSysDevsel].dpxHdl)
            continue;

        // Under Windows, EP1 sometimes has a USB data toggle bit problem.
        // If the USB port is closed after an odd number of 64B EP1 packets, then the next EP1 transaction fails with a toggle bit error.
        // The only way I've found to fix this is to always I/O an even number of 64B USB packets before calling usb_close().
        // We'll just always I/O an even number of 64B packets, by appending a single 64B packet if necessary.
        // Special function register for port E is innocuous enough.  Writing an output enable of 0 is the default hard reset value.  Leaves JTAG tristate.
        // Of course, if we are closing because we just told the EZ-USB to initiate a hardware reset, then don't try to do any USB accesses.
        if (!gDoingHardwareReset) {
            if (dpxDeviceTable[dpxSysDevsel].nEP1Reads & 1)
                EZReadSFR(EZ_SFR_OEE);      // This has to come first, since it does both a write and a read
            if (dpxDeviceTable[dpxSysDevsel].nEP1Writes & 1)
                EZWriteSFR(EZ_SFR_OEE, 0);
        }
        dpxDeviceTable[dpxSysDevsel].nEP1Reads = 0;
        dpxDeviceTable[dpxSysDevsel].nEP1Writes = 0;

        // Note that usb_close() takes care of calling usb_release_interface(),
        // so there's no more cleanup required here.
#ifdef USE_LIB01
        usb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#else
		libusb_close(dpxDeviceTable[dpxSysDevsel].dpxHdl);
#endif
    }

    // Clear entire table in 1 fell swoop
    memset(dpxDeviceTable, 0, sizeof(dpxDeviceTable));
    
    // Leave with an invalid devsel to catch any illegal accesses later
    dpxSysDevsel = DPX_DEVSEL_INVALID;
}


// Non-0 if DPxOpen() succeeded, and found a DATAPixx with firmware, and a well-configured FPGA
int DPxIsReady()
{
    // It's possible that a previous call to an illegal function for a given device left dpxSysDevsel at DPX_DEVSEL_INVALID.
    // I saw this in PTB when setup code set vertical stereo to 0 on a PROPixx.
    // Just in case that happened, we'll restore dpxSysDevsel to any valid device here.
    DPxSelectSysDevice(DPX_DEVSEL_ANY);

    // If device selection succeeded, return true
    if (dpxSysDevsel >= DPX_DEVSEL_FIRST_DEVICE && dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE  && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga)
        return 1;

    // If device selection failed, we return 0.
    // Note that DPxSelectSysDevice() probably just set a global error.
    // We don't want this error to propagate.
    // It is a valid operation to call DPxIsReady() when no device is open.
    // We'll just clear the error.
    DPxClearError();
    return 0;
}


// TPxGetImagePtr:	Allocates memory for the image data to be
//					stored in ram and return a char pointing to it.
//@return:			char* ptr, a pointer pointing to the image data
//					NULL, if problem with malloc of ram reading
unsigned char* TPxGetImagePtr()
{
	unsigned short blockLength;
	unsigned address;
	unsigned length;
	unsigned char *buffPtr = (unsigned char*)malloc(1280*1024*sizeof(char));

	address = 0;
	length = 1280*1024;	

    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return NULL;

	// Break into largest supported tram chunks
	while (length) {
		if (length > DPX_RWRAM_BLOCK_SIZE)
			blockLength = DPX_RWRAM_BLOCK_SIZE;
		else
			blockLength = (unsigned short)length;
		ep2out_Tram[0] = '^';
		ep2out_Tram[1] = EP2OUT_READRAM;
		ep2out_Tram[2] = 6;
		ep2out_Tram[3] = 0;
		ep2out_Tram[4] = (address >>  0) & 0xFF;
		ep2out_Tram[5] = (address >>  8) & 0xFF;
		ep2out_Tram[6] = (address >> 16) & 0xFF;
		ep2out_Tram[7] = (address >> 24) & 0xFF;
		ep2out_Tram[8] = LSB(blockLength);
		ep2out_Tram[9] = MSB(blockLength);
		if (EZWriteEP2Tram(ep2out_Tram, EP6IN_READRAM, blockLength)) {
			DPxDebugPrint0("ERROR: DPxReadRam() call to EZWriteEP2Tram() failed\n");
			DPxSetError(DPX_ERR_RAM_READ_USB_ERROR);
			return NULL;
		}

		if ((void*)buffPtr != (void*)(ep6in_Tram+4))	// Users are allowed to read directly from ep6in_Tram to save memcpy
			memcpy(buffPtr, ep6in_Tram+4, blockLength);
		address += blockLength;
		buffPtr += blockLength;
		length  -= blockLength;
	}

	return buffPtr;
}


// Read a block of VPixx device RAM into a local buffer
void DPxReadRam(unsigned address, unsigned length, void* buffer)
{
	unsigned short blockLength;
	char* buffPtr = (char*)buffer;

    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;

	// Validate args
	if (address & 1) {
		DPxDebugPrint1("ERROR: DPxReadRam() argument address 0x%x is not an even number\n", address);
		DPxSetError(DPX_ERR_RAM_READ_ADDR_ODD);
		return;
	}
	if (length & 1) {
		DPxDebugPrint1("ERROR: DPxReadRam() argument length 0x%x is not an even number\n", length);
		DPxSetError(DPX_ERR_RAM_READ_LEN_ODD);
		return;
	}
	if (address + length > DPxGetRamSize()) {
		DPxDebugPrint2("ERROR: DPxReadRam() argument address 0x%x plus length 0x%x exceeds DATAPixx memory size\n", address, length);
		DPxSetError(DPX_ERR_RAM_READ_TOO_HIGH);
		return;
	}
	if (!buffer) {
		DPxDebugPrint0("ERROR: DPxReadRam() argument buffer address is null\n");
		DPxSetError(DPX_ERR_RAM_READ_BUFFER_NULL);
		return;
	}

	// Break into largest supported tram chunks
	while (length) {
		if (length > DPX_RWRAM_BLOCK_SIZE)
			blockLength = DPX_RWRAM_BLOCK_SIZE;
		else
			blockLength = (unsigned short)length;
		ep2out_Tram[0] = '^';
		ep2out_Tram[1] = EP2OUT_READRAM;
		
		if (DPxIsA10Arch())
			ep2out_Tram[2] = 8;
		else
			ep2out_Tram[2] = 6;

		ep2out_Tram[3] = 0;
		ep2out_Tram[4] = (address >>  0) & 0xFF;
		ep2out_Tram[5] = (address >>  8) & 0xFF;
		ep2out_Tram[6] = (address >> 16) & 0xFF;
		ep2out_Tram[7] = (address >> 24) & 0xFF;
		ep2out_Tram[8] = LSB(blockLength);
		ep2out_Tram[9] = MSB(blockLength);

		if (DPxIsA10Arch())
		{
			ep2out_Tram[10] = 0;
			ep2out_Tram[11] = 0;
		}

		if (EZWriteEP2Tram(ep2out_Tram, EP6IN_READRAM, blockLength)) {
			DPxDebugPrint0("ERROR: DPxReadRam() call to EZWriteEP2Tram() failed\n");
			DPxSetError(DPX_ERR_RAM_READ_USB_ERROR);
			return;
		}

		if ((void*)buffPtr != (void*)(ep6in_Tram+4))	// Users are allowed to read directly from ep6in_Tram to save memcpy
			memcpy(buffPtr, ep6in_Tram+4, blockLength);
		address += blockLength;
		buffPtr += blockLength;
		length  -= blockLength;
	}
}


// Write a local buffer to DATAPixx RAM
void DPxWriteRam(unsigned address, unsigned length, void* buffer)
{    
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
    DPxWriteRamNoDevsel(address, length, buffer);
}


// Write a local buffer to DATAPixx RAM
void DPxWriteRamNoDevsel(unsigned address, unsigned length, void* buffer)
{
	unsigned short blockLength, payloadLength;
	char* buffPtr = (char*)buffer;

	// Validate args
	if (address & 1) {
		DPxDebugPrint1("ERROR: DPxWriteRam() argument address 0x%x is not an even number\n", address);
		DPxSetError(DPX_ERR_RAM_WRITE_ADDR_ODD);
		return;
	}
	if (length & 1) {
		DPxDebugPrint1("ERROR: DPxWriteRam() argument length 0x%x is not an even number\n", length);
		DPxSetError(DPX_ERR_RAM_WRITE_LEN_ODD);
		return;
	}
	if (address + length > DPxGetRamSizeNoDevsel()) {
		DPxDebugPrint2("ERROR: DPxWriteRam() argument address 0x%x plus length 0x%x exceeds DATAPixx memory size\n", address, length);
		DPxSetError(DPX_ERR_RAM_WRITE_TOO_HIGH);
		return;
	}
	if (!buffer) {
		DPxDebugPrint0("ERROR: DPxWriteRam() argument buffer address is null\n");
		DPxSetError(DPX_ERR_RAM_WRITE_BUFFER_NULL);
		return;
	}

	// Break into largest supported tram chunks
	while (length) {
		if (length > DPX_RWRAM_BLOCK_SIZE)
			blockLength = DPX_RWRAM_BLOCK_SIZE;
		else
			blockLength = (unsigned short)length;
		payloadLength = blockLength + 4;
		ep2out_Tram[0] = '^';
		ep2out_Tram[1] = EP2OUT_WRITERAM;
		ep2out_Tram[2] = LSB(payloadLength);
		ep2out_Tram[3] = MSB(payloadLength);
		ep2out_Tram[4] = (address >>  0) & 0xFF;
		ep2out_Tram[5] = (address >>  8) & 0xFF;
		ep2out_Tram[6] = (address >> 16) & 0xFF;
		ep2out_Tram[7] = (address >> 24) & 0xFF;
		if ((void*)(ep2out_Tram+8) != (void*)buffPtr)		// Users are allowed to write directly into ep2out_Tram to save memcpy
			memcpy(ep2out_Tram+8, buffPtr, blockLength);
		if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
			DPxDebugPrint0("ERROR: DPxWriteRam() call to EZWriteEP2Tram() failed\n");
			DPxSetError(DPX_ERR_RAM_WRITE_USB_ERROR);
			return;
		}

		address += blockLength;
		buffPtr += blockLength;
		length  -= blockLength;
	}
}


// Address of API internal read RAM buffer.
// Caller can access this buffer directly to avoid an extra memcpy when reading Datapixx RAM.
void* DPxGetReadRamBuffAddr()
{
	return (void*)((char*)ep6in_Tram + 4);
}


// Number of bytes in internal read RAM buffer.
// Users of the buffer returned by DPxGetReadRamBuffAddr() must limit their transaction size to this value.
int DPxGetReadRamBuffSize()
{
	return DPX_RWRAM_BLOCK_SIZE;
}


// Address of API internal write RAM buffer.
// Caller can access this buffer directly to avoid an extra memcpy when writing Datapixx RAM.
void* DPxGetWriteRamBuffAddr()
{
	return (void*)((char*)ep2out_Tram + 8);
}


// Number of bytes in internal write RAM buffer.
// Users of the buffer returned by DPxGetWriteRamBuffAddr() must limit their transaction size to this value.
int DPxGetWriteRamBuffSize()
{
	return DPX_RWRAM_BLOCK_SIZE;
}


// Set a 16-bit register's value in dpxRegisterCache[]
void DPxSetReg16(int regAddr, int regValue)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	if (regAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetReg16() argument register address 0x%x is not an even number\n", regAddr);
		DPxSetError(DPX_ERR_SETREG16_ADDR_ODD);
		return;
	}
	if (regAddr < 0 || regAddr >= (int)DPxGetRegisterSpaceSizeNoDevsel()) {
		DPxDebugPrint2("ERROR: DPxSetReg16() argument register address 0x%x is not in range 0 to 0x%X\n", regAddr, DPxGetRegisterSpaceSizeNoDevsel()-2);
		DPxSetError(DPX_ERR_SETREG16_ADDR_RANGE);
		return;
	}
	if (regValue < -32768 || regValue > 65535) {
		DPxDebugPrint1("ERROR: DPxSetReg16() argument register value 0x%x is out of range\n", regValue);
		DPxSetError(DPX_ERR_SETREG16_DATA_RANGE);
		return;
	}

	dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2] = regValue;
	dpxDeviceTable[dpxSysDevsel].dpxRegisterModified[regAddr/2] = 1;
}


// Read a 16-bit register's value from dpxRegisterCache[].
// Note that this returns an _unsigned_ 16-bit value.
int DPxGetReg16(int regAddr)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	if (regAddr & 1) {
		DPxDebugPrint1("ERROR: DPxGetReg16() argument register address 0x%x is not an even number\n", regAddr);
		DPxSetError(DPX_ERR_GETREG16_ADDR_ODD);
		return 0;
	}
	if (regAddr < 0 || regAddr >= (int)DPxGetRegisterSpaceSizeNoDevsel()) {
		DPxDebugPrint2("ERROR: DPxGetReg16() argument register address 0x%x is not in range 0 to 0x%X\n", regAddr, DPxGetRegisterSpaceSizeNoDevsel()-2);
		DPxSetError(DPX_ERR_GETREG16_ADDR_RANGE);
		return 0;
	}
	return dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2];
}


// Set a 32-bit register's value in dpxRegisterCache[]
// Assumes 32-bit registers have 32-bit address alignment.
void DPxSetReg32(int regAddr, unsigned regValue)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	if (regAddr & 3) {
		DPxDebugPrint1("ERROR: DPxSetReg32() argument register address 0x%x is not 32-bit aligned\n", regAddr);
		DPxSetError(DPX_ERR_SETREG32_ADDR_ALIGN);
		return;
	}
	if (regAddr < 0 || regAddr >= (int)DPxGetRegisterSpaceSizeNoDevsel()) {
		DPxDebugPrint2("ERROR: DPxSetReg32() argument register address 0x%x is not in range 0 to 0x%X\n", regAddr, DPxGetRegisterSpaceSizeNoDevsel()-4);
		DPxSetError(DPX_ERR_SETREG32_ADDR_RANGE);
		return;
	}
	dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2  ] = LSW(regValue);
	dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2+1] = MSW(regValue);
	dpxDeviceTable[dpxSysDevsel].dpxRegisterModified[regAddr/2  ] = 1;
	dpxDeviceTable[dpxSysDevsel].dpxRegisterModified[regAddr/2+1] = 1;
}


// Read a 32-bit register's value from dpxRegisterCache[]
unsigned DPxGetReg32(int regAddr)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	if (regAddr & 3) {
		DPxDebugPrint1("ERROR: DPxGetReg32() argument register address 0x%x is not 32-bit aligned\n", regAddr);
		DPxSetError(DPX_ERR_GETREG32_ADDR_ALIGN);
		return 0;
	}
	if (regAddr < 0 || regAddr >= (int)DPxGetRegisterSpaceSizeNoDevsel()) {
		DPxDebugPrint2("ERROR: DPxGetReg32() argument register address 0x%x is not in range 0 to 0x%X\n", regAddr, DPxGetRegisterSpaceSizeNoDevsel()-4);
		DPxSetError(DPX_ERR_GETREG32_ADDR_RANGE);
		return 0;
	}
	return (dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2+1] << 16) + dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[regAddr/2];
}


int DPxGetRegSize(int regAddr)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
	if (regAddr >= DPXREG_NANOTIME_15_0 && regAddr <= DPXREG_NANOMARKER_63_48+1)	return 8;
	if (DPxIsTrackpixx()) return 2;
	if (DPxIsTrackpixxBridge()) return 2;
	if (regAddr >= DPXREG_VID_VPERIOD_L && regAddr <= DPXREG_VID_VPERIOD_H+1)		return 4;

    // PROPixx is unique
    if (DPxIsPropixx()) {
        if (regAddr >= PPXREG_TSCOPE_SCHED_ONSET_L	&& regAddr <= PPXREG_TSCOPE_SCHED_CTRL_H+1)
            return 4;
        return 2;    // Rest of PROPixx register map contains 16-bit registers
    }

    // DATAPixx/DATAPixx2/VIEWPixx/PROPixxCtrl share I/O systems
	if (regAddr >= DPXREG_DAC_BUFF_BASEADDR_L	&& regAddr <= DPXREG_DAC_SCHED_CTRL_H+1)	return 4;
	if (regAddr >= DPXREG_ADC_CHANREF_L			&& regAddr <= DPXREG_ADC_CHANREF_H+1)		return 4;
	if (regAddr >= DPXREG_ADC_BUFF_BASEADDR_L	&& regAddr <= DPXREG_ADC_SCHED_CTRL_H+1)	return 4;
	if (regAddr >= DPXREG_DOUT_DATA_L			&& regAddr <= DPXREG_DOUT_DATA_H+1)			return 4;
	if (regAddr >= DPXREG_DOUT_BUFF_BASEADDR_L	&& regAddr <= DPXREG_DOUT_SCHED_CTRL_H+1)	return 4;
	if (regAddr >= DPXREG_DIN_DATA_L			&& regAddr <= DPXREG_DIN_DATAOUT_H+1)		return 4;
	if (regAddr >= DPXREG_DIN_BUFF_BASEADDR_L	&& regAddr <= DPXREG_DIN_SCHED_CTRL_H+1)	return 4;
	if (regAddr >= DPXREG_AUD_BUFF_BASEADDR_L	&& regAddr <= DPXREG_AUX_SCHED_CTRL_H+1)	return 4;
	if (regAddr >= DPXREG_MIC_BUFF_BASEADDR_L	&& regAddr <= DPXREG_MIC_SCHED_CTRL_H+1)	return 4;

    // TPx has same I/O space, but also has a second block of registers
    if (DPxIsTrackpixxCtrl()) {
        if (regAddr >= 0x320 && regAddr < 0x330) return 4;
    }

	return 2;
}


// Data structures for accumulating a composite USB message over multiple API calls.
// Other DP users (eg: CODEC I2C) could be sending ep2out traffic between the buildUsbMsg calls,
// so we need our own static copy of the message buffer, and message pointer.
UInt16	dpxBuildUsbMsgBuff[4096];			// size is overkill if we're just using this for register updates
UInt16*	dpxBuildUsbMsgPtr = dpxBuildUsbMsgBuff;
int		dpxBuildUsbMsgHasReadback = 0;


// Start accumulating a composite USB message
void DPxBuildUsbMsgBegin()
{
	dpxBuildUsbMsgPtr = dpxBuildUsbMsgBuff;
	dpxBuildUsbMsgHasReadback = 0;
}


// Append composite USB message to write modified registers from local cache to DATAPixx.
// Combines contiguous modified registers into single trams.
void DPxBuildUsbMsgWriteRegs()
{
	unsigned iReg;
	int lastModifiedRegIndex = -2;	// -1 could make reg(0) think it's a follower.
	UInt16* payloadPtr = 0;

	// Check each register to see if it has been modified in the local cache
	for (iReg = 0; iReg < DPxGetRegisterSpaceSizeNoDevsel()/2; iReg++) {

		// Construct trams for writing modified registers to DP
		if (dpxDeviceTable[dpxSysDevsel].dpxRegisterModified[iReg]) {
			dpxDeviceTable[dpxSysDevsel].dpxRegisterModified[iReg] = 0;	// Indicates that DP is getting new modified value

			if (dpxDeviceTable[dpxSysDevsel].dpxIsDatapixx3){
				// Send as 32bit words
				*dpxBuildUsbMsgPtr++ = (EP2OUT_WRITEREGS << 8) + '^';	// Tram header for a register write
				*dpxBuildUsbMsgPtr++ = 8;								// Tram payload initialized for register index only
				*dpxBuildUsbMsgPtr++ = iReg;							// Index of first register to write with tram
				*dpxBuildUsbMsgPtr++ = 0;								// Fill MSB with 0
				*dpxBuildUsbMsgPtr++ = dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[iReg];
				*dpxBuildUsbMsgPtr++ = 0;								// Fill MSB with 0
			}
			else
			{
				if (lastModifiedRegIndex != iReg-1) {						// If reg is not contiguous with previous modified register, start a new write tram 
					*dpxBuildUsbMsgPtr++ = (EP2OUT_WRITEREGS << 8) + '^';	// Tram header for a register write
					*dpxBuildUsbMsgPtr++ = 2;								// Tram payload initialized for register index only
					*dpxBuildUsbMsgPtr++ = iReg;							// Index of first register to write with tram
					payloadPtr = dpxBuildUsbMsgPtr - 2;						// So we can increment as we add contig registers
				}

				// Add the modified register write to the current tram
				*dpxBuildUsbMsgPtr++ = dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[iReg];
				(*payloadPtr) += 2;
				lastModifiedRegIndex = iReg;
			}
		}
	}

    // Some register bits are one-shots, and must be manually reset to 0 once they are written
	if (dpxDeviceTable[dpxSysDevsel].dpxIsPropixx) {
		dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[PPXREG_SLEEP/2] = 0;
	}
	else {
		dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[DPXREG_SCHED_STARTSTOP/2] = 0;                     // Starting/stopping schedules
		dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[DPXREG_CTRL/2] &= ~DPXREG_CTRL_CALIB_RELOAD;       // SPI calibration table reload
		dpxDeviceTable[dpxSysDevsel].dpxRegisterCache[DPXREG_VID_VESA/2] &= ~DPXREG_VID_VESA_LEFT_WEN;   // Writing VESA Left/Right bit
	}
}


// Append composite USB message to read Datapixx register set.
void DPxBuildUsbMsgReadRegs()
{
	*dpxBuildUsbMsgPtr++ = (EP2OUT_READREGS << 8) + '^';			// Tram header for a register read
	*dpxBuildUsbMsgPtr++ = 0;										// No payload for a register read command
	dpxBuildUsbMsgHasReadback = 1;
}


// Append composite USB message to read Datapixx register set, but read will occur later.
void DPxBuildUsbMsgReadRegsDelayed()
{
	*dpxBuildUsbMsgPtr++ = (EP2OUT_READREGS << 8) + '^';			// Tram header for a register read
	*dpxBuildUsbMsgPtr++ = 0;										// No payload for a register read command
}


// Append composite USB message to freeze Datapixx USB message treatment
// until next leading edge of video vertical sync pulse.
// ***What to do if the DATAPixx is not receiving any video?  This could freeze an application on next register readback!
// If there's no video, we'll set an error code, and will not implement the video sync.
// On VIEWPixx/PROPixx there is no such requirement, since a software or hardware test pattern will always be running.
// Not quite true anymore.  Display could be sleeping.  In that case it will freeze.
void DPxBuildUsbMsgVideoSync()
{
    // Caller is responsible for ensuring valid dpxSysDevsel
    if (DPxIsVidDviActive() || DPxIsS3Arch()) {
        *dpxBuildUsbMsgPtr++ = (EP2OUT_VSYNC << 8) + '^';				// Tram header for vertical sync
        *dpxBuildUsbMsgPtr++ = 0;										// No payload
    }
    else
        DPxSetError(DPX_ERR_VID_VSYNC_WITHOUT_VIDEO);
}


// Append composite USB message to freeze Datapixx USB message treatment
// until a prespecified RGB pixel sequence is seen at the video input.
// The timeout argument specifies the maximum number of video frames which the Datapixx will wait.
// After this time, USB message treatment will continue, and DPxIsPsyncTimeout() will return true.
// pixelData contains a list of 8-bit RGB component values; ie: R0, G0, B0, R1, G1, B1...
void DPxBuildUsbMsgPixelSync(int nPixels, unsigned char* pixelData, int timeout)
{
	UInt16* sizePtr;

	if (nPixels < 1 || nPixels > 8) {
		DPxDebugPrint0("ERROR: DPxBuildUsbMsgPixelSync() nPixels argument must be in the range 1-8\n");
		DPxSetError(DPX_ERR_VID_PSYNC_NPIXELS_ARG_ERROR);
		return;
	}
	if (timeout < 0 || timeout > 65535) {
		DPxDebugPrint0("ERROR: DPxBuildUsbMsgPixelSync() timeout must be in the range 0-65535\n");
		DPxSetError(DPX_ERR_VID_PSYNC_TIMEOUT_ARG_ERROR);
		return;
	}

	*dpxBuildUsbMsgPtr++ = (EP2OUT_WRITEPSYNC << 8) + '^';	// Tram header for pixel sync sequence
	sizePtr = dpxBuildUsbMsgPtr;							// Save size pointer for later use
	*dpxBuildUsbMsgPtr++ = nPixels * 6;						// payload contains 16-bit RGB pixel components

	while (nPixels--) {
		*dpxBuildUsbMsgPtr++ = *pixelData++ << 8;			// Red
		*dpxBuildUsbMsgPtr++ = *pixelData++ << 8;			// Green
		*dpxBuildUsbMsgPtr++ = *pixelData++ << 8;			// Blue
	}

	*dpxBuildUsbMsgPtr++ = (EP2OUT_PSYNC << 8) + '^';		// Tram header for pixel sync
	*dpxBuildUsbMsgPtr++ = 2;								// 2-byte payload
	*dpxBuildUsbMsgPtr++ = timeout;

	// If we are using the FX3 USB interface, then send data at modulo 4bytes
	if (DPxIsA10Arch()){
		if (*sizePtr % 4){
			*dpxBuildUsbMsgPtr++ = 0;
			sizePtr++;
		}
	}

}


// Transmit the message we just built,
// and possibly wait for an incoming register read USB message.
void DPxBuildUsbMsgEnd()
{
	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif

    // Caller is responsible for ensuring valid dpxSysDevsel

	// It's possible that user called DPxBuildUsbMsgWriteRegs() but no registers were modified.
    // Also no vsync, psync, or register readback requested.
    // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	packetSize = (int)((char*)dpxBuildUsbMsgPtr - (char*)dpxBuildUsbMsgBuff);
	if (!packetSize)
		return;

#if 0
    // Support remote (ethernet) PROPixx
    if (dpxDevSel) {
        memmove(dpxBuildUsbMsgBuff+3, dpxBuildUsbMsgBuff, packetSize);
        dpxBuildUsbMsgBuff[0] = ('X' << 8) + '^';	// Tram header for ethernet redirection
        dpxBuildUsbMsgBuff[1] = packetSize;			// Write payload
        dpxBuildUsbMsgBuff[2] = dpxBuildUsbMsgHasReadback ? DPX_REG_SPACE_MAX+4 : 0; // Expected return tram size
        packetSize += 6;
    }
#endif

	// Write the packet with the trams to VPixx Device
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)dpxBuildUsbMsgBuff, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (unsigned char*)dpxBuildUsbMsgBuff, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxBuildUsbMsgEnd() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxBuildUsbMsgEnd() call to usb_bulk_write() failed: %s\n", usb_strerror());
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
			dpxEp2WrFails++;
			return;
		}
	}

	// If reading, go get the new register values, and copy into local cache
	if (dpxBuildUsbMsgHasReadback)
        DPxGetRegCache();
}


// Read registers over USB
void DPxGetRegCache()
{
    if (EZReadEP6Tram(EP6IN_READREGS, DPxGetRegisterSpaceSizeNoDevsel()) < 0) {
        DPxDebugPrint0("ERROR: DPxGetRegCache() call to EZReadEP6Tram() failed\n");
        DPxSetError(DPX_ERR_USB_REG_BULK_READ);
        return;
    }
    memcpy(dpxDeviceTable[dpxSysDevsel].dpxRegisterCache, ep6in_Tram+4, DPxGetRegisterSpaceSizeNoDevsel());
}


// Write local register cache to Datapixx over USB
void DPxWriteRegCache()
{
    int saveDevsel;
    
    // If user has requested DPX_DEVSEL_AUTO, then we write all device's caches
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        saveDevsel = dpxSysDevsel;
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        dpxSysDevsel = saveDevsel;
    }
    
    // If user has requested a specific device, we only write that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;
        
        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgEnd();
    }
}


// Write local register cache to Datapixx over USB, then read back DATAPixx registers over USB into local cache
void DPxUpdateRegCache()
{
    int saveDevsel;

    // If user has requested DPX_DEVSEL_AUTO, then we update all device's caches
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        saveDevsel = dpxSysDevsel;
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgReadRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        dpxSysDevsel = saveDevsel;
    }
    
    // If user has requested a specific device, we only update that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;

        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgReadRegs();
        DPxBuildUsbMsgEnd();
    }
}


// Write local register cache to Datapixx over USB.
// This routine sends the USB message and returns quickly,
// but Datapixx only writes the registers on leading edge of next vertical sync pulse.
void DPxWriteRegCacheAfterVideoSync()
{
    int saveDevsel;
    
    // If user has requested DPX_DEVSEL_AUTO, then we write all device's caches
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
			saveDevsel = dpxSysDevsel;    
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgVideoSync();
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgEnd();
            }
			dpxSysDevsel = saveDevsel;
        }
    }
    
    // If user has requested a specific device, we only write that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;
        
        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgVideoSync();
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgEnd();
    }
}


// Write local register cache to DATAPixx over USB,
// then read back DATAPixx registers over USB into local cache.
// DATAPixx blocks until leading edge of next vertical sync pulse before writing/reading registers.
void DPxUpdateRegCacheAfterVideoSync()
{
    int saveSysDevsel;
    
    // If user has requested DPX_DEVSEL_AUTO, then we update all device's caches
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        saveSysDevsel = dpxSysDevsel;
        
        // Here's the thing.  If we have multiple devices in the system, which video sync should we be waiting for?
        // In the case of PPC/PPX, the vsync's are the same.
        // The thing is though, we need to get in both cache writes _before_ we do either cache read;
        // otherwise the second cache write won't be done until the first cache read returns after vsync.
        // So, split up into multiple cache writes, followed by multiple cache reads.
        // We will only return after both systems have seen their vsync.
        // Note that the first device's returned timer value will be a perfect vsync timetag,
        // but later timer values may not correspond to a vsync timetag.
        // To get around this, can just DPxSelectDevice() for a specific device instead of DPX_DEVSEL_AUTO.
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
			dpxUsrDevsel = dpxSysDevsel;
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgVideoSync();
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
			dpxUsrDevsel = dpxSysDevsel;
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgReadRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        dpxSysDevsel = saveSysDevsel;
		dpxUsrDevsel = DPX_DEVSEL_AUTO;
    }
    
    // If user has requested a specific device, we only update that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;

        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgVideoSync();
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgReadRegs();
        DPxBuildUsbMsgEnd();
    }
}


// Write local register cache to Datapixx over USB.
// This routine sends the USB message and returns quickly,
// but Datapixx only writes the registers when the previously defined pixel sync sequence has been displayed.
void DPxWriteRegCacheAfterPixelSync(int nPixels, unsigned char* pixelData, int timeout)
{
    int saveDevsel;
    
    // If user has requested DPX_DEVSEL_AUTO, then we write all device's caches.
    // In the case of PROPixxCtrl/PROPixx, the PPC normally sends a copy of the video to PPX, so they should both psync correctly.
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        saveDevsel = dpxSysDevsel;
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgPixelSync(nPixels, pixelData, timeout);
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        dpxSysDevsel = saveDevsel;
    }
    
    // If user has requested a specific device, we only write that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;
        
        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgPixelSync(nPixels, pixelData, timeout);
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgEnd();
    }
}


// Write local register cache to Datapixx over USB,
// then read back DATAPixx registers over USB into local cache.
// DATAPixx blocks until the specified pixel sync sequence has been displayed before writing/reading registers.
void DPxUpdateRegCacheAfterPixelSync(int nPixels, unsigned char* pixelData, int timeout)
{
    int saveDevsel;
    
    // If user has requested DPX_DEVSEL_AUTO, then we update all device's caches
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO) {
        saveDevsel = dpxSysDevsel;
        
        // Here's the thing.  If we have multiple devices in the system, which psync should we be waiting for?
        // In the case of PROPixxCtrl/PROPixx, the PPC normally sends a copy of the video to PPX, so they should both psync correctly.
        // The thing is though, we need to get in both cache writes _before_ we do either cache read;
        // otherwise the second cache write won't be done until the first cache read returns after psync.
        // So, split up into multiple cache writes, followed by multiple cache reads.
        // We will only return after both systems have seen their psync.
        // Note that the first device's returned timer value will be a perfect psync timetag,
        // but later timer values may not correspond to a psync timetag.
        // To get around this, can just DPxSelectDevice() for a specific device instead of DPX_DEVSEL_AUTO.
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgPixelSync(nPixels, pixelData, timeout);
                DPxBuildUsbMsgWriteRegs();
                DPxBuildUsbMsgEnd();
            }
        }
        for (dpxSysDevsel = DPX_DEVSEL_FIRST_DEVICE; dpxSysDevsel <= DPX_DEVSEL_LAST_DEVICE; dpxSysDevsel++) {
            
            // Don't use DPxSelectSysDevice(), or missing devices will print error messages.
            if (dpxSysDevsel != DPX_DEVSEL_UNCONFIGURED && dpxDeviceTable[dpxSysDevsel].dpxGoodFpga) {
                DPxBuildUsbMsgBegin();
                DPxBuildUsbMsgReadRegs();
                dpxActivePSyncTimeout = timeout;	// Let low-level know that the timeout could be large
                DPxBuildUsbMsgEnd();
                dpxActivePSyncTimeout = -1;			// Let low-level return timeout to normal value
            }
        }
        dpxSysDevsel = saveDevsel;
    }
    
    // If user has requested a specific device, we only update that device's cache.
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
            return;
        
        DPxBuildUsbMsgBegin();
        DPxBuildUsbMsgPixelSync(nPixels, pixelData, timeout);
        DPxBuildUsbMsgWriteRegs();
        DPxBuildUsbMsgReadRegs();
        dpxActivePSyncTimeout = timeout;	// Let low-level know that the timeout could be large
        DPxBuildUsbMsgEnd();
        dpxActivePSyncTimeout = -1;			// Let low-level return timeout to normal value
    }
}


// Get all DATAPixx registers, and save them in a local copy.
// Note that there is only 1 copy, so this should not be done with dpxSysDevsel set to AUTO with multiple devices.
void DPxSaveRegs()
{
    // Caller must ensure only 1 device is updated
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        printf("WARNING: DPxSaveRegs() only supports a single device\n");
	DPxUpdateRegCache();
	memcpy(dpxSavedRegisters, dpxDeviceTable[dpxSysDevsel].dpxRegisterCache, sizeof(dpxSavedRegisters));
}


// Write the local copy back to the DATAPixx
void DPxRestoreRegs()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        printf("WARNING: DPxRestoreRegs() only supports a single device\n");
	memcpy(dpxDeviceTable[dpxSysDevsel].dpxRegisterCache, dpxSavedRegisters, sizeof(dpxSavedRegisters));
	memset(dpxDeviceTable[dpxSysDevsel].dpxRegisterModified, 1, sizeof(dpxDeviceTable[dpxSysDevsel].dpxRegisterModified));
	DPxUpdateRegCache();
}


// Set an 8-bit I2C register in audio CODEC IC
void DPxSetCodecReg(int regAddr, int regValue)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;

	regAddr &= 0x7F;
	DPxSetI2cReg(regAddr, regValue);

	// Sometimes we want to readback cached CODEC I2C register values
	cachedCodecRegs[regAddr] = (unsigned char)regValue;
}


// Read an 8-bit I2C register from audio CODEC IC
int DPxGetCodecReg(int regAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetI2cReg(regAddr & 0x7F);
}


// Read an 8-bit I2C register from CODEC cache, instead of from audio CODEC IC hardware.
// This is much faster, but could be wrong if another app has written the register.
int DPxGetCachedCodecReg(int regAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return cachedCodecRegs[regAddr & 0x7F];
}


// Set an 8-bit I2C register in Silicon Image DVI IC
void DPxSetDviReg(int regAddr, int regValue)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;

	DPxSetI2cReg(regAddr | 0x80, regValue);
}


// Read an 8-bit I2C register from Silicon Image DVI IC
int DPxGetDviReg(int regAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetI2cReg(regAddr | 0x80);
}


// Set an 8-bit register in audio CODEC IC, DVI receiver or PROPixx Projector via DDC I2C port
void DPxSetI2cReg(int regAddr, int regValue)
{
	UInt16* tramPtr = (UInt16*)ep2out_Tram;
	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif
	

    // Caller is responsible for ensuring valid dpxSysDevsel
	*tramPtr++ = (EP2OUT_WRITEI2C << 8) + '^';	// Tram header for an I2C register write
	if (regAddr < 0x100)
	{
		*tramPtr++ = 2;								// Tram payload has 1 byte for register number, and 1 byte for datum
		*tramPtr++ = ((unsigned char)regAddr << 8) + (unsigned char)regValue;
		if (DPxIsDatapixx3())
		{
			*tramPtr++ = 0;
		}
	}
	else
	{
		*tramPtr++ = 4;								// Tram payload has 3 byte for register number, and 1 byte for datum
		*tramPtr++ = ((unsigned char)regAddr << 24);
		*tramPtr++ = ((unsigned char)regAddr << 8) + (unsigned char)regValue;
	}

	packetSize = (int)((char*)tramPtr - (char*)ep2out_Tram);   // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxSetI2cReg() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxSetI2cReg() call to usb_bulk_write() failed: %s\n", usb_strerror());
			dpxEp2WrFails++;
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
			return;
		}
	}
}


// Read an 8-bit register from audio CODEC IC
int DPxGetI2cReg(int regAddr)
{
	UInt16* tramPtr = (UInt16*)ep2out_Tram;
	unsigned int recvSize = 2;

	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif

    // Caller is responsible for ensuring valid dpxSysDevsel
	*tramPtr++ = (EP2OUT_READI2C << 8) + '^';			// Tram header for an I2C register read
	
	// Payload length is 4bytes when in datapixx3 in order to send 32bit datum
	if (DPxIsDatapixx3())
	{
		*tramPtr++ = 4;										// 2-byte payload for an I2C register read command
	}
	else
	{
		*tramPtr++ = 2;										// 2-byte payload for an I2C register read command
	}

	*tramPtr++ = regAddr << 8;
	
	// Pad 0 for 16/32bit FX3 access
	if (DPxIsDatapixx3())
	{
		*tramPtr++ = 0;
	}
	
	packetSize = (int)((char*)tramPtr - (char*)ep2out_Tram);   // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxGetI2cReg() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxGetI2cReg() call to usb_bulk_write() failed: %s\n", usb_strerror());
			dpxEp2WrFails++;
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
			return -1;
		}
	}

	if (DPxIsDatapixx3())
	{
		recvSize += 2;
	}

	if (EZReadEP6Tram(EP6IN_READI2C, recvSize) < 0) {
		DPxDebugPrint0("ERROR: DPxGetI2cReg() call to EZReadEP6Tram() failed\n");
		DPxSetError(DPX_ERR_USB_REG_BULK_READ);
		return -1;
	}
	return (unsigned)ep6in_Tram[4];
}


// Set an 16-bit register in LUX CMOS IC.
// ADDRESS: 8 bit
// bit#7   = LUX A or B
// bit#6-0 = LUX register add
//
// I have no idea why, but if my host software sends a software reset to the LUX,
// then under certain conditions, the LUX skip bus is left in an unworking state.
// Fortunately, a second software reset corrects the situation.
// We will manually convert a single reset into two resets.
void DPxSetLuxRegInt(int regAddr, int regValue);
void DPxSetLuxRegInt(int regAddr, int regValue)
{
	UInt16* tramPtr = (UInt16*)ep2out_Tram;
	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif

    // Caller is responsible for ensuring valid dpxSysDevsel
	*tramPtr++ = (EP2OUT_WRITELUX << 8) + '^';	// Tram header for an LUX register write

		*tramPtr++ = 4;								// Tram payload has 2 byte for register number, and 2 byte for datum
		*tramPtr++ = regAddr;
		*tramPtr++ = regValue;

	packetSize = (int)((char*)tramPtr - (char*)ep2out_Tram);   // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxSetLuxReg() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxSetLuxReg() call to usb_bulk_write() failed: %s\n", usb_strerror());
			dpxEp2WrFails++;
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
			return;
		}
	}
}


void DPxSetLuxReg(int regAddr, int regValue)
{
    DPxSetLuxRegInt(regAddr, regValue);
    if (regAddr == 0x7E && regValue == 0)
        DPxSetLuxRegInt(regAddr, regValue);
}


// Read an 16-bit register from LUX CMOS IC (TRACKPixx)
int DPxGetLuxReg(int regAddr)
{
	UInt16* tramPtr = (UInt16*)ep2out_Tram;
	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif


    // Caller is responsible for ensuring valid dpxSysDevsel
	*tramPtr++ = (EP2OUT_READLUX << 8) + '^';			// Tram header for an LUX register read
	*tramPtr++ = 2;										// 2-byte payload for an LUX register read command
	*tramPtr++ = regAddr;
	packetSize = (int)((char*)tramPtr - (char*)ep2out_Tram);   // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxGetLuxReg() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxGetLuxReg() call to usb_bulk_write() failed: %s\n", usb_strerror());
			dpxEp2WrFails++;
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
			return -1;
		}
	}
	if (EZReadEP6Tram(EP6IN_READLUX, 2) < 0) {
		DPxDebugPrint0("ERROR: DPxGetLuxReg() call to EZReadEP6Tram() failed\n");
		DPxSetError(DPX_ERR_USB_REG_BULK_READ);
		return -1;
	}
	return (ep6in_Tram[5]<<8) + ep6in_Tram[4];
}

// Get the VPixx identifier code
int DPxGetID()
{
    int id;

    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
    id = DPxGetReg16(DPXREG_DPID);
    
    // If there's a bad SPI text string, we will convert it into a VIEWPixx
    if (id == DPXREG_DPID_BAD_SPI_TEXT) {
        printf("Please write SPI manufacturing text\n");
        id = DPXREG_DPID_VP;
    }

	return id;
}


// The DPxIs*() routines return non-zero if the VPixx hardware is of the specified class.
// If multiple VPixx devices are detected in the system,
// and DPxSelectSysDevice() has been set to its default value of DPX_DEVSEL_AUTO,
// then multiple DPxIs*() queries could return non-zero.
// These returns are based on USB product ID instead of FPGA register set, so it's valid even if FPGA has not been programmed.

// Returns non-0 for original DATAPixx CRT driver
int DPxIsDatapixx()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_DPX].dpxIsDatapixx;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsDatapixx;
}


// Returns non-0 for DATAPixx2.
int DPxIsDatapixx2()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_DP2].dpxIsDatapixx2;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsDatapixx2;
}

// Returns non-0 for DATAPixx3.
int DPxIsDatapixx3()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_DP3].dpxIsDatapixx3;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsDatapixx3;
}

// Returns non-0 if DATAPixx is embedded in a VIEWPixx OR a VIEWPixx3D OR a VIEWPixxEEG
int DPxIsViewpixx()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_VPX].dpxIsViewpixx;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsViewpixx;
}


// Returns non-0 if DATAPixx is embedded in a VIEWPixx3D.
// Note that VP/VP3D have same USB product ID, so decision must be based on configured FPGA.
int DPxIsViewpixx3D()
{
    int isViewpixx3D = 0;
    int saveDpxSysDevsel;
    int partNumber;
	
	if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
	{ 	// Want to use dpxSysDevSel

		partNumber = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
	}
	else // Not auto, want to use dpxUsrDevsel
	{
		saveDpxSysDevsel    = dpxSysDevsel;
		dpxSysDevsel        = dpxUsrDevsel;
		partNumber          = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
		dpxSysDevsel        = saveDpxSysDevsel;
	}
	if (partNumber == DPXREG_OPTIONS_PART_3DLITE || partNumber == DPXREG_OPTIONS_PART_3DFULL)
            isViewpixx3D = 1;
	return isViewpixx3D;
    /*
    if (DPxIsViewpixx()) {
        saveDpxSysDevsel    = dpxSysDevsel;
        dpxSysDevsel        = DPX_DEVSEL_VPX;
        partNumber          = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
        dpxSysDevsel        = saveDpxSysDevsel;
        if (partNumber == DPXREG_OPTIONS_PART_3DLITE || partNumber == DPXREG_OPTIONS_PART_3DFULL)
            isViewpixx3D = 1;
    }
	
    return isViewpixx3D; */
}


// Returns non-0 if DATAPixx is embedded in a VIEWPixxEEG.
// Note that VP/VP3D/VPEEG have same USB product ID, so decision must be based on configured FPGA.
int DPxIsViewpixxEeg()
{
    int isViewpixxEeg = 0;
    int saveDpxSysDevsel;
    int partNumber;
    
	if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
	{ 	// Want to use dpxSysDevSel

		partNumber = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
	}
	else // Not auto, want to use dpxUsrDevsel
	{
		saveDpxSysDevsel    = dpxSysDevsel;
		dpxSysDevsel        = dpxUsrDevsel;
		partNumber          = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
		dpxSysDevsel        = saveDpxSysDevsel;
	}
	if (partNumber == DPXREG_OPTIONS_PART_EEG)
            isViewpixxEeg = 1;
    return isViewpixxEeg;

	/* OLD
    if (DPxIsViewpixx()) {
        saveDpxSysDevsel    = dpxSysDevsel;
        dpxSysDevsel        = DPX_DEVSEL_VPX;
        partNumber          = DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK;
        dpxSysDevsel        = saveDpxSysDevsel;
        if (partNumber == DPXREG_OPTIONS_PART_EEG)
            isViewpixxEeg = 1;
    }
    return isViewpixxEeg; */
}


// Returns non-0 if a PROPixx (not the Propixx Controller, but the actual projector unit)
int DPxIsPropixx()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_PPX].dpxIsPropixx;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsPropixx;
 }


// Returns non-0 if DATAPixx is embedded in a PROPixx Controller
int DPxIsPropixxCtrl()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_PPC].dpxIsPropixxCtrl;
    else
        return dpxDeviceTable[dpxSysDevsel].dpxIsPropixxCtrl;
}

// Returns non-0 if Trackpixx Controller device
int DPxIsTrackpixxCtrl()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_TPC].DPxIsTrackpixxCtrl;
    else
        return dpxDeviceTable[dpxSysDevsel].DPxIsTrackpixxCtrl;
}

int DPxIsSysDevSelTrackpixxCtrl()
{ //  In case where we are in AUTO and we are going through dpxSysDevSel, this funciton will return 0
	if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
		return dpxDeviceTable[dpxSysDevsel].DPxIsTrackpixxCtrl;
	else
		return dpxDeviceTable[dpxUsrDevsel].DPxIsTrackpixxCtrl;
}

// Returns non-0 if Trackpixx Bridge device
int DPxIsTrackpixxBridge()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_TPB].DPxIsTrackpixxBridge;
    else
        return dpxDeviceTable[dpxSysDevsel].DPxIsTrackpixxBridge;
}

// Returns non-0 if Trackpixx device
int DPxIsTrackpixx()
{
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO)
        return dpxDeviceTable[DPX_DEVSEL_TPX].DPxIsTrackpixx;
    else
        return dpxDeviceTable[dpxSysDevsel].DPxIsTrackpixx;
}


// Get the number of bytes of register space in the VPixx system
unsigned DPxGetRegisterSpaceSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;

    return DPxGetRegisterSpaceSizeNoDevsel();
}

// Get the number of bytes of register space in the VPixx system, without calling DPxSelectSysDevice(DPX_DEVSEL_ANY)
unsigned DPxGetRegisterSpaceSizeNoDevsel()
{
	unsigned registerSize = DPX_REG_SPACE;

	if (DPxIsSysDevSelTrackpixxCtrl()) //  We can't use this here as it will get TRUE if DEVSEL == AUTO and There is a TPXCtrl connected
		registerSize = TPC_REG_SPACE; // Giving us a 992 register space for a non TPXCtrl device...same if DPx3+ anything else...
	else if (DPxIsDatapixx3())
		registerSize = DP3_REG_SPACE;

	return registerSize;

	// if devsel == auto 
	//	if dpxsysdevsel == TPxCtrl numbers
	//		return TPC_REG_SPACE
	//  if dpxsysdevsel == DPx3 numbers
	//		return DP3_REG_SPACE
	//  retrun DPX_REG_SPACE
	// else do what is above
}

// Get the number of bytes of RAM in the VPixx system
unsigned DPxGetRamSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;

    return DPxGetRamSizeNoDevsel();
}

// Get the number of bytes of RAM in the VPixx system, without calling DPxSelectSysDevice(DPX_DEVSEL_ANY)
unsigned DPxGetRamSizeNoDevsel()
{
	int ramMB;
    
	switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_RAM_MASK) {
		case DPXREG_OPTIONS_RAM_0     : ramMB = 0;    break;
		case DPXREG_OPTIONS_RAM_32M   : ramMB = 32;   break;
		case DPXREG_OPTIONS_RAM_64M   : ramMB = 64;   break;
		case DPXREG_OPTIONS_RAM_128M  : ramMB = 128;  break;
		case DPXREG_OPTIONS_RAM_256M  : ramMB = 256;  break;
		case DPXREG_OPTIONS_RAM_512M  : ramMB = 512;  break;
		case DPXREG_OPTIONS_RAM_1024M : ramMB = 1024; break;
		case DPXREG_OPTIONS_RAM_2048M : ramMB = 2048; break;
		case DPXREG_OPTIONS_RAM_4096M : ramMB = 4096; break;
		case DPXREG_OPTIONS_RAM_8192M : ramMB = 8192; break;
		default: ramMB = 0;
	}
	if (ramMB == 0) {
		DPxDebugPrint0("ERROR: DPxGetRamSize() doesn't recognize RAM size\n");
		DPxSetError(DPX_ERR_RAM_UNKNOWN_SIZE);
	}
	return ramMB * (1 << 20);
}


// Get the Vpixx device part number
int DPxGetPartNumber()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
    if (DPxIsViewpixx()) {
        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            case DPXREG_OPTIONS_PART_LITE   : return VPX_PARTNUM_VPX_LITE;
            case DPXREG_OPTIONS_PART_FULL   : return VPX_PARTNUM_VPX_FULL;
            case DPXREG_OPTIONS_PART_3DLITE : return VPX_PARTNUM_V3D_LITE;
            case DPXREG_OPTIONS_PART_3DFULL : return VPX_PARTNUM_V3D_FULL;
			case DPXREG_OPTIONS_PART_EEG	: return VPX_PARTNUM_VPX_EEG;				
        }
    }
    else if (DPxIsPropixxCtrl()) {
        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            case DPXREG_OPTIONS_PART_PPCLITE : return VPX_PARTNUM_PPC_LITE;
            case DPXREG_OPTIONS_PART_PPCFULL : return VPX_PARTNUM_PPC_FULL;
        }
    }
    else if (DPxIsPropixx()) {
        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            case DPXREG_OPTIONS_PART_PPX : return VPX_PARTNUM_PPX;  // PROPixx projector
        }
    }
	else if (DPxIsDatapixx2()) {
        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            case DPXREG_OPTIONS_PART_DP2LITE : return VPX_PARTNUM_DP2_LITE;
            case DPXREG_OPTIONS_PART_DP2FULL : return VPX_PARTNUM_DP2_FULL;
        }
    }
	else if (DPxIsTrackpixx()) {
			return VPX_PARTNUM_TPX;  // TRACKPixx eye tracker
    }
	else if (DPxIsTrackpixxCtrl()) {
			return VPX_PARTNUM_TPC;  // TRACKPixx Controller
    }
	else if (DPxIsTrackpixxBridge()) {
		switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            //case DPXREG_OPTIONS_PART_TPB_TX : return VPX_PARTNUM_TPB_TX;
            case DPXREG_OPTIONS_PART_TPB_RX : return VPX_PARTNUM_TPB_RX;
            default: return 0;
        }
    }
	else if (DPxIsDatapixx3()) {
        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
			case DPXREG_OPTIONS_PART_DP3LITE : return VPX_PARTNUM_DP3_LITE;
            case DPXREG_OPTIONS_PART_DP3FULL : return VPX_PARTNUM_DP3_FULL;
        }
    }

    else {  // Assume a DATAPixx
        if (DPxGetFirmwareRev() < 14)
            return VPX_PARTNUM_DPX_FULL;	// DATAPixx options reg only has part number starting with VHDL rev 14.

        switch (DPxGetReg16(DPXREG_OPTIONS) & DPXREG_OPTIONS_PART_MASK) {
            case DPXREG_OPTIONS_PART_LITE : return VPX_PARTNUM_DPX_LITE;
            case DPXREG_OPTIONS_PART_FULL : return VPX_PARTNUM_DPX_FULL;
        }
    }
	DPxDebugPrint0("ERROR: DPxGetPartNumber() doesn't recognize part number\n");
	DPxSetError(DPX_ERR_UNKNOWN_PART_NUMBER);

	return 0;
}


// Get the DATAPixx firmware revision
int DPxGetFirmwareRev()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_FIRMWARE_REV);
}


// Get voltage being supplied from +5V supply
double DPxGetSupplyVoltage()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return MSB(DPxGetReg16(DPXREG_POWER)) / 256.0 * 6.65;
}


// Get current being supplied from +5V supply.
double DPxGetSupplyCurrent()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
    return LSB(DPxGetReg16(DPXREG_POWER)) / 256.0 * ((DPxIsViewpixx() || DPxIsPropixxCtrl() || DPxIsDatapixx2()) ? 21.168 : 10.584);    // DATAPixx uses a 10 mOhm current sense resistor; VIEWPixx/PROPixxCtrl use 5 mOhms
}


// Get voltage being supplied from +12V supply.  Only implemented on VIEWPixx.
double DPxGetSupply2Voltage()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_TPC))
        return 0;
    
	return MSB(DPxGetReg16(DPXREG_POWER2)) / 256.0 * 26.52;
}


// Get current being supplied from +12V supply.  Only implemented on VIEWPixx.
double DPxGetSupply2Current()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_TPC))
        return 0;
    
    return LSB(DPxGetReg16(DPXREG_POWER2)) / 256.0 * 21.168;
}


// Returns non-0 if VESA and Analog I/O +5V pins are trying to draw more than 500 mA
int DPxIs5VFault()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_5V_FAULT;
}


// Returns non-0 if last pixel sync wait timed out
int DPxIsPsyncTimeout()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_PSYNC_TIMEOUT;
}


// Returns non-0 if DDR SDRAM controller has not yet brought memory system online
int DPxIsRamOffline()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_RAM_OFFLINE;
}


// Get temperature inside of VPixx device chassis, in degrees Celcius
double DPxGetTempCelcius()
{
    if (DPxIsPropixx())
        return DPxGetPPxTemperature(PPX_TEMP_RX_DVI);
    else
        return (double)(signed char)(LSB(DPxGetReg16(DPXREG_TEMP)));
}


// Get temperature2 inside of VPixx device chassis, in degrees Celcius
double DPxGetTemp2Celcius()
{
    if (DPxIsPropixx())
        return DPxGetPPxTemperature(PPX_TEMP_FPGA2);
    else
        return (double)(signed char)(MSB(DPxGetReg16(DPXREG_TEMP)));
}


// Get temperature of FPGA die, in degrees Celcius
double DPxGetTemp3Celcius()
{
    if (DPxIsPropixx())
        return DPxGetPPxTemperature(PPX_TEMP_FPGA);
    else
        return (double)(signed char)(MSB(DPxGetReg16(DPXREG_STATUS)));
}


// Get temperature inside of VPixx device chassis, in degrees Farenheit
double DPxGetTempFarenheit()
{
	return DPxGetTempCelcius() * 9 / 5 + 32;
}


// Get double precision seconds since powerup
double DPxGetTime()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxMakeFloat64FromTwoUInt32(DPxGetReg32(DPXREG_NANOTIME_47_32), DPxGetReg32(DPXREG_NANOTIME_15_0)) * 1.0e-9;
}


// Latch the current NanoTime value into the marker register
void DPxSetMarker()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_NANOMARKER_15_0, 0);	// Write any value to the register to latch nanotime
}


// Get double precision seconds when DPxSetNanoMark() was last called
double DPxGetMarker()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxMakeFloat64FromTwoUInt32(DPxGetReg32(DPXREG_NANOMARKER_47_32), DPxGetReg32(DPXREG_NANOMARKER_15_0)) * 1.0e-9;
}


// Get low/high UInt32 nanoseconds since powerup
void DPxGetNanoTime(unsigned *nanoHigh32, unsigned *nanoLow32)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	if (!nanoHigh32 || !nanoLow32) {
		DPxDebugPrint0("ERROR: DPxGetNanoTime() argument is null\n");
		DPxSetError(DPX_ERR_NANO_TIME_NULL_PTR);
		return;
	}

	*nanoHigh32 = DPxGetReg32(DPXREG_NANOTIME_47_32);
	*nanoLow32  = DPxGetReg32(DPXREG_NANOTIME_15_0);
}


// Get high/low UInt32 nanosecond marker
void DPxGetNanoMarker(unsigned *nanoHigh32, unsigned *nanoLow32)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	if (!nanoHigh32 || !nanoLow32) {
		DPxDebugPrint0("ERROR: DPxGetNanoMarker() argument is null\n");
		DPxSetError(DPX_ERR_NANO_MARK_NULL_PTR);
		return;
	}

	*nanoHigh32 = DPxGetReg32(DPXREG_NANOMARKER_47_32);
	*nanoLow32  = DPxGetReg32(DPXREG_NANOMARKER_15_0);
}


/********************************************************************************/
/*																				*/
/*	DAC Subsystem																*/
/*																				*/
/********************************************************************************/


// Returns number of DAC channels in system
int DPxGetDacNumChans()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPX_DAC_NCHANS;
}


// Set the 16-bit 2's complement signed value for one DAC channel (0-3)
void DPxSetDacValue(int value, int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (value < -32768 || value > 65535) {	// We'll permit full combined range of SInt16 and UInt16.
		DPxDebugPrint1("ERROR: DPxSetDacValue() argument value %d is out of 16-bit range\n", value);
		DPxSetError(DPX_ERR_DAC_SET_BAD_VALUE);
		return;
	}
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxSetDacValue() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_SET_BAD_CHANNEL);
		return;
	}
	DPxSetReg16(DPXREG_DAC_DATA0+channel*2, value);
}


// Get the 16-bit 2's complement signed value for one DAC channel (0-3);
int DPxGetDacValue(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxGetDacValue() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_GET_BAD_CHANNEL);
		return 0;
	}
	return (SInt16)DPxGetReg16(DPXREG_DAC_DATA0+channel*2);
}


// Get the voltage min/max range for a DAC channel (0-3).
// Note that the true max value will really be 1 LSB less than the reported max value.
void DPxGetDacRange(int channel, double *minV, double *maxV)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (!minV) {
		DPxDebugPrint0("ERROR: DPxGetDacRange() minV argument is null\n");
		DPxSetError(DPX_ERR_DAC_RANGE_NULL_PTR);
		return;
	}
	if (!maxV) {
		DPxDebugPrint0("ERROR: DPxGetDacRange() maxV argument is null\n");
		DPxSetError(DPX_ERR_DAC_RANGE_NULL_PTR);
		return;
	}
	
	switch(channel) {
		case 0:
		case 1: *minV = -10; *maxV = 10; break;
		case 2:
		case 3:
            if (DPxIsViewpixx() || DPxIsPropixxCtrl() || DPxIsDatapixx2() || DPxIsDatapixx3())
                { *minV = -10; *maxV = 10; }
            else
                { *minV = -5; *maxV = 5; }  // DATAPixx
            break;
		default:
			*minV = -1; *maxV = 1;		// Use +-1 to protect user against divide by 0
			DPxDebugPrint2("ERROR: DPxGetDacRange() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
			DPxSetError(DPX_ERR_DAC_RANGE_BAD_CHANNEL);
			return;
	}
}


// Set the voltage for one DAC channel +-10V for ch0/1, +-5V for ch2/3
void DPxSetDacVoltage(double voltage, int channel)
{
	double minV, maxV, fValue;
	int iValue;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxSetDacVoltage() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_SET_BAD_CHANNEL);
		return;
	}
	ReturnIfError(DPxGetDacRange(channel, &minV, &maxV));
	if (voltage < minV || voltage > maxV) {
		DPxDebugPrint3("ERROR: DPxSetDacVoltage() argument voltage %g is not in range %g to %g\n", voltage, minV, maxV);
		DPxSetError(DPX_ERR_DAC_SET_BAD_VALUE);
		return;
	}

	fValue = (voltage - minV) / (maxV - minV) - 0.5;	// -0.5 to +0.5
	iValue = (int)floor(fValue * 65536 + 0.5);			// -32768 to +32768 with rounding (not truncation)
	if (iValue == 32768)
		iValue = 32767;									// Since maxV is 1 LSB over top
	DPxSetDacValue(iValue, channel);
}


// Get the voltage for one DAC channel +-10V for ch0/1, +-5V for ch2/3
double DPxGetDacVoltage(int channel)
{
	int iValue;
	double minV, maxV;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	Return0IfError(iValue = DPxGetDacValue(channel));
	Return0IfError(DPxGetDacRange(channel, &minV, &maxV));
	return ((double)iValue + 32768) / 65536 * (maxV - minV) + minV;
}


// Enable RAM buffering of a DAC channel
void DPxEnableDacBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxEnableDacBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_BUFF_BAD_CHANNEL);
		return;
	}
	DPxSetReg16(DPXREG_DAC_CHANSEL, DPxGetReg16(DPXREG_DAC_CHANSEL) | (1 << channel));
}


// Disable RAM buffering of a DAC channel
void DPxDisableDacBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxDisableDacBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_BUFF_BAD_CHANNEL);
		return;
	}
	DPxSetReg16(DPXREG_DAC_CHANSEL, DPxGetReg16(DPXREG_DAC_CHANSEL) & ~(1 << channel));	
}


// Disable RAM buffering of all DAC channels
void DPxDisableDacBuffAllChans()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DAC_CHANSEL, 0);	
}


// Returns non-0 if RAM buffering is enabled for a channel
int DPxIsDacBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (channel < 0 || channel > DPX_DAC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxIsDacBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_DAC_NCHANS-1);
		DPxSetError(DPX_ERR_DAC_BUFF_BAD_CHANNEL);
		return 0;
	}
	return DPxGetReg16(DPXREG_DAC_CHANSEL) & (1 << channel);
}


// Enable DAC "raw" mode, causing DAC data to bypass hardware calibration
void DPxEnableDacCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DAC_CTRL, DPxGetReg16(DPXREG_DAC_CTRL) | DPXREG_DAC_CTRL_CALIB_RAW);
}


// Disable DAC "raw" mode, causing normal DAC hardware calibration
void DPxDisableDacCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DAC_CTRL, DPxGetReg16(DPXREG_DAC_CTRL) & ~DPXREG_DAC_CTRL_CALIB_RAW);
}


// Returns non-0 if DAC data is bypassing hardware calibration
int DPxIsDacCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DAC_CTRL) & DPXREG_DAC_CTRL_CALIB_RAW;
}


// Set DAC RAM buffer base address.  Must be an even value.
void DPxSetDacBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DAC_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DAC_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DAC_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get DAC RAM buffer base address
unsigned DPxGetDacBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_BUFF_BASEADDR_L);
}


// Set RAM address from which next DAC datum will be read.  Must be an even value.
void DPxSetDacBuffReadAddr(unsigned buffReadAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffReadAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffReadAddr(0x%x) illegal odd address\n", buffReadAddr);
		DPxSetError(DPX_ERR_DAC_BUFF_ODD_READADDR);
		return;
	}
	if (buffReadAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffReadAddr(0x%x) exceeds DATAPixx RAM\n", buffReadAddr);
		DPxSetError(DPX_ERR_DAC_BUFF_READADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DAC_BUFF_READADDR_L, buffReadAddr);
}


// Get RAM address from which next DAC datum will be read
unsigned DPxGetDacBuffReadAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_BUFF_READADDR_L);
}


// Set DAC RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffReadAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies spooled playback, or the continuous playback of periodic waveforms.
void DPxSetDacBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_DAC_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDacBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_DAC_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_DAC_BUFF_SIZE_L, buffSize);
}


// Get DAC RAM buffer size in bytes
unsigned DPxGetDacBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetDacBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDacBuffBaseAddr(buffAddr);
	DPxSetDacBuffReadAddr(buffAddr);
	DPxSetDacBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first DAC update
void DPxSetDacSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DAC_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first DAC update
unsigned DPxGetDacSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_SCHED_ONSET_L);
}


// Set DAC schedule update rate and units.
// Documentation limits to 1 MHz (but in practice I've tested up to 2 MHz).
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 1 MHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 1 MHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 1000 ns
void DPxSetDacSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 1000000) {
				DPxDebugPrint1("ERROR: DPxSetDacSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DAC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 1000000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetDacSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DAC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 1000) {
				DPxDebugPrint1("ERROR: DPxSetDacSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_DAC_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetDacSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_DAC_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(DPXREG_DAC_SCHED_CTRL_L, (DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_DAC_SCHED_RATE_L,  rateValue);
}


// Get DAC schedule update rate (and optionally get rate units)
unsigned DPxGetDacSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_DAC_SCHED_RATE_L);
}


// Set DAC schedule update count
void DPxSetDacSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DAC_SCHED_COUNT_L,  count);
}


// Get DAC schedule update count
unsigned DPxGetDacSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableDacSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DAC_SCHED_CTRL_L, DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableDacSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DAC_SCHED_CTRL_L, DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsDacSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetDacSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDacSchedOnset(onset);
	DPxSetDacSchedRate(rateValue, rateUnits);
	DPxSetDacSchedCount(count);
	if (count)
		DPxEnableDacSchedCountdown();
	else
		DPxDisableDacSchedCountdown();
}


// Start running a DAC schedule
void DPxStartDacSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DAC)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_DAC));
}


// Stop running a DAC schedule
void DPxStopDacSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DAC)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_DAC));
}


// Returns non-0 if DAC schedule is currently running
int DPxIsDacSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DAC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


/********************************************************************************/
/*																				*/
/*	ADC Subsystem																*/
/*																				*/
/********************************************************************************/


// Returns number of ADC channels in system, excluding REF0/1
int DPxGetAdcNumChans()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPX_ADC_NCHANS;
}


// Get the 16-bit 2's complement signed value for one ADC channel (0-17)
int DPxGetAdcValue(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS+1) {
		DPxDebugPrint2("ERROR: DPxGetAdcValue() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS+1);
		DPxSetError(DPX_ERR_ADC_GET_BAD_CHANNEL);
		return 0;
	}
	return (SInt16)DPxGetReg16(DPXREG_ADC_DATA0+channel*2);
}


// Get the voltage min/max range for an ADC channel (0-17).
// Note that the true max value will really be 1 LSB less than the reported max value.
void DPxGetAdcRange(int channel, double *minV, double *maxV)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (!minV) {
		DPxDebugPrint0("ERROR: DPxGetAdcRange() minV argument is null\n");
		DPxSetError(DPX_ERR_ADC_RANGE_NULL_PTR);
		return;
	}
	if (!maxV) {
		DPxDebugPrint0("ERROR: DPxGetAdcRange() maxV argument is null\n");
		DPxSetError(DPX_ERR_ADC_RANGE_NULL_PTR);
		return;
	}

	if (channel >= 0 && channel <= DPX_ADC_NCHANS+1) {
		*minV = -10;
		*maxV = 10;
	}
	else {
		*minV = -1;		// Use +-1 to protect user against divide by 0
		*maxV = 1;
		DPxDebugPrint2("ERROR: DPxGetAdcRange() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS+1);
		DPxSetError(DPX_ERR_ADC_RANGE_BAD_CHANNEL);
	}
}


// Get the voltage for one ADC channel
double DPxGetAdcVoltage(int channel)
{
	int iValue;
	double minV, maxV;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;

	Return0IfError(iValue = DPxGetAdcValue(channel));
	Return0IfError(DPxGetAdcRange(channel, &minV, &maxV));
	return ((double)iValue + 32768) / 65536 * (maxV - minV) + minV;
}


// Set a channel's differential reference source (only valid for channels 0-15)
// chanRef is one of the following predefined constants:
//		DPXREG_ADC_CHANREF_GND		: Referenced to ground
//		DPXREG_ADC_CHANREF_DIFF		: Referenced to adjacent analog input 
//		DPXREG_ADC_CHANREF_REF0		: Referenced to REF0 analog input
//		DPXREG_ADC_CHANREF_REF1		: Referenced to REF1 analog input
void DPxSetAdcBuffChanRef(int channel, int chanRef)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxSetAdcBuffChanRef() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS-1);
		DPxSetError(DPX_ERR_ADC_REF_BAD_CHANNEL);
		return;
	}
	
	switch (chanRef) {
		case DPXREG_ADC_CHANREF_GND:
		case DPXREG_ADC_CHANREF_DIFF:
		case DPXREG_ADC_CHANREF_REF0:
		case DPXREG_ADC_CHANREF_REF1:
			DPxSetReg32(DPXREG_ADC_CHANREF_L, (DPxGetReg32(DPXREG_ADC_CHANREF_L) & ~(3 << (channel*2))) | (chanRef << (channel*2)));
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetAdcBuffChanRef() unrecognized chanRef %d\n", chanRef);
			DPxSetError(DPX_ERR_ADC_BAD_CHAN_REF);
	}
}


// Get a channel's differential reference source (only valid for channels 0-15)
int DPxGetAdcBuffChanRef(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxGetAdcBuffChanRef() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS-1);
		DPxSetError(DPX_ERR_ADC_REF_BAD_CHANNEL);
		return 0;
	}
	return (DPxGetReg32(DPXREG_ADC_CHANREF_L) >> (channel*2)) & 3;
}


// Enable RAM buffering of an ADC channel (only valid for channels 0-15)
void DPxEnableAdcBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxEnableAdcBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS-1);
		DPxSetError(DPX_ERR_ADC_BUFF_BAD_CHANNEL);
		return;
	}
	DPxSetReg16(DPXREG_ADC_CHANSEL, DPxGetReg16(DPXREG_ADC_CHANSEL) | (1 << channel));	
}


// Disable RAM buffering of an ADC channel (only valid for channels 0-15)
void DPxDisableAdcBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxDisableAdcBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS-1);
		DPxSetError(DPX_ERR_ADC_BUFF_BAD_CHANNEL);
		return;
	}
	DPxSetReg16(DPXREG_ADC_CHANSEL, DPxGetReg16(DPXREG_ADC_CHANSEL) & ~(1 << channel));	
}


// Disable RAM buffering of all ADC channels
void DPxDisableAdcBuffAllChans()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CHANSEL, 0);	
}


// Returns non-0 if RAM buffering is enabled for an ADC channel (only valid for channels 0-15)
int DPxIsAdcBuffChan(int channel)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (channel < 0 || channel > DPX_ADC_NCHANS-1) {
		DPxDebugPrint2("ERROR: DPxIsAdcBuffChan() argument channel %d is not in range 0 to %d\n", channel, DPX_ADC_NCHANS-1);
		DPxSetError(DPX_ERR_ADC_BUFF_BAD_CHANNEL);
		return 0;
	}
	return DPxGetReg16(DPXREG_ADC_CHANSEL) & (1 << channel);
}


// Enable ADC "raw" mode, causing ADC data to bypass hardware calibration
void DPxEnableAdcCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) | DPXREG_ADC_CTRL_CALIB_RAW);
}


// Disable ADC "raw" mode, causing normal ADC hardware calibration
void DPxDisableAdcCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) & ~DPXREG_ADC_CTRL_CALIB_RAW);
}


// Returns non-0 if ADC data is bypassing hardware calibration
int DPxIsAdcCalibRaw()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_ADC_CTRL) & DPXREG_ADC_CTRL_CALIB_RAW;
}


// ADC data readings are looped back internally from programmed DAC voltages:
//		DAC_DATA0 => ADC_DATA0/2/4/6/8/10/12/14
//		DAC_DATA1 => ADC_DATA1/3/5/7/9/11/13/15
//		DAC_DATA2 => ADC_REF0
//		DAC_DATA3 => ADC_REF1
void DPxEnableDacAdcLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) | DPXREG_ADC_CTRL_DAC_LOOPBACK);
}


// Disable ADC loopback, causing ADC readings to reflect real analog inputs
void DPxDisableDacAdcLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) & ~DPXREG_ADC_CTRL_DAC_LOOPBACK);
}


// Returns non-0 if ADC inputs are looped back from DAC outputs
int DPxIsDacAdcLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_ADC_CTRL) & DPXREG_ADC_CTRL_DAC_LOOPBACK;
}


// ADC's convert continuously (can add up to 4 microseconds random latency to scheduled samples)
void DPxEnableAdcFreeRun()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) | DPXREG_ADC_CTRL_FREE_RUN);
}


// ADC's only convert on schedule ticks (for microsecond-precise sampling)
void DPxDisableAdcFreeRun()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_ADC_CTRL, DPxGetReg16(DPXREG_ADC_CTRL) & ~DPXREG_ADC_CTRL_FREE_RUN);
}


// Returns non-0 if ADC's are performing continuous conversions
int DPxIsAdcFreeRun()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_ADC_CTRL) & DPXREG_ADC_CTRL_FREE_RUN;
}


// Set ADC RAM buffer start address.  Must be an even value.
void DPxSetAdcBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_ADC_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_ADC_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_ADC_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get ADC RAM buffer start address
unsigned DPxGetAdcBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_BUFF_BASEADDR_L);
}


// Set RAM address to which next ADC datum will be written.  Must be an even value.
void DPxSetAdcBuffWriteAddr(unsigned buffWriteAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffWriteAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffWriteAddr(0x%x) illegal odd address\n", buffWriteAddr);
		DPxSetError(DPX_ERR_ADC_BUFF_ODD_WRITEADDR);
		return;
	}
	if (buffWriteAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffWriteAddr(0x%x) exceeds DATAPixx RAM\n", buffWriteAddr);
		DPxSetError(DPX_ERR_ADC_BUFF_WRITEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_ADC_BUFF_WRITEADDR_L, buffWriteAddr);
}


// Get RAM address to which next ADC datum will be written
unsigned DPxGetAdcBuffWriteAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_BUFF_WRITEADDR_L);
}


// Set ADC RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffWriteAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies continuous spooled acquisition.
void DPxSetAdcBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_ADC_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAdcBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_ADC_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_ADC_BUFF_SIZE_L, buffSize);
}


// Get ADC RAM buffer size in bytes
unsigned DPxGetAdcBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetAdcBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAdcBuffBaseAddr(buffAddr);
	DPxSetAdcBuffWriteAddr(buffAddr);
	DPxSetAdcBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first ADC sample
void DPxSetAdcSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first ADC sample
unsigned DPxGetAdcSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_SCHED_ONSET_L);
}


// Set ADC schedule sample rate and units
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 200 kHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 200 kHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 5000 ns
void DPxSetAdcSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 200000) {
				DPxDebugPrint1("ERROR: DPxSetAdcSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_ADC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 200000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetAdcSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_ADC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 5000) {
				DPxDebugPrint1("ERROR: DPxSetAdcSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_ADC_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetAdcSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_ADC_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(DPXREG_ADC_SCHED_CTRL_L, (DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_ADC_SCHED_RATE_L,  rateValue);
}


// Get ADC schedule update rate (and optionally get rate units)
unsigned DPxGetAdcSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_ADC_SCHED_RATE_L);
}


// Set ADC schedule update count
void DPxSetAdcSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_COUNT_L,  count);
}


// Get ADC schedule update count
unsigned DPxGetAdcSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableAdcSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_CTRL_L, DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableAdcSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_CTRL_L, DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsAdcSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetAdcSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAdcSchedOnset(onset);
	DPxSetAdcSchedRate(rateValue, rateUnits);
	DPxSetAdcSchedCount(count);
	if (count)
		DPxEnableAdcSchedCountdown();
	else
		DPxDisableAdcSchedCountdown();
}


// Start running an ADC schedule
void DPxStartAdcSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_ADC)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_ADC));
}


// Stop running an ADC schedule
void DPxStopAdcSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_ADC)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_ADC));
}


// Returns non-0 if ADC schedule is currently running
int DPxIsAdcSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Each buffered ADC sample is preceeded with a 64-bit nanosecond timetag
void DPxEnableAdcLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_CTRL_L, DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Buffered data has no timetags
void DPxDisableAdcLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_ADC_SCHED_CTRL_L, DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
int DPxIsAdcLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_ADC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_TIMETAG;
}


/********************************************************************************/
/*																				*/
/*	DOUT (Digital Output) Subsystem												*/
/*																				*/
/********************************************************************************/


// Returns number of digital output bits in system (24 in current implementation)
int DPxGetDoutNumBits()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return 24;
}


// For each of the 24 bits set in bitMask, set the DOUT to the value in the corresponding bitValue.
void DPxSetDoutValue(int bitValue, int bitMask)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	// If user specified unimplemented bits, it's not a fatal error so don't return.
	// At least set the implemented bits.
	if (bitMask & 0xFF000000) {
		DPxDebugPrint2("ERROR: DPxSetDoutValue() argument bitMask %08X includes unimplemented bits %08X\n", bitMask, bitMask & 0xFF000000);
		DPxSetError(DPX_ERR_DOUT_SET_BAD_MASK);
	}

	// Internally, the 24 DOUT bits are implemented as a single 32-bit register.
	// Note that a user write, and a schedule, can step over each other.
	if (bitMask)
		DPxSetReg32(DPXREG_DOUT_DATA_L, (DPxGetReg32(DPXREG_DOUT_DATA_L) & ~bitMask) | (bitValue & bitMask));
}


// Get the values of the 24 DOUT bits
int DPxGetDoutValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_DATA_L);
}


// Enable automatic DOUT schedules upon DIN button presses
void DPxEnableDoutButtonSchedules()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) | DPXREG_DOUT_CTRL_BUTTON_SCHEDULES);
}


// Disable automatic DOUT schedules upon DIN button presses
void DPxDisableDoutButtonSchedules()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) & ~DPXREG_DOUT_CTRL_BUTTON_SCHEDULES);
}


// Returns non-0 if automatic DOUT schedules occur upon DIN button presses
int DPxIsDoutButtonSchedules()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DOUT_CTRL) & DPXREG_DOUT_CTRL_BUTTON_SCHEDULES;
}


// LCD backlight LED enables are gated by DOUT15.  Can be used to make a tachistoscope by pulsing DOUT15 with a schedule.
void DPxEnableDoutBacklightPulse()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) | DPXREG_DOUT_CTRL_BACKLIGHT_PULSE);
}


// LCD backlight LEDs are unaffected by DOUT system.
void DPxDisableDoutBacklightPulse()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) & ~DPXREG_DOUT_CTRL_BACKLIGHT_PULSE);
}


// Returns non-0 if LCD backlight LED enables are gated by DOUT15
int DPxIsDoutBacklightPulse()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return 0;
    
	return DPxGetReg16(DPXREG_DOUT_CTRL) & DPXREG_DOUT_CTRL_BACKLIGHT_PULSE;
}


// Digital outputs show RGB value of first upper left pixel of the screen
void DPxEnableDoutPixelMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return;
    }
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) | DPXREG_DOUT_CTRL_PIXEL_MODE);
}


// Digital outputs back to normal mode
void DPxDisableDoutPixelMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return;
    }
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) & ~DPXREG_DOUT_CTRL_PIXEL_MODE);
}


// Returns non-0 if Digital Out pixel mode is enabled
int DPxIsDoutPixelMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return 0;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return 0;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return 0;
    }
    
	return DPxGetReg16(DPXREG_DOUT_CTRL) & DPXREG_DOUT_CTRL_PIXEL_MODE;
}

// Enable Digital outputs DOUT VSYNC Mode where BIT#23 shows the VSYNC value
void DPxEnableDoutVsyncMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return;
    }
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) | DPXREG_DOUT_CTRL_VSYNC_MODE);
}


// Disable DOUT VSYNC mode
void DPxDisableDoutVsyncMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return;
    }
    
	DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) & ~DPXREG_DOUT_CTRL_VSYNC_MODE);
}


// Returns non-0 if DOUT VSYNC mode is enabled -- VIEWPixx Rev >= 46 only
int DPxIsDoutVsyncMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return 0;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return 0;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return 0;
    }
    
	return DPxGetReg16(DPXREG_DOUT_CTRL) & DPXREG_DOUT_CTRL_VSYNC_MODE;
}


void DPxSetDoutButtonSchedulesMode(int mode)
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return;
    }
    
	if (mode == DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE_RISING_EDGE)
		DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) | DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE);
	else
		DPxSetReg16(DPXREG_DOUT_CTRL, DPxGetReg16(DPXREG_DOUT_CTRL) & ~DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE);
}



unsigned DPxGetDoutButtonSchedulesMode()
{
    if (DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return 0;
    }
    else if (DPxIsDatapixx2()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
            return 0;
    }
    else {
        if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
            return 0;
    }
    
	return DPxGetReg16(DPXREG_DOUT_CTRL) & DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE;
}


// Set DOUT RAM buffer start address.  Must be an even value.
void DPxSetDoutBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DOUT_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DOUT_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DOUT_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get DOUT RAM buffer start address
unsigned DPxGetDoutBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_BUFF_BASEADDR_L);
}


// Set RAM address from which next DOUT datum will be read.  Must be an even value.
void DPxSetDoutBuffReadAddr(unsigned buffReadAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffReadAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffReadAddr(0x%x) illegal odd address\n", buffReadAddr);
		DPxSetError(DPX_ERR_DOUT_BUFF_ODD_READADDR);
		return;
	}
	if (buffReadAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffReadAddr(0x%x) exceeds DATAPixx RAM\n", buffReadAddr);
		DPxSetError(DPX_ERR_DOUT_BUFF_READADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DOUT_BUFF_READADDR_L, buffReadAddr);
}


// Get RAM address from which next DOUT datum will be read
unsigned DPxGetDoutBuffReadAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_BUFF_READADDR_L);
}




// Set DOUT RAM buffer size in bytes.  Must be an even value.  Buffer wraps to Base after Size.
// The hardware will automatically wrap the BuffReadAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies spooled playback, or the continuous playback of periodic waveforms.
void DPxSetDoutBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_DOUT_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDoutBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_DOUT_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_DOUT_BUFF_SIZE_L, buffSize);
}


// Get DOUT RAM buffer size in bytes
unsigned DPxGetDoutBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetDoutBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDoutBuffBaseAddr(buffAddr);
	DPxSetDoutBuffReadAddr(buffAddr);
	DPxSetDoutBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first DOUT update
void DPxSetDoutSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DOUT_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first DOUT update
unsigned DPxGetDoutSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_SCHED_ONSET_L);
}


// Set DOUT schedule update rate and units.
// Documentation limits to 10 MHz (but in practice I've tested higher, even 100 MHz if waveform fits inside 1 64B cacheline).
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 10 MHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 10 MHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 100 ns
void DPxSetDoutSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 10000000) {
				DPxDebugPrint1("ERROR: DPxSetDoutSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DOUT_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 10000000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetDoutSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DOUT_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 100) {
				DPxDebugPrint1("ERROR: DPxSetDoutSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_DOUT_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetDoutSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_DOUT_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(DPXREG_DOUT_SCHED_CTRL_L, (DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_DOUT_SCHED_RATE_L,  rateValue);
}


// Get DOUT schedule update rate (and optionally get rate units)
unsigned DPxGetDoutSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_DOUT_SCHED_RATE_L);
}


// Set DOUT schedule update count
void DPxSetDoutSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DOUT_SCHED_COUNT_L,  count);
}


// Get DOUT schedule update count
unsigned DPxGetDoutSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableDoutSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DOUT_SCHED_CTRL_L, DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableDoutSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DOUT_SCHED_CTRL_L, DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsDoutSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetDoutSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDoutSchedOnset(onset);
	DPxSetDoutSchedRate(rateValue, rateUnits);
	DPxSetDoutSchedCount(count);
	if (count)
		DPxEnableDoutSchedCountdown();
	else
		DPxDisableDoutSchedCountdown();
}


// Start running a DOUT schedule
void DPxStartDoutSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DOUT)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_DOUT));
}


// Stop running a DOUT schedule
void DPxStopDoutSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DOUT)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_DOUT));
}


// Returns non-0 if DOUT schedule is currently running
int DPxIsDoutSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DOUT_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


/********************************************************************************/
/*																				*/
/*	DIN (Digital Input) Subsystem												*/
/*																				*/
/********************************************************************************/


// Returns number of digital input bits in system (24 in current implementation)
int DPxGetDinNumBits()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return 24;
}


// Get the values of the 24 DIN bits
int DPxGetDinValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_DATA_L);
}


// Set 24-bit port direction mask.  Set mask bits to 1 for each bit which should drive its port.
void DPxSetDinDataDir(int directionMask)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	// If user specified unimplemented bits, it's not a fatal error so don't return.
	// At least set the implemented bits.
	if (directionMask & 0xFF000000) {
		DPxDebugPrint2("ERROR: DPxSetDinDataDir() argument directionMask %08X includes unimplemented bits %08X\n", directionMask, directionMask & 0xFF000000);
		DPxSetError(DPX_ERR_DIN_SET_BAD_MASK);
	}
	DPxSetReg32(DPXREG_DIN_DIR_L, directionMask);
}


// Get 24-bit port direction mask
int DPxGetDinDataDir()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_DIR_L);
}


// Set the data which should be driven on each port whose output has been enabled by DPxSetDinDataDir()
void DPxSetDinDataOut(int dataOut)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_DATAOUT_L, dataOut);
}


// Get the data which is being driven on each output port
int DPxGetDinDataOut()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_DATAOUT_L);
}


// Set drive strength of driven outputs.  Range is 0-1.  Implementation uses 1/16 up to 16/16.
void DPxSetDinDataOutStrength(double strength)
{
	int iStrength;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (strength < 0 || strength > 1) {
		DPxDebugPrint1("ERROR: DPxSetDinDataOutStrength(%f) illegal value\n", strength);
		DPxSetError(DPX_ERR_DIN_BAD_STRENGTH);
		return;
	}

	// Convert to 0/16 to 16/16.
	iStrength = (int)floor(strength * 16 + 0.5);
	
	// 0/16 has to map to 1/16 (the minimum value),
	// and 16/16 has to map to a register value of 0, which means full strength.
	if (iStrength == 0)
		iStrength = 1;
	else if (iStrength == 16)
		iStrength = 0;

	// And write the value to the register
	DPxSetReg16(DPXREG_DIN_CTRL, (DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_PWM) | (iStrength << 8));
}


// Get drive strength of driven outputs.  Range is 0-1.  Implementation uses 1/16 up to 16/16.
double DPxGetDinDataOutStrength()
{
    int pwmReg;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	pwmReg = (DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_PWM) >> 8;
	return pwmReg ? pwmReg/16.0 : 1.0;
}


// DIN transitions are only recognized after entire DIN bus has been stable for 80 ns.
// (good for deskewing parallel busses, and ignoring transmission line reflections).
void DPxEnableDinStabilize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_STABILIZE);
}


// Immediately recognize all DIN transitions
void DPxDisableDinStabilize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_STABILIZE);
}


// Returns non-0 if DIN transitions are being debounced
int DPxIsDinStabilize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_STABILIZE;
}


// When a DIN transitions, ignore further DIN transitions for next 30 milliseconds (good for response buttons)
void DPxEnableDinDebounce()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_DEBOUNCE);
}


// Immediately recognize all DIN transitions
void DPxDisableDinDebounce()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_DEBOUNCE);
}


// Returns non-0 if DIN transitions are being debounced
int DPxIsDinDebounce()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_DEBOUNCE;
}


// Enable loopback between digital output ports and digital inputs
void DPxEnableDoutDinLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_DOUT_LOOPBACK);
}


// Disable loopback between digital outputs and digital inputs
void DPxDisableDoutDinLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_DOUT_LOOPBACK);
}


// Returns non-0 if digital inputs are driven by digital output ports
int DPxIsDoutDinLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_DOUT_LOOPBACK;
}


// Set DIN RAM buffer start address.  Must be an even value.
void DPxSetDinBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DIN_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_DIN_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DIN_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get DIN RAM buffer start address
unsigned DPxGetDinBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_BUFF_BASEADDR_L);
}


// Set RAM address to which next DIN datum will be written.  Must be an even value.
void DPxSetDinBuffWriteAddr(unsigned buffWriteAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffWriteAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffWriteAddr(0x%x) illegal odd address\n", buffWriteAddr);
		DPxSetError(DPX_ERR_DIN_BUFF_ODD_WRITEADDR);
		return;
	}
	if (buffWriteAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffWriteAddr(0x%x) exceeds DATAPixx RAM\n", buffWriteAddr);
		DPxSetError(DPX_ERR_DIN_BUFF_WRITEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_DIN_BUFF_WRITEADDR_L, buffWriteAddr);
}


// Get RAM address to which next DIN datum will be written
unsigned DPxGetDinBuffWriteAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_BUFF_WRITEADDR_L);
}


// Set DIN RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffWriteAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies continuous spooled acquisition.
void DPxSetDinBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_DIN_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetDinBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_DIN_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_DIN_BUFF_SIZE_L, buffSize);
}


// Get DIN RAM buffer size in bytes
unsigned DPxGetDinBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetDinBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDinBuffBaseAddr(buffAddr);
	DPxSetDinBuffWriteAddr(buffAddr);
	DPxSetDinBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first DIN sample
void DPxSetDinSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first DIN sample
unsigned DPxGetDinSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_ONSET_L);
}


// Set DIN schedule sample rate and units
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 1 MHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 1 MHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 1000 ns
// DIN scheduling is probably good to 2 MHz now.  I can certainly log edges at about 3 MHz.
// If I want to do any better, I'll have to stop flushing DPR cache to RAM after every sample.
void DPxSetDinSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 1000000) {
				DPxDebugPrint1("ERROR: DPxSetDinSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DIN_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 1000000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetDinSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_DIN_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 1000) {
				DPxDebugPrint1("ERROR: DPxSetDinSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_DIN_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetDinSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_DIN_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, (DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_DIN_SCHED_RATE_L,  rateValue);
}


// Get DIN schedule update rate (and optionally get rate units)
unsigned DPxGetDinSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_DIN_SCHED_RATE_L);
}


// Set DIN schedule update count
void DPxSetDinSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_COUNT_L,  count);
}


// Get DIN schedule update count
unsigned DPxGetDinSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableDinSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableDinSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsDinSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetDinSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetDinSchedOnset(onset);
	DPxSetDinSchedRate(rateValue, rateUnits);
	DPxSetDinSchedCount(count);
	if (count)
		DPxEnableDinSchedCountdown();
	else
		DPxDisableDinSchedCountdown();
}


// Start running an DIN schedule
void DPxStartDinSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DIN)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_DIN));
}


// Stop running an DIN schedule
void DPxStopDinSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_DIN)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_DIN));
}


// Returns non-0 if DIN schedule is currently running
int DPxIsDinSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Each buffered DIN sample is preceeded with a 64-bit nanosecond timetag
void DPxEnableDinLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Buffered data has no timetags
void DPxDisableDinLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_TIMETAG);
}


// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
int DPxIsDinLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_TIMETAG;
}


// Each DIN transition is automatically logged (no schedule is required.  Best way to log response buttons)
void DPxEnableDinLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_EVENTS);
    DPxDisableTouchpixxLogEvents(); // Can't have both running simultaneously
}


// Disable automatic logging of DIN transitions
void DPxDisableDinLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_EVENTS);
}


// Returns non-0 if DIN transitions are being logged to RAM buffer
int DPxIsDinLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_EVENTS;
}


/********************************************************************************/
/*																				*/
/*	TOUCHPixx Subsystem															*/
/*																				*/
/********************************************************************************/


void DPxEnableTouchpixx()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_TOUCHPIXX);
}


void DPxDisableTouchpixx()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_TOUCHPIXX);
}


int DPxIsTouchpixx()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_TOUCHPIXX;
}


// Set TOUCHPixx RAM buffer start address.  Must be an even value.
void DPxSetTouchpixxBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxSetDinBuffBaseAddr(buffBaseAddr);
}


// Get TOUCHPixx RAM buffer start address
unsigned DPxGetTouchpixxBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetDinBuffBaseAddr();
}


// Set RAM address to which next TOUCHPixx datum will be written.  Must be an even value.
void DPxSetTouchpixxBuffWriteAddr(unsigned buffWriteAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxSetDinBuffWriteAddr(buffWriteAddr);
}


// Get RAM address to which next TOUCHPixx datum will be written
unsigned DPxGetTouchpixxBuffWriteAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
    return DPxGetDinBuffWriteAddr();
}


// Set TOUCHPixx RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffWriteAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies continuous spooled acquisition.
void DPxSetTouchpixxBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxSetDinBuffSize(buffSize);
}


// Get TOUCHPixx RAM buffer size in bytes
unsigned DPxGetTouchpixxBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
    return DPxGetDinBuffSize();
}


// Shortcut which assigns Size/BaseAddr/WriteAddr
void DPxSetTouchpixxBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxSetDinBuff(buffAddr, buffSize);
}


void DPxEnableTouchpixxLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_LOG_TOUCHPIXX);
    DPxDisableDinLogEvents(); // Can't have both running simultaneously
}


void DPxDisableTouchpixxLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_DIN_SCHED_CTRL_L, DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_LOG_TOUCHPIXX);
}


int DPxIsTouchpixxLogEvents()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_DIN_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_LOG_TOUCHPIXX;
}


// Each buffered TOUCHPixx sample is preceeded with a 64-bit nanosecond timetag
void DPxEnableTouchpixxLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxEnableDinLogTimetags();
}


// Buffered data has no timetags
void DPxDisableTouchpixxLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
    DPxDisableDinLogTimetags();
}


// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
int DPxIsTouchpixxLogTimetags()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
    return DPxIsDinLogTimetags();
}


void DPxEnableTouchpixxLogContinuousMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_TOUCHPIXX_CONT);
}


void DPxDisableTouchpixxLogContinuousMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_TOUCHPIXX_CONT);
}


int DPxIsTouchpixxLogContinuousMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_TOUCHPIXX_CONT;
}


void DPxSetTouchpixxType(int touchPanelMode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;

    if (touchPanelMode) 
		DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) | DPXREG_DIN_CTRL_TOUCHPIXX_TYPE); // Capacitive, type == 1
	else
		DPxSetReg16(DPXREG_DIN_CTRL, DPxGetReg16(DPXREG_DIN_CTRL) & ~DPXREG_DIN_CTRL_TOUCHPIXX_TYPE); // Resistive, type == 0
}


int DPxGetTouchpixxType(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return (DPxGetReg16(DPXREG_DIN_CTRL) & DPXREG_DIN_CTRL_TOUCHPIXX_TYPE) >> 14;
}


// Stabilization should default to off.
// This causes the least surprise to someone who presses the touch panel,
// then calls IsTouchpixxPressed() or DPxGetTouchpixxCoords() expecting to get a nice touch.
double touchpixxStabilizeDuration = 0.0;

void DPxSetTouchpixxStabilizeDuration(double duration)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	touchpixxStabilizeDuration = duration;
}


double DPxGetTouchpixxStabilizeDuration()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return touchpixxStabilizeDuration;
}


// Returns non-0 if touch panel is pressed
int DPxIsTouchpixxPressed()
{
	int x, y;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	DPxGetTouchpixxCoords(&x, &y);
    if (x == 0 && y == 0)
		return	0;
	else
		return	1;
}


#define	TOUCHPIXX_STABILIZE_DISTANCE 1500


void DPxGetTouchpixxCoords(int* x, int* y)
{
	static int lastXRead = 0, lastYRead = 0;
	static double startTime = 0;
	static int minValX, maxValX, minValY, maxValY;
	
	int currentX = 0, currentY = 0;
	double currentTimer;	
    
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	currentTimer = DPxGetTime();
	currentX = DPxGetReg16(DPXREG_DIN_DATAOUT_L);
	currentY = DPxGetReg16(DPXREG_DIN_DATAOUT_H);
    
	if (currentX == 0 && currentY == 0)
		startTime = DPxGetTime();	// reset timer
    
	//printf("\nStart=%8.3f Now=%8.3f   X= %6d LastX= %6d  ***  Y= %6d  LastY= %6d", startTime, currentTimer, currentX, lastXRead, currentY, lastYRead);
    
	// New pressed touch screen detected
	if (lastXRead == 0 && lastYRead == 0 && currentX != 0 && currentY != 0)
	{
		startTime = DPxGetTime();
        if (DPxGetTouchpixxStabilizeDuration() == 0) {
            *x = currentX;
            *y = currentY;
        }
        else {
            *x = 0;
            *y = 0;
        }
	}
	
	//was touched and still touched
	else if (lastXRead != 0 && lastYRead != 0 && currentX != 0 && currentY != 0) 
	{
		// Outside if limits!
		if (currentX > maxValX || currentX < minValX || currentY > maxValY || currentY < minValY)
		{
            startTime = DPxGetTime();
			*x = 0;
			*y = 0;
		}
		else
		{
			// wanted time is reached?
			if (currentTimer >= (startTime + DPxGetTouchpixxStabilizeDuration()))
			{
				*x = currentX;
				*y = currentY;
			}
			else
			{
				// we still return 0s because time is not reached...
				*x = 0;
				*y = 0;
			}
		}
	}
	// was not touched and not still touched
	else
	{
		*x = 0;
		*y = 0;
	}
    
    lastXRead = currentX;
    lastYRead = currentY;
    minValX = currentX - TOUCHPIXX_STABILIZE_DISTANCE;
    maxValX = currentX + TOUCHPIXX_STABILIZE_DISTANCE;
    minValY = currentY - TOUCHPIXX_STABILIZE_DISTANCE;
    maxValY = currentY + TOUCHPIXX_STABILIZE_DISTANCE;
}


/********************************************************************************/
/*																				*/
/*	AUD (Audio Output) Subsystem												*/
/*																				*/
/********************************************************************************/

// Call DPxInitAudCodec() once before other Aud/Mic routines, to configure initial audio CODEC state.
// Can also call this at any time to return CODEC to its initial state.
// Note that the first time this routine is called after reset,
// it can pause up to 0.6 seconds while CODEC internal amplifiers are powering up.
// This delay garantees that the CODEC is ready for stable playback immediately upon return.
void DPxInitAudCodec()
{
    int iRead;
	double timer;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    

	// Here's the thing.  It appears that the CODEC output can hang if we update timing registers while DACs are powered up.
	// I especially see this when going from non-double-rate to double-rate mode, but I also see this when only programming divisor.
	// When the CODEC hangs, it seems to never generate audio out until it has been reprogrammed back to non-double-rate mode.
	// It seems that I can prevent this from happening by powering down the DACs while reprogramming the rate or divisor.
	// ADC's also need to be powered down, or audio input and output can both get screwed up.
	// Optimal solution here is just to read the register.
	// InitAudCodec is called rarely, so don't worry about a few milliseconds to read register 2.
	// We take the time to read the register so that we can avoid the pop caused by DAC shutdown.
	if (DPxGetCodecReg(2) != 0x22) {
		DPxSetCodecReg(19, 0x78);	// Powerdown Left ADC
		DPxSetCodecReg(22, 0x78);	// Powerdown Right ADC
		DPxSetCodecReg(37, 0x20);	// Powerdown L/R DACs, configure HPLCOM as an independant output (goes to DP speaker)

        // ***This caused Michael Barnett Cowan's bug...
        // Hmmm.  CODEC doesn't always power down when we tell it to.  Maybe when we've stopped MCLK?
        // The final solution is just to stay in double-rate mode.  It can present 8 kHz audio samples w/o distortion.
        // Also, user might program this rate during an animation.  We don't want to be hanging here.
        // ...Unfortunately, it started hanging again with VHDL rev 7.
        // Now I'm putting back this wait, and it doesn't seem to hang anymore.
        // I've counted that it takes anywhere from 3 to 21 reads of register 94 before it says IC has powered down.
        // Note that we only stay here when the user is actually _changing_ the frequency,
        // so the user can still call this function a million times during an animation,
        // and we'll only pause here if we're really changing the rate.
        // After long-run tests, the CODEC seems rock solid now that I've enabled this wait.
        // Never seems to stop producing sound.
        // NOPE.  If VPixx program calls this at the beginning of each trial, then sometimes I see the program hanging here.
        // Not quite sure what the best strategy would be now.
        // We'll try just waiting here for a maximum of 25 times, since emperical tests show that when it succeeds it needs a maximum of 21 reads.
        //		while ((DPxGetCodecReg(94) & 0xC0) != 0x00)
        //			(void)0;
        for (iRead = 0; iRead < 25; iRead++) {
            if ((DPxGetCodecReg(94) & 0xC0) == 0x00) {
                break;
            }
        }
	}
    
	// Stereo DAC outputs are routed to either DAC_L/R1 or DAC_L/R2 or DAC_L/R3.
	// DAC_L/R1 are general purpose internal routing, which can go to any of the output channels.
	// DAC_L/R2 are dedicated routings to the HPL/ROUT (high-power outputs).
	// DAC_L/R3 are dedicated routings to the LEFT/RIGHT_LO (line outputs).
	// So, we definitely have to route to DAC_L/R1.
	DPxSetCodecReg( 0, 0x00);	// Ensure that we are programming page 0
//	DPxSetCodecReg( 1, 0x80);	// Perform self-clearing software reset.  NOT.  Causes CODEC to fail later.  I would probably need a delay after this reset.
	DPxSetCodecReg( 2, 0x22);	// DAC/ADC Fs = Fsref/2.  This is what the DP configures itself to on powerup reset.
	DPxSetCodecReg( 3, 0x20);	// PLL disabled, Q=4 (4 is the minimum value for double-rate DAC mode)

	//  Not using CODEC PLL anymore, so just program with reset values
	DPxSetCodecReg( 4, 0x04);
	DPxSetCodecReg( 5, 0x00);
	DPxSetCodecReg( 6, 0x00);

	// CODEC can hang if it is programmed for dual mode,
	// and we reprogram for non-dual mode without powering down the DACs.
	// For this reason, we are ALWAYS in dual mode!
	DPxSetCodecReg( 7, 0x6A);	// L/R DAC datapaths play left/right channel input data, dual-rate mode. ***If I don't spec dual-rate here, and CODEC is doing dual-rate now, can hang CODEC!
	DPxSetCodecReg( 8, 0x00);	// BCLK and WCLK are inputs (slave mode)
	DPxSetCodecReg( 9, 0x4E);	// Serial bus uses DSP mode with 16-bit L/R data.  x256 BCLK, Resync DAC/ADC if group delay drifts (gives smaller catchup pops).
	DPxSetCodecReg(10, 0x00);	// Serial bus data has 0 BCLK offset from WCLK pulses
	DPxSetCodecReg(11, 0x01);	// PLL R=1
	DPxSetCodecReg(12, 0x00);	// Disable ADC highpass filters, DAC digital effects, DAC de-emphasis filters
	DPxSetCodecReg(14, 0x80);	// Configure high-power outputs for AC-coupled.  Don't know what this does, but headphones are AC coupled.

	// ADC setup
	DPxSetCodecReg(15,   80);	// Left ADC PGA not muted, and gain is 40 dB (good for microphone).
	DPxSetCodecReg(16,   80);	// Right ADC PGA not muted, and gain is also 40 dB.
	DPxSetCodecReg(17, 0xFF);	// MIC3 is not connected to Left ADC
	DPxSetCodecReg(18, 0xFF);	// MIC3 is not connected to Right ADC
	DPxSetCodecReg(19, 0x04);	// Connect LINE1L (MIC) to left ADC, and powerup ADC.
	DPxSetCodecReg(20, 0x78);	// LINE2L (audio line in left) not connected to Left ADC
	DPxSetCodecReg(21, 0x78);	// MIC1R/LINE1R not connected to Left ADC
	DPxSetCodecReg(22, 0x04);	// Connect LINE1R (MIC) to right ADC, and powerup ADC.
	DPxSetCodecReg(23, 0x78);	// LINE2R (audio line in right) not connected to Right ADC
	DPxSetCodecReg(24, 0x78);	// MIC1L/LINE1L not connected to Right ADC
	DPxSetCodecReg(25, 0x40);	// 2.0V MIC bias
	DPxSetCodecReg(26, 0x00);	// No AGC Left.  Online groups say AGC can go into space and never come back...
	DPxSetCodecReg(27, 0x00);	// No AGC Left
	DPxSetCodecReg(28, 0x00);	// No AGC Left
	DPxSetCodecReg(29, 0x00);	// No AGC Right
	DPxSetCodecReg(30, 0x00);	// No AGC Right
	DPxSetCodecReg(31, 0x00);	// No AGC Right
	DPxSetCodecReg(32, 0x00);	// Left AGC gain 0 dB
	DPxSetCodecReg(33, 0x00);	// Right AGC gain 0 dB
	DPxSetCodecReg(34, 0x00);	// Left AGC debounce off
	DPxSetCodecReg(35, 0x00);	// Right AGC debounce off

	//DPxSetCodecReg(36, 0x00);	// A read-only status register
	DPxSetCodecReg(37, 0xE0);	// Powerup L/R DACs, configure HPLCOM as an independant output (goes to DP speaker)
	DPxSetCodecReg(38, 0x1C);	// HPRCOM is differential of HPLCOM (goes to DP speaker), and enable short-circuit protection on HP outputs.
	DPxSetCodecReg(40, 0x80);	// Output common-mode voltage is 1.65V.  Should give greatest swing and power with 3.3V supplies.  Enable some volume soft-stepping to reduce pop when setting volume.
	DPxSetCodecReg(41, 0x00);	// Left/Right DAC outputs routed to DAC_L/R1 paths, and Left/Right DAC channels have independent volume controls.
	DPxSetCodecReg(42, 0x8C);	// 400 ms driver poweron, 4 ms driver rampup step.  Supposed to reduce pop.  I don't see any diff.
	DPxSetCodecReg(43, 0x00);	// Left DAC at full volume
	DPxSetCodecReg(44, 0x00);	// Right DAC at full volume
	DPxSetCodecReg(45, 0x00);	// LINE2L is not routed to HPLOUT
	DPxSetCodecReg(46, 0x00);	// PGA_L is not routed to HPLOUT
	DPxSetCodecReg(47, 0xA8);	// DAC_L1 is routed to HPLOUT at full volume.  NOPE.  Reducing to -8 dB removes a lot of hiss.  -20 dB for safely.
	DPxSetCodecReg(48, 0x00);	// LINE2R is not routed to HPLOUT
	DPxSetCodecReg(49, 0x00);	// PGA_R is not routed to HPLOUT
	DPxSetCodecReg(50, 0x00);	// DAC_R1 is not routed to HPLOUT
	DPxSetCodecReg(51, 0x09);	// HPLOUT is full power, no overdrive
	DPxSetCodecReg(52, 0x00);	// LINE2L is not routed to HPLCOM
	DPxSetCodecReg(53, 0x00);	// PGA_L is not routed to HPLCOM
	DPxSetCodecReg(54, 0x90);	// DAC_L1 is routed to HPLCOM at -8dB.  -6dB is half, but seems to saturate my speaker, peaks cause large oscillations.
	DPxSetCodecReg(55, 0x00);	// LINE2R is not routed to HPLCOM
	DPxSetCodecReg(56, 0x00);	// PGA_R is not routed to HPLCOM
	DPxSetCodecReg(57, 0x90);	// DAC_R1 is routed to HPLCOM at -8dB.  -6dB is half, but seems to saturate my speaker, peaks cause large oscillations.
//	DPxSetCodecReg(58, 0x01);	// Keep HPLCOM powered down until a schedule starts; otherwise FPGA is not driving I2S, and speaker gets a DC voltage and hisses.
	DPxSetCodecReg(58, 0x09);	// Now VHDL drives I2S from reset, so now HPLCOM is full power, no overdrive
	DPxSetCodecReg(59, 0x00);	// LINE2L is not routed to HPROUT
	DPxSetCodecReg(60, 0x00);	// PGA_L is not routed to HPROUT
	DPxSetCodecReg(61, 0x00);	// DAC_L1 is not routed to HPROUT
	DPxSetCodecReg(62, 0x00);	// LINE2R is not routed to HPROUT
	DPxSetCodecReg(63, 0x00);	// PGA_R is not routed to HPROUT
	DPxSetCodecReg(64, 0xA8);	// DAC_R1 is routed to HPROUT at full volume.  NOPE.  Reducing to -8 dB removes a lot of hiss.  -20 dB for safely.
	DPxSetCodecReg(65, 0x09);	// HPROUT is full power, no overdrive
	DPxSetCodecReg(66, 0x00);	// LINE2L is not routed to HPRCOM
	DPxSetCodecReg(67, 0x00);	// PGA_L is not routed to HPRCOM
	DPxSetCodecReg(68, 0x00);	// DAC_L1 is not routed to HPRCOM
	DPxSetCodecReg(69, 0x00);	// LINE2R is not routed to HPRCOM
	DPxSetCodecReg(70, 0x00);	// PGA_R is not routed to HPRCOM
	DPxSetCodecReg(71, 0x80);	// DAC_R1 is not routed to HPRCOM
	DPxSetCodecReg(72, 0x09);	// HPROUT is full power, no overdrive
	DPxSetCodecReg(101, 0x01);	// CODEC_CLKIN comes from CLKDIV_OUT, instead of PLL_OUT
	DPxSetCodecReg(102, 0x02);	// CLKDIV_IN comes from MCLK

	// The CODEC has internal subsystems which can be powered up and down (for power savings in mobile apps).
	// If the internal systems were powered down before the call to DPxInitAudCodec(), then they are now powering up.
	// We will wait here until the systems have fully powered up, so the user is free to start using the CODEC immediately upon return.
	// Measurements show that the DACs power up fast, but the HPLOUT/HPROUT/HPLCOM/HPPRCOM take about 0.6 seconds.
	// Note that this is not much of a concern now, because the CODEC system automatically configures itself,
	// and powers itself up at reset.
	// The only way it could be powered down would be if user did a direct I2C CODEC register write.
	ReturnIfError(DPxUpdateRegCache());
	timer = DPxGetTime();
	for ( ; ; ) {

		// Update timestamp _before_ reading registers,
		// so we know that the CODEC registers were definitely read _after_ the measured delay.
		ReturnIfError(DPxUpdateRegCache());
	
		// Check powerup bits for internal systems.  If all set, we're done.
		if ((DPxGetCodecReg(94) & 0xC6) == 0xC6 && (DPxGetCodecReg(95) & 0x0C) == 0x0C)
			break;

		// If not powered up after 1 second, something's probably wrong.
		if (DPxGetTime() - timer > 1) {
			DPxDebugPrint0("ERROR: DPxInitAudCodec() timeout waiting for CODEC to powerup\n");
			DPxSetError(DPX_ERR_AUD_CODEC_POWERUP);
			break;
		}
	}
}


// Set the 16-bit 2's complement signed value for left audio output channel
void DPxSetAudLeftValue(int value)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (value < -32768 || value > 32767) {	// Restrict to SInt16 range
		DPxDebugPrint1("ERROR: DPxSetAudLeftValue() argument value %d is out of signed 16-bit range\n", value);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VALUE);
		return;
	}
	DPxSetReg16(DPXREG_AUD_DATA_LEFT , value);
}


// Set the 16-bit 2's complement signed value for right audio output channel
void DPxSetAudRightValue(int value)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (value < -32768 || value > 32767) {	// Restrict to SInt16 range
		DPxDebugPrint1("ERROR: DPxSetAudRightValue() argument value %d is out of signed 16-bit range\n", value);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VALUE);
		return;
	}
	DPxSetReg16(DPXREG_AUD_DATA_RIGHT , value);
}


// Get the 16-bit 2's complement signed value for left audio output channel
int DPxGetAudLeftValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return (SInt16)DPxGetReg16(DPXREG_AUD_DATA_LEFT);
}


// Get the 16-bit 2's complement signed value for right audio output channel
int DPxGetAudRightValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return (SInt16)DPxGetReg16(DPXREG_AUD_DATA_RIGHT);
}


// Set volume for Left audio channel
void DPxSetAudLeftVolume(double volume)
{
	int iVolume;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	// Check for out-of-range values, and set resulting error state;
	// but still continue to set the clipped volume.
	if (volume < 0) {
		DPxDebugPrint1("ERROR: DPxSetAudLeftVolume() argument volume %g is under range 0 to 1\n", volume);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VOLUME);
		volume = 0;
	}
	else if (volume > 1) {
		DPxDebugPrint1("ERROR: DPxSetAudLeftVolume() argument volume %g is over range 0 to 1\n", volume);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VOLUME);
		volume = 1;
	}

	// round to the closest possible integer volume
	iVolume = (int)(volume * 65536 + 0.5);
	if (iVolume < 65536) {
		DPxSetReg16(DPXREG_AUD_CTRL, DPxGetReg16(DPXREG_AUD_CTRL) & ~DPXREG_AUD_CTRL_MAXVOL_LEFT);	// Enables volume control
		DPxSetReg16(DPXREG_AUD_VOLUME_LEFT , iVolume);
	}
	else {
		DPxSetReg16(DPXREG_AUD_CTRL, DPxGetReg16(DPXREG_AUD_CTRL) | DPXREG_AUD_CTRL_MAXVOL_LEFT);	// Forces maximum volume
		DPxSetReg16(DPXREG_AUD_VOLUME_LEFT , 65535);
	}
}


// Get volume for the Left audio output channel
double DPxGetAudLeftVolume()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_AUD_CTRL) & DPXREG_AUD_CTRL_MAXVOL_LEFT ? 1.0 : DPxGetReg16(DPXREG_AUD_VOLUME_LEFT) / 65536.0;
}


// Set volume for Right audio channel
void DPxSetAudRightVolume(double volume)
{
	int iVolume;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	// Check for out-of-range values, and set resulting error state;
	// but still continue to set the clipped volume.
	if (volume < 0) {
		DPxDebugPrint1("ERROR: DPxSetAudRightVolume() argument volume %g is under range 0 to 1\n", volume);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VOLUME);
		volume = 0;
	}
	else if (volume > 1) {
		DPxDebugPrint1("ERROR: DPxSetAudRightVolume() argument volume %g is over range 0 to 1\n", volume);
		DPxSetError(DPX_ERR_AUD_SET_BAD_VOLUME);
		volume = 1;
	}

	// round to the closest possible integer volume
	iVolume = (int)(volume * 65536 + 0.5);
	if (iVolume < 65536) {
		DPxSetReg16(DPXREG_AUD_CTRL, DPxGetReg16(DPXREG_AUD_CTRL) & ~DPXREG_AUD_CTRL_MAXVOL_RIGHT);	// Enables volume control
		DPxSetReg16(DPXREG_AUD_VOLUME_RIGHT , iVolume);
	}
	else {
		DPxSetReg16(DPXREG_AUD_CTRL, DPxGetReg16(DPXREG_AUD_CTRL) | DPXREG_AUD_CTRL_MAXVOL_RIGHT);	// Forces maximum volume
		DPxSetReg16(DPXREG_AUD_VOLUME_RIGHT , 65535);
	}
}


// Get volume for the Right audio output channel
double DPxGetAudRightVolume()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_AUD_CTRL) & DPXREG_AUD_CTRL_MAXVOL_RIGHT ? 1.0 : DPxGetReg16(DPXREG_AUD_VOLUME_RIGHT) / 65536.0;
}


// Set volume for both Left/Right audio channels
void DPxSetAudVolume(double volume)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudLeftVolume(volume);
	DPxSetAudRightVolume(volume);
}


// Get volume for both Left/Right audio channels (or Left channel, if Left/Right are different)
double DPxGetAudVolume()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_AUD_CTRL) & DPXREG_AUD_CTRL_MAXVOL_LEFT ? 1.0 : DPxGetReg16(DPXREG_AUD_VOLUME_LEFT) / 65536.0;
}


int DPxAudCodecVolumeToReg(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	// Convert volume from ratio to dB
	if (!dBUnits)
		volume = 20 * log10(volume);

	// Return closest corresponding register value
	if (volume >= 0) return 0x80;
	if (volume <= -63.5) return 0xFF;
	return 0x80 + (int)floor(-2 * volume + 0.5);
}


double DPxAudCodecRegToVolume(int reg, int dBUnits)
{
	double volumedB = (reg & 0x7F) / -2.0;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	// Get the volume in dB
	
	return dBUnits ? volumedB : pow(10, volumedB / 20);
}


// Set volume for the DATAPixx Audio OUT Left channel, range 0-1
void DPxSetAudCodecOutLeftVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetCodecReg(47, DPxAudCodecVolumeToReg(volume, dBUnits));
}


// Get volume for the DATAPixx Audio OUT Left channel, range 0-1
double DPxGetAudCodecOutLeftVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxAudCodecRegToVolume(DPxGetCodecReg(47), dBUnits);
}


// Set volume for the DATAPixx Audio OUT Right channel, range 0-1
void DPxSetAudCodecOutRightVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetCodecReg(64, DPxAudCodecVolumeToReg(volume, dBUnits));
}


// Get volume for the DATAPixx Audio OUT Right channel, range 0-1
double DPxGetAudCodecOutRightVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxAudCodecRegToVolume(DPxGetCodecReg(64), dBUnits);
}


// Set volume for the DATAPixx Audio OUT Left and Right channels, range 0-1
void DPxSetAudCodecOutVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudCodecOutLeftVolume(volume, dBUnits);
	DPxSetAudCodecOutRightVolume(volume, dBUnits);
}


// Get volume for the DATAPixx Audio OUT Left and Right channels (or Left channel, if Left/Right are different)
double DPxGetAudCodecOutVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetAudCodecOutLeftVolume(dBUnits);
}


// Set volume for the DATAPixx Speaker Left channel, range 0-1
void DPxSetAudCodecSpeakerLeftVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetCodecReg(54, DPxAudCodecVolumeToReg(volume, dBUnits));
}


// Get volume for the DATAPixx Speaker Left channel, range 0-1
double DPxGetAudCodecSpeakerLeftVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxAudCodecRegToVolume(DPxGetCodecReg(54), dBUnits);
}


// Set volume for the DATAPixx Speaker Right channel, range 0-1
void DPxSetAudCodecSpeakerRightVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetCodecReg(57, DPxAudCodecVolumeToReg(volume, dBUnits));
}


// Get volume for the DATAPixx Speaker Right channel, range 0-1
double DPxGetAudCodecSpeakerRightVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxAudCodecRegToVolume(DPxGetCodecReg(57), dBUnits);
}


// Set volume for the DATAPixx Speaker Left and Right channels, range 0-1
void DPxSetAudCodecSpeakerVolume(double volume, int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudCodecSpeakerLeftVolume(volume, dBUnits);
	DPxSetAudCodecSpeakerRightVolume(volume, dBUnits);
}


// Get volume for the DATAPixx Speaker Left and Right channels (or Left channel, if Left/Right are different)
double DPxGetAudCodecSpeakerVolume(int dBUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetAudCodecSpeakerLeftVolume(dBUnits);
}


// Configure how audio Left/Right channels are updated by schedule data
// lrMode is one of the following predefined constants:
//		DPXREG_AUD_CTRL_LRMODE_MONO		: Each AUD schedule datum goes to left and right channels
//		DPXREG_AUD_CTRL_LRMODE_LEFT		: Each AUD schedule datum goes to left channel only
//		DPXREG_AUD_CTRL_LRMODE_RIGHT	: Each AUD schedule datum goes to right channel only
//		DPXREG_AUD_CTRL_LRMODE_STEREO_1	: Pairs of AUD data are copied to left/right channels
//		DPXREG_AUD_CTRL_LRMODE_STEREO_2	: AUD data goes to left channel, AUX data goes to right
void DPxSetAudLRMode(int lrMode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (lrMode) {
		case DPXREG_AUD_CTRL_LRMODE_MONO:
		case DPXREG_AUD_CTRL_LRMODE_LEFT:
		case DPXREG_AUD_CTRL_LRMODE_RIGHT:
		case DPXREG_AUD_CTRL_LRMODE_STEREO_1:
		case DPXREG_AUD_CTRL_LRMODE_STEREO_2:
			DPxSetReg16(DPXREG_AUD_CTRL, (DPxGetReg16(DPXREG_AUD_CTRL) & ~DPXREG_AUD_CTRL_LRMODE_MASK) | lrMode);
			break;

		default:
			DPxDebugPrint1("ERROR: DPxSetAudLRMode() unrecognized lrMode %d\n", lrMode);
			DPxSetError(DPX_ERR_AUD_SET_BAD_LRMODE);
	}
}


int DPxGetAudLRMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_AUD_CTRL) & DPXREG_AUD_CTRL_LRMODE_MASK;
}


// Set AUD RAM buffer base address.  Must be an even value.
void DPxSetAudBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_AUD_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_AUD_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_AUD_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get AUD RAM buffer base address
unsigned DPxGetAudBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_BUFF_BASEADDR_L);
}


// Set RAM address from which next AUD datum will be read.  Must be an even value.
void DPxSetAudBuffReadAddr(unsigned buffReadAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffReadAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffReadAddr(0x%x) illegal odd address\n", buffReadAddr);
		DPxSetError(DPX_ERR_AUD_BUFF_ODD_READADDR);
		return;
	}
	if (buffReadAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffReadAddr(0x%x) exceeds DATAPixx RAM\n", buffReadAddr);
		DPxSetError(DPX_ERR_AUD_BUFF_READADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_AUD_BUFF_READADDR_L, buffReadAddr);
}


// Get RAM address from which next AUD datum will be read
unsigned DPxGetAudBuffReadAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_BUFF_READADDR_L);
}


// Set AUD RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffReadAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies spooled playback, or the continuous playback of periodic waveforms.
void DPxSetAudBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_AUD_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAudBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_AUD_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_AUD_BUFF_SIZE_L, buffSize);
}


// Get AUD RAM buffer size in bytes
unsigned DPxGetAudBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetAudBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudBuffBaseAddr(buffAddr);
	DPxSetAudBuffReadAddr(buffAddr);
	DPxSetAudBuffSize(buffSize);
}


// Set AUX RAM buffer base address.  Must be an even value.
void DPxSetAuxBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_AUX_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_AUX_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_AUX_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get AUX RAM buffer base address
unsigned DPxGetAuxBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_BUFF_BASEADDR_L);
}


// Set RAM address from which next AUX datum will be read.  Must be an even value.
void DPxSetAuxBuffReadAddr(unsigned buffReadAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffReadAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffReadAddr(0x%x) illegal odd address\n", buffReadAddr);
		DPxSetError(DPX_ERR_AUX_BUFF_ODD_READADDR);
		return;
	}
	if (buffReadAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffReadAddr(0x%x) exceeds DATAPixx RAM\n", buffReadAddr);
		DPxSetError(DPX_ERR_AUX_BUFF_READADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_AUX_BUFF_READADDR_L, buffReadAddr);
}


// Get RAM address from which next AUX datum will be read
unsigned DPxGetAuxBuffReadAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_BUFF_READADDR_L);
}


// Set AUX RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffReadAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies spooled playback, or the continuous playback of periodic waveforms.
void DPxSetAuxBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_AUX_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetAuxBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_AUX_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_AUX_BUFF_SIZE_L, buffSize);
}


// Get AUX RAM buffer size in bytes
unsigned DPxGetAuxBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetAuxBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAuxBuffBaseAddr(buffAddr);
	DPxSetAuxBuffReadAddr(buffAddr);
	DPxSetAuxBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first AUD update
void DPxSetAudSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUD_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first AUD update
unsigned DPxGetAudSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_SCHED_ONSET_L);
}


// Set AUD schedule update rate.  Range is 8-96 kHz.
// We'll limit maximum frequency to 96 kHz for 3 reasons:
// 1) This is the maximum spec'd frequency of the CODEC.
// 2) CODEC BCLK min high/low time specs are 35 ns, so I need 8 CLK100 = 12.5 MHz BCLK / 128 = 97.7 kHz max.
// 3) CODEC MCLK is spec'd for 50 MHz, but I get CODEC noise if I run in n=1.5 with MCLK > 25 MHz.
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 96 kHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 96 kHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 10417 ns
void DPxSetAudSchedRate(unsigned rateValue, int rateUnits)
{
	int multMClk, pllDual;
	int regDivisor;
	//regPllDual;
	int modifyingDivisor;
	int savedReg19, savedReg22, savedReg37;
	double freq, divisor;
    int iRead;
    int poweredDown;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:		freq = rateValue;						break;
		case DPXREG_SCHED_CTRL_RATE_XVID:	freq = rateValue * DPxGetVidVFreq();	break;
		case DPXREG_SCHED_CTRL_RATE_NANO:	freq = 1.0e9 / rateValue;				break;
		default:
			DPxDebugPrint1("ERROR: DPxSetAudSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_AUD_SCHED_BAD_RATE_UNITS);
			return;
	}

	if (freq < 8000) {		// Min frequency supported by CODEC
		DPxDebugPrint1("ERROR: DPxSetAudSchedRate() frequency too low %g\n", freq);
		DPxSetError(DPX_ERR_AUD_SCHED_TOO_SLOW);
		return;
	}
	if (freq > 96000) {	// Max frequency supported by CODEC.  Also, 25 MHz MCLK / 256 = 97.7 kHz.
		DPxDebugPrint1("ERROR: DPxSetAudSchedRate() frequency too high %g\n", freq);
		DPxSetError(DPX_ERR_AUD_SCHED_TOO_FAST);
		return;
	}

	// If given the choice, we would like to generate all frequencies in double-rate mode.
	// This gives better interpolation of waveforms, pushing noise into higher frequencies.
	// CODEC datasheet says that Fsref should be within the range 39-53 kHz,
	// so the lowest frequency we can make in double-rate is 39 kHz * 2(for double-rate) / 6.
	// Actually, we'll just always force double-rate.
	// Otherwise CODEC can hang up when switching from non-double-rate to double-rate.
	// Empirical tests show that we can run CODEC with 8 kHz audio samples = 24 kHz Fsref w/o distortion.
//	if (freq >= 39000.0 / 3)	// 13000 Hz
		pllDual = 1;
//	else
//		pllDual = 0;

	// Scan through the 11 divisors from largest to smallest, looking for the first range which can generate the desired freq.
	// We want the largest possible divisor, because this will result in the smoothest interpolation between sample values.
	// The frequency ranges attainable for the different /n almost all overlap, except for 2 small holes at the top.
	// We fill these holes by allowing the Fsref min to go below 39 kHz.
	// The worst case is for frequencies just above 65104 Hz, which need an Fsref of 32552 Hz.
	// I think it's best to _reduce_ Fsref below its spec'd frequency, rather than increasing it.
	// I was getting some bad noise out of CODEC for MCLK's which exceeded 25 MHz.
	for (divisor = 6; divisor > 1; divisor -= 0.5)
		if (freq <= 25.0e6 / (pllDual ? 256 : 512) / divisor)
			break;

	// Ratio between CODEC MCLK frequency, and WCLK frequency.  Remember minimum Q is 4 in dual, so we always use Q = 4.
	multMClk = (int)((pllDual ? 256 : 512) * divisor);

	// TLV320AIC32 register values
//	regPllDual = (pllDual << 6) | (pllDual << 5) | 0x0A;					// Sets dual bits, and enables left/right DAC datapaths
	regDivisor = (int)(divisor * 2 - 2) | ((int)(divisor * 2 - 2) << 4);	// DAC and ADC get same divisor

	// Here's the thing.  It appears that the CODEC output can hang if we update timing registers while DACs are powered up.
	// I especially see this when going from non-double-rate to double-rate mode, but I also see this when only programming divisor.
	// When the CODEC hangs, it seems to never generate audio out until it has been reprogrammed back to non-double-rate mode.
	// It seems that I can prevent this from happening by powering down the DACs while reprogramming the rate or divisor.
	// ADC's also need to be powered down, or audio input and output can both get screwed up.
	savedReg19 = cachedCodecRegs[19];
	savedReg22 = cachedCodecRegs[22];
	savedReg37 = cachedCodecRegs[37];
	modifyingDivisor = (regDivisor != cachedCodecRegs[2]);
	if (modifyingDivisor) {
		DPxSetCodecReg(19, 0x78);	// Powerdown Left ADC
		DPxSetCodecReg(22, 0x78);	// Powerdown Right ADC
		DPxSetCodecReg(37, 0x20);	// Powerdown L/R DACs, configure HPLCOM as an independant output (goes to DP speaker)

		// Hmmm.  CODEC doesn't always power down when we tell it to.  Maybe when we've stopped MCLK?
		// The final solution is just to stay in double-rate mode.  It can present 8 kHz audio samples w/o distortion.
		// Also, user might program this rate during an animation.  We don't want to be hanging here.
		// ...Unfortunately, it started hanging again with VHDL rev 7.
		// Now I'm putting back this wait, and it doesn't seem to hang anymore.
		// I've counted that it takes anywhere from 3 to 21 reads of register 94 before it says IC has powered down.
		// Note that we only stay here when the user is actually _changing_ the frequency,
		// so the user can still call this function a million times during an animation,
		// and we'll only pause here if we're really changing the rate.
		// After long-run tests, the CODEC seems rock solid now that I've enabled this wait.
		// Never seems to stop producing sound.
        // NOPE.  If VPixx program calls this at the beginning of each trial, then sometimes I see the program hanging here.
        // Not quite sure what the best strategy would be now.
        // We'll try just waiting here for a maximum of 25 times, since emperical tests show that when it succeeds it needs a maximum of 21 reads.
//		while ((DPxGetCodecReg(94) & 0xC0) != 0x00)
//			(void)0;
        poweredDown = 0;
        for (iRead = 0; iRead < 25; iRead++) {
            if ((DPxGetCodecReg(94) & 0xC0) == 0x00) {
                poweredDown = 1;
                break;
            }
        }

#if 0
        // OK, powerdown didn't work.  Now what?
        if (!poweredDown) {
            printf("Powerdown failed, CR 37 = %02X, CR 94 = %02X\n", DPxGetCodecReg(37), DPxGetCodecReg(94));
        }
#endif
	}

	// AUD channel scheduler holds frequency, and multiplier to get us up to MCLK frequency.
	DPxSetReg32(DPXREG_AUD_SCHED_CTRL_L, (DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_AUD_SCHED_RATE_L,  rateValue);
	DPxSetReg16(DPXREG_AUD_CTRL, (DPxGetReg16(DPXREG_AUD_CTRL) & ~DPXREG_AUD_CTRL_BCLK_RATIO) | (multMClk >> 7));

	// AUX channel rate is tied to AUD rate.  They are supplying simultaneous L/R data to a single audio CODEC, so their rates can't be different.
	DPxSetReg32(DPXREG_AUX_SCHED_CTRL_L, (DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_AUX_SCHED_RATE_L,  rateValue);

//	DPxSetCodecReg( 4, regPllJ);		// We're no longer using CODEC PLL.
//	DPxSetCodecReg( 5, regPllDH);
//	DPxSetCodecReg( 6, regPllDL);
//	DPxSetCodecReg( 7, regPllDual);		// Always keeps the 0x6A from init

	if (modifyingDivisor) {
		DPxSetCodecReg( 2, regDivisor);
		DPxSetCodecReg(19, savedReg19);	// Probably powering up Left ADC
		DPxSetCodecReg(22, savedReg22);	// Probably powering up Right ADC
		DPxSetCodecReg(37, savedReg37);	// Probably powering up L/R DACs

		// Wait until the 2 DACs are fully powered up before allowing user to start audio playback.
		// We don't have a timeout here, because we don't want to call DPxUpdateRegCache() for the user.
		// There doesn't seem to be an equivalent register to confirm ADC powerup.
		while ((DPxGetCodecReg(94) & 0xC0) != 0xC0)
			(void)0;
	}
}


// Get AUD schedule update rate (and optionally get rate units)
unsigned DPxGetAudSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_AUD_SCHED_RATE_L);
}


// Set AUD schedule update count
void DPxSetAudSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUD_SCHED_COUNT_L,  count);
}


// Get AUD schedule update count
unsigned DPxGetAudSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableAudSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUD_SCHED_CTRL_L, DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableAudSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUD_SCHED_CTRL_L, DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsAudSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetAudSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudSchedOnset(onset);
	DPxSetAudSchedRate(rateValue, rateUnits);
	DPxSetAudSchedCount(count);
	if (count)
		DPxEnableAudSchedCountdown();
	else
		DPxDisableAudSchedCountdown();
}


// Start running an AUD schedule
void DPxStartAudSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_AUD)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_AUD));
}


// Stop running an AUD schedule
void DPxStopAudSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_AUD)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_AUD));
}


// Returns non-0 if AUD schedule is currently running
int DPxIsAudSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUD_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Set nanosecond delay between schedule start and first AUX update
void DPxSetAuxSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUX_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first AUX update
unsigned DPxGetAuxSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_SCHED_ONSET_L);
}


// Set AUX (and AUD) schedule update rate.  Range is 8-96 kHz.
void DPxSetAuxSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAudSchedRate(rateValue, rateUnits);	// Will assign same rate for both AUD and AUX
}


// Get AUX schedule update rate (and optionally get rate units)
unsigned DPxGetAuxSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_AUX_SCHED_RATE_L);
}


// Set AUX schedule update count
void DPxSetAuxSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUX_SCHED_COUNT_L,  count);
}


// Get AUX schedule update count
unsigned DPxGetAuxSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableAuxSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUX_SCHED_CTRL_L, DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableAuxSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_AUX_SCHED_CTRL_L, DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsAuxSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Count.
// If Count > 0, enables Countdown mode.
void DPxSetAuxSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetAuxSchedOnset(onset);
	DPxSetAuxSchedRate(rateValue, rateUnits);
	DPxSetAuxSchedCount(count);
	if (count)
		DPxEnableAuxSchedCountdown();
	else
		DPxDisableAuxSchedCountdown();
}


// Start running a AUX schedule
void DPxStartAuxSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_AUX)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_AUX));
	DPxSetCodecReg(58, 0x09);	// To prevent hiss, HPLCOM is powered down after Init().  Make sure it's on now.
}


// Stop running a AUX schedule
void DPxStopAuxSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_AUX)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_AUX));
}


// Returns non-0 if AUX schedule is currently running
int DPxIsAuxSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_AUX_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Returns CODEC Audio OUT group delay in seconds.
// This is the time between when a schedule sends a data sample to the CODEC,
// and when that sample has greatest output at the "Audio OUT" jack of the Datapixx.
// Due to the way in which CODECs operate, this delay is a function of the sample rate (eg: 48000).
// Note that my empirical data shows a slope of 23.665, not 21.665.
// That's because my test waveform started with 2 samples of "0" data.
double DPxGetAudGroupDelay(double sampleRate)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return 21.665 / sampleRate + 7.86e-6;
}


/********************************************************************************/
/*																				*/
/*	MIC Subsystem																*/
/*																				*/
/********************************************************************************/


// Select the source of the microphone input.
// source is one of the following predefined constants:
//		DPX_MIC_SRC_MIC_IN	: Microphone level input
//		DPX_MIC_SRC_LINE_IN	: Line level audio input.
// Valid gain values are 1-1000, or 0-60 dB.
// Typical gain values would be around 100 for a microphone input,
// and probably 1 for line-level input.
void DPxSetMicSource(int source, double gain, int dBUnits)
{
	double gainDb, gainReg;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	// Convert gain to dB
	gainDb = dBUnits ? gain : 20 * log10(gain);

	// Convert gain to a valid register value
	gainReg = floor(gainDb * 2 + 0.5);
	if (gainReg < 0) {
		DPxDebugPrint1("ERROR: DPxSetMicSource() gain of %g is too low\n", gain);
		DPxSetError(DPX_ERR_MIC_SET_GAIN_TOO_LOW);
		return;
	}
	if (gainReg > 120) {	// 60 dB
		DPxDebugPrint1("ERROR: DPxSetMicSource() gain of %g is too high\n", gain);
		DPxSetError(DPX_ERR_MIC_SET_GAIN_TOO_HIGH);
		return;
	}

	// Select the requested source
	if (source == DPX_MIC_SRC_MIC_IN) {
		DPxSetCodecReg(19, 0x04);	// MIC1L connected to left ADC with no attenuation, and powerup ADC.
		DPxSetCodecReg(20, 0x78);	// LINE2L not connected to Left ADC
		DPxSetCodecReg(22, 0x04);	// MIC1R connected to right ADC with no attenuation, and powerup ADC.
		DPxSetCodecReg(23, 0x78);	// LINE2R not connected to Right ADC
	}
	else if (source == DPX_MIC_SRC_LINE_IN) {
		DPxSetCodecReg(19, 0x7C);	// MIC1L not connected to left ADC, and powerup ADC.
		DPxSetCodecReg(20, 0x00);	// LINE2L connected to Left ADC with no attenuation
		DPxSetCodecReg(22, 0x7C);	// MIC1R not connected to right ADC, and powerup ADC.
		DPxSetCodecReg(23, 0x00);	// LINE2R connected to Right ADC with no attenuation
	}
	else {
		DPxDebugPrint1("ERROR: DPxSetMicSource() %d is not a valid audio input source\n", source);
		DPxSetError(DPX_ERR_MIC_SET_BAD_SOURCE);
		return;
	}

	// Implement gain (only if source was valid).
	// For now, we use the same gain for both left/right channels.
	DPxSetCodecReg(15, (int)gainReg);	// Left ADC PGA gain setting
	DPxSetCodecReg(16, (int)gainReg);	// Right ADC PGA gain setting
}


// Get the source of the microphone input, and optionally return the source's gain.
// Set dBUnits to non-zero to return the gain in dB.
int DPxGetMicSource(double *gain, int dBUnits)
{
	double gainDb;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	// Get the gain.  Only need to read the left channel.
	if (gain) {
		gainDb = (DPxGetCodecReg(15) & 0x7F) / 2.0;
		*gain = dBUnits ? gainDb : pow(10, gainDb / 20);
	}

	// Return the input source
	if ((DPxGetCodecReg(19) & 0x78) != 0x78)
		return DPX_MIC_SRC_MIC_IN;
	if ((DPxGetCodecReg(20) & 0x78) != 0x78)
		return DPX_MIC_SRC_LINE_IN;
	return DPX_MIC_SRC_UNKNOWN;
}


// Get the 16-bit 2's complement signed value for left MIC channel
int DPxGetMicLeftValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return (SInt16)DPxGetReg16(DPXREG_MIC_DATA_LEFT);
}


// Get the 16-bit 2's complement signed value for right MIC channel
int DPxGetMicRightValue()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return (SInt16)DPxGetReg16(DPXREG_MIC_DATA_RIGHT);
}


// Configure how microphone Left/Right channels are stored to schedule buffer.
// lrMode is one of the following predefined constants:
//		DPXREG_MIC_CTRL_LRMODE_MONO		: Mono data is written to schedule buffer (average of Left/Right CODEC data)
//		DPXREG_MIC_CTRL_LRMODE_LEFT		: Left data is written to schedule buffer
//		DPXREG_MIC_CTRL_LRMODE_RIGHT	: Right data is written to schedule buffer
//		DPXREG_MIC_CTRL_LRMODE_STEREO	: Left and Right data are both written to schedule buffer
void DPxSetMicLRMode(int lrMode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (lrMode) {
		case DPXREG_MIC_CTRL_LRMODE_MONO:
		case DPXREG_MIC_CTRL_LRMODE_LEFT:
		case DPXREG_MIC_CTRL_LRMODE_RIGHT:
		case DPXREG_MIC_CTRL_LRMODE_STEREO:
			DPxSetReg16(DPXREG_MIC_CTRL, (DPxGetReg16(DPXREG_MIC_CTRL) & ~DPXREG_MIC_CTRL_LRMODE_MASK) | lrMode);
			break;

		default:
			DPxDebugPrint1("ERROR: DPxSetMicLRMode() unrecognized lrMode %d\n", lrMode);
			DPxSetError(DPX_ERR_MIC_SET_BAD_LRMODE);
	}
}


int DPxGetMicLRMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_MIC_CTRL) & DPXREG_MIC_CTRL_LRMODE_MASK;
}


// Enable loopback between audio outputs and microphone inputs
void DPxEnableAudMicLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_MIC_CTRL, DPxGetReg16(DPXREG_MIC_CTRL) | DPXREG_MIC_CTRL_AUD_LOOPBACK);
}


// Disable loopback between audio outputs and microphone inputs
void DPxDisableAudMicLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_MIC_CTRL, DPxGetReg16(DPXREG_MIC_CTRL) & ~DPXREG_MIC_CTRL_AUD_LOOPBACK);
}


// Returns non-0 if microphone inputs are driven by audio outputs
int DPxIsAudMicLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg16(DPXREG_MIC_CTRL) & DPXREG_MIC_CTRL_AUD_LOOPBACK;
}


// Set MIC RAM buffer start address.  Must be an even value.
void DPxSetMicBuffBaseAddr(unsigned buffBaseAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffBaseAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffBaseAddr(0x%x) illegal odd address\n", buffBaseAddr);
		DPxSetError(DPX_ERR_MIC_BUFF_ODD_BASEADDR);
		return;
	}
	if (buffBaseAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffBaseAddr(0x%x) exceeds DATAPixx RAM\n", buffBaseAddr);
		DPxSetError(DPX_ERR_MIC_BUFF_BASEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_MIC_BUFF_BASEADDR_L, buffBaseAddr);
}


// Get MIC RAM buffer start address
unsigned DPxGetMicBuffBaseAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_BUFF_BASEADDR_L);
}


// Set RAM address to which next MIC datum will be written.  Must be an even value.
void DPxSetMicBuffWriteAddr(unsigned buffWriteAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffWriteAddr & 1) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffWriteAddr(0x%x) illegal odd address\n", buffWriteAddr);
		DPxSetError(DPX_ERR_MIC_BUFF_ODD_WRITEADDR);
		return;
	}
	if (buffWriteAddr >= DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffWriteAddr(0x%x) exceeds DATAPixx RAM\n", buffWriteAddr);
		DPxSetError(DPX_ERR_MIC_BUFF_WRITEADDR_TOO_HIGH);
		return;
	}
	DPxSetReg32(DPXREG_MIC_BUFF_WRITEADDR_L, buffWriteAddr);
}


// Get RAM address to which next MIC datum will be written
unsigned DPxGetMicBuffWriteAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_BUFF_WRITEADDR_L);
}


// Set MIC RAM buffer size in bytes.  Must be an even value.
// The hardware will automatically wrap the BuffWriteAddr, when it gets to BuffBaseAddr+BuffSize, back to BuffBaseAddr.
// This simplifies continuous spooled acquisition.
void DPxSetMicBuffSize(unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	if (buffSize & 1) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffSize(0x%x) illegal odd size\n", buffSize);
		DPxSetError(DPX_ERR_MIC_BUFF_ODD_SIZE);
		return;
	}
	if (buffSize > DPxGetRamSize()) {
		DPxDebugPrint1("ERROR: DPxSetMicBuffSize(0x%x) exceeds DATAPixx RAM\n", buffSize);
		DPxSetError(DPX_ERR_MIC_BUFF_TOO_BIG);
		return;
	}
	DPxSetReg32(DPXREG_MIC_BUFF_SIZE_L, buffSize);
}


// Get MIC RAM buffer size in bytes
unsigned DPxGetMicBuffSize()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_BUFF_SIZE_L);
}


// Shortcut which assigns Size/BaseAddr/ReadAddr
void DPxSetMicBuff(unsigned buffAddr, unsigned buffSize)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetMicBuffBaseAddr(buffAddr);
	DPxSetMicBuffWriteAddr(buffAddr);
	DPxSetMicBuffSize(buffSize);
}


// Set nanosecond delay between schedule start and first MIC sample
void DPxSetMicSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_MIC_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first MIC sample
unsigned DPxGetMicSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_SCHED_ONSET_L);
}


// Set MIC schedule sample rate and units
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 96 kHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 96 kHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 10417 ns
// Note that the MIC system shares CODEC timing resources with the AUD system,
// and calling DPxSetMicSchedRate() must indirectly call DPxSetAudSchedRate() with the same frequency.
// I don't much like this relationship, and it may disappear in future hardware revs.
void DPxSetMicSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 96000) {
				DPxDebugPrint1("ERROR: DPxSetMicSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_MIC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 96000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetMicSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_MIC_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 10417) {
				DPxDebugPrint1("ERROR: DPxSetMicSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_MIC_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetMicSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_MIC_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(DPXREG_MIC_SCHED_CTRL_L, (DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(DPXREG_MIC_SCHED_RATE_L,  rateValue);

	// Audio subsystem must absolutely run at the same rate as the MIC system,
	// since it's the AUD system which paces the CODEC I2S bus,
	// and configures CODEC registers.
	DPxSetAudSchedRate(rateValue, rateUnits);
}


// Get MIC schedule update rate (and optionally get rate units)
unsigned DPxGetMicSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(DPXREG_MIC_SCHED_RATE_L);
}


// Set MIC schedule update count
void DPxSetMicSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_MIC_SCHED_COUNT_L,  count);
}


// Get MIC schedule update count
unsigned DPxGetMicSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnableMicSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_MIC_SCHED_CTRL_L, DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisableMicSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg32(DPXREG_MIC_SCHED_CTRL_L, DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsMicSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetMicSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetMicSchedOnset(onset);
	DPxSetMicSchedRate(rateValue, rateUnits);
	DPxSetMicSchedCount(count);
	if (count)
		DPxEnableMicSchedCountdown();
	else
		DPxDisableMicSchedCountdown();
}


// Start running an MIC schedule
void DPxStartMicSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;

	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_MIC)) |
										(DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_MIC));
}


// Stop running an MIC schedule
void DPxStopMicSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_MIC)) |
										(DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_MIC));
}


// Returns non-0 if MIC schedule is currently running
int DPxIsMicSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return DPxGetReg32(DPXREG_MIC_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Returns CODEC Microphone IN group delay in seconds.
// This is the time between when a voltage appears at the MIC IN jack of the DATAPixx,
// and when an audio input schedule will acquire the voltage.
// Due to the way in which CODECs operate, this delay is a function of the sample rate (eg: 48000).
// Note that I measure the same group delay for both the "MIC IN" and the "Audio IN" jacks.
double DPxGetMicGroupDelay(double sampleRate)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	return 19.335 / sampleRate - 7.86e-6;
}


/********************************************************************************/
/*																				*/
/*	Video Subsystem																*/
/*																				*/
/********************************************************************************/


// Set the video processing mode.
// vidMode is one of the following predefined constants:
//		DPXREG_VID_CTRL_MODE_C24	Straight passthrough from DVI 8-bit (or HDMI "deep" 10/12-bit) RGB to VGA 8/10/12-bit RGB
//		DPXREG_VID_CTRL_MODE_L48	DVI RED[7:0] is used as an index into a 256-entry 16-bit RGB colour lookup table
//		DPXREG_VID_CTRL_MODE_M16	DVI RED[7:0] & GREEN[7:0] concatenate into a VGA 16-bit value sent to all three RGB components
//		DPXREG_VID_CTRL_MODE_C48	Even/Odd pixel RED/GREEN/BLUE[7:0] concatenate to generate 16-bit RGB components at half the horizontal resolution
//		DPXREG_VID_CTRL_MODE_L48D	DVI RED[7:4] & GREEN[7:4] concatenate to form an 8-bit index into a 256-entry 16-bit RGB colour lookup table
//		DPXREG_VID_CTRL_MODE_M16D	DVI RED[7:3] & GREEN[7:3] & BLUE[7:2] concatenate into a VGA 16-bit value sent to all three RGB components
//		DPXREG_VID_CTRL_MODE_C36D	Even/Odd pixel RED/GREEN/BLUE[7:2] concatenate to generate 12-bit RGB components at half the horizontal resolution
//		DPXREG_VID_CTRL_MODE_RB24	DVI RED[7:0] & GREEN[7:4] concatenate to form 12-bit RED value, DVI BLUE[7:0] & GREEN[3:0] concatenate to form 12-bit BLUE value, GREEN is forced to 0 (VIEWPixx Rev >= 21 only)
//      DPXREG_VID_CTRL_MODE_RB3D   Straight passthrough from DVI 8-bit but sdds an overlay for the console display in RB3D mode. Green is the CLUT index (PPC Only)
void DPxSetVidMode(int vidMode)
{
	if (vidMode & ~DPXREG_VID_CTRL_MODE_MASK) {
        DPxDebugPrint1("ERROR: DPxSetVidMode() unrecognized vidMode %d\n", vidMode);
        DPxSetError(DPX_ERR_VID_SET_BAD_MODE);
        return;
	}

    // If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then set the video mode for both.
    // We want the PPC console to have the same video mode as the PPX.
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_MODE_MASK) | vidMode);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_MODE_MASK) | vidMode);
        return;
    }

    // Normal processing for all other cases
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_MODE_MASK) | vidMode);
}


// Get the video processing mode
int DPxGetVidMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;

	return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_MODE_MASK;
}


// Pass 256*3 = 768 16-bit values, in order R0,G0,B0,R1,G1,B1...
// DPxSetVidClut() returns immediately, and CLUT is implemented at next vertical blanking interval.
void DPxSetVidClut(UInt16* clutData)
{
	int payloadLength = 256 * 3 * 2;

	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_WRITECLUT;
	ep2out_Tram[2] = LSB(payloadLength);
	ep2out_Tram[3] = MSB(payloadLength);
	memcpy(ep2out_Tram+4, clutData, payloadLength);
    
    // If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then send same CLUT to both.
    // We want the PPC console to have the same CLUT as the PPX.
    if ((dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) || 
		((dpxUsrDevsel == DPX_DEVSEL_PPX || dpxUsrDevsel == DPX_DEVSEL_PPC) && dpxDeviceTable[DPX_DEVSEL_PPX].dpxIsPropixx && dpxDeviceTable[DPX_DEVSEL_PPC].dpxIsPropixxCtrl)){
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        return;
    }
    
    // Normal processing for all other cases
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
	if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
		DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
	}
}


// Similar to DPxSetVidClut(), except pass 512*3 (=1536) 16-bit video DAC data to fill 2 channel CLUTs with independent data, in order R0,G0,B0,R1,G1,B1...
// Low CLUT goes to main display, and high CLUT goes to console.
void DPxSetVidCluts(UInt16* clutData)
{
	int payloadLength;

    // If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then send display CLUT to PPX, and console CLUT to PPC.
    if ((dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) || 
		((dpxUsrDevsel == DPX_DEVSEL_PPX || dpxUsrDevsel == DPX_DEVSEL_PPC) && dpxDeviceTable[DPX_DEVSEL_PPX].dpxIsPropixx && dpxDeviceTable[DPX_DEVSEL_PPC].dpxIsPropixxCtrl)){
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        payloadLength = 256 * 3 * 2;
        ep2out_Tram[0] = '^';
        ep2out_Tram[1] = EP2OUT_WRITECLUT;
        ep2out_Tram[2] = LSB(payloadLength);
        ep2out_Tram[3] = MSB(payloadLength);
        memcpy(ep2out_Tram+4, clutData, payloadLength);
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidCluts() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        memcpy(ep2out_Tram+4, clutData+768, payloadLength);
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidCluts() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        return;
    }

    // Normal case for all other processing
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	payloadLength = 512 * 3 * 2;
	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_WRITECLUT;
	ep2out_Tram[2] = LSB(payloadLength);
	ep2out_Tram[3] = MSB(payloadLength);
	memcpy(ep2out_Tram+4, clutData, payloadLength);
	if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
		DPxDebugPrint0("ERROR: DPxSetVidCluts() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
	}
}


void DPxPPxEnableCalibrationMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	DPxSetReg16(PPXREG_VID_CALIB, DPxGetReg16(PPXREG_VID_CALIB) | PPXREG_VID_CALIB_MODE_EN);
}


void DPxPPxDisableCalibrationMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CALIB, (DPxGetReg16(PPXREG_VID_CALIB) & ~PPXREG_VID_CALIB_MODE_EN) );
}


int DPxPPxIsCalibrationModeEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))    // Non-PROPixx systems will return 0
        return 0;
    
	return DPxGetReg16(PPXREG_VID_CALIB) & PPXREG_VID_CALIB_MODE_EN;
}


// Pass 4096*3 = 12288 16-bit values, in order 
// Writes both the SPI, and HS_LUT_RAM.  
// SPI is loaded automatically next read.
void DPxSetPPxHotSpotLut(int hotSpotCenterX, int hotSpotCenterY, UInt16* hotSpotLutData)
{
	int payloadLength = 4096 * 3 * 2;
	unsigned char hotSpotCenter[5];
	unsigned char hotSpotLutDataSPI[24576];

	// PROPixx only function
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	if (hotSpotCenterX < -960 || hotSpotCenterX > 960 || hotSpotCenterY < -540 || hotSpotCenterY > 540) {	// Sanity check
		fprintf(stderr,"ERROR: X or Y hot spot coordonate out of range\n");
		return;
	}

	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_WRITEHSLUT;
	ep2out_Tram[2] = LSB(payloadLength);
	ep2out_Tram[3] = MSB(payloadLength);
	memcpy(ep2out_Tram+4, hotSpotLutData, payloadLength);
    
	if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
		DPxDebugPrint0("ERROR: DPxSetVidHotSpotLut() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
	}

	memcpy(hotSpotLutDataSPI, hotSpotLutData, payloadLength);

    DPxSpiErase(SPI_ADDR_PPX_HS_LUT, payloadLength, NULL);
    DPxSpiWrite(SPI_ADDR_PPX_HS_LUT, payloadLength, (char*)hotSpotLutDataSPI , NULL);

	// Set the register
	DPxSetPPxHotSpotCenter(hotSpotCenterX, hotSpotCenterY);

	// realign to unsigned coordinate
	hotSpotCenterX = hotSpotCenterX + 960;
	hotSpotCenterY = hotSpotCenterY + 540;

	hotSpotCenter[0] = LSB(hotSpotCenterX);
	hotSpotCenter[1] = MSB(hotSpotCenterX);
	hotSpotCenter[2] = LSB(hotSpotCenterY);
	hotSpotCenter[3] = MSB(hotSpotCenterY);

	// Also, save the current projection mode
	hotSpotCenter[4] = DPxIsPPxRearProjection() + DPxIsPPxCeilingMount();

    DPxSpiWrite(SPI_ADDR_PPX_HS_POS, 5, (char*)hotSpotCenter, NULL); 
}


void DPxEnablePPxHotSpotCorrection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	DPxSetReg16(PPXREG_HS_CTRL, DPxGetReg16(PPXREG_HS_CTRL) | PPXREG_HS_CTRL_CORRECTION_EN);
}


void DPxDisablePPxHotSpotCorrection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_HS_CTRL, (DPxGetReg16(PPXREG_HS_CTRL) & ~PPXREG_HS_CTRL_CORRECTION_EN) );
}


int DPxIsPPxHotSpotCorrection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))    // Non-PROPixx systems will return 0
        return 0;
    
	return DPxGetReg16(PPXREG_HS_CTRL) & PPXREG_HS_CTRL_CORRECTION_EN;
}

//(where 0,0 is center of screen)
void DPxSetPPxHotSpotCenter(int x, int y)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	if (x < -960 || x > 960 || y < -540 || y > 540) {	// Sanity check
		fprintf(stderr,"ERROR: X or Y hot spot coordonate out of range\n");
		return;
	}
    
	DPxSetReg16(PPXREG_HS_CENTER_X_COORD, x + 960);
	DPxSetReg16(PPXREG_HS_CENTER_Y_COORD, y + 540);
}

void DPxGetPPxHotSpotCenter(int *x, int *y)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	*x = DPxGetReg16(PPXREG_HS_CENTER_X_COORD) - 960;
	*y = DPxGetReg16(PPXREG_HS_CENTER_Y_COORD) - 540;
}


void DPxSetVidClutTransparencyColor(UInt16 red, UInt16 green, UInt16 blue)
{
	// Not working on DATAPixx
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;

	ep2out_Tram[0]  = '^';
	ep2out_Tram[1]  = EP2OUT_WRITECLUTRANS;
	ep2out_Tram[2]  = 6; // always 3 16-bit word
	ep2out_Tram[3]  = 0;
	ep2out_Tram[4]  = LSB(red);
	ep2out_Tram[5]  = MSB(red);
	ep2out_Tram[6]  = LSB(green);
	ep2out_Tram[7]  = MSB(green);
	ep2out_Tram[8]  = LSB(blue);
	ep2out_Tram[9]  = MSB(blue);
    
    // If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then send same CLUT Transparency color to both.
    // We want the PPC console to have the same CLUT as the PPX.
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()){
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
            DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
            DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
        }
        return;
    }
    
    // Normal processing for all other cases
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
	if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
		DPxDebugPrint0("ERROR: DPxSetVidClut() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_VID_CLUT_WRITE_USB_ERROR);
	}
}


void DPxGetVidClutTransparencyColor(UInt16* red, UInt16* green, UInt16* blue)
{
	UInt16* tramPtr = (UInt16*)ep2out_Tram;
	int packetSize, iRetry;
	#ifndef USE_LIB01
		int packetSizeWrite;
	#endif


	// Not working on DATAPixx
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;

    // Caller is responsible for ensuring valid dpxSysDevsel
	*tramPtr++ = (EP2OUT_READCLUTRANS << 8) + '^';			// Tram header for a CLUT Color Transparency read
	*tramPtr++ = 0;		// 0-byte payload for an CLUT Transparent Color read command

	packetSize = (int)((char*)tramPtr - (char*)ep2out_Tram);   // Use explicit (int) cast because 64-bit builds probably have 64-bit addresses, but only 32-bit integers.
	for (iRetry = 0; ; iRetry++) {
#ifdef USE_LIB01
		if (usb_bulk_write(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, 1000) == packetSize)
#else
		if ((libusb_bulk_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, 2, (char*)ep2out_Tram, packetSize, &packetSizeWrite, 1000) == 0) &&
			(packetSize == packetSizeWrite))
#endif
			break;
		else if (iRetry < MAX_RETRIES) {
			DPxDebugPrint1("ERROR: DPxGetVidClutTransparencyColor() call to usb_bulk_write() retried: %s\n", usb_strerror());
			dpxEp2WrRetries++;
		}
		else {
			DPxDebugPrint1("ERROR: DPxGetVidClutTransparencyColor() call to usb_bulk_write() failed: %s\n", usb_strerror());
			dpxEp2WrFails++;
			DPxSetError(DPX_ERR_USB_REG_BULK_WRITE);
		}
	}
	if (EZReadEP6Tram(EP6IN_READCLUTRANS, 6) < 0) {
		DPxDebugPrint0("ERROR: DPxGetVidClutTransparencyColor() call to EZReadEP6Tram() failed\n");
		DPxSetError(DPX_ERR_USB_REG_BULK_READ);
	}
	
	*red	= ep6in_Tram[4] + (ep6in_Tram[5] << 8);
	*green	= ep6in_Tram[6] + (ep6in_Tram[7] << 8);
	*blue	= ep6in_Tram[8] + (ep6in_Tram[9] << 8);
}


// Enable video CLUT transparency color mode
void DPxEnableVidClutTransparencyColorMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
	
    // If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then enable the CLUT Transparency Color mode for both.
    // We want the PPC console to have the same CLUT Transparency Color mode as the PPX.
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN);
        return;
    }

    // Normal processing for all other cases
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN);
}


// Disable video CLUT transparency color mode
void DPxDisableVidClutTransparencyColorMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;    
	
	// If user is in DPX_DEVSEL_AUTO, and both PPC and PPX are present, then disable the CLUT Transparency Color mode for both.
    // We want the PPC console to have the same CLUT Transparency Color mode as the PPX.
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) {
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN));
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
        DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN));
        return;
    }

    // Normal processing for all other cases
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN));
}


// Returns non-0 if video CLUT Transparency Color mode is enabled
int DPxIsVidClutTransparencyColorMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_CLUT_TRANS_MODE_EN;
}


// VGA 1 shows left half of video image, VGA 2 shows right half of video image
void DPxEnableVidHorizSplit()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2_DPX))
        return;

	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_HSPLIT_MAN | DPXREG_VID_CTRL_HSPLIT);
}


// VGA 1 and VGA 2 both show entire video image (hardware video mirroring)
void DPxDisableVidHorizSplit()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_HSPLIT_MAN) & ~DPXREG_VID_CTRL_HSPLIT);
}


// DATAPixx will automatically split video across the two VGA outputs if the horizontal resolution is at least twice the vertical resolution (default mode)
void DPxAutoVidHorizSplit()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) & ~(DPXREG_VID_CTRL_HSPLIT_MAN | DPXREG_VID_CTRL_HSPLIT));
}


// Returns non-0 if video is being split across the two VGA outputs.
int DPxIsVidHorizSplit()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))    // Non-DATAPixx systems will return 0
        return 0;
    
	return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_HSPLIT;
}


// Top/bottom halves of input image are output in two sequential video frames.
// VESA L/R output is set to 1 when first frame (left eye) is displayed,
// and set to 0 when second frame (right eye) is displayed.
void DPxEnableVidVertStereo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_VSTEREO_MAN | DPXREG_VID_CTRL_VSTEREO);
}


// Normal display (no hardware vertical stereo)
void DPxDisableVidVertStereo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, (DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_VSTEREO_MAN) & ~DPXREG_VID_CTRL_VSTEREO);
}


// Vertical stereo is enabled automatically when vertical resolution > horizontal resolution (default mode)
void DPxAutoVidVertStereo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) & ~(DPXREG_VID_CTRL_VSTEREO_MAN | DPXREG_VID_CTRL_VSTEREO));
}


// Returns non-0 if DATAPixx is separating input into sequencial left/right stereo images.
int DPxIsVidVertStereo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))    // Non-DATAPixx systems will return 0
        return 0;
    
	return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_VSTEREO;
}


// VGA 1 and VGA 2 both show an overlay composite of the left/right halves of the video image
void DPxEnableVidHorizOverlay()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) | DPXREG_VID_CTRL_HOVERLAY);
}


// Horizontal overlay is disabled
void DPxDisableVidHorizOverlay()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_HOVERLAY);
}


// Returns non-0 if the left/right halves of the video image are being overlayed
int DPxIsVidHorizOverlay()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))    // Non-DATAPixx systems will return 0
        return 0;
    
	return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_HOVERLAY;
}

// Set bounding rectangle within left half image whose contents are composited with right half image
void DPxSetVidHorizOverlayBounds(int X1, int Y1, int X2, int Y2)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_HOVERLAY_X1, X1);
	DPxSetReg16(DPXREG_VID_HOVERLAY_Y1, Y1);
	DPxSetReg16(DPXREG_VID_HOVERLAY_X2, X2);
	DPxSetReg16(DPXREG_VID_HOVERLAY_Y2, Y2);
}


// Get bounding rectangle of horizontal overlay window
void DPxGetVidHorizOverlayBounds(int* X1, int* Y1, int* X2, int* Y2)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	*X1 = DPxGetReg16(DPXREG_VID_HOVERLAY_X1);
	*Y1 = DPxGetReg16(DPXREG_VID_HOVERLAY_Y1);
	*X2 = DPxGetReg16(DPXREG_VID_HOVERLAY_X2);
	*Y2 = DPxGetReg16(DPXREG_VID_HOVERLAY_Y2);
}


// Set 1024 16-bit video alpha values, in order X0,X1..X511,Y0,Y1...Y511
void DPxSetVidHorizOverlayAlpha(UInt16* alphaData)
{
	int payloadLength = 2048;

    if (!DPxSelectSysDevice(DPX_DEVSEL_DPX))
        return;
    
	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_WRITEALPHA;
	ep2out_Tram[2] = LSB(payloadLength);
	ep2out_Tram[3] = MSB(payloadLength);
	memcpy(ep2out_Tram+4, alphaData, payloadLength);
	if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
		DPxDebugPrint0("ERROR: DPxSetVidHorizOverlayAlpha() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_VID_ALPHA_WRITE_USB_ERROR);
	}
}

// Enable the VIEWPixx visual indicator (red flashing square on the top left of the screen) if the incoming video is rescanned and not lockable
void DPxEnableVidRescanWarning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;

	DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) & ~DPXREG_VID_CTRL2_NO_VLOCK_LAMP);
}

// Disable the VIEWPixx visual indicator (red flashing square on the top left of the screen) if the incoming video is rescanned and not lockable
void DPxDisableVidRescanWarning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL2, (DPxGetReg16(DPXREG_VID_CTRL2) | DPXREG_VID_CTRL2_NO_VLOCK_LAMP));
}

// Return non-0 if the visual indicator is enabled when incoming video is rescanned and not lockable 
int DPxIsVidRescanWarning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))    // Non-VIEWPixx systems will return 0
        return 0;
    
	return ~DPxGetReg16(DPXREG_VID_CTRL2) & DPXREG_VID_CTRL2_NO_VLOCK_LAMP;
}


// Get number of video dot times in one horizontal scan line (includes horizontal blanking interval).
// Note that this register is already multiplied by 2 if dual-link is active.
int DPxGetVidHTotal()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_HTOTAL);
}


// Get number of video lines in one vertical frame (includes vertical blanking interval)
int DPxGetVidVTotal()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VTOTAL);
}


// Get number of visible pixels in one horizontal scan line
int DPxGetVidHActive()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_HACTIVE);
}


// Get number of visible lines in one vertical frame
int DPxGetVidVActive()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VACTIVE);
}


// Get video vertical frame period in nanoseconds
unsigned DPxGetVidVPeriod()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg32(DPXREG_VID_VPERIOD_L) * 10;	// The DP register counts units of 10 ns.
}


// Get video vertical frame rate in Hz
double DPxGetVidVFreq()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return 1.0e9 / DPxGetVidVPeriod();
}


// Get video horizontal line rate in Hz
double DPxGetVidHFreq()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetVidVFreq() * DPxGetVidVTotal();
}


// Get video dot frequency in Hz
double DPxGetVidDotFreq()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetVidHFreq() * DPxGetVidHTotal();
}


// Returns non-0 if DATAPixx is currently receiving video data over DVI link
int DPxIsVidDviActive()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_STATUS) & DPXREG_VID_STATUS_DVI_ACTIVE;
}


// Returns non-0 if DATAPixx is currently receiving video data over dual-link DVI
int DPxIsVidDviActiveDual()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_STATUS) & DPXREG_VID_STATUS_DVI_ACTIVE_DUAL;
}


// Returns non-0 if VIEWPixx is currently receiving video whose timing can directly drive display.
// Resolution must be 1920x1200, with refresh rate in the range 100-120Hz.
int DPxIsVidDviLockable()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_STATUS) & DPXREG_VID_STATUS_DVI_LOCKABLE;
}


// Returns non-0 if DATAPixx is receiving video at too high a clock frequency
int DPxIsVidOverClocked()
{
	double dotFreq;
	
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	// OK if we're unplugged
	if (!DPxIsVidDviActive())
		return 0;

	// DVI standard says that maximum DVI_CLK is 165 MHz,
	// but I think an HDMI source could provide digital data up to 225 MHz.
	// Our DVI receiver can decode it, but FPGA DVI_CLK fmax is 165 MHz.
	dotFreq = DPxGetVidDotFreq();
	if (!DPxIsVidDviActiveDual())
		return dotFreq > 165e6;
		
	// OK, so we're receiving dual link DVI (HDMI is only single link).
	// If we are outputting pixels at half the input pixel rate, then we're OK up to 330 MHz.
	// This happens when in C48/C36D modes, or in horizontal split screen.
	if (DPxGetVidMode() == DPXREG_VID_CTRL_MODE_C48 ||
		DPxGetVidMode() == DPXREG_VID_CTRL_MODE_C36D ||
		DPxIsVidHorizSplit())
		return dotFreq > 330e6;

	// We're in dual link DVI, and we're outputting pixels at the input pixel rate.
	// FPGA calls this "turbo" mode, and its fmax is up around 300 MHz.
	// The limiting factor however is the video DAC ASIC which is spec'd at 200 MHz.
	// (In practice, the video DACs seem to be good for up to 240-250 MHz).
    // Of course, this is only a limit on the original DATAPixx.  The VIEWPixx isn't affected.
    if (!DPxIsS3Arch())
        return dotFreq > 200e6;
    
    // Otherwise, we're not overclocked
    return 0;
}


// VESA connector outputs left-eye signal
void DPxSetVidVesaLeft()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
    // Now all VESA writes are done at DPXREG_VID_VESA, and writing VESA_LR bit requires setting a write-enable bit
    DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) | DPXREG_VID_VESA_LEFT | DPXREG_VID_VESA_LEFT_WEN);
}


// VESA connector outputs right-eye signal
void DPxSetVidVesaRight()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
//    if (DPxIsS3Arch())
//        DPxSetReg16(DPXREG_VID_CTRL, DPxGetReg16(DPXREG_VID_CTRL) & ~DPXREG_VID_CTRL_VESA_LEFT);
//    else
//        DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) & ~DPXREG_VID_VESA_LEFT);

    // Now all VESA writes are done at DPXREG_VID_VESA, and writing VESA_LR bit requires setting a write-enable bit
    DPxSetReg16(DPXREG_VID_VESA, (DPxGetReg16(DPXREG_VID_VESA) & ~DPXREG_VID_VESA_LEFT) | DPXREG_VID_VESA_LEFT_WEN);
}


// Returns non-0 if VESA connector has left-eye signal
int DPxIsVidVesaLeft()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;
    
    if (DPxIsViewpixx() && DPxGetFirmwareRev() <= 12)
        return DPxGetReg16(DPXREG_VID_CTRL) & DPXREG_VID_CTRL_VESA_LEFT;
    else
        return DPxGetReg16(DPXREG_VID_VESA) & DPXREG_VID_VESA_LEFT;
}


// VESA 3D output interprets middle pixel on last raster line as a blueline code
void DPxEnableVidVesaBlueline()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) | DPXREG_VID_VESA_BLUELINE);
}


// VESA 3D output is not dependent on video content
void DPxDisableVidVesaBlueline()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) & ~DPXREG_VID_VESA_BLUELINE);
}


// Returns non-0 if VESA 3D output is dependent on video blueline codes
int DPxIsVidVesaBlueline()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VESA) & DPXREG_VID_VESA_BLUELINE;
}

// Returns non-0 if VESA 3D output is dependent on NVIDIA 3D Sync
int DPxIsVidVesaNv3dSynced()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VESA) & DPXREG_VID_VESA_NV3D_SYNCED;
}

// Enable PROPixx 3D VESA output freeRun enable bit
void DPxEnableVidVesaFreeRun()
{
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX)) // for PROPixx only
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) | DPXREG_VID_VESA_PPX_FREERUN);
}


// Disable PROPixx 3D VESA output freeRun enable bit
void DPxDisableVidVesaFreeRun()
{
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX)) // for PROPixx only
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, DPxGetReg16(DPXREG_VID_VESA) & ~DPXREG_VID_VESA_PPX_FREERUN);
}


// Returns non-0 if PROPixx VESA 3D output enabled by the freeRun register bit
int DPxIsVidVesaFreeRun()
{
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX)) // for PROPixx only
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VESA) & DPXREG_VID_VESA_PPX_FREERUN; 
}



// Set the waveform which will be sent to the DPX/VPX/DP2/PPC VESA 3D port
// waveform is one of the following predefined constants:
//      DPXREG_VID_VESA_WAVEFORM_LR             : VESA port drives straight L/R squarewave for 3rd party emitter
//      DPXREG_VID_VESA_WAVEFORM_CRYSTALEYES    : VESA port drives 3DPixx IR emitter for CrystalEyes 3D goggles
//      DPXREG_VID_VESA_WAVEFORM_NVIDIA         : VESA port drives 3DPixx IR emitter for NVIDIA 3D goggles
// We also set a reasonable VESA phase for the given waveform.
// User can override this phase afterwards by calling DPxSetVidVesaPhase().
void DPxSetVidVesaWaveform(int waveform)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, (DPxGetReg16(DPXREG_VID_VESA) & ~DPXREG_VID_VESA_WAVEFORM_MASK) | waveform);

    // Set phase of 3D glasses
    if (DPxIsPropixx())
        DPxSetVidVesaPhase(0);      // For PROPixx/DepthQ, no phase adjust
    else if (DPxIsViewpixx())
        DPxSetVidVesaPhase(100);    // For VIEWPixx, delay goggles 78% of video frame to wait for scanning backlight
    else
        DPxSetVidVesaPhase(245);    // For DATAPixx and DATAPixx2, flip goggles 9% of video frame early so they finish transitioning during vertical blanking
}


// Get the waveform which is being sent to the VPixx Product VESA 3D port
int DPxGetVidVesaWaveform()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_VESA) & DPXREG_VID_VESA_WAVEFORM_MASK;
}


// Set the 8-bit unsigned phase of the VESA 3D waveform
void DPxSetVidVesaPhase(int phase)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;
    
	DPxSetReg16(DPXREG_VID_VESA, (DPxGetReg16(DPXREG_VID_VESA) & 0x00FF) | (phase << 8));    
}


// Get the 8-bit unsigned phase of the VESA 3D waveform
int DPxGetVidVesaPhase()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;
    
	return (DPxGetReg16(DPXREG_VID_VESA) & 0xFF00) >> 8;
}


// Read pixels from the VPixx device line buffer, and return a pointer to the data.
// For each pixel, the buffer contains 16 bit R/G/B/U (where U is unused).
// The returned data could be overwritten by the next DPx* call.
UInt16* DPxGetVidLine()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_DPX_TPC))
        return 0;
    
	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_READVIDLINE;
	ep2out_Tram[2] = 0;
	ep2out_Tram[3] = 0;
	if (EZWriteEP2Tram(ep2out_Tram, EP6IN_READVIDLINE, 16384)) {
		DPxDebugPrint0("ERROR: DPxGetVidLine() call to EZWriteEP2Tram() failed\n");
		DPxSetError(DPX_ERR_RAM_READ_USB_ERROR);
		return 0;
	}

	return (UInt16*)(ep6in_Tram+4);
}


// Set the raster line on which pixel sync sequence is expected
void DPxSetVidPsyncRasterLine(int line)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	if (line & ~DPXREG_VID_PSYNC_RASTER_LINE) {
		DPxDebugPrint1("ERROR: DPxSetVidPsyncRasterLine() line %d out of range\n", line);
		DPxSetError(DPX_ERR_VID_PSYNC_LINE_ARG_ERROR);
		return;
	}

	if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) 
	{
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_PSYNC, (DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_RASTER_LINE) | line);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
		DPxSetReg16(DPXREG_VID_PSYNC, (DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_RASTER_LINE) | line);
        return;
    }

	// Normal case for all other devices.
	if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
		
	DPxSetReg16(DPXREG_VID_PSYNC, (DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_RASTER_LINE) | line);
}


// Get the raster line on which pixel sync sequence is expected
int DPxGetVidPsyncRasterLine()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_PSYNC) & DPXREG_VID_PSYNC_RASTER_LINE;
}


// Pixel sync is only recognized on a single raster line
void DPxEnableVidPsyncSingleLine()
{

	// Special case when PROPixx + Ctrl
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) 
	{
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_SINGLE_LINE);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
		DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_SINGLE_LINE);
        return;
    }

	// Normal case for all other devices.
	if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
		
	DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_SINGLE_LINE);
	
	
}


// Pixel sync is recognized on any raster line
void DPxDisableVidPsyncSingleLine()
{

	// Special case when PROPixx + Ctrl
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) 
	{
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_SINGLE_LINE);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
		DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_SINGLE_LINE);
        return;
    }

	// Normal case for all other devices.
	if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
		
	DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_SINGLE_LINE);
	
}


// Returns non-0 if pixel sync is only recognized on a single raster line
int DPxIsVidPsyncSingleLine()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_PSYNC) & DPXREG_VID_PSYNC_SINGLE_LINE;
}


// Pixel sync raster line is always displayed black
void DPxEnableVidPsyncBlankLine()
{
	// Special case when PROPixx + Ctrl
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) 
	{
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_BLANK_LINE);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
		DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_BLANK_LINE);
        return;
    }

	// Normal case for all other devices.
	if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) | DPXREG_VID_PSYNC_BLANK_LINE);
}


// Pixel sync raster line is displayed normally
void DPxDisableVidPsyncBlankLine()
{
	// Special case when PROPixx + Ctrl
    if (dpxUsrDevsel == DPX_DEVSEL_AUTO && DPxIsPropixx() && DPxIsPropixxCtrl()) 
	{
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
            return;
        	DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_BLANK_LINE);
        if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
            return;
			DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_BLANK_LINE);
        return;
    }

	// Normal case for all other devices.
	if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_PSYNC, DPxGetReg16(DPXREG_VID_PSYNC) & ~DPXREG_VID_PSYNC_BLANK_LINE);
}


// Returns non-0 if pixel sync raster line is always displayed black
int DPxIsVidPsyncBlankLine()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_PSYNC) & DPXREG_VID_PSYNC_BLANK_LINE;
}

// Enable VIEWPixx scanning backlight
void DPxEnableVidScanningBacklight()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
    DPxSetReg16(DPXREG_VID_BL_SCAN_CTRL, 0xD5FF);
    DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) | DPXREG_VID_CTRL2_PIXELDRIVE | DPXREG_VID_CTRL2_PIXELDRIVE_ACCUM);
}


// Disable VIEWPixx scanning backlight
void DPxDisableVidScanningBacklight()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
    DPxSetReg16(DPXREG_VID_BL_SCAN_CTRL, 0x00FF);
    DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) & ~(DPXREG_VID_CTRL2_PIXELDRIVE | DPXREG_VID_CTRL2_PIXELDRIVE_ACCUM));
}


// Returns non-0 if VIEWPixx scanning backlight is enabled
int DPxIsVidScanningBacklight()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return 0;

    return DPxGetReg16(DPXREG_VID_BL_SCAN_CTRL) & 0x8000;
}


// Enable 3D pixel polarity inversion
void DPxEnableVidLcd3D60Hz()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) | DPXREG_VID_CTRL2_LCD_3D60HZ);
}


// Return to normal pixel polarity inversion
void DPxDisableVidLcd3D60Hz()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) & ~DPXREG_VID_CTRL2_LCD_3D60HZ);
}


// Returns non-0 if 3D pixel polarity inversion is enabled
int DPxIsVidLcd3D60Hz()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return 0;
    
    return DPxGetReg16(DPXREG_VID_CTRL2) & DPXREG_VID_CTRL2_LCD_3D60HZ;
}


// Set source of video pattern to be displayed
void DPxSetVidSource(int vidSource)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_VID_SRC, vidSource);
}


// Get source of video pattern being displayed
int DPxGetVidSource()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_SRC);
}


void DPxSetVidSwtp(int address)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return;
    
    DPxSetVidSource(DPXREG_VID_SRC_SWTP | (address >> (DPxIsPropixx() ? 20 : 16)));
}


void DPxSetVidSwtp3D(int address)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return;
    
    DPxSetVidSource(DPXREG_VID_SRC_SWTP_3D | (address >> (DPxIsPropixx() ? 20 : 16)));
}


int DPxGetVidSwtpAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return 0;
    
    return (DPxGetVidSource() & 0x0FF0) << (DPxIsPropixx() ? 20 : 16);
}


void DPxSetVidBacklightIntensity(int intensity)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return;

	if (intensity < 0 || intensity > 255) {
		DPxDebugPrint1("ERROR: DPxSetVidBacklightIntensity() argument value %d is out of range (0-255)\n", intensity);
		DPxSetError(DPX_ERR_VID_BL_INTENSITY_ARG_ERROR);
		return;
	}
    
    DPxSetReg16(DPXREG_VID_BL_INTENSITY, (DPxGetReg16(DPXREG_VID_BL_INTENSITY) & 0x00FF) | ((0xFF-intensity) << 8));
}


int DPxGetVidBacklightIntensity()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_VPX))
        return 0;
    
    return (0xFF-MSB(DPxGetReg16(DPXREG_VID_BL_INTENSITY) & DPXREG_VID_BL_INTENSITY_MASK));
}

void DPxSetVidGreyscaleMode(int greyscaleMode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return;

	if (greyscaleMode < DPXREG_VID_CTRL3_GREY_MODE_DISABLE || greyscaleMode > DPXREG_VID_CTRL3_GREY_MODE_BLU) {
		DPxDebugPrint1("ERROR: DPxSetVidGreyscaleMode() invalid mode: %d\n", greyscaleMode);
		DPxSetError(DPX_ERR_VID_GREY_MODE_ARG_ERROR);
		return;
	}
    DPxSetReg16(DPXREG_VID_CTRL3, (DPxGetReg16(DPXREG_VID_CTRL3) & ~DPXREG_VID_CTRL3_GREY_MODE_MASK) | greyscaleMode);
}

int DPxGetVidGreyscaleMode()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX))
        return 0;
    
    return (DPxGetReg16(DPXREG_VID_CTRL3) & DPXREG_VID_CTRL3_GREY_MODE_MASK);
}





// Shortcut to stop running all DAC/ADC/DOUT/DIN/AUD/AUX/MIC/TACH schedules
void DPxStopAllScheds()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, 0xAAAA);
}


typedef struct {
    UInt16          ctrl;       // 15:Unused, 14:DE, 13:HSync, 12:VSync, 11-8:BlueLSB, 7-4:GreenLSB, 3-0:RedLSB
    unsigned char   redE;
    unsigned char   greenE;
    unsigned char   blueE;
    unsigned char   redO;
    unsigned char   greenO;
    unsigned char   blueO;
} ScopePixel;

// VScope video timing bit locations
#define SCOPE_CTRL_DE       0x4000
#define SCOPE_CTRL_HSYNC    0x2000
#define SCOPE_CTRL_VSYNC    0x1000
#define SCOPE_CTRL_DDC_SCL  0x0002
#define SCOPE_CTRL_DDC_SDA  0x0001

#define N_SCOPE_TEST_FRAMES 10
#define MAX_SCOPE_HMSGS     20
#define SCOPE_BUFF_SIZE ((165000000/120)*(N_SCOPE_TEST_FRAMES+2))      // Make sure we read in enough data to get all the frames we need, even under bizarre video format problems.
ScopePixel scopePixelBuff[SCOPE_BUFF_SIZE]; 

FILE *fpVScope;
int vsLoop = 0;

static void PrintScopePixels(int inBuffIndex, int nPixels);
void DPxVideoScope(int toFile)
{
    char *displayName;

    int regVidCtrl2;
    int timHActive, timHBl, timHFp, timHSync, timHBp, timHTotal, timVActive, timVBl, timVFp, timVSync, timVTotal, timFTotal;    
    int i, j, k;
    int inVBlank, vFrameEnd, vFrame, vFrameMin, vFrameMax, nVFrame, nVFrameErrors;
    int hActiveStart, hActiveEnd, hActive, hActiveMin, hActiveMax, nHActive, nHActiveErrors;
    int hFPEnd, hFP, hFPMin, hFPMax, nHFP, nHFPErrors;
    int hSyncEnd, hSync, hSyncMin, hSyncMax, nHSync, nHSyncErrors;
    int hBPEnd, hBP, hBPMin, hBPMax, nHBP, nHBPErrors;
    int hTotalEnd, hTotal, hTotalMin, hTotalMax, nHTotal, nHTotalErrors;
    double hActiveSum, hFPSum, hSyncSum, hBPSum, hTotalSum, vFrameSum, hPeriodSum, vSyncSum;
    int hLineNumber, hLastErrorLine, enableHMessages, hMsgNumber;
    int lastFramePixel, yCoord;
    UInt16* vidLineData;
    int vertState, lineIsActive, vertLineCount, vertFpStartLine, vertSyncStartLine, vertBpStartLine;
    int vertFp, vertSync, vertBp, vertFpMin, vertFpMax, vertSyncMin, vertSyncMax, vertBpMin, vertBpMax;
    int i2, vSyncStartPhase, vSyncStartPhaseMin, vSyncStartPhaseMax, vSyncEndPhase, vSyncEndPhaseMin, vSyncEndPhaseMax;
    double hiddenRed, hiddenGrn, hiddenBlu;
    int hPeriod, hPeriodStart, hPeriodMin, hPeriodMax, nHPeriod, nHPeriodErrors;
    int vSyncStart, vSync, vSyncMin, vSyncMax, nVSync;
	char resp[3];

    vertFpStartLine = 0;
    vertSyncStartLine = 0;
    vertBpStartLine = 0;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC_VPX_TPC))
        return;

	printf("\n");
	printf("\nVSCOPE require a connected DVI video input to work");
	printf("\n");
    printf("\nAre you sure that you want to start VSCOPE? <Y/N>: ");

	do {
		fgets(resp, 3, stdin);
	} while (toupper(resp[0]) != 'Y' && toupper(resp[0]) != 'N');

    if (toupper(resp[0]) == 'N') {
		printf("\n"); 
		return;
	}

    // Get expected video characteristics
    if (DPxIsViewpixx3D()) {
        displayName = "VIEWPixx3D";
        timHActive  = 1920 / 2;
        timHBl      = 160 / 2;
        timHFp      = 16 / 2;
        timHSync    = 32 / 2;
        timVActive  = 1080;
        timVBl      = 163;
        timVFp      = 3;
        timVSync    = 6;
    }
    else if (DPxIsViewpixx()) {
        displayName = "VIEWPixx";
        timHActive  = 1920 / 2;
        timHBl      = 160 / 2;
        timHFp      = 48 / 2;
        timHSync    = 32 / 2;
        timVActive  = 1200;
        timVBl      = DPxGetReg16(DPXREG_VID_VTOTAL) < 1200 + (35+281)/2 ? 35 : 281;  // OS could be running us at 100Hz or 120Hz
        timVFp      = 3;
        timVSync    = 6;
    }
    else if (DPxIsPropixx()) {
        displayName = "PROPixx";
        timHActive  = 1920 / 2;
        timHBl      = 280 / 2;
        timHFp      = 88 / 2;
        timHSync    = 40 / 2;
        timVActive  = 1080;
        timVBl      = 47;
        timVFp      = 3;
        timVSync    = 6;
    }
    else if (DPxIsPropixxCtrl()) {
        displayName = "PROPixxCtrl";
        timHActive  = 1920 / 2;
        timHBl      = 280 / 2;
        timHFp      = 88 / 2;
        timHSync    = 40 / 2;
        timVActive  = 1080;
        timVBl      = 47;
        timVFp      = 3;
        timVSync    = 6;
    }
	else if (DPxIsDatapixx2()) {
        displayName = "DATAPixx2";
        timHActive  = 1920 / 2;
        timHBl      = 280 / 2;
        timHFp      = 88 / 2;
        timHSync    = 40 / 2;
        timVActive  = 1080;
        timVBl      = 47;
        timVFp      = 3;
        timVSync    = 6;
    }
    else {
        printf("ERROR: Unknown display type\n");
        return;
    }

    timHBp      = timHBl - timHFp - timHSync;
    timHTotal   = timHActive + timHBl;
    timVTotal   = timVActive + timVBl;
    timFTotal   = timHTotal * timVTotal;    

    if (toFile)
        fpVScope = fopen("listing.txt", "wt");
    else
        fpVScope = stdout;
Restart:
    inVBlank        = 0;
    vFrameEnd       = 0;
    vFrameMin       = 0x7FFFFFFF;
    vFrameMax       = -1;
    nVFrame         = 0;
    nVFrameErrors   = 0;
    vFrameSum       = 0;

    hActiveStart    = 0;
    hActiveEnd      = 0;
    hActiveMin      = 0x7FFFFFFF;
    hActiveMax      = -1;
    nHActive        = 0;
    nHActiveErrors  = 0;
    hActiveSum      = 0;

    hFPEnd          = 0;
    hFPMin          = 0x7FFFFFFF;
    hFPMax          = -1;
    nHFP            = 0;
    nHFPErrors      = 0;
    hFPSum          = 0;

    hSyncEnd        = 0;
    hSyncMin        = 0x7FFFFFFF;
    hSyncMax        = -1;
    nHSync          = 0;
    nHSyncErrors    = 0;
    hSyncSum        = 0;

    hBPEnd          = 0;
    hBPMin          = 0x7FFFFFFF;
    hBPMax          = -1;
    nHBP            = 0;
    nHBPErrors      = 0;
    hBPSum          = 0;

    hTotalEnd       = 0;
    hTotalMin       = 0x7FFFFFFF;
    hTotalMax       = -1;
    nHTotal         = 0;
    nHTotalErrors   = 0;
    hTotalSum       = 0;
    
    hPeriodStart    = 0;
    hPeriodMin      = 0x7FFFFFFF;
    hPeriodMax      = -1;
    nHPeriod        = 0;
    nHPeriodErrors  = 0;
    hPeriodSum      = 0;

    hLineNumber     = 0;
    hLastErrorLine  = -1;
    hMsgNumber       = 0;
    enableHMessages  = 1;

    vSync         = 0;
    vSyncStart    = -1;
    vSyncMin      = 0x7FFFFFFF;
    vSyncMax      = -1;
    nVSync        = 0;
    vSyncSum      = 0;

    lastFramePixel = 0;
    yCoord = 0;

    vertState = 0;
    lineIsActive = 0;
    vertLineCount = 0;
    vertFpMin = 1000000;
    vertFpMax = -1;
    vertSyncMin = 1000000;
    vertSyncMax = -1;
    vertBpMin = 1000000;
    vertBpMax = -1;

    vSyncStartPhaseMin = 1000000000;
    vSyncStartPhaseMax = -1;
    vSyncEndPhaseMin = 1000000000;
    vSyncEndPhaseMax = -1;

    hiddenRed = 0.0;
    hiddenGrn = 0.0;
    hiddenBlu = 0.0;

    // Pixel drive will clobber VScope, so turn it off
    regVidCtrl2 = DPxGetReg16(DPXREG_VID_CTRL2);
    DPxSetReg16(DPXREG_VID_CTRL2, regVidCtrl2 & !(DPXREG_VID_CTRL2_PIXELDRIVE | DPXREG_VID_CTRL2_PIXELDRIVE_ACCUM));
    DPxUpdateRegCache();
    
    // Initiate frame grab
    DPxSetReg16(DPXREG_VID_SCOPE, 1);
    DPxUpdateRegCache();

    // Wait until frame grab done
    do {
        DPxUpdateRegCache();
    } while (DPxGetReg16(DPXREG_VID_SCOPE));

    // Get buffer
    DPxReadRam(0, sizeof(scopePixelBuff), scopePixelBuff);
    
    // Restore normal operation
    DPxSetReg16(DPXREG_VID_CTRL2, regVidCtrl2);
    DPxUpdateRegCache();

    // Get first video line from line grabber
    vidLineData = DPxGetVidLine();

    // Do a little analysis of I2C activity on DDC bus
    for (i = 1; i < SCOPE_BUFF_SIZE; i++) {
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SCL) != (scopePixelBuff[i-1].ctrl & SCOPE_CTRL_DDC_SCL)) {
            fprintf(fpVScope, "SCL=%d, i=%d\n", (scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SCL) ? 1 : 0, i);
        }
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SDA) != (scopePixelBuff[i-1].ctrl & SCOPE_CTRL_DDC_SDA)) {
            fprintf(fpVScope, "   SDA=%d, i=%d\n", (scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SDA) ? 1 : 0, i);
        }
    }

    // Parse the buffer
    for (i = 0; nVFrame < N_SCOPE_TEST_FRAMES; i++) {

        // Did we reach the end of our buffer without finding the desired number of video frames?
        if (i >= SCOPE_BUFF_SIZE) {
            if (nVFrame == 0) {
                fprintf(fpVScope, "***ERROR: No VSYNC found.  Is %s connected to video source?\n", displayName);
                goto endAnalyzeVideo;
            }
            else {
                fprintf(fpVScope, "***ERROR: Only %d of %d VSYNC found in buffer.\n", nVFrame, N_SCOPE_TEST_FRAMES);
                goto endAnalyzeVideo;
            }
        }        
        
        // We will use the convention that the first active pixel after vertical blanking is the start of a video frame, and the start of a video line
        if (scopePixelBuff[i].ctrl & SCOPE_CTRL_VSYNC)
            inVBlank = 1;
        lastFramePixel = 0;
        if (inVBlank && (scopePixelBuff[i+1].ctrl & SCOPE_CTRL_DE)) {   // We're in the last VBL pixel before a new frame
            inVBlank = 0;
            lastFramePixel = 1;
            if (vFrameEnd == 0) {   // Will this be the first frame we analyze?
                fprintf(fpVScope, "First frame starts at address 0x%x\n", (unsigned int)((i+1)*sizeof(ScopePixel)));
                vFrameEnd = i;
                hTotalEnd = i;      // So that first video line can calculate HTOTAL
                continue;           // Don't process last pixel in VBL
            }
            else {                  // This is not the first frame we process, so we can do frame length analysis
                vFrame = i - vFrameEnd;
                vFrameSum += vFrame;
                nVFrame++;
                if (vFrame != timFTotal) {
                    nVFrameErrors++;
                    fprintf(fpVScope, "***ERROR: Frame %d FTOTAL[nom=%d] = %d\n", nVFrame, timFTotal, vFrame);
                }
                if (vFrameMin > vFrame)
                    vFrameMin = vFrame;
                if (vFrameMax < vFrame)
                    vFrameMax = vFrame;
                vFrameEnd = i;
            }
        }

        // We will not do any more processing of the buffer data until we have found the first frame start
        if (!vFrameEnd)
            continue;

        // We are adding simplified calculation of vertical timing, updated on leading edge of each HSYNC
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_HSYNC) && !(scopePixelBuff[i-1].ctrl & SCOPE_CTRL_HSYNC)) {
            vertLineCount++;
            lineIsActive = scopePixelBuff[i-480].ctrl & SCOPE_CTRL_DE;  // 200 has to be 
            switch (vertState) {
                case 0: // Startup, waiting for active video before beginning processing
                    if (lineIsActive)
                        vertState = 1;
                    break;
                    
                case 1:     // In active region, waiting for start of VBL
                    if (!lineIsActive) {
                        vertFpStartLine = vertLineCount;
                        vertState = 2;
                    }
                    else    // Don't break if in VFP, in case VFP = 0
                        break;
                    
                case 2:     // In vertical front porch, waiting for VSYNC
                    if (scopePixelBuff[i-100].ctrl & SCOPE_CTRL_VSYNC) {
                        vertSyncStartLine = vertLineCount;
                        vertState = 3;
                    }
                    break;
                    
                case 3:     // In vertical sync, waiting for vertical back porch
                    if (!(scopePixelBuff[i-100].ctrl & SCOPE_CTRL_VSYNC)) {
                        vertBpStartLine = vertLineCount;
                        vertState = 4;
                    }
                    else    // Don't break if going into VBP, in case VBP = 0
                        break;
                    
                case 4:     // In vertical back porch, waiting for active
                    if (lineIsActive) {
                        vertFp = vertSyncStartLine - vertFpStartLine;
                        vertSync = vertBpStartLine - vertSyncStartLine;
                        vertBp = vertLineCount - vertBpStartLine;
                        if (vertFpMin > vertFp)
                            vertFpMin = vertFp;
                        if (vertFpMax < vertFp)
                            vertFpMax = vertFp;
                        if (vertSyncMin > vertSync)
                            vertSyncMin = vertSync;
                        if (vertSyncMax < vertSync)
                            vertSyncMax = vertSync;
                        if (vertBpMin > vertBp)
                            vertBpMin = vertBp;
                        if (vertBpMax < vertBp)
                            vertBpMax = vertBp;
                        vertState = 1;
                    }
                    break;
            }

            lineIsActive = 0;   // Reset for active test for next line
        }
        else if (scopePixelBuff[i].ctrl & SCOPE_CTRL_DE)    // Detect if line leading up to HSYNC is active
            lineIsActive = 1;

        // What's the phase relationship between VSYNC and HSYNC
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_VSYNC) && !(scopePixelBuff[i-1].ctrl & SCOPE_CTRL_VSYNC)) {
            for (i2 = i; i2 > 0; i2--) {
                if ((scopePixelBuff[i2].ctrl & SCOPE_CTRL_HSYNC) && !(scopePixelBuff[i2-1].ctrl & SCOPE_CTRL_HSYNC)) {
                    vSyncStartPhase = i - i2;
//                    printf("vSyncStartPhase = %d, i = %d, i2 = %d\n", vSyncStartPhase, i, i2);
                    if (vSyncStartPhaseMin > vSyncStartPhase)
                        vSyncStartPhaseMin = vSyncStartPhase;
                    if (vSyncStartPhaseMax < vSyncStartPhase)
                        vSyncStartPhaseMax = vSyncStartPhase;
                    break;
                }
            }
        }
        else if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_VSYNC) && (scopePixelBuff[i-1].ctrl & SCOPE_CTRL_VSYNC)) {
            for (i2 = i; i2 > 0; i2--) {
                if ((scopePixelBuff[i2].ctrl & SCOPE_CTRL_HSYNC) && !(scopePixelBuff[i2-1].ctrl & SCOPE_CTRL_HSYNC)) {
                    vSyncEndPhase = i - i2;
                    if (vSyncEndPhaseMin > vSyncEndPhase)
                        vSyncEndPhaseMin = vSyncEndPhase;
                    if (vSyncEndPhaseMax < vSyncEndPhase)
                        vSyncEndPhaseMax = vSyncEndPhase;
                    break;
                }
            }
        }

        // Report non-zero RGB data received during blanking
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].redE)
            hiddenRed++;
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].redO)
            hiddenRed++;
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].greenE)
            hiddenGrn++;
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].greenO)
            hiddenGrn++;
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].blueE)
            hiddenBlu++;
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && scopePixelBuff[i].blueO)
            hiddenBlu++;

        // Detect start of Horizontal Active.
        if (!(scopePixelBuff[i-1].ctrl & SCOPE_CTRL_DE) && (scopePixelBuff[i].ctrl & SCOPE_CTRL_DE))
            hActiveStart = i;

        // Horizontal Active.
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && !(scopePixelBuff[i+1].ctrl & SCOPE_CTRL_DE)) {
            hActiveEnd = i;
            if (hActiveStart) {
                hActive = hActiveEnd-hActiveStart+1;
                hActiveSum += hActive;
                nHActive++;
                if (hActive != timHActive) {
                    nHActiveErrors++;
                    if (enableHMessages) {
                        if (hLastErrorLine != hLineNumber) {
                            hLastErrorLine = hLineNumber;
                            fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                        }
                        else
                            fprintf(fpVScope, ", ");
                        fprintf(fpVScope, "HACTIVE[nom=%d] = %d", timHActive, hActive);
                    }
                }
                if (hActiveMin > hActive)
                    hActiveMin = hActive;
                if (hActiveMax < hActive)
                    hActiveMax = hActive;
            }
        }

        // Horizontal Front Porch
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_HSYNC) && (scopePixelBuff[i+1].ctrl & SCOPE_CTRL_HSYNC)) {
            hFPEnd = i;
            if (hActiveEnd) {
                hFP = hFPEnd-hActiveEnd;
                if (hFP < timHTotal) {     // Ignore vertical blank periods
                    hFPSum += hFP;
                    nHFP++;
                    if (hFP != timHFp) {
                        nHFPErrors++;
                        if (enableHMessages) {
                            if (hLastErrorLine != hLineNumber) {
                                hLastErrorLine = hLineNumber;
                                fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                            }
                            else
                                fprintf(fpVScope, ", ");
                            fprintf(fpVScope, "HFP[nom=%d] = %d", timHFp, hFP);
                        }
                    }
                    if (hFPMin > hFP)
                        hFPMin = hFP;
                    if (hFPMax < hFP)
                        hFPMax = hFP;
                }
            }
        }

        // Horizontal Sync
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_HSYNC) && !(scopePixelBuff[i+1].ctrl & SCOPE_CTRL_HSYNC)) {
            hSyncEnd = i;
            if (hFPEnd) {
                hSync = hSyncEnd-hFPEnd;
                hSyncSum += hSync;
                nHSync++;
                if (hSync != timHSync) {
                    nHSyncErrors++;
                    if (enableHMessages) {
                        if (hLastErrorLine != hLineNumber) {
                            hLastErrorLine = hLineNumber;
                            fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                        }
                        else
                            fprintf(fpVScope, ", ");
                        fprintf(fpVScope, "HSYNC[nom=%d] = %d", timHSync, hSync);
                    }
                }
                if (hSyncMin > hSync)
                    hSyncMin = hSync;
                if (hSyncMax < hSync)
                    hSyncMax = hSync;
            }
        }

        // Horizontal Back Porch
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_DE) && (scopePixelBuff[i+1].ctrl & SCOPE_CTRL_DE)) {
            hBPEnd = i;
            if (hSyncEnd) {
                hBP = hBPEnd-hSyncEnd;
                hBPSum += hBP;
                nHBP++;
                if (hBP != timHBp) {
                    nHBPErrors++;
                    if (enableHMessages) {
                        if (hLastErrorLine != hLineNumber) {
                            hLastErrorLine = hLineNumber;
                            fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                        }
                        else
                            fprintf(fpVScope, ", ");
                        fprintf(fpVScope, "HBP[nom=%d] = %d", timHBp, hBP);
                    }
                }
                if (hBPMin > hBP)
                    hBPMin = hBP;
                if (hBPMax < hBP)
                    hBPMax = hBP;
            }

            // While we're at the end of the back porch, we'll also collects stats for entire lines
            if (hTotalEnd) {
                hTotal = i - hTotalEnd;
                if (hTotal < timHTotal*timVSync) {     // Ignore vertical blank periods
                    hTotalSum += hTotal;
                    nHTotal++;
                    if (hTotal != timHTotal) {
                        nHTotalErrors++;
                        if (enableHMessages) {
                            if (hLastErrorLine != hLineNumber) {
                                hLastErrorLine = hLineNumber;
                                fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                            }
                            else
                                fprintf(fpVScope, ", ");
                            fprintf(fpVScope, "HTOTAL[nom=%d] = %d", timHTotal, hTotal);
                        }
                    }
                    if (hTotalMin > hTotal)
                        hTotalMin = hTotal;
                    if (hTotalMax < hTotal)
                        hTotalMax = hTotal;
                }
            }
            hTotalEnd = i;

            // End-of-line message processing
            if (hLastErrorLine == hLineNumber) {
                fprintf(fpVScope, ", RGB[");
                PrintScopePixels(hActiveStart, 2);
                fprintf(fpVScope, "]..[");
                PrintScopePixels(hActiveEnd-1, 2);
                fprintf(fpVScope, "]\n");
                if (++hMsgNumber >= MAX_SCOPE_HMSGS)
                    enableHMessages = 0;
            }
            if (nVFrame < N_SCOPE_TEST_FRAMES && yCoord < 4) { // Show for every frame, so that we can detect FRC
                if (yCoord == 0)
                    fprintf(fpVScope, "Frame %d\n", nVFrame);
                fprintf(fpVScope, "   Line %d RGB[", yCoord);
                PrintScopePixels(hActiveStart, 2);
                fprintf(fpVScope, "]..[");
                PrintScopePixels(hActiveEnd-1, 2);
                fprintf(fpVScope, "]\n");
            }

            // Print out one complete line if we're writing to a file
            if (toFile && hLineNumber == 8) {
                for (j = hActiveStart; j <= hActiveEnd; j += 2) {
                    fprintf(fpVScope, "2clk[%d]=[", j-hActiveStart);
                    PrintScopePixels(j, 2);
                    fprintf(fpVScope, "]\n");
                }
             }

            // A quick self-test to see if video line readback works.
            if (hLineNumber == 0) {
                for (j = 0; j < (hActiveEnd-hActiveStart+1)/2; j++) {
                    k = DPxIsVidDviActiveDual() ? j : j*2+1;
                    if (scopePixelBuff[hActiveStart+k].redE   != vidLineData[j*8+0] >> 8 ||
                        scopePixelBuff[hActiveStart+k].greenE != vidLineData[j*8+1] >> 8 ||
                        scopePixelBuff[hActiveStart+k].blueE  != vidLineData[j*8+2] >> 8 ||
                        scopePixelBuff[hActiveStart+k].redO   != vidLineData[j*8+4] >> 8 ||
                        scopePixelBuff[hActiveStart+k].greenO != vidLineData[j*8+5] >> 8 ||
                        scopePixelBuff[hActiveStart+k].blueO  != vidLineData[j*8+6] >> 8) {
                        printf("***LineBuff error on frame %d, pixel pair %d***\n", nVFrame, j);
                        printf("DDR: (%02X,%02X,%02X),(%02X,%02X,%02X)\n", scopePixelBuff[hActiveStart+k].redE, scopePixelBuff[hActiveStart+k].greenE, scopePixelBuff[hActiveStart+k].blueE,
                                scopePixelBuff[hActiveStart+k].redO, scopePixelBuff[hActiveStart+k].greenO, scopePixelBuff[hActiveStart+k].blueO);
                        printf("LIN: (%02X,%02X,%02X),(%02X,%02X,%02X)\n", vidLineData[j*8+0] >> 8, vidLineData[j*8+1] >> 8, vidLineData[j*8+2] >> 8, vidLineData[j*8+4] >> 8, vidLineData[j*8+5] >> 8, vidLineData[j*8+6] >> 8);
                    }
                }
            }

            // Prep for next line
            hLineNumber++;
            yCoord = lastFramePixel ? 0 : yCoord + 1;
        }
        
        // HPeriod calculation which is independent of DE
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_HSYNC) && (scopePixelBuff[i+1].ctrl & SCOPE_CTRL_HSYNC)) {
            if (hPeriodStart) {
                hPeriod = i - hPeriodStart;
                hPeriodSum += hPeriod;
                nHPeriod++;
                if (hPeriod != timHTotal) {
                    nHPeriodErrors++;
                    if (enableHMessages) {
                        if (hLastErrorLine != hLineNumber) {
                            hLastErrorLine = hLineNumber;
                            fprintf(fpVScope, "***ERROR: Line %d, y=%d: ", hLineNumber, yCoord);
                        }
                        else
                            fprintf(fpVScope, ", ");
                        fprintf(fpVScope, "HPERIOD[nom=%d] = %d", timHTotal, hPeriod);
                    }
                }
                if (hPeriodMin > hPeriod)
                    hPeriodMin = hPeriod;
                if (hPeriodMax < hPeriod)
                    hPeriodMax = hPeriod;
            }
            hPeriodStart = i;
        }

        
        // VSync duration
        if (!(scopePixelBuff[i].ctrl & SCOPE_CTRL_VSYNC) && (scopePixelBuff[i+1].ctrl & SCOPE_CTRL_VSYNC))
            vSyncStart = i;
        else if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_VSYNC) && !(scopePixelBuff[i+1].ctrl & SCOPE_CTRL_VSYNC)) {
            if (vSyncStart > 0) {
                vSync = i - vSyncStart;
                vSyncSum += vSync;
                nVSync++;
                if (vSyncMin > vSync)
                    vSyncMin = vSync;
                if (vSyncMax < vSync)
                    vSyncMax = vSync;
            }
        }
    }

endAnalyzeVideo:

    // Print summary stats
    fprintf(fpVScope, "\n");
    if (nVFrameErrors == 0)
        fprintf(fpVScope, "All %d VFRAME=%d\n", nVFrame, timFTotal);
    else
        fprintf(fpVScope, "***ERROR: VFRAME[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timFTotal, vFrameMin, vFrameMax, vFrameSum/nVFrame, nVFrameErrors, nVFrame);

    if (nHActiveErrors == 0)
        fprintf(fpVScope, "All %d HACTIVE=%d\n", nHActive, timHActive);
    else
        fprintf(fpVScope, "***ERROR: HACTIVE[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHActive, hActiveMin, hActiveMax, hActiveSum/nHActive, nHActiveErrors, nHActive);
    if (nHActive != N_SCOPE_TEST_FRAMES*timVActive)
        fprintf(fpVScope, "***ERROR: Expected %d HACTIVE, but recorded %d\n", N_SCOPE_TEST_FRAMES*timVActive, nHActive);

    if (nHFPErrors == 0)
        fprintf(fpVScope, "All %d HFP=%d\n", nHFP, timHFp);
    else
        fprintf(fpVScope, "***ERROR: HFP[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHFp, hFPMin, hFPMax, hFPSum/nHFP, nHFPErrors, nHFP);
    if (nHFP != N_SCOPE_TEST_FRAMES*timVActive)
        fprintf(fpVScope, "***ERROR: Expected %d HFP, but recorded %d\n", N_SCOPE_TEST_FRAMES*timVActive, nHFP);

    if (nHSyncErrors == 0)
        fprintf(fpVScope, "All %d HSYNC=%d\n", nHSync, timHSync);
    else
        fprintf(fpVScope, "***ERROR: HSYNC[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHSync, hSyncMin, hSyncMax, hSyncSum/nHSync, nHSyncErrors, nHSync);
    if (nHSync != N_SCOPE_TEST_FRAMES*timVTotal)
        fprintf(fpVScope, "***ERROR: Expected %d HSYNC, but recorded %d\n", N_SCOPE_TEST_FRAMES*timVTotal, nHSync);

    if (nHBPErrors == 0)
        fprintf(fpVScope, "All %d HBP=%d\n", nHBP, timHBp);
    else
        fprintf(fpVScope, "***ERROR: HBP[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHBp, hBPMin, hBPMax, hBPSum/nHBP, nHBPErrors, nHBP);
    if (nHBP != N_SCOPE_TEST_FRAMES*timVActive)
        fprintf(fpVScope, "***ERROR: Expected %d HBP, but recorded %d\n", N_SCOPE_TEST_FRAMES*timVActive, nHBP);

    if (nHTotalErrors == 0)
        fprintf(fpVScope, "All %d HTOTAL=%d\n", nHTotal, timHTotal);
    else
        fprintf(fpVScope, "***ERROR: HTOTAL[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHTotal, hTotalMin, hTotalMax, hTotalSum/nHTotal, nHTotalErrors, nHTotal);
    if (nHTotal != N_SCOPE_TEST_FRAMES*(timVActive-1)) // VBL scraps one of our HTOTAL data
        fprintf(fpVScope, "***ERROR: Expected %d HTOTAL, but recorded %d\n", N_SCOPE_TEST_FRAMES*(timVActive-1), nHTotal);
    if (nHPeriodErrors == 0)
        fprintf(fpVScope, "All %d HPERIOD=%d\n", nHPeriod, timHTotal);
    else
        fprintf(fpVScope, "***ERROR: HPERIOD[nom=%d] range=%d-%d, avg=%.3f, %d/%d errors\n", timHTotal, hPeriodMin, hPeriodMax, hPeriodSum/nHPeriod, nHPeriodErrors, nHPeriod);

    fprintf(fpVScope, "VFP range=%d-%d\n", vertFpMin, vertFpMax);
    fprintf(fpVScope, "VSYNC range=%d-%d\n", vertSyncMin, vertSyncMax);
    fprintf(fpVScope, "VBP range=%d-%d\n", vertBpMin, vertBpMax);
    fprintf(fpVScope, "VSYNC start phase range=%d-%d\n", vSyncStartPhaseMin, vSyncStartPhaseMax);
    fprintf(fpVScope, "VSYNC end phase range=%d-%d\n", vSyncEndPhaseMin, vSyncEndPhaseMax);
    fprintf(fpVScope, "VSYNC duration (in pixel clock time) range=%d-%d, avg of %d VSYNC =%.3f\n", vSyncMin, vSyncMax, nVSync, vSyncSum/nVSync);

#if 0
    if (hiddenRed > 0 || hiddenGrn > 0 || hiddenBlu > 0)
        fprintf(fpVScope, "***Hidden (R,G,B) = (%g,%g,%g)\n", hiddenRed, hiddenGrn, hiddenBlu);
#endif
    
    if (vsLoop) {
        if (vSyncMin < 6400)
            fprintf(fpVScope, "Found VScope error\n");
        else
            goto Restart;
            
    }

//endAnalyzeVideo:
    
    // Do a little analysis of I2C activity on DDC bus
    for (i = 1; i < SCOPE_BUFF_SIZE; i++) {
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SCL) != (scopePixelBuff[i-1].ctrl & SCOPE_CTRL_DDC_SCL)) {
            fprintf(fpVScope, "SCL=%d, i=%d\n", (scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SCL) ? 1 : 0, i);
        }
        if ((scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SDA) != (scopePixelBuff[i-1].ctrl & SCOPE_CTRL_DDC_SDA)) {
            fprintf(fpVScope, "   SDA=%d, i=%d\n", (scopePixelBuff[i].ctrl & SCOPE_CTRL_DDC_SDA) ? 1 : 0, i);
        }
    }

    if (fpVScope != stdout)
        fclose(fpVScope);
}


void PrintScopePixels(int inBuffIndex, int nPixels)
{
    int i;
    
    for (i = inBuffIndex; i < inBuffIndex+nPixels; i++) {
        if (i > inBuffIndex)
            fprintf(fpVScope, ",");
        fprintf(fpVScope, "(%02X,%02X,%02X),(%02X,%02X,%02X)", scopePixelBuff[i].redE, scopePixelBuff[i].greenE, scopePixelBuff[i].blueE,
                                                      scopePixelBuff[i].redO, scopePixelBuff[i].greenO, scopePixelBuff[i].blueO);
    }
}

// Get DATAPixx3 Temperature
double DPxGetDP3Temperature(int TempNum)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP3))
        return 0;

	switch(TempNum) {
        case DP3_TEMP_DP:			return (double)(LSB(DPxGetReg16(DPXREG_TEMP)));
        case DP3_TEMP_FPGA_EXT:     return (double)(MSB(DPxGetReg16(DPXREG_TEMP)));
        case DP3_TEMP_USB:			return (double)(LSB(DPxGetReg16(DPXREG_POWER2)));
        case DP3_TEMP_ADC:			return (double)(MSB(DPxGetReg16(DPXREG_POWER2)));
        default:
            DPxDebugPrint2("ERROR: DPxGetDP3Temperature() argument TempNum %d is not in range 0 to %d\n", TempNum, DP3_TEMP_ADC);
            DPxSetError(DPX_ERR_PPX_BAD_TEMP);
            return 0.0;
	}
}


// Get PROPixx Temperature
double DPxGetPPxTemperature(int TempNum)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	switch(TempNum) {
        case PPX_TEMP_LED_RED:			return (double)(LSB(DPxGetReg16(PPXREG_TEMP_LED_GRN_RED)));
        case PPX_TEMP_LED_GRN:          return (double)(MSB(DPxGetReg16(PPXREG_TEMP_LED_GRN_RED)));
        case PPX_TEMP_LED_BLU:			return (double)(LSB(DPxGetReg16(PPXREG_TEMP_LED_ALT_BLU)));
        case PPX_TEMP_LED_ALT:			return (double)(MSB(DPxGetReg16(PPXREG_TEMP_LED_ALT_BLU)));
        case PPX_TEMP_POWER_BOARD:		return (double)(LSB(DPxGetReg16(PPXREG_TEMP_DMD_POW)));
		case PPX_TEMP_DMD:				return (double)(MSB(DPxGetReg16(PPXREG_TEMP_DMD_POW)));
        case PPX_TEMP_LED_POWER_BOARD:	return (double)(LSB(DPxGetReg16(PPXREG_TEMP_RX_DVI_LED_POW_BD)));
        case PPX_TEMP_RX_DVI:			return (double)(MSB(DPxGetReg16(PPXREG_TEMP_RX_DVI_LED_POW_BD)));
        case PPX_TEMP_FPGA:				return (double)(LSB(DPxGetReg16(PPXREG_TEMP_DDC4100_FPGA)));
        case PPX_TEMP_FPGA2:			return (double)(MSB(DPxGetReg16(PPXREG_TEMP_DDC4100_FPGA)));
        default:
            DPxDebugPrint2("ERROR: DPxGetPPTemperature() argument TempNum %d is not in range 0 to %d\n", TempNum, PPX_TEMP_FPGA2);
            DPxSetError(DPX_ERR_PPX_BAD_TEMP);
            return 0.0;
	}
}



// Get Voltage Monitor
double DPxGetPPxVoltageMonitor(int voltageNum)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	switch(voltageNum) {
        case PPX_POWER_5V:		return (double)(DPxGetReg16(PPXREG_POWER_5V)) * 2 * 0.00030518;
        case PPX_POWER_2P5V:	return (double)(DPxGetReg16(PPXREG_POWER_2P5V)) * 0.00030518;
        case PPX_POWER_1P8V:	return (double)(DPxGetReg16(PPXREG_POWER_1P8V)) * 0.00030518;
        case PPX_POWER_1P5V:	return (double)(DPxGetReg16(PPXREG_POWER_1P5V)) * 0.00030518;
        case PPX_POWER_1P1V:	return (double)(DPxGetReg16(PPXREG_POWER_1P1V)) * 0.00030518;
        case PPX_POWER_1V:		return (double)(DPxGetReg16(PPXREG_POWER_1P0V)) * 0.00030518;
        case PPX_POWER_12V:		return (double)(DPxGetReg16(PPXREG_POWER_12V)) * 4 * 0.00030518;
        //case PPX_POWER_VCC:		return (double)((DPxGetReg16(PPXREG_POWER_VCC)) * 0.00030518) + 2.5;
        default:
            DPxDebugPrint2("ERROR: DPxGetPPVoltageMonitor() argument VoltageNum %d is not in range 0 to %d\n", voltageNum, PPX_POWER_12V);
            DPxSetError(DPX_ERR_PPX_BAD_VOLTAGE);
            return 0.0;
	}
}

float L11_to_float(unsigned int input_val)
{
	// extract exponent as MS 5 bits
	char exponent = input_val >> 11;
	// extract mantissa as LS 11 bits
	short int mantissa = input_val & 0x7ff;
	// sign extend exponent from 5 to 8 bits
	if( exponent > 0x0F ) exponent |= 0xE0;
	// sign extend mantissa from 11 to 16 bits
	if( mantissa > 0x03FF ) mantissa |= 0xF800;

	// compute value as mantissa * 2^(exponent)
	return mantissa * pow(2,exponent);
}

float L16_to_float(char exponent, unsigned int mantissa)
{
	// sign extend exponent
	if( exponent > 0x0F ) exponent |= 0xE0;
	// compute value as mantissa * 2^(exponent)
	return mantissa * pow(2,exponent);
}

void DPxSetPPxLedCurrent(int ledNum, double current)
{
    int vDac;
    
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    // Technically a DAC value of 0xFFFF would specify a current of 83A, but we'll clamp LLAPI to a value of 50A.
    if (current < 0 || current > 50) {
        DPxDebugPrint1("ERROR: DPxSetPPxLedCurrent() argument current %g is not in range 0 to 50A\n", current);
        DPxSetError(DPX_ERR_PPX_BAD_LED_CURRENT);
        return;
    }
    vDac = (int)floor(current * PPX_LEDCUR_TO_VDAC + 0.5);
    
	switch(ledNum) {
        case PPX_LED_CUR_RED_L:	DPxSetReg16(PPXREG_LED_DAC_RED_L, vDac); break;
        case PPX_LED_CUR_RED_H:	DPxSetReg16(PPXREG_LED_DAC_RED_H, vDac); break;
        case PPX_LED_CUR_GRN_L:	DPxSetReg16(PPXREG_LED_DAC_GRN_L, vDac); break;
        case PPX_LED_CUR_GRN_H:	DPxSetReg16(PPXREG_LED_DAC_GRN_H, vDac); break;
        case PPX_LED_CUR_BLU_L:	DPxSetReg16(PPXREG_LED_DAC_BLU_L, vDac); break;
        case PPX_LED_CUR_BLU_H:	DPxSetReg16(PPXREG_LED_DAC_BLU_H, vDac); break;
        case PPX_LED_CUR_ALT_L:	DPxSetReg16(PPXREG_LED_DAC_ALT_L, vDac); break;
        case PPX_LED_CUR_ALT_H:	DPxSetReg16(PPXREG_LED_DAC_ALT_H, vDac); break;
        default:
            DPxDebugPrint2("ERROR: DPxSetPPxLedCurrent() argument LedNum %d is not in range 0 to %d\n", ledNum, PPX_LED_CUR_ALT_H);
            DPxSetError(DPX_ERR_PPX_BAD_LED);
	}
}


// Get PROPixx LED Current
double DPxGetPPxLedCurrent(int ledNum)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	switch(ledNum) {
        case PPX_LED_CUR_RED_L:	return (double)(DPxGetReg16(PPXREG_LED_DAC_RED_L)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_RED_H:	return (double)(DPxGetReg16(PPXREG_LED_DAC_RED_H)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_GRN_L:	return (double)(DPxGetReg16(PPXREG_LED_DAC_GRN_L)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_GRN_H:	return (double)(DPxGetReg16(PPXREG_LED_DAC_GRN_H)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_BLU_L:	return (double)(DPxGetReg16(PPXREG_LED_DAC_BLU_L)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_BLU_H:	return (double)(DPxGetReg16(PPXREG_LED_DAC_BLU_H)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_ALT_L:	return (double)(DPxGetReg16(PPXREG_LED_DAC_ALT_L)) / PPX_LEDCUR_TO_VDAC;
        case PPX_LED_CUR_ALT_H:	return (double)(DPxGetReg16(PPXREG_LED_DAC_ALT_H)) / PPX_LEDCUR_TO_VDAC;
        default:
            DPxDebugPrint2("ERROR: DPxGetPPLedCurrent() argument LedNum %d is not in range 0 to %d\n", ledNum, PPX_LED_CUR_ALT_H);
            DPxSetError(DPX_ERR_PPX_BAD_LED);
            return 0.0;
	}
}


double DPxGetPPxFanTachometer(int fanNum)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	switch(fanNum) {
        case PPX_FAN_TACH_1: if (LSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)) == 0xFF || LSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(LSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)))));
        case PPX_FAN_TACH_2: if (MSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)) == 0xFF || MSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(MSB(DPxGetReg16(PPXREG_FAN_TACH_1_2)))));
        case PPX_FAN_TACH_3: if (LSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)) == 0xFF || LSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(LSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)))));
        case PPX_FAN_TACH_4: if (MSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)) == 0xFF || MSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(MSB(DPxGetReg16(PPXREG_FAN_TACH_3_4)))));
        case PPX_FAN_TACH_5: if (LSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)) == 0xFF || LSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(LSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)))));
        case PPX_FAN_TACH_6: if (MSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)) == 0xFF || MSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)) == 0x00) return 0.0; else return (double)((60*3900)/(2*(MSB(DPxGetReg16(PPXREG_FAN_TACH_5_6)))));
        default:
            DPxDebugPrint2("ERROR: DPxGetPPxFanTachometer() argument FanNum %d is not in range 0 to %d\n", fanNum, PPX_FAN_TACH_6);
            DPxSetError(DPX_ERR_PPX_BAD_FAN);
            return 0.0;
	}
}

double DPxGetPPxFanPwm()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return (double)((((DPxGetReg16(PPXREG_FAN_CONFIG)) & PPXREG_FAN_CONFIG_PWM)*100)/256);
}


int DPxIsPPxVidSeqEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_SEQ_CSR) & PPXREG_VID_SEQ_CSR_EN;
}


// Set 3D crosstalk (0-1) which should be subtracted from stereoscopic stimuli
// Only works on PROPixx in RB3D mode.
void DPxSetPPx3dCrosstalk(double crosstalk)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    if (crosstalk > 1)
        crosstalk = 1;
    else if (crosstalk < 0)
        crosstalk = 0;
    DPxSetReg16(PPXREG_3D_CROSSTALK, (int)floor(crosstalk * 65535 + 0.5));
}


// Get 3D crosstalk (0-1) which is being subtracted from stereoscopic stimuli
// Only works on PROPixx in RB3D mode.
double DPxGetPPx3dCrosstalk(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_3D_CROSSTALK) / 65535.0;
}


// Set 3D crosstalk (0-1) of left-eye image which should be subtracted from stereoscopic right-eye image
// Only works on PROPixx in RB3D mode.
void DPxSetPPx3dCrosstalkLR(double crosstalk)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    if (crosstalk > 1)
        crosstalk = 1;
    else if (crosstalk < 0)
        crosstalk = 0;
    DPxSetReg16(PPXREG_3D_CROSSTALK_LR, (int)floor(crosstalk * 65535 + 0.5));
}


// Get 3D crosstalk (0-1) of left-eye image which is being subtracted from stereoscopic right-eye image
// Only works on PROPixx in RB3D mode.
double DPxGetPPx3dCrosstalkLR(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_3D_CROSSTALK_LR) / 65535.0;
}


// Set 3D crosstalk (0-1) of right-eye image which should be subtracted from stereoscopic left-eye image
// Only works on PROPixx in RB3D mode.
void DPxSetPPx3dCrosstalkRL(double crosstalk)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    if (crosstalk > 1)
        crosstalk = 1;
    else if (crosstalk < 0)
        crosstalk = 0;
    DPxSetReg16(PPXREG_3D_CROSSTALK_RL, (int)floor(crosstalk * 65535 + 0.5));
}


// Get 3D crosstalk (0-1) of right-eye image which is being subtracted from stereoscopic left-eye image
// Only works on PROPixx in RB3D mode.
double DPxGetPPx3dCrosstalkRL(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_3D_CROSSTALK_RL) / 65535.0;
}


int DPxIsPPxAsyncResetEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_DDC_CFG) & PPXREG_VID_DDC_CFG_ARST;
}


int DPxIsPPxPowerFloatEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_DDC_CFG) & PPXREG_VID_DDC_CFG_PWR_FLOAT;
}


void DPxEnablePPxRearProjection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) | DPXREG_VID_CTRL3_REAR_PROJECTION);
}


void DPxDisablePPxRearProjection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) & ~DPXREG_VID_CTRL3_REAR_PROJECTION);
}


int DPxIsPPxRearProjection()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(DPXREG_VID_CTRL3) & DPXREG_VID_CTRL3_REAR_PROJECTION;
}


void DPxEnablePPxCeilingMount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) | DPXREG_VID_CTRL3_CEILING_MOUNT);
}


void DPxDisablePPxCeilingMount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) & ~DPXREG_VID_CTRL3_CEILING_MOUNT);
}


int DPxIsPPxCeilingMount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;

    return DPxGetReg16(DPXREG_VID_CTRL3) & DPXREG_VID_CTRL3_CEILING_MOUNT;
}

void DPxEnablePPxLampLed()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_SLEEP, PPXREG_SLEEP_LAMP_LED_ON);
}

void DPxDisablePPxLampLed()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_SLEEP, PPXREG_SLEEP_LAMP_LED_OFF);
}


int DPxIsPPxLampLedEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;

    return DPxGetReg16(PPXREG_SLEEP) & PPXREG_SLEEP_LAMP_LED_ON_STATUS;
}

void DPxSetPPxAwake()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_SLEEP, PPXREG_SLEEP_GOTO_AWAKE);
}

void DPxSetPPxSleep()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_SLEEP, PPXREG_SLEEP_GOTO_SLEEP);
}


int DPxIsPPxAwake()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;

    return DPxGetReg16(PPXREG_SLEEP) & PPXREG_SLEEP_AWAKE_STATUS;
}


// (PROPixx Rev >= 19 only)
// Enable quiet fan mode where fans run at a lower speed
void DPxEnablePPxQuietFanMode(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_FAN_CONFIG2, DPxGetReg16(PPXREG_FAN_CONFIG2) | PPXREG_FAN_CONFIG2_FAN_QUIET_MODE);
}

// Disable quiet fan mode
void DPxDisablePPxQuietFanMode(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

    DPxSetReg16(PPXREG_FAN_CONFIG2, DPxGetReg16(PPXREG_FAN_CONFIG2) & ~PPXREG_FAN_CONFIG2_FAN_QUIET_MODE);
}

// Returns non-0 if quiet fan mode is enabled
int	DPxIsPPxQuietFanMode(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;

    return DPxGetReg16(PPXREG_FAN_CONFIG2) & PPXREG_FAN_CONFIG2_FAN_QUIET_MODE;
}


// (PROPixx Rev >= 28 only)
void DPxEnablePPxSwtpLoad(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_SWTP_LOAD, DPxGetReg16(PPXREG_VID_SWTP_LOAD) | PPXREG_VID_SWTP_LOAD_EN);
}

void DPxDisablePPxSwtpLoad(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

    DPxSetReg16(PPXREG_VID_SWTP_LOAD, DPxGetReg16(PPXREG_VID_SWTP_LOAD) & ~PPXREG_VID_SWTP_LOAD_EN);
}

int	DPxIsPPxSwtpLoadEnabled(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;

    return DPxGetReg16(PPXREG_VID_SWTP_LOAD) & PPXREG_VID_SWTP_LOAD_EN;
}

void DPxSetPPxSwtpLoadPage(int page)
{    
	
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	if (page < 0 || page > 127) {
		DPxDebugPrint1("ERROR: DPxSetPPxSwtpLoadPage() argument value %d is out of range [0-127]\n", page);
		DPxSetError(DPX_ERR_PPX_SWTP_LOAD_PAGE_BAD_VALUE);
		return;
	}
	
	DPxSetReg16(PPXREG_VID_SWTP_LOAD, (DPxGetReg16(PPXREG_VID_SWTP_LOAD) & ~PPXREG_VID_SWTP_LOAD_PAGE_MASK) | page);
}


int DPxGetPPxSwtpLoadPage()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_VID_SWTP_LOAD) & PPXREG_VID_SWTP_LOAD_PAGE_MASK;
}



// PROPixx Tachistoscope sub-system
// Enable the PROPixx T-Scope subsystem.
// This will terminate normal display of incoming video.
// Contains an implicit call to DPxUpdateRegCache().
void DPxEnablePPxTScope()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	// Put low current values to avoid any LED current problem
	DPxSetPPxLedCurrent(PPX_LED_CUR_RED_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_GRN_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_BLU_H, 1);
    DPxUpdateRegCache();
    
    DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) | PPXREG_TSCOPE_CSR_EN);
    
    // T-Scope requires different LED currents
    // Note that this calls DPxUpdateRegCache()
    DPxSetPPxDefaultLedCurrents();
}


// Disable T-Scope subsystem, and return to normal video display.
// Contains an implicit call to DPxUpdateRegCache().
void DPxDisablePPxTScope()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
	// Put low current values to avoid any LED current problem
	DPxSetPPxLedCurrent(PPX_LED_CUR_RED_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_GRN_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_BLU_H, 1);
    DPxUpdateRegCache();
    DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) & ~PPXREG_TSCOPE_CSR_EN);
    // T-Scope requires different LED currents
    // Note that this calls DPxUpdateRegCache()
    DPxSetPPxDefaultLedCurrents();
}


// Returns non-zero if T-Scope subsystem is enabled
int DPxIsPPxTScopeEnabled()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_CSR) & PPXREG_TSCOPE_CSR_EN;
}


// Set the PROPixx RAM page containing the T-Scope "cover" image
void DPxSetPPxTScopeBuffBasePage(unsigned page)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_BUFF_BASEPAGE, page);
}


// Get the RAM page for the T-Scope "cover" image
unsigned DPxGetPPxTScopeBuffBasePage()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_BUFF_BASEPAGE);
}


// Set the number of images in the downloaded T-Scope movie
void DPxSetPPxTScopeBuffNPages(unsigned nPages)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_BUFF_NPAGES, nPages);
}


// Get the number of pages in the uploaded T-Scope movie
unsigned DPxGetPPxTScopeBuffNPages()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_BUFF_NPAGES);
}


// Download 1 or more T-Scope images to PROPixx RAM
void DPxWritePPxTScopePages(unsigned startPage, unsigned nPages, void* pageData)
{
    unsigned iPage;
    unsigned ramAddress;
    void *pageDataPtr;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

    // Here's the thing.  If we got to here, then we have definitely selected a PROPixx.
    // Unfortunately, calling DPxWriteRam() will itself call DPxSelectSysDevice(DPX_DEVSEL_ANY),
    // and that will probably select something other than the PROPixx.
    // We'll call a version which does not do a devsel.
    // We have to write one page at a time, because pageData contains packed data, whereas RAM stores successive pages at offsets of 256kB.
    for (iPage = 0; iPage < nPages; iPage++) {
        ramAddress = (startPage + iPage) * PPX_TSCOPE_PAGE_STEP_SIZE;
        pageDataPtr = ((char*)pageData) + iPage * PPX_TSCOPE_PAGE_DATA_SIZE;
        DPxWriteRamNoDevsel(ramAddress, PPX_TSCOPE_PAGE_DATA_SIZE, pageDataPtr);
    }
}


// Wait for current video frame to terminate, then load and show cover page of T-Scope movie
void DPxEnablePPxTScopePrepReq()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) | PPXREG_TSCOPE_CSR_PREP_REQ);    
}


// Terminate T-Scope movie on current page
void DPxDisablePPxTScopePrepReq() {
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) & ~PPXREG_TSCOPE_CSR_PREP_REQ);
}


// Returns non-zero if a T-Scope movie request is active
int DPxIsPPxTScopePrepReq()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_CSR) & PPXREG_TSCOPE_CSR_PREP_REQ;
}


// Goes non-zero when cover page has been presented, and T-Scope schedule can be started
int DPxIsPPxTScopePrepAck()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_CSR) & PPXREG_TSCOPE_CSR_PREP_ACK;
}


// Set nanosecond delay between schedule start and first TACH update
void DPxSetPPxTScopeSchedOnset(unsigned onset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	DPxSetReg32(PPXREG_TSCOPE_SCHED_ONSET_L, onset);
}


// Get nanosecond delay between schedule start and first TACH update
unsigned DPxGetPPxTScopeSchedOnset()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg32(PPXREG_TSCOPE_SCHED_ONSET_L);
}


// Set TACH schedule update rate and units.
// DMD limits to about 10kHz
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 10 kHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 10 kHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 100,000 ns
void DPxSetPPxTScopeSchedRate(unsigned rateValue, int rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	switch (rateUnits) {
		case DPXREG_SCHED_CTRL_RATE_HZ:
			if (rateValue > 10000) {
				DPxDebugPrint1("ERROR: DPxSetPPxTScopeSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_PPX_TSCOPE_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_XVID:
			if (rateValue > 10000/DPxGetVidVFreq()) {
				DPxDebugPrint1("ERROR: DPxSetPPxTScopeSchedRate() frequency too high %u\n", rateValue);
				DPxSetError(DPX_ERR_PPX_TSCOPE_SCHED_TOO_FAST);
				return;
			}
			break;
		case DPXREG_SCHED_CTRL_RATE_NANO:
			if (rateValue < 100000) {
				DPxDebugPrint1("ERROR: DPxSetPPxTScopeSchedRate() period too low %u\n", rateValue);
				DPxSetError(DPX_ERR_PPX_TSCOPE_SCHED_TOO_FAST);
				return;
			}
			break;
		default:
			DPxDebugPrint1("ERROR: DPxSetPPxTScopeSchedRate() unrecognized rateUnits %d\n", rateUnits);
			DPxSetError(DPX_ERR_PPX_TSCOPE_SCHED_BAD_RATE_UNITS);
			return;
	}
	DPxSetReg32(PPXREG_TSCOPE_SCHED_CTRL_L, (DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_RATE_MASK) | rateUnits);
	DPxSetReg32(PPXREG_TSCOPE_SCHED_RATE_L,  rateValue);
}


// Get TACH schedule update rate (and optionally get rate units)
unsigned DPxGetPPxTScopeSchedRate(int *rateUnits)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	if (rateUnits)
		*rateUnits = DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RATE_MASK;
	return DPxGetReg32(PPXREG_TSCOPE_SCHED_RATE_L);
}


// Set TACH schedule update count
void DPxSetPPxTScopeSchedCount(unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg32(PPXREG_TSCOPE_SCHED_COUNT_L,  count);
}


// Get TACH schedule update count
unsigned DPxGetPPxTScopeSchedCount()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg32(PPXREG_TSCOPE_SCHED_COUNT_L);
}


// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
void DPxEnablePPxTScopeSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg32(PPXREG_TSCOPE_SCHED_CTRL_L, DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) | DPXREG_SCHED_CTRL_COUNTDOWN);
}


// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
void DPxDisablePPxTScopeSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg32(PPXREG_TSCOPE_SCHED_CTRL_L, DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) & ~DPXREG_SCHED_CTRL_COUNTDOWN);
}


// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
int DPxIsPPxTScopeSchedCountdown()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_COUNTDOWN;
}


// Shortcut which assigns Onset/Rate/Count.
// If Count > 0, enables Countdown mode.
void DPxSetPPxTScopeSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetPPxTScopeSchedOnset(onset);
	DPxSetPPxTScopeSchedRate(rateValue, rateUnits);
	DPxSetPPxTScopeSchedCount(count);
	if (count)
		DPxEnablePPxTScopeSchedCountdown();
	else
		DPxDisablePPxTScopeSchedCountdown();
}


// Start running a TACH schedule
void DPxStartPPxTScopeSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_TACH)) |
                (DPXREG_SCHED_STARTSTOP_START << DPXREG_SCHED_STARTSTOP_SHIFT_TACH));
}


// Stop running a TACH schedule
void DPxStopPPxTScopeSched()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(DPXREG_SCHED_STARTSTOP, (DPxGetReg16(DPXREG_SCHED_STARTSTOP) & ~(DPXREG_SCHED_STARTSTOP_MASK << DPXREG_SCHED_STARTSTOP_SHIFT_TACH)) |
                (DPXREG_SCHED_STARTSTOP_STOP << DPXREG_SCHED_STARTSTOP_SHIFT_TACH));
}


// Returns non-0 if TACH schedule is currently running
int DPxIsPPxTScopeSchedRunning()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg32(PPXREG_TSCOPE_SCHED_CTRL_L) & DPXREG_SCHED_CTRL_RUNNING;
}


// Set start address of T-Scope microcode program.  T-Scope currently supports up to 1024 instructions
void DPxSetPPxTScopeProgAddr(unsigned progAddr)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_PROG_ADDR, progAddr);
}


// Get start address of T-Scope microcode program
unsigned DPxGetPPxTScopeProgAddr()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_PROG_ADDR);
}


// Set a constant value which is added to T-Scope microcode instruction pages
void DPxSetPPxTScopeProgOffsetPage(unsigned offset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_TSCOPE_PROG_OFFSET_PAGE, offset);
}


// Get the constant value which is added to T-Scope microcode instruction pages
unsigned DPxGetPPxTScopeProgOffsetPage()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_PROG_OFFSET_PAGE);
}


// Set the PROPixx T-Scope operating mode.
// mode is one of the following predefined constants:
//      PPXREG_TSCOPE_CTRL_MODE_BIN_LIST        : T-Scope presents a list of binary images stored in PROPixx DRAM
//      PPXREG_TSCOPE_CTRL_MODE_VID_SINGLE      : T-Scope presents a single video image pointed to by calling DPxSetPPxTScopeBuffBasePage()
//      PPXREG_TSCOPE_CTRL_MODE_VID_PROG        : T-Scope presents a sequence of video images, controlled by microcode uploaded to the PROPixx
void DPxSetPPxTScopeMode(int mode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_TSCOPE_CSR, (DPxGetReg16(PPXREG_TSCOPE_CSR) & ~PPXREG_TSCOPE_CTRL_MODE_MASK) | mode);    
}


// Get the PROPixx T-Scope operating mode
int DPxGetPPxTScopeMode(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return DPxGetReg16(PPXREG_TSCOPE_CSR & PPXREG_TSCOPE_CTRL_MODE_MASK);
}


// Only one quadrant of an uploaded image is presented at a time
void DPxEnablePPxTScopeQuad()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) | PPXREG_TSCOPE_CSR_QUAD);
}


// The entire uploaded image is presented
void DPxDisablePPxTScopeQuad()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_TSCOPE_CSR, DPxGetReg16(PPXREG_TSCOPE_CSR) & ~PPXREG_TSCOPE_CSR_QUAD);
}


// Returns non-0 if only one quadrant of an uploaded image is presented at a time
int DPxIsPPxTScopeQuad()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_TSCOPE_CSR) & PPXREG_TSCOPE_CSR_QUAD;
}


// Pass nCmds*2 (max 1024x2) 16-bit microcode words, in order page0,nFrames0,page1,nFrames1...
// The microcode is executed until it reaches an instruction with nFrames=0.
void DPxSetPPxTScopeProg(UInt16* progData, int nCmds)
{
    int payloadLength;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	payloadLength = nCmds * 4;
	ep2out_Tram[0] = '^';
	ep2out_Tram[1] = EP2OUT_WRITEPPXPGM;
	ep2out_Tram[2] = LSB(payloadLength);
	ep2out_Tram[3] = MSB(payloadLength);
	memcpy(ep2out_Tram+4, progData, payloadLength);
    if (EZWriteEP2Tram(ep2out_Tram, 0, 0)) {
        printf("***ERROR: DPxSetPPxTScopeProg() call to EZWriteEP2Tram() failed\n");
    }
}


// PROPixx Gaze-Contingent Display is shifted horizontally and vertically
void DPxEnablePPxGcdShift()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) | PPXREG_VID_CTRL2_GCD_SHIFT_EN);
}


// PROPixx display is not shifted
void DPxDisablePPxGcdShift()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) & ~PPXREG_VID_CTRL2_GCD_SHIFT_EN);
}


// Returns non-0 if PROPixx Gaze-Contingent Display is shifted
int DPxIsPPxGcdShift()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_CTRL2) & PPXREG_VID_CTRL2_GCD_SHIFT_EN;
}


// PROPixx Gaze-Contingent Display can modify image shift on every DLP sub-frame
void DPxEnablePPxGcdShiftSubframe()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) | PPXREG_VID_CTRL2_GCD_SHIFT_SUBFRAME);
}


// PROPixx Gaze-Contingent Display only modifies shift once per video frame
void DPxDisablePPxGcdShiftSubframe()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) & ~PPXREG_VID_CTRL2_GCD_SHIFT_SUBFRAME);
}


// Returns non-0 if PROPixx Gaze-Contingent Display can modify image shift on every DLP sub-frame
int DPxIsPPxGcdShiftSubframe()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_CTRL2) & PPXREG_VID_CTRL2_GCD_SHIFT_SUBFRAME;
}


// PROPixx Gaze-Contingent Display shift is controlled by hardware
void DPxEnablePPxGcdShiftHardware()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) | PPXREG_VID_CTRL2_GCD_SHIFT_HARDWARE);
}


// PROPixx Gaze-Contingent Display shift is controlled by software registers
void DPxDisablePPxGcdShiftHardware()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_CTRL2, DPxGetReg16(PPXREG_VID_CTRL2) & ~PPXREG_VID_CTRL2_GCD_SHIFT_HARDWARE);
}


// Returns non-0 if PROPixx Gaze-Contingent Display shift is controlled by hardware
int DPxIsPPxGcdShiftHardware()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
	return DPxGetReg16(PPXREG_VID_CTRL2) & PPXREG_VID_CTRL2_GCD_SHIFT_HARDWARE;
}


// Set the PROPixx Gaze-Contingent Display horizontal/vertical image shift
void DPxSetPPxGcdShift(int xShift, int yShift)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_GCD_SHIFTX, xShift);
	DPxSetReg16(PPXREG_GCD_SHIFTY, yShift);
}


// Get the PROPixx Gaze-Contingent Display horizontal/vertical image shift
void DPxGetPPxGcdShift(int* xShift, int* yShift)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    *xShift = (short)DPxGetReg16(PPXREG_GCD_SHIFTX);    // Need "short" cast so negative numbers get sign extended
    *yShift = (short)DPxGetReg16(PPXREG_GCD_SHIFTY);
}


// PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display shifting
void DPxEnableGcdShiftHardwareBridge()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) | DPXREG_VID_CTRL2_TX_DVI_TPX_DATA);
}


// // PROPixx Controller does not send hardware data to PROPixx for Gaze-Contingent Display shifting
void DPxDisableGcdShiftHardwareBridge()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) & ~DPXREG_VID_CTRL2_TX_DVI_TPX_DATA);
}


// Returns non-0 if PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display shifting
int DPxIsGcdShiftHardwareBridge()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return 0;
    
	return DPxGetReg16(DPXREG_VID_CTRL2) & DPXREG_VID_CTRL2_TX_DVI_TPX_DATA;
}


// Specify the way that the PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display X/Y shifting
// mode is one of the following predefined constants:
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_NULL      : X/Y are forced to 0
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_ADC       : X/Y are signed 16-bit numbers coming from ADC0/1
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_DIN       : X/Y are signed 12-bit numbers coming from DIN 11:0/23:12
void DPxSetGcdShiftHardwareMode(int mode)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
	DPxSetReg16(DPXREG_CTRL, (DPxGetReg16(DPXREG_CTRL) & ~DPXREG_CTRL_GCD_SHIFT_HW_MODE_MASK) | mode);    
}


// Returns the method that the PROPixx Controller bridges hardware inputs for Gaze-Contingent Display X/Y shifting
int DPxGetGcdShiftHardwareMode(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return 0;
    
    return DPxGetReg16(DPXREG_CTRL & DPXREG_CTRL_GCD_SHIFT_HW_MODE_MASK);
}


// PROPixx Controller implements a gain and an offset when mapping hardware inputs to x/y pixel shifts.
// The Gain terms indicate how many pixels should be shifted for a full-scale hardware input, divided by 4.
// For example, the default value of 512 means that a full-scale input from the ADC (+10V) will shift the image 2048 pixels to the right.
// The Offset terms are in straight pixels.  For example, assigning a value of 10 would cause the above +10V ADC to shift the image by 2058 instead of 1048 pixels.
// Note that these gains and offsets are signed 16-bit values.
void DPxSetGcdShiftHardwareTransform(int xGain, int xOffset, int yGain, int yOffset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;

	DPxSetReg16(DPXREG_GCD_SHIFT_HW_X_GAIN, xGain);
	DPxSetReg16(DPXREG_GCD_SHIFT_HW_X_OFFSET, xOffset);
	DPxSetReg16(DPXREG_GCD_SHIFT_HW_Y_GAIN, yGain);
	DPxSetReg16(DPXREG_GCD_SHIFT_HW_Y_OFFSET, yOffset);
}


void DPxGetGcdShiftHardwareTransform(int* xGain, int* xOffset, int* yGain, int* yOffset)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
    *xGain = (short)DPxGetReg16(DPXREG_GCD_SHIFT_HW_X_GAIN);    // Need "short" cast so negative numbers get sign extended
    *xOffset = (short)DPxGetReg16(DPXREG_GCD_SHIFT_HW_X_OFFSET);
    *yGain = (short)DPxGetReg16(DPXREG_GCD_SHIFT_HW_Y_GAIN);
    *yOffset = (short)DPxGetReg16(DPXREG_GCD_SHIFT_HW_Y_OFFSET);
}


//zzz


void DPxEnableTxDviPassthru()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) & ~DPXREG_VID_CTRL3_TX_DVI_NO_PASSTHRU);
}

void DPxDisableTxDviPassthru()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL3, DPxGetReg16(DPXREG_VID_CTRL3) | DPXREG_VID_CTRL3_TX_DVI_NO_PASSTHRU);
}

int DPxIsTxDviPassthru()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPC))
        return 0;
    
    return DPxGetReg16(DPXREG_VID_CTRL3) & ~DPXREG_VID_CTRL3_REAR_PROJECTION;
}


// DATAPixx2 Rev >= 47 only
void DPxEnableRtcOnVideo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
        return;
    
	DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) & ~DPXREG_VID_CTRL2_TX_DVI_TPX_DATA);
}

// DATAPixx2 Rev >= 47 only
void DPxDisableRtcOnVideo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
        return;
    
    DPxSetReg16(DPXREG_VID_CTRL2, DPxGetReg16(DPXREG_VID_CTRL2) | DPXREG_VID_CTRL2_TX_DVI_TPX_DATA);
}

// DATAPixx2 Rev >= 47 only
int DPxIsRtcOnVideo()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_DP2))
        return 0;
    
    return DPxGetReg16(DPXREG_VID_CTRL2) & ~DPXREG_VID_CTRL2_TX_DVI_TPX_DATA;
}



// Set PROPixx DLP Sequencer program.
// Contains an implicit call to DPxUpdateRegCache().
void DPxSetPPxDlpSeqPgrm(int program)
{    
	UInt16 bitWeightCalGetValue[36];

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	// Put low current values to avoid any LED current problem
	DPxSetPPxLedCurrent(PPX_LED_CUR_RED_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_GRN_H, 1);
    DPxSetPPxLedCurrent(PPX_LED_CUR_BLU_H, 1);
    DPxUpdateRegCache();
	
	// Sequencer RGB_HBD can't be used if the PPX does not has the LED Driver Board rev 2
	// It can't be selected too if there is no calibration done yet.
	if (program == PPXREG_VID_SEQ_CSR_PGRM_RGB_CAL_HBD) {		
		
		if (MSB(DPxGetReg16(PPXREG_PART_NUM2)) == 0x22) {
			DPxSetError(DPX_ERR_PPX_UNSUPPORTED_DLP_SEQ_PGRM);
			return;
		}

		DPxSpiRead(SPI_ADDR_PPX_BWCAL, sizeof(bitWeightCalGetValue), (char*)bitWeightCalGetValue, NULL);

		// No correction found.
		if (bitWeightCalGetValue[0] == 0xFFFF) {
			DPxSetError(DPX_ERR_PPX_NO_HIGH_BIT_DEPTH_CAL_FOUND);
			return;
		}		
	}	
	
    DPxSetReg16(PPXREG_VID_SEQ_CSR, (DPxGetReg16(PPXREG_VID_SEQ_CSR) & ~PPXREG_VID_SEQ_CSR_PGRM_MASK) | program);
	
	// Write the new sequencer immediately, to load the right LED currents.
    DPxUpdateRegCache();

    // Some sequencers require different LED currents
    // Note that this calls DPxUpdateRegCache()
    DPxSetPPxDefaultLedCurrents();
}

// Get PROPixx DLP Sequencer program
int DPxGetPPxDlpSeqPgrm()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return (DPxGetReg16(PPXREG_VID_SEQ_CSR) & PPXREG_VID_SEQ_CSR_PGRM_CURRENT_MASK) >> 4;
}


// Set LED currents depending on command sequence or T-Scope.
// There's a subtlety here.
// If we need to return to calibrated white point, that will require a SPI read, which will do a DPxUpdateRegCache().
// This will have a side effect of implementing any cached register writes, including sequencer changes.
// We'll make sure everything gets implemented simultaneously by always doing an DPxUpdateRegCache() here.

void DPxSetPPxDefaultLedCurrents()
{

	unsigned char dataBuff[6];
    int redDac, grnDac, bluDac;
	int iRgb, greySeq;
    double redCur, grnCur, bluCur;
	double maxLedCur[3];
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	greySeq = 0;
	// Get the calibrated D65 white point for High Bit Depth sequencer
	if (DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_RGB_CAL_HBD) {
		
		if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXD65_CAL_HBD, 6, (char*)dataBuff, NULL)) {
			fprintf(stderr, "ERROR: Could not read SPI\n");
			return;
		}
	}
	// Get the calibrated D65 white point for grey sequencer
	else if (DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_RB3D     || DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_GREY_3X ||
			 DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_GREY_720 || DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_QUAD12X || 
			(DPxIsPPxTScopeEnabled() && (DPxGetReg16(0x12E) & 6) != 2 && (DPxGetReg16(0x12E) & 6) != 4)) { 
		
		greySeq = 1;
		if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXD65_GREY0, 6, (char*)dataBuff, NULL)) {
			fprintf(stderr, "ERROR: Could not read SPI\n");
			return;
		}
	}
	// Get the calibrated D65 white point for RGB sequencer
	else {		
		if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXD65, 6, (char*)dataBuff, NULL)) {
			fprintf(stderr, "ERROR: Could not read SPI\n");
			return;
		}
    }

	redDac = (dataBuff[0] << 8) + dataBuff[1];
	grnDac = (dataBuff[2] << 8) + dataBuff[3];
	bluDac = (dataBuff[4] << 8) + dataBuff[5];
	redCur = (double)redDac / PPX_LEDCUR_TO_VDAC;
	grnCur = (double)grnDac / PPX_LEDCUR_TO_VDAC;
	bluCur = (double)bluDac / PPX_LEDCUR_TO_VDAC;

	// Special conditions override calibrated white point.
    // Basically these are situations (greyMode) in which all 3 LEDs are on simultaneously,
    // and the power supplies could never handle that.
	if (greySeq && (redCur > 18 || grnCur > 18 || bluCur > 18)) {

		// Get the maximum permissable LED currents
		if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXCUR, 6, (char*)dataBuff, NULL)) {
			printf("ERROR: Could not read LEDMAXCUR from SPI\n");
			return;
		}

		// if grey mode, maximum current is 18A. 45% of 40A (a real 18A)
		for (iRgb = 0; iRgb < 3; iRgb++) {
			maxLedCur[iRgb] = ((dataBuff[iRgb*2] << 8) + dataBuff[iRgb*2+1]) * 0.45; 
		}

		// 45% of "real" 40A = 18A
		redCur = maxLedCur[0]/PPX_LEDCUR_TO_VDAC;
		grnCur = maxLedCur[1]/PPX_LEDCUR_TO_VDAC;
		bluCur = maxLedCur[2]/PPX_LEDCUR_TO_VDAC;

	}
	else {
		// LLAPI just ignores writes if current is over 50A.
		if (redCur > 50) redCur = 50;
		if (grnCur > 50) grnCur = 50;
		if (bluCur > 50) bluCur = 50;
	}

    // Set Current
    DPxSetPPxLedCurrent(PPX_LED_CUR_RED_H, redCur);
    DPxSetPPxLedCurrent(PPX_LED_CUR_GRN_H, grnCur);
    DPxSetPPxLedCurrent(PPX_LED_CUR_BLU_H, bluCur);

    // Write the new currents immediately, to align with the current dlp sequencer which was just written. 
    DPxUpdateRegCache();
}


void DPxSetPPxGreyLedCurrents(int index)
{
	unsigned char dataBuff[6];
    int redDac, grnDac, bluDac;
    double redCur, grnCur, bluCur;
    
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
	
	// All 3 LEDs are on simultaneously,
	if (DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_RB3D     || DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_GREY_3X ||
		DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_GREY_720 || DPxGetPPxDlpSeqPgrm() == PPXREG_VID_SEQ_CSR_PGRM_QUAD12X || 
		DPxIsPPxTScopeEnabled()) {
	
		if (index == 0) {
			// Get the calibrated D65 white point
			if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXD65_GREY0, 6, (char*)dataBuff, NULL)) {
				fprintf(stderr, "ERROR: Could not read SPI\n");
				return;
			}
		}
		else {		
			if (DPxSpiRead(SPI_ADDR_PPX_LEDMAXD65_GREY1, 6, (char*)dataBuff, NULL)) {
				fprintf(stderr, "ERROR: Could not read SPI\n");
				return;
			}
		}
	}

	redDac = (dataBuff[0] << 8) + dataBuff[1];
	grnDac = (dataBuff[2] << 8) + dataBuff[3];
	bluDac = (dataBuff[4] << 8) + dataBuff[5];

	redCur = (double)redDac / PPX_LEDCUR_TO_VDAC;
	grnCur = (double)grnDac / PPX_LEDCUR_TO_VDAC;
	bluCur = (double)bluDac / PPX_LEDCUR_TO_VDAC;

    // LLAPI just ignores writes if current is over 18A.
    if (redCur > 18) redCur = 18;
    if (grnCur > 18) grnCur = 18;
    if (bluCur > 18) bluCur = 18;

    // Implement LED currents
    DPxSetPPxLedCurrent(PPX_LED_CUR_RED_H, redCur);
    DPxSetPPxLedCurrent(PPX_LED_CUR_GRN_H, grnCur);
    DPxSetPPxLedCurrent(PPX_LED_CUR_BLU_H, bluCur);

    // Write the new currents immediately, to align with the current dlp sequencer which was just written. 
    DPxUpdateRegCache();
}

void DPxSetPPxLedMask(int mask)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	if (mask < 0 || mask > 7) {
		DPxDebugPrint1("ERROR: DPxSetPPxLedMask() argument value %d is out of range (0-7)\n", mask);
		DPxSetError(DPX_ERR_VID_LED_MASK_ARG_ERROR);
		return;
	}
    
	DPxSetReg16(PPXREG_VID_LED_MASK, (DPxGetReg16(PPXREG_VID_LED_MASK) & ~PPXREG_VID_RGB_LED_MASK) | mask << 4);
}


int DPxGetPPxLedMask()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return (DPxGetReg16(PPXREG_VID_LED_MASK) & PPXREG_VID_RGB_LED_MASK) >> 4;
}


void DPxSetCustomStartupConfig()
{
	unsigned short DefRegVal[DPX_REG_SPACE_MAX/2];
    int spiAddr;
	unsigned i;

    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return;

	DPxUpdateRegCache();				// Get a VPixx Device register snapshot

	if (DPxGetError() != DPX_SUCCESS) {
		DPxDebugPrint1("Fail: RegisterRead() call to DPxReadRegsToCacheUSB() failed with error %d\n", DPxGetError());
	}

	for (i = 0; i < DPxGetRegisterSpaceSizeNoDevsel()/2; i++ ) {
		DefRegVal[i] = (unsigned short)DPxGetReg16(i*2);
    }

	spiAddr = SPI_ADDR_VPX_REGDEF;
	if (DPxIsA10Arch())
		spiAddr = SPI_ADDR_DP3_REGDEF;

    DPxSpiErase(spiAddr, DPxGetRegisterSpaceSizeNoDevsel(), NULL);
    DPxSpiWrite(spiAddr, DPxGetRegisterSpaceSizeNoDevsel(), (char*)DefRegVal, NULL);
}

int DPxIsCustomStartupConfig()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_ANY))
        return 0;

	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_CUSTOM_REG_VAL;
}

void DPxSetFactoryStartupConfig()
{
    int spiAddr;
	
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return;

	spiAddr = SPI_ADDR_VPX_REGDEF;

	if (DPxIsA10Arch())
		spiAddr = SPI_ADDR_DP3_REGDEF;

    DPxSpiErase(spiAddr, DPxGetRegisterSpaceSizeNoDevsel(), NULL);
}

int DPxIsCustomEdid()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;

	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_USER_EDID;
}

int DPxIsNvidia3dVisionReady()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX_PPC_VPX_DPX))
        return 0;

	return DPxGetReg16(DPXREG_STATUS) & DPXREG_STATUS_NV3D_READY_EDID;
}

// Set 36 16-bit bit weight calibration values.
void DPxSetPPxBitWeightCalibration(UInt16* bitWeightCalData)
{
	UInt16 bitWeightCalValue[36];
	int payloadLength = 72;

    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

	memcpy(bitWeightCalValue, bitWeightCalData, payloadLength);

	DPxSpiErase(SPI_ADDR_PPX_BWCAL, payloadLength, NULL);
    DPxSpiWrite(SPI_ADDR_PPX_BWCAL, payloadLength, (char*)bitWeightCalValue, NULL);
	
	DPxSetReg16(PPXREG_DDC_STATUS, PPXREG_DDC_STATUS_RELOAD_BWC);
	DPxUpdateRegCache(); 
}

void DPxClearPPxBitWeightCalibration()
{
	if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;

    DPxSpiErase(SPI_ADDR_PPX_BWCAL, 36 * 2, NULL);
	DPxSetReg16(PPXREG_DDC_STATUS, PPXREG_DDC_STATUS_RELOAD_BWC);
	DPxUpdateRegCache(); 
}

void DPxEnableTPBridgeLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_TPB))
        return;
    
	DPxSetReg16(0x30, 1);
}


void DPxDisableTPBridgeLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_TPB))
        return;
    
	DPxSetReg16(0x30, 0);
}

// Returns non-0 if TP Bridge internal Loopback is enabled
int DPxIsTPBridgeLoopback()
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_TPB))
        return 0;
    
	return (int) DPxGetReg16(0x30) & 1;
}


int DPxGetNbrDevices(void)
{
	int i=0, device_count=0;
	for(i=0; i< DPX_DEVSEL_CNT_SIZE; i++) { device_count += devselCnt[i]; }

	return device_count;
}

void FX3TransferEP0(unsigned char type, unsigned char opcode, unsigned long addr, int len, unsigned char *buf)
{
    // Caller is responsible for ensuring valid dpxSysDevsel
#ifdef USE_LIB01
		usb_control_msg(dpxDeviceTable[dpxSysDevsel].dpxHdl, type, opcode, addr & 0xffff, addr >> 16, (char*)buf, len, 1000);
#else
		libusb_control_transfer(dpxDeviceTable[dpxSysDevsel].dpxHdl, type, opcode, addr & 0xffff, addr >> 16, (unsigned char*)buf, len, 1000);
#endif

}

// Returns 1 if 3D is enabled for Quad4X
int DPxPPxIsQuad4x3D(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return 0;
    
    return (DPxGetReg16(PPXREG_VID_QUAD_3D) & 0x01);
}

// Enables the 3D driving of the polarizer in Quad4X
void DPxPPxEnableQuad4x3D(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
	DPxSetReg16(PPXREG_VID_QUAD_3D, DPxGetReg16(PPXREG_VID_QUAD_3D) | 0x01);
}

// Disables the 3D driving of the polarizer in Quad4X
void DPxPPxDisableQuad4x3D(void)
{
    if (!DPxSelectSysDevice(DPX_DEVSEL_PPX))
        return;
    
    DPxSetReg16(PPXREG_VID_QUAD_3D, DPxGetReg16(PPXREG_VID_QUAD_3D) & 0xFE);
}
