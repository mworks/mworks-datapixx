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

//	-API functions take "int" arguments and return "int" values whenever the value fits in a SInt32.
//	 UInt32 data does not fit in a SInt32, so it is passed as type "unsigned".
//	 This strategy minimizes the number of type casts required by API users.
//	-This API should work OK with both 32-bit and 64-bit integers.
//	-Unless otherwise specified, the reset value for all DPxSet*() parameters is 0, and all "DPxEnable*()" are disabled.
//	-This API assumes it is being compiled for a LITTLE-ENDIAN architecture; eg: Intel Mac or PC.
//	 This assumption comes into play when reading and writing little-endian USB messages.
//

#ifndef __LIBDPX_H__
#define __LIBDPX_H__

#ifdef __cplusplus
extern "C" {
#endif

// USE_LIB10 is conditional on FACTORY_10 for now
#ifndef FACTORY_10
#define USE_LIB01
#endif

#ifdef USE_LIB01
#include "usb.h"                        // Don't want this in libdpx_i.h, or firmware compiles fail
#else
#include "libusb.h"
struct usb_dev_handle;
typedef struct usb_dev_handle usb_dev_handle;
#endif

#include "libdpx_i.h"					// internal API includes field definitions; eg: DPXREG_SCHED_CTRL_RATE_HZ

#ifdef MAKE_DLL
#define EX_DLL __declspec(dllexport)	// Used to compile DLL for Python
#else
#define EX_DLL
#endif

typedef struct TestStruct {
    unsigned char  array[1080][1920][3];
} TestStruct;


#define LEFT_X_VECTOR_LOW	8
#define LEFT_X_VECTOR_HIGH	9
#define LEFT_Y_VECTOR_LOW	10
#define LEFT_Y_VECTOR_HIGH	11
#define LEFT_A_SIZE_LOW		12
#define LEFT_A_SIZE_HIGH	13	
#define LEFT_B_SIZE_LOW		14
#define LEFT_B_SIZE_HIGH	15	
#define LEFT_ANGLE_LOW		16
#define LEFT_ANGLE_HIGH		17
#define RIGHT_X_VECTOR_LOW	18
#define RIGHT_X_VECTOR_HIGH	19
#define RIGHT_Y_VECTOR_LOW	20
#define RIGHT_Y_VECTOR_HIGH	21
#define RIGHT_A_SIZE_LOW	22
#define RIGHT_A_SIZE_HIGH	23
#define RIGHT_B_SIZE_LOW	24
#define RIGHT_B_SIZE_HIGH	25
#define RIGHT_ANGLE_LOW		26
#define RIGHT_ANGLE_HIGH	27
#define DIN_VALUE_LOW		28
#define DIN_VALUE_HIGH      29
#define EYE_BLINK			30



#define INDEX_INCEMENT 32 // 8 bytes for timetag, 20 bytes for the data


// TRACKPIXX Functions
EX_DLL void TPxGetEyePositionDuringCalib(float screen_x, float screen_y);
EX_DLL void TPxFinishCalibration(void);
EX_DLL void TPxFinishCalibration_new(void);
EX_DLL void TPxFinishCalibration_newPoly(void);
//EX_DLL void TPxFinishCalibration_OLD(void);
EX_DLL void TPxGetEyePosition(float* eyeReturn);
EX_DLL void TPxGetEyePosition_new(float* eyeReturn);
EX_DLL void TPxGetEyePosition_newPoly(float* eyeReturn);
EX_DLL void TPxGetEyePosition_newPoly_valid(float vx[2], float vy[2], float vcoeff_x_eye1[5], float vcoeff_y_eye1[7], float vcoeff_x_eye2[5],  float vcoeff_y_eye2[7], float* eyeReturn);

EX_DLL int TPxIsDeviceCalibrated(void);
EX_DLL float TPxFilterFunctionLevelOne(float* x2, float* x1, float* x);
EX_DLL void TPxTestlibtpxFunctions();

EX_DLL void TPxBestPolyFinishCalibration(void);
EX_DLL void TPxBestPolyGetEyePosition(float* eyeReturn);

EX_DLL void		TPxSetBuffBaseAddr(unsigned buffBaseAddr);				
EX_DLL unsigned	TPxGetBuffBaseAddr(void);								
EX_DLL void		TPxSetBuffWriteAddr(unsigned buffWriteAddr);				
EX_DLL unsigned	TPxGetBuffWriteAddr(void);								
EX_DLL void		TPxSetBuffSize(unsigned buffSize);						
EX_DLL unsigned	TPxGetBuffSize(void);									
EX_DLL void		TPxSetBuff(unsigned buffAddr, unsigned buffSize);		

EX_DLL void		TPxEnableLogTimetags(void);								
EX_DLL void		TPxDisableLogTimetags(void);								
EX_DLL int		TPxIsLogTimetags(void);	
EX_DLL void		TPxEnableFreeRun(void);			
EX_DLL void		TPxDisableFreeRun(void);								
EX_DLL int		TPxIsFreeRun(void);	

EX_DLL void		TPxSetWantedData(void);
EX_DLL int		DisableTpxAcquisition(int enable);//debug

EX_DLL unsigned TPxSaveToCsv(unsigned address, const char* fileName);


EX_DLL void PPxDownloadTestPattern(TestStruct* testPattern, int loadAddr, int page);
EX_DLL unsigned char* TPxGetImagePtr(void);
EX_DLL unsigned char* createImageChar(void);



//	Setup functions
EX_DLL int      DPxDetectDevice(int devsel);    // Returns non-zero if a given VPixx hardware device exists in the system.  "devsel" takes on one of the following values:
                                                //  DPX_DEVSEL_DPX      -DATAPixx
                                                //  DPX_DEVSEL_VPX      -VIEWPixx
                                                //  DPX_DEVSEL_PPC      -PROPixx Controller
                                                //  DPX_DEVSEL_PPX      -PROPixx
                                                //  DPX_DEVSEL_TPC      -TRACKPixx Controller
                                                //  DPX_DEVSEL_TPX      -TRACKPixx
												//  DPX_DEVSEL_TPB      -TRACKPixx Bridge
EX_DLL int		DPxSelectDevice(int devsel);    // Select which VPixx device to access.  "devsel" takes on one of the above values.  Returns non-zero if device found
                                                //  DPX_DEVSEL_AUTO     -Let library dynamically choose target device for each function call (default value)
                                                //  DPX_DEVSEL_DPX      -First DATAPixx found in system
                                                //  DPX_DEVSEL_VPX      -First VIEWPixx found in system
                                                //  DPX_DEVSEL_PPC      -First PROPixx Controller found in system
                                                //  DPX_DEVSEL_PPX      -First PROPixx found in system
                                                //  DPX_DEVSEL_TPC      -First TRACKPixx Controller found in system
EX_DLL int		DPxSelectDeviceSubName(int devtype, char* devname); // Select which VPixx device to access.  "devtype" takes on one of the above values. "devname" must me a valid custom device name. Returns non-zero if device found.

EX_DLL void		DPxOpen(void);					// Call before any VPixx devices
EX_DLL void		DPxClose(void);					// Close all VPixx devices
EX_DLL int		DPxIsReady(void);				// Returns non-0 if a VPixx device has been successfully opened

//	Functions for reading and writing DATAPixx RAM
EX_DLL void		DPxReadRam(unsigned address, unsigned length, void* buffer);	// Read a block of DATAPixx RAM into a local buffer
EX_DLL void		DPxWriteRam(unsigned address, unsigned length, void* buffer);	// Write a local buffer to DATAPixx RAM
EX_DLL void*	DPxGetReadRamBuffAddr(void);									// Address of API internal read RAM buffer
EX_DLL int		DPxGetReadRamBuffSize(void);									// Number of bytes in internal read RAM buffer
EX_DLL void*	DPxGetWriteRamBuffAddr(void);									// Address of API internal write RAM buffer
EX_DLL int		DPxGetWriteRamBuffSize(void);									// Number of bytes in internal write RAM buffer

//	The DPxSet*() DPxEnable*(), and DPxDisable*() functions write new register values to a local cache, then flag these registers as "modified".
//	DPxWriteRegCache() downloads modified registers in the local cache back to the DATAPixx.
//	Averages about 125 microseconds (probably one 125us USB microframe) on a Mac Pro.
//	DPxUpdateRegCache() does a DPxWriteRegCache(),
//	then uploads a snapshot of the DATAPixx register map back into the local register cache.
//	Once in the local cache, DPxGet*() and DPxIs*() functions can quickly examine the cached values.
//	This averages about 250 microseconds (probably 2 x 125us USB microframes) on a Mac Pro.
//	The AfterVideoSync() versions do the same thing, except that the Datapixx waits until
//	the leading edge of the next vertical sync pulse before treating the command.
//	The AfterPixelSync() works similarly, except that it waits for a specific pixel sequence.
EX_DLL void		DPxWriteRegCache(void);					// Write local register cache to Datapixx over USB
EX_DLL void		DPxUpdateRegCache(void);				// DPxWriteRegCache, then readback registers over USB into local cache
EX_DLL void		DPxWriteRegCacheAfterVideoSync(void);	// Like DPxWriteRegCache, but Datapixx only writes the registers on leading edge of next vertical sync pulse
EX_DLL void		DPxUpdateRegCacheAfterVideoSync(void);	// Like DPxUpdateRegCache, but Datapixx only writes the registers on leading edge of next vertical sync pulse
EX_DLL void		DPxWriteRegCacheAfterPixelSync(int nPixels, unsigned char* pixelData, int timeout);		// Like DPxWriteRegCache, but waits for a pixel sync sequence.
																										// pixelData is passed in order R0,G0,B0,R1,G1,B1...
																										// A pixel sync sequence may contain a maximum of 8 pixels.
																										// Timeout is in video frames.
EX_DLL void		DPxUpdateRegCacheAfterPixelSync(int nPixels, unsigned char* pixelData, int timeout);	// Like DPxUpdateRegCache, but waits for a pixel sync sequence

//	API to read and write individual fields within the local register cache.
//	First set of registers is global system information.
EX_DLL int		DPxGetID(void);							// Get the DATAPixx identifier code
EX_DLL int		DPxIsDatapixx(void);					// Returns non-0 for the original DATAPixx CRT driver
EX_DLL int		DPxIsDatapixx2(void);					// Returns non-0 for DATAPixx2 digital video driver
EX_DLL int		DPxIsDatapixx3(void);					// Returns non-0 for DATAPixx3 I/O hub
EX_DLL int		DPxIsViewpixx(void);					// Returns non-0 if DATAPixx is embedded in a VIEWPixx OR a VIEWPixx3D OR a VIEWPixxEEG
EX_DLL int		DPxIsViewpixx3D(void);					// Returns non-0 if DATAPixx is embedded in a VIEWPixx3D
EX_DLL int		DPxIsViewpixxEeg(void);					// Returns non-0 if DATAPixx is embedded in a VIEWPixxEEG
EX_DLL int		DPxIsPropixxCtrl(void);                 // Returns non-0 if DATAPixx is embedded in a PROPixx controller
EX_DLL int		DPxIsPropixx(void);                     // Returns non-0 for a PROPixx projector (not controller)
EX_DLL int		DPxIsTrackpixxCtrl(void);				// Returns non-0 for a TRACKPixx Controller
EX_DLL int		DPxIsTrackpixxBridge(void);				// Returns non-0 for a TRACKPixx Bridge
EX_DLL int		DPxIsTrackpixx(void);					// Returns non-0 for a TRACKPixx eye tracker
EX_DLL unsigned DPxGetRamSize(void);					// Get the number of bytes of RAM in the VPixx device
EX_DLL unsigned DPxGetRegisterSpaceSize(void);			// Get the number of bytes of register space in the VPixx system
EX_DLL int		DPxGetPartNumber(void);					// Get the integer part number of this VPixx Device
EX_DLL int		DPxGetFirmwareRev(void);				// Get the VPixx Device firmware revision
EX_DLL double	DPxGetSupplyVoltage(void);				// Get voltage being supplied from +5V supply
EX_DLL double	DPxGetSupplyCurrent(void);				// Get current being supplied from +5V supply
EX_DLL double	DPxGetSupply2Voltage(void);				// Get voltage being supplied from +12V supply (+24V for the VIEWPixxEEG)
EX_DLL double	DPxGetSupply2Current(void);				// Get current being supplied from +12V supply (+24V for the VIEWPixxEEG)
EX_DLL int		DPxIs5VFault(void);						// Returns non-0 if VESA and Analog I/O +5V pins are trying to draw more than 500 mA
EX_DLL int		DPxIsPsyncTimeout(void);				// Returns non-0 if last pixel sync wait timed out
EX_DLL int		DPxIsRamOffline(void);					// Returns non-0 if DDR SDRAM controller has not yet brought memory system online
EX_DLL double	DPxGetTempCelcius(void);				// Get temperature inside of DATAPixx chassis, in degrees Celcius
EX_DLL double	DPxGetTemp2Celcius(void);				// Get temperature2 inside of VIEWPixx chassis, in degrees Celcius
EX_DLL double	DPxGetTemp3Celcius(void);				// Get temperature of VIEWPixx FPGA die, in degrees Celcius
EX_DLL double	DPxGetTempFarenheit(void);				// Get temperature inside of DATAPixx chassis, in degrees Farenheit
EX_DLL double	DPxGetTime(void);						// Get double precision seconds since powerup
EX_DLL void		DPxSetMarker(void);						// Latch the current time value into the marker register
EX_DLL double	DPxGetMarker(void);						// Get double precision seconds when DPxSetMarker() was last called
EX_DLL void		DPxGetNanoTime(unsigned *nanoHigh32, unsigned *nanoLow32); // Get high/low UInt32 nanoseconds since powerup
EX_DLL void		DPxGetNanoMarker(unsigned *nanoHigh32, unsigned *nanoLow32); // Get high/low UInt32 nanosecond marker


// API routines dedicated to PROPixx
EX_DLL void		DPxSetPPx3dCrosstalk(double crosstalk);				// Set 3D crosstalk (0-1) which should be subtracted from stereoscopic stimuli
EX_DLL double	DPxGetPPx3dCrosstalk(void);							// Get 3D crosstalk (0-1) which is being subtracted from stereoscopic stimuli
EX_DLL void		DPxSetPPx3dCrosstalkLR(double crosstalk);			// Set 3D crosstalk (0-1) of left-eye image which should be subtracted from stereoscopic right-eye image
EX_DLL double	DPxGetPPx3dCrosstalkLR(void);						// Get 3D crosstalk (0-1) of left-eye image which is being subtracted from stereoscopic right-eye image
EX_DLL void		DPxSetPPx3dCrosstalkRL(double crosstalk);			// Set 3D crosstalk (0-1) of right-eye image which should be subtracted from stereoscopic left-eye image
EX_DLL double	DPxGetPPx3dCrosstalkRL(void);						// Get 3D crosstalk (0-1) of right-eye image which is being subtracted from stereoscopic left-eye image

EX_DLL void		DPxEnablePPxCeilingMount(void);						// PROPixx flips image horizontally and vertically
EX_DLL void		DPxDisablePPxCeilingMount(void);					// PROPixx does not flip image horizontally and vertically
EX_DLL int		DPxIsPPxCeilingMount(void);							// Returns non-0 if PROPixx is flipping image horizontally and vertically
EX_DLL void		DPxEnablePPxRearProjection(void);					// PROPixx flips image horizontally
EX_DLL void		DPxDisablePPxRearProjection(void);					// PROPixx does not flip image horizontally
EX_DLL int		DPxIsPPxRearProjection(void);						// Returns non-0 if PROPixx is flipping image horizontally

 
EX_DLL void     DPxSetPPxDlpSeqPgrm(int program);                   // Set PROPixx DLP Sequencer program (PROPixx Rev >= 6 only)
                                                                    //  Recognized values:
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_RGB
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_RB3D		(PROPixx Rev >=  5 only)
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_RGB240      (PROPixx Rev >= 10 only)
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_RGB180      (PROPixx Rev >= 10 only)
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_QUAD4X      (PROPixx Rev >=  9 only)
                                                                    //          PPXREG_VID_SEQ_CSR_PGRM_QUAD12X     (PROPixx Rev >= 11 only)
																	//          PPXREG_VID_SEQ_CSR_PGRM_RGB_CAL_HBD (PROPixx Rev >= 24 only)
																	//          PPXREG_VID_SEQ_CSR_PGRM_GREY_3X     (PROPixx Rev >= 26 only)
																	//          PPXREG_VID_SEQ_CSR_PGRM_GREY_720    (PROPixx Rev >= 26 only)
                                                                    // Contains an implicit call to DPxUpdateRegCache().
EX_DLL int      DPxGetPPxDlpSeqPgrm(void);                          // Get PROPixx DLP Sequencer program


// (PROPixx Rev >= 12 only)
EX_DLL void		DPxEnablePPxLampLed(void);							// Enable PROPixx Lamp LEDs 
EX_DLL void		DPxDisablePPxLampLed(void);							// Disable PROPixx Lamp LEDs
EX_DLL int		DPxIsPPxLampLedEnabled(void);						// Returns non-0 if PROPixx Lamp LED is enabled

// (PROPixx Rev >= 12 only)
EX_DLL void		DPxSetPPxAwake(void);								// Awake PROPixx Projector (Turn on)
EX_DLL void		DPxSetPPxSleep(void);								// Sleep PROPixx Projector (Turn off)
EX_DLL int		DPxIsPPxAwake(void);								// Returns non-0 if PROPixx is awake

// (PROPixx Rev >= 19 only)
EX_DLL void		DPxEnablePPxQuietFanMode(void);						// Enable quiet fan mode where fans run at a lower speed
EX_DLL void		DPxDisablePPxQuietFanMode(void);					// Disable quiet fan mode
EX_DLL int		DPxIsPPxQuietFanMode(void);							// Returns non-0 if quiet fan mode is enabled


// PROPixx Tachistoscope sub-system
EX_DLL void     DPxEnablePPxTScope(void);                           // Enable the PROPixx T-Scope subsystem.
                                                                    // This will terminate normal display of incoming video.
                                                                    // Contains an implicit call to DPxUpdateRegCache().
EX_DLL void     DPxDisablePPxTScope(void);                          // Disable T-Scope subsystem, and return to normal video display.
                                                                    // Contains an implicit call to DPxUpdateRegCache().
EX_DLL int      DPxIsPPxTScopeEnabled(void);                        // Returns non-zero if T-Scope subsystem is enabled
EX_DLL void     DPxSetPPxTScopeBuffBasePage(unsigned page);         // Set the PROPixx RAM page containing the T-Scope "cover" image
EX_DLL unsigned DPxGetPPxTScopeBuffBasePage(void);                  // Get the RAM page for the T-Scope "cover" image
EX_DLL void     DPxSetPPxTScopeBuffNPages(unsigned nPages);         // Set the number of images in the downloaded T-Scope movie
                                                                    // If the playback schedule exceeds this number of pages,
                                                                    // the T-Scope movie will wrap back to the page after the base (cover) page.
                                                                    // The maximum number of pages available to T-Scope movies depends on how much RAM is available in the PROPixx.
                                                                    // Currently the PROPixx contains 2GB of RAM.  The top 32MB is reserved, leaving 2016 MB for the T-Scope.
                                                                    // T-Scope pages are stored on 256kB boundaries, so a maximum of 8064 T-Scope pages are available.
EX_DLL unsigned DPxGetPPxTScopeBuffNPages(void);                    // Get the number of pages in the uploaded T-Scope movie

#define PPX_TSCOPE_PAGE_DATA_SIZE (1920*1080/8)
#define PPX_TSCOPE_PAGE_STEP_SIZE (256*1024)
// T-Scope images are 1920 * 1080 bits = 2,073,600 bits (=259,200 bytes).
// T-Scope images are stored little-endian, so bit 0 of byte 0 controls the top-left pixel.
// T-Scope images are stored in PROPixx RAM at increments of 256kB (=262,144 bytes)
EX_DLL void     DPxWritePPxTScopePages(unsigned startPage, unsigned nPages, void* pageData);    // Download 1 or more T-Scope images to PROPixx RAM
                                                                                                // pageData must point to a buffer whose size is at least nPages*PPX_TSCOPE_PAGE_DATA_SIZE bytes
EX_DLL void     DPxEnablePPxTScopePrepReq(void);                    // Wait for current video frame to terminate, then load and show cover page of T-Scope movie
EX_DLL void     DPxDisablePPxTScopePrepReq(void);                   // Terminate T-Scope movie on current page
EX_DLL int      DPxIsPPxTScopePrepReq(void);                        // Returns non-zero if a T-Scope movie request is active
EX_DLL int      DPxIsPPxTScopePrepAck(void);                        // Goes non-zero when cover page has been presented, and T-Scope schedule can be started
EX_DLL void		DPxSetPPxTScopeSchedOnset(unsigned onset);			// Set nanosecond delay between schedule start and first T-Scope update
EX_DLL unsigned	DPxGetPPxTScopeSchedOnset(void);					// Get nanosecond delay between schedule start and first T-Scope update
EX_DLL void		DPxSetPPxTScopeSchedRate(unsigned rateValue, int rateUnits);	// Set T-Scope schedule update rate and units
// rateUnits is one of the following predefined constants:
//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is updates per second, maximum 10 kHz
//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is updates per video frame, maximum 10 kHz
//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is update period in nanoseconds, minimum 100,000 ns
EX_DLL unsigned	DPxGetPPxTScopeSchedRate(int *rateUnits);           // Get T-Scope schedule update rate (and optionally get rate units)
EX_DLL void		DPxSetPPxTScopeSchedCount(unsigned count);          // Set T-Scope schedule update count
EX_DLL unsigned	DPxGetPPxTScopeSchedCount(void);                    // Get T-Scope schedule update count
EX_DLL void		DPxEnablePPxTScopeSchedCountdown(void);             // SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisablePPxTScopeSchedCountdown(void);			// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsPPxTScopeSchedCountdown(void);                 // Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetPPxTScopeSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartPPxTScopeSched(void);                       // Start running a T-Scope schedule
EX_DLL void		DPxStopPPxTScopeSched(void);						// Stop running a T-Scope schedule
EX_DLL int		DPxIsPPxTScopeSchedRunning(void);					// Returns non-0 if T-Scope schedule is currently running

// New TScope functionality added to PROPixx Rev 30
EX_DLL void		DPxSetPPxTScopeProgAddr(unsigned progAddr);         // Set start address of T-Scope microcode program.  T-Scope currently supports up to 1024 instructions
EX_DLL unsigned	DPxGetPPxTScopeProgAddr(void);                      // Get start address of T-Scope microcode program
EX_DLL void		DPxSetPPxTScopeProgOffsetPage(unsigned offset);     // Set a constant value which is added to T-Scope microcode instruction pages
EX_DLL unsigned	DPxGetPPxTScopeProgOffsetPage(void);                // Get the constant value which is added to T-Scope microcode instruction pages
EX_DLL void		DPxSetPPxTScopeMode(int mode);                      // Set the PROPixx T-Scope operating mode
// mode is one of the following predefined constants:
//      PPXREG_TSCOPE_CTRL_MODE_BIN_LIST        : T-Scope presents a list of binary images stored in PROPixx DRAM
//      PPXREG_TSCOPE_CTRL_MODE_VID_SINGLE      : T-Scope presents a single video image pointed to by calling DPxSetPPxTScopeBuffBasePage()
//      PPXREG_TSCOPE_CTRL_MODE_VID_PROG        : T-Scope presents a sequence of video images, controlled by microcode uploaded to the PROPixx
EX_DLL int      DPxGetPPxTScopeMode(void);                          // Get the PROPixx T-Scope operating mode
EX_DLL void		DPxEnablePPxTScopeQuad(void);                       // Only one quadrant of an uploaded image is presented at a time
EX_DLL void		DPxDisablePPxTScopeQuad(void);                      // The entire uploaded image is presented
EX_DLL int		DPxIsPPxTScopeQuad(void);                           // Returns non-0 if only one quadrant of an uploaded image is presented at a time
EX_DLL void		DPxSetPPxTScopeProg(UInt16* progData, int nCmds);	// Pass nCmds*2 (max 1024x2) 16-bit microcode words, in order page0,nFrames0,page1,nFrames1...
                                                                    // The microcode is executed until it reaches an instruction with nFrames=0.

// New Gaze-Contingent Display functionality added to PROPixx Rev 30
EX_DLL void		DPxEnablePPxGcdShift(void);                         // PROPixx Gaze-Contingent Display is shifted horizontally and vertically
EX_DLL void		DPxDisablePPxGcdShift(void);                        // PROPixx display is not shifted
EX_DLL int		DPxIsPPxGcdShift(void);                             // Returns non-0 if PROPixx Gaze-Contingent Display is shifted
EX_DLL void		DPxEnablePPxGcdShiftSubframe(void);                 // PROPixx Gaze-Contingent Display can modify image shift on every DLP sub-frame
EX_DLL void		DPxDisablePPxGcdShiftSubframe(void);                // PROPixx Gaze-Contingent Display only modifies shift once per video frame
EX_DLL int		DPxIsPPxGcdShiftSubframe(void);                     // Returns non-0 if PROPixx Gaze-Contingent Display can modify image shift on every DLP sub-frame
EX_DLL void		DPxEnablePPxGcdShiftHardware(void);                 // PROPixx Gaze-Contingent Display shift is controlled by hardware
EX_DLL void		DPxDisablePPxGcdShiftHardware(void);                // PROPixx Gaze-Contingent Display shift is controlled by software registers
EX_DLL int		DPxIsPPxGcdShiftHardware(void);                     // Returns non-0 if PROPixx Gaze-Contingent Display shift is controlled by hardware
EX_DLL void		DPxSetPPxGcdShift(int xShift, int yShift);          // Set the PROPixx Gaze-Contingent Display horizontal/vertical image shift
EX_DLL void		DPxGetPPxGcdShift(int* xShift, int* yShift);        // Get the PROPixx Gaze-Contingent Display horizontal/vertical image shift

// New Gaze-Contingent Display functionality added to PROPixx Ctrl Rev 42
EX_DLL void		DPxEnableGcdShiftHardwareBridge(void);              // PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display shifting
EX_DLL void		DPxDisableGcdShiftHardwareBridge(void);             // PROPixx Controller does not send hardware data to PROPixx for Gaze-Contingent Display shifting
EX_DLL int		DPxIsGcdShiftHardwareBridge(void);                  // Returns non-0 if PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display shifting
EX_DLL void		DPxSetGcdShiftHardwareMode(int mode);               // Specify the way that the PROPixx Controller bridges hardware inputs to PROPixx for Gaze-Contingent Display X/Y shifting
// mode is one of the following predefined constants:
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_NULL      : X/Y are forced to 0
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_ADC       : X/Y are signed 16-bit numbers coming from ADC0/1
//      DPXREG_CTRL_GCD_SHIFT_HW_MODE_DIN       : X/Y are signed 12-bit numbers coming from DIN 11:0/23:12
EX_DLL int      DPxGetGcdShiftHardwareMode(void);                   // Returns the method that the PROPixx Controller bridges hardware inputs for Gaze-Contingent Display X/Y shifting

// PROPixx Controller implements a gain and an offset when mapping hardware inputs to x/y pixel shifts.
// The Gain terms indicate how many pixels should be shifted for a full-scale hardware input, divided by 4.
// For example, the default value of 512 means that a full-scale input from the ADC (+10V) will shift the image 2048 pixels to the right.
// The Offset terms are in straight pixels.  For example, assigning a value of 10 would cause the above +10V ADC to shift the image by 2058 instead of 1048 pixels.
// Note that these gains and offsets are signed 16-bit values.
EX_DLL void     DPxSetGcdShiftHardwareTransform(int xGain, int xOffset, int yGain, int yOffset);
EX_DLL void     DPxGetGcdShiftHardwareTransform(int* xGain, int* xOffset, int* yGain, int* yOffset);

//  QUAD4x + 3D mode functions
EX_DLL void		DPxPPxEnableQuad4x3D(void);
EX_DLL void		DPxPPxDisableQuad4x3D(void);
EX_DLL int		DPxPPxIsQuad4x3D(void);

    
// PROPixx Rev >= 6 only and VIEWPixx/PROPixxCTRL Rev >= 27 only
EX_DLL void		DPxSetCustomStartupConfig(void);	    			// Take a snapshot of the current 240 registers values and save it to use it as new startup configuration.
EX_DLL void		DPxSetFactoryStartupConfig(void);	    			// Set back the factory startup configuration.

// PROPixx Rev >= 8 only and VIEWPixx/PROPixxCTRL Rev >= 27 only
EX_DLL int		DPxIsCustomStartupConfig(void);						// Returns non-0 if VPixx device has loaded custom startup register values


EX_DLL void		DPxSetPPxBitWeightCalibration(UInt16* bitWeightCalData);
EX_DLL void		DPxClearPPxBitWeightCalibration(void);	    			// 

EX_DLL void		DPxEnablePPxSwtpLoad(void);	    				// Enable PROPixx software test pattern load mode
EX_DLL void		DPxDisablePPxSwtpLoad(void);	    			// Disable PROPixx software test pattern load mode
EX_DLL int		DPxIsPPxSwtpLoadEnabled(void);	    			// Returns non-0 if PROPixx software test pattern load mode
EX_DLL void		DPxSetPPxSwtpLoadPage(int page);	    		// Set PROPixx software test pattern load page [0-127]
EX_DLL int		DPxGetPPxSwtpLoadPage(void);   					// return PROPixx software test pattern load page value


//	DAC (Digital to Analog Converter) subsystem
//	4 16-bit DACs can be written directly by user, or updated by a DAC schedule.
//	A DAC schedule is used to automatically copy a waveform from DATAPixx RAM to the DACs.
//	Typical scheduling steps are:
//		-DPxWriteRam() to write one or more 16-bit waveforms into DATAPixx RAM
//		-DPxSetDacBuffReadAddr() specifies the RAM address where the first DAC datum should be read
//		-If the waveform buffer is expected to wrap, then DPxSetDacBuffBaseAddr() and DPxSetDacBuffSize() controls buffer address wrapping
//		-DPxEnableDacBuffChan() for each DAC channel which should be updated from RAM buffer
//		-DPxSetDacSchedOnset() specifies nanosecond onset delay between schedule start and first update of DACs enabled in mask
//		-DPxSetDacSchedRate() sets rate at which schedule should update enabled DACs.  Interpretation of rateValue depends on rateUnits:
//		-To schedule a fixed number of DAC updates, call DPxEnableDacSchedCountdown() and set DPxSetDacSchedCount() to desired number of DAC updates;
//		 or for an unlimited number of DAC updates, call DPxDisableDacSchedCountdown() to count updates forever.
//		-DPxStartDacSched()
//		-The schedule will now wait for the onset delay, then copy the RAM waveform to the enabled DACs at the requested rate
//		-If Countdown mode is disabled, then manually DPxDacSchedStop() when desired
//
EX_DLL int		DPxGetDacNumChans(void);									// Returns number of DAC channels in system (4 in current implementation)
EX_DLL void		DPxSetDacValue(int value, int channel);						// Set the 16-bit 2's complement signed value for one DAC channel
EX_DLL int		DPxGetDacValue(int channel);								// Get the 16-bit 2's complement signed value for one DAC channel
EX_DLL void     DPxGetDacRange(int channel, double *minV, double *maxV);	// Return voltage range; For VIEWPixx: +-10V, for DATAPixx: +-10V for ch0/1, +-5V for ch2/3
EX_DLL void     DPxSetDacVoltage(double voltage, int channel);				// Set the voltage for one DAC channel (+-10V for ch0/1, +-5V for ch2/3)
EX_DLL double	DPxGetDacVoltage(int channel);								// Get the voltage for one DAC channel (+-10V for ch0/1, +-5V for ch2/3)
EX_DLL void		DPxEnableDacCalibRaw(void);									// Enable DAC "raw" mode, causing DAC data to bypass hardware calibration
EX_DLL void		DPxDisableDacCalibRaw(void);								// Disable DAC "raw" mode, causing normal DAC hardware calibration
EX_DLL int		DPxIsDacCalibRaw(void);										// Returns non-0 if DAC data is bypassing hardware calibration
EX_DLL void		DPxEnableDacBuffChan(int channel);							// Enable RAM buffering of a DAC channel
EX_DLL void		DPxDisableDacBuffChan(int channel);							// Disable RAM buffering of a DAC channel
EX_DLL void		DPxDisableDacBuffAllChans(void);							// Disable RAM buffering of all DAC channels
EX_DLL int		DPxIsDacBuffChan(int channel);								// Returns non-0 if RAM buffering is enabled for a DAC channel
EX_DLL void		DPxSetDacBuffBaseAddr(unsigned buffBaseAddr);				// Set DAC RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetDacBuffBaseAddr(void);								// Get DAC RAM buffer start address
EX_DLL void		DPxSetDacBuffReadAddr(unsigned buffReadAddr);				// Set RAM address from which next DAC datum will be read.  Must be an even value.
EX_DLL unsigned	DPxGetDacBuffReadAddr(void);								// Get RAM address from which next DAC datum will be read
EX_DLL void		DPxSetDacBuffSize(unsigned buffSize);						// Set DAC RAM buffer size in bytes.  Must be an even value.  Buffer wraps to Base after Size.
EX_DLL unsigned	DPxGetDacBuffSize(void);									// Get DAC RAM buffer size in bytes
EX_DLL void		DPxSetDacBuff(unsigned buffAddr, unsigned buffSize);		// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetDacSchedOnset(unsigned onset);						// Set nanosecond delay between schedule start and first DAC update
EX_DLL unsigned	DPxGetDacSchedOnset(void);									// Get nanosecond delay between schedule start and first DAC update
EX_DLL void		DPxSetDacSchedRate(unsigned rateValue, int rateUnits);		// Set DAC schedule update rate and units
																			// rateUnits is one of the following predefined constants:
																			// DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is updates per second, maximum 1 MHz
																			// DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is updates per video frame, maximum 1 MHz
																			// DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is update period in nanoseconds, minimum 1000 ns
EX_DLL unsigned	DPxGetDacSchedRate(int *rateUnits);							// Get DAC schedule update rate (and optionally get rate units)
EX_DLL void		DPxSetDacSchedCount(unsigned count);						// Set DAC schedule update count
EX_DLL unsigned	DPxGetDacSchedCount(void);									// Get DAC schedule update count
EX_DLL void		DPxEnableDacSchedCountdown(void);							// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableDacSchedCountdown(void);							// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsDacSchedCountdown(void);								// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetDacSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartDacSched(void);										// Start running a DAC schedule
EX_DLL void		DPxStopDacSched(void);										// Stop running a DAC schedule
EX_DLL int		DPxIsDacSchedRunning(void);									// Returns non-0 if DAC schedule is currently running

//	ADC (Analog to Digital Converter) subsystem
//	The ADC subsystem has 18 simultaneously sampled analog inputs.
//	Inputs 0-15 make up the channel dataset, whose samples can be scheduled and stored to RAM.
//	Inputs 16-17 can be used as differential reference inputs for the 16 data channels.
//	For each of the 16 channels, call DPxSetAdcBuffChanRef() to specify what it's analog refence should be.
//	An ADC schedule is used to automatically store ADC samples into RAM.
//	Typical usage steps are the same as for DAC schedules,
//	except that DPxReadRam() should be used to copy the acquired data from the DATAPixx back to host memory.
//	A schedule can optionally store a 64-bit nanosecond timetag with each sample.
//
EX_DLL int		DPxGetAdcNumChans(void);									// Returns number of ADC channels in system (18 in current implementation)
EX_DLL int		DPxGetAdcValue(int channel);								// Get the 16-bit 2's complement signed value for one ADC channel (0-17)
EX_DLL void		DPxGetAdcRange(int channel, double *minV, double *maxV);	// Return voltage range (+-10V for all channels)
EX_DLL double	DPxGetAdcVoltage(int channel);								// Get the voltage for one ADC channel
EX_DLL void		DPxSetAdcBuffChanRef(int channel, int chanRef);				// Set a channel's differential reference source (only valid for channels 0-15)
																			// chanRef is one of the following predefined constants:
																			//		DPXREG_ADC_CHANREF_GND		: Referenced to ground
																			//		DPXREG_ADC_CHANREF_DIFF		: Referenced to adjacent analog input 
																			//		DPXREG_ADC_CHANREF_REF0		: Referenced to REF0 analog input
																			//		DPXREG_ADC_CHANREF_REF1		: Referenced to REF1 analog input
EX_DLL int		DPxGetAdcBuffChanRef(int channel);							// Get a channel's differential reference source (only valid for channels 0-15)
EX_DLL void		DPxEnableAdcBuffChan(int channel);							// Enable RAM buffering of an ADC channel (only valid for channels 0-15)
EX_DLL void		DPxDisableAdcBuffChan(int channel);							// Disable RAM buffering of an ADC channel (only valid for channels 0-15)
EX_DLL void		DPxDisableAdcBuffAllChans(void);							// Disable RAM buffering of all ADC channels
EX_DLL int		DPxIsAdcBuffChan(int channel);								// Returns non-0 if RAM buffering is enabled for an ADC channel (only valid for channels 0-15)
EX_DLL void		DPxEnableAdcCalibRaw(void);									// Enable ADC "raw" mode, causing ADC data to bypass hardware calibration
EX_DLL void		DPxDisableAdcCalibRaw(void);								// Disable ADC "raw" mode, causing normal ADC hardware calibration
EX_DLL int		DPxIsAdcCalibRaw(void);										// Returns non-0 if ADC data is bypassing hardware calibration
EX_DLL void		DPxEnableDacAdcLoopback(void);								// ADC data readings are looped back internally from programmed DAC voltages:
																			//		DAC_DATA0 => ADC_DATA0/2/4/6/8/10/12/14
																			//		DAC_DATA1 => ADC_DATA1/3/5/7/9/11/13/15
																			//		DAC_DATA2 => ADC_REF0
																			//		DAC_DATA3 => ADC_REF1
EX_DLL void		DPxDisableDacAdcLoopback(void);								// Disable ADC loopback, causing ADC readings to reflect real analog inputs
EX_DLL int		DPxIsDacAdcLoopback(void);									// Returns non-0 if ADC inputs are looped back from DAC outputs
EX_DLL void		DPxEnableAdcFreeRun(void);									// ADC's convert continuously (can add up to 4 microseconds random latency to scheduled samples)
EX_DLL void		DPxDisableAdcFreeRun(void);									// ADC's only convert on schedule ticks (for microsecond-precise sampling)
EX_DLL int		DPxIsAdcFreeRun(void);										// Returns non-0 if ADC's are performing continuous conversions
EX_DLL void		DPxSetAdcBuffBaseAddr(unsigned buffBaseAddr);				// Set ADC RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetAdcBuffBaseAddr(void);								// Get ADC RAM buffer start address
EX_DLL void		DPxSetAdcBuffWriteAddr(unsigned buffWriteAddr);				// Set RAM address to which next ADC datum will be written.  Must be an even value.
EX_DLL unsigned	DPxGetAdcBuffWriteAddr(void);								// Get RAM address to which next ADC datum will be written
EX_DLL void		DPxSetAdcBuffSize(unsigned buffSize);						// Set ADC RAM buffer size in bytes.  Must be an even value.  Buffer wraps after Size.
EX_DLL unsigned	DPxGetAdcBuffSize(void);									// Get ADC RAM buffer size in bytes
EX_DLL void		DPxSetAdcBuff(unsigned buffAddr, unsigned buffSize);		// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetAdcSchedOnset(unsigned onset);						// Set nanosecond delay between schedule start and first ADC sample
EX_DLL unsigned	DPxGetAdcSchedOnset(void);									// Get nanosecond delay between schedule start and first ADC sample
EX_DLL void		DPxSetAdcSchedRate(unsigned rateValue, int rateUnits);		// Set ADC schedule sample rate and units
																			// rateUnits is one of the following predefined constants:
																			//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 200 kHz
																			//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 200 kHz
																			//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 5000 ns
EX_DLL unsigned	DPxGetAdcSchedRate(int *rateUnits);							// Get ADC schedule sample rate (and optionally get rate units)
EX_DLL void		DPxSetAdcSchedCount(unsigned count);						// Set ADC schedule sample count
EX_DLL unsigned	DPxGetAdcSchedCount(void);									// Get ADC schedule sample count
EX_DLL void		DPxEnableAdcSchedCountdown(void);							// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableAdcSchedCountdown(void);							// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsAdcSchedCountdown(void);								// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetAdcSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartAdcSched(void);										// Start running an ADC schedule
EX_DLL void		DPxStopAdcSched(void);										// Stop running an ADC schedule
EX_DLL int		DPxIsAdcSchedRunning(void);									// Returns non-0 if ADC schedule is currently running
EX_DLL void		DPxEnableAdcLogTimetags(void);								// Each buffered ADC sample is preceeded with a 64-bit nanosecond timetag
EX_DLL void		DPxDisableAdcLogTimetags(void);								// Buffered data has no timetags
EX_DLL int		DPxIsAdcLogTimetags(void);									// Returns non-0 if buffered datasets are preceeded with nanosecond timetag

//	DOUT (Digital Output) subsystem
//	The DATAPixx has 24 TTL outputs.
//	The low 16 bits can be written directly by the user, or updated by a DOUT schedule.
//	The high 8 bits can only be written by the user, not by a schedule.
//	A DOUT schedule is used to automatically copy a waveform from DATAPixx RAM to the low 16 digital outputs.
//	Typical usage steps are the same as for DAC schedules.
//
EX_DLL int		DPxGetDoutNumBits(void);								// Returns number of digital output bits in system (24 in current implementation)
EX_DLL void		DPxSetDoutValue(int bitValue, int bitMask);				// For each of the 24 bits set in bitMask, set the DOUT to the value in the corresponding bitValue
EX_DLL int		DPxGetDoutValue(void);									// Get the values of the 24 DOUT bits
EX_DLL void		DPxEnableDoutButtonSchedules(void);						// Enable automatic DOUT schedules upon DIN button presses
EX_DLL void		DPxDisableDoutButtonSchedules(void);					// Disable automatic DOUT schedules upon DIN button presses
EX_DLL int		DPxIsDoutButtonSchedules(void);							// Returns non-0 if automatic DOUT schedules occur upon DIN button presses
EX_DLL void		DPxSetDoutButtonSchedulesMode(int mode);				// If mode == DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE_FALLING_EDGE (0), for standard, else MRI (1)
EX_DLL unsigned	DPxGetDoutButtonSchedulesMode(void);					// Returns DPXREG_DOUT_CTRL_BUTTON_SCHED_MODE_FALLING_EDGE (0) for standard, 1 for RISING_EDGE
EX_DLL void		DPxEnableDoutBacklightPulse(void);						// LCD backlight LED enables are gated by DOUT15.  Can be used to make a tachistoscope by pulsing DOUT15 with a schedule.
EX_DLL void		DPxDisableDoutBacklightPulse(void);                     // LCD backlight LEDs are unaffected by DOUT system.
EX_DLL int		DPxIsDoutBacklightPulse(void);							// Returns non-0 if LCD backlight LED enables are gated by DOUT15
EX_DLL void		DPxEnableDoutPixelMode(void);							// Enable pixel mode where DOUT show RGB value of first upper left pixel of the screen	-- VIEWPixx Rev >= 31 only
EX_DLL void		DPxDisableDoutPixelMode(void);							// Disable pixel mode -- VIEWPixx Rev >= 31 only
EX_DLL int		DPxIsDoutPixelMode(void);								// Returns non-0 if DOUT pixel mode is enabled -- VIEWPixx Rev >= 31 only
EX_DLL void		DPxEnableDoutVsyncMode(void);							// Enable VSYNC mode where DOUT[23] show the VSYNC value -- VIEWPixx Rev >= 46 only
EX_DLL void		DPxDisableDoutVsyncMode(void);							// Disable VSYNC mode -- VIEWPixx Rev >= 46 only *** PIXEL MODE TAKES PRIORIY ON VSYNC MODE!
EX_DLL int		DPxIsDoutVsyncMode(void);								// Returns non-0 if DOUT VSYNC mode is enabled -- VIEWPixx Rev >= 46 only
EX_DLL void		DPxSetDoutBuffBaseAddr(unsigned buffBaseAddr);			// Set DOUT RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetDoutBuffBaseAddr(void);							// Get DOUT RAM buffer start address
EX_DLL void		DPxSetDoutBuffReadAddr(unsigned buffReadAddr);			// Set RAM address from which next DOUT datum will be read.  Must be an even value.
EX_DLL unsigned	DPxGetDoutBuffReadAddr(void);							// Get RAM address from which next DOUT datum will be read
EX_DLL void		DPxSetDoutBuffSize(unsigned buffSize);					// Set DOUT RAM buffer size in bytes.  Must be an even value.  Buffer wraps to Base after Size.
EX_DLL unsigned	DPxGetDoutBuffSize(void);								// Get DOUT RAM buffer size in bytes
EX_DLL void		DPxSetDoutBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetDoutSchedOnset(unsigned onset);					// Set nanosecond delay between schedule start and first DOUT update
EX_DLL unsigned	DPxGetDoutSchedOnset(void);								// Get nanosecond delay between schedule start and first DOUT update
EX_DLL void		DPxSetDoutSchedRate(unsigned rateValue, int rateUnits);	// Set DOUT schedule update rate and units
																		// rateUnits is one of the following predefined constants:
																		//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is updates per second, maximum 10 MHz
																		//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is updates per video frame, maximum 10 MHz
																		//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is update period in nanoseconds, minimum 100 ns
EX_DLL unsigned	DPxGetDoutSchedRate(int *rateUnits);					// Get DOUT schedule update rate (and optionally get rate units)
EX_DLL void		DPxSetDoutSchedCount(unsigned count);					// Set DOUT schedule update count
EX_DLL unsigned	DPxGetDoutSchedCount(void);								// Get DOUT schedule update count
EX_DLL void		DPxEnableDoutSchedCountdown(void);						// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableDoutSchedCountdown(void);						// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsDoutSchedCountdown(void);							// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetDoutSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartDoutSched(void);								// Start running a DOUT schedule
EX_DLL void		DPxStopDoutSched(void);									// Stop running a DOUT schedule
EX_DLL int		DPxIsDoutSchedRunning(void);							// Returns non-0 if DOUT schedule is currently running

//	DIN (Digital Input) subsystem
//	The DATAPixx has 24 TTL inputs.
//	Transitions on the low 16 bits can be acquired to a RAM buffer.
//	Alternatively, the low 16 bits can be acquired at a fixed frequency using a DIN schedule.
//	Typical usage steps for DIN schedules are the same as for DAC schedules.
//	A 64-bit nanosecond timetag can optionally be stored with each sample in the RAM buffer.
//	The DIN ports are actually bidirectional, and each bit can be programmed to drive its port.
//	Users can program a drive strength, which could be used to vary the intensity of driven LEDs.
//
EX_DLL int		DPxGetDinNumBits(void);									// Returns number of digital input bits in system (24 in current implementation)
EX_DLL int		DPxGetDinValue(void);									// Get the values of the 24 DIN bits
EX_DLL void		DPxSetDinDataDir(int directionMask);					// Set 24-bit port direction mask.  Set mask bits to 1 for each bit which should drive its port.
EX_DLL int		DPxGetDinDataDir(void);									// Get 24-bit port direction mask
EX_DLL void		DPxSetDinDataOut(int dataOut);							// Set the data which should be driven on each port whose output has been enabled by DPxSetDinDataDir()
EX_DLL int		DPxGetDinDataOut(void);									// Get the data which is being driven on each output port
EX_DLL void		DPxSetDinDataOutStrength(double strength);				// Set strength of driven outputs.  Range is 0-1.  Implementation uses 1/16 up to 16/16.
EX_DLL double	DPxGetDinDataOutStrength(void);							// Get strength of driven outputs.  Range is 0-1.  Implementation uses 1/16 up to 16/16.
EX_DLL void		DPxEnableDinStabilize(void);							// DIN transitions are only recognized after entire DIN bus has been stable for 80 ns (good for deskewing parallel busses, and ignoring transmission line reflections).
EX_DLL void		DPxDisableDinStabilize(void);							// Immediately recognize all DIN transitions, possibly with debouncing.
EX_DLL int		DPxIsDinStabilize(void);								// Returns non-0 if DIN transitions are being stabilized
EX_DLL void		DPxEnableDinDebounce(void);								// When a DIN transitions, ignore further DIN transitions for next 30 milliseconds (good for response buttons)
EX_DLL void		DPxDisableDinDebounce(void);							// Immediately recognize all DIN transitions (after possible stabilization)
EX_DLL int		DPxIsDinDebounce(void);									// Returns non-0 if DIN transitions are being debounced
EX_DLL void		DPxEnableDoutDinLoopback(void);							// Enable loopback between digital output ports and digital inputs
EX_DLL void		DPxDisableDoutDinLoopback(void);						// Disable loopback between digital outputs and digital inputs
EX_DLL int		DPxIsDoutDinLoopback(void);								// Returns non-0 if digital inputs are driven by digital output ports
EX_DLL void		DPxSetDinBuffBaseAddr(unsigned buffBaseAddr);			// Set DIN RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetDinBuffBaseAddr(void);							// Get DIN RAM buffer start address
EX_DLL void		DPxSetDinBuffWriteAddr(unsigned buffWriteAddr);			// Set RAM address to which next DIN datum will be written.  Must be an even value.
EX_DLL unsigned	DPxGetDinBuffWriteAddr(void);							// Get RAM address to which next DIN datum will be written
EX_DLL void		DPxSetDinBuffSize(unsigned buffSize);					// Set DIN RAM buffer size in bytes.  Must be an even value.  Buffer wraps after Size.
EX_DLL unsigned	DPxGetDinBuffSize(void);								// Get DIN RAM buffer size in bytes
EX_DLL void		DPxSetDinBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetDinSchedOnset(unsigned onset);					// Set nanosecond delay between schedule start and first DIN sample
EX_DLL unsigned	DPxGetDinSchedOnset(void);								// Get nanosecond delay between schedule start and first DIN sample
EX_DLL void		DPxSetDinSchedRate(unsigned rateValue, int rateUnits);	// Set DIN schedule sample rate and units
																		// rateUnits is one of the following predefined constants:
																		//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 1 MHz
																		//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 1 MHz
																		//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 1000 ns
EX_DLL unsigned	DPxGetDinSchedRate(int *rateUnits);						// Get DIN schedule sample rate (and optionally get rate units)
EX_DLL void		DPxSetDinSchedCount(unsigned count);					// Set DIN schedule sample count
EX_DLL unsigned	DPxGetDinSchedCount(void);								// Get DIN schedule sample count
EX_DLL void		DPxEnableDinSchedCountdown(void);						// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableDinSchedCountdown(void);						// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsDinSchedCountdown(void);							// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetDinSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartDinSched(void);									// Start running an DIN schedule
EX_DLL void		DPxStopDinSched(void);									// Stop running an DIN schedule
EX_DLL int		DPxIsDinSchedRunning(void);								// Returns non-0 if DIN schedule is currently running
EX_DLL void		DPxEnableDinLogTimetags(void);							// Each buffered DIN sample is preceeded with a 64-bit nanosecond timetag
EX_DLL void		DPxDisableDinLogTimetags(void);							// Buffered data has no timetags
EX_DLL int		DPxIsDinLogTimetags(void);								// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
EX_DLL void		DPxEnableDinLogEvents(void);							// Each DIN transition is automatically logged (no schedule is required.  Best way to log response buttons)
EX_DLL void		DPxDisableDinLogEvents(void);							// Disable automatic logging of DIN transitions
EX_DLL int		DPxIsDinLogEvents(void);								// Returns non-0 if DIN transitions are being logged to RAM buffer

//	TOUCHPixx Subsystem
//  Note that the TOUCHPixx subsystem uses some of the Digital Input resources,
//  so, for example, TOUCHPixx logging cannot be used simultaneously with digital input logging.
//
EX_DLL void		DPxEnableTouchpixx(void);								// Enables the TOUCHPixx touch panel hardware subsystem
EX_DLL void		DPxDisableTouchpixx(void);								// Disables the TOUCHPixx touch panel hardware subsystem
EX_DLL int		DPxIsTouchpixx(void);									// Returns non-0 if TOUCHPixx touch panel hardware is present and enabled
EX_DLL void		DPxSetTouchpixxType(int touchPanelMode);				// Set the actual Touchpixx type (VPX/DP2/PPC Rev >= 37 only)
																		//  Recognized values: 
																		//       DPXREG_DIN_CTRL_TOUCHPIXX_TYPE_RES		0
																		//       DPXREG_DIN_CTRL_TOUCHPIXX_TYPE_CAP		1
EX_DLL int		DPxGetTouchpixxType(void);								// Returns the actual Touchpixx type
EX_DLL void		DPxSetTouchpixxBuffBaseAddr(unsigned buffBaseAddr);		// Set TOUCHPixx RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetTouchpixxBuffBaseAddr(void);						// Get TOUCHPixx RAM buffer start address
EX_DLL void		DPxSetTouchpixxBuffWriteAddr(unsigned buffWriteAddr);	// Set RAM address to which next TOUCHPixx datum will be written.  Must be an even value.
EX_DLL unsigned	DPxGetTouchpixxBuffWriteAddr(void);						// Get RAM address to which next TOUCHPixx datum will be written
EX_DLL void		DPxSetTouchpixxBuffSize(unsigned buffSize);				// Set TOUCHPixx RAM buffer size in bytes.  Must be an even value.  Buffer wraps after Size.
EX_DLL unsigned	DPxGetTouchpixxBuffSize(void);							// Get TOUCHPixx RAM buffer size in bytes
EX_DLL void		DPxSetTouchpixxBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/WriteAddr
EX_DLL void		DPxEnableTouchpixxLogEvents(void);						// Each TOUCHPixx panel press and release is automatically logged to RAM buffer
EX_DLL void		DPxDisableTouchpixxLogEvents(void);						// Disable automatic logging of TOUCHPixx activity
EX_DLL int		DPxIsTouchpixxLogEvents(void);							// Returns non-0 if TOUCHPixx activity is being logged to RAM buffer
EX_DLL void		DPxEnableTouchpixxLogTimetags(void);					// Each buffered TOUCHPixx sample is preceeded with a 64-bit nanosecond timetag
EX_DLL void		DPxDisableTouchpixxLogTimetags(void);					// Buffered data has no timetags
EX_DLL int		DPxIsTouchpixxLogTimetags(void);						// Returns non-0 if buffered datasets are preceeded with nanosecond timetag
EX_DLL void		DPxEnableTouchpixxLogContinuousMode(void);				// TOUCHPixx logging returns continuous position updates during a panel press
EX_DLL void		DPxDisableTouchpixxLogContinuousMode(void);				// TOUCHPixx logging only returns initial press and release events
EX_DLL int		DPxIsTouchpixxLogContinuousMode(void);					// Returns non-0 if TOUCHPixx logging returns continuous position updates during a panel press
EX_DLL void     DPxSetTouchpixxStabilizeDuration(double duration);      // Set duration in seconds that TOUCHPixx panel coordinates must be stable before being recognized as a touch in DPxGetTouchpixxCoords()
EX_DLL double   DPxGetTouchpixxStabilizeDuration(void);                 // Return duration in seconds that TOUCHPixx panel coordinates must be stable before being recognized as a touch in DPxGetTouchpixxCoords()
EX_DLL int      DPxIsTouchpixxPressed(void);                            // Returns non-0 if touch panel is currently pressed
EX_DLL void		DPxGetTouchpixxCoords(int* x, int* y);                  // Get the current touch panel X/Y coordinates.  Returns (0,0) if panel not pressed.

//	AUD (Audio Output) subsystem
//	The DATAPixx has a stereo output system, viewed as two output channels (Left/Right).
//	Data should be provided in 16-bit 2's complement format, typically as a playback schedule.
//	Typical schedule usage steps are the same as for DAC schedules.
//	An AUD buffer and schedule can supply data to both the Left/Right channels;
//	or, AUD can supply just the Left channel, and a second AUX buffer and schedule can supply the Right channel.
//	Use the AUX channel to implement separate RAM buffers for Left/Right data streams,
//	and separate schedule onset delays for Left/Right waveforms.
//
//	Call DPxInitAudCodec() once before other Aud/Mic routines, to configure initial audio CODEC state.
//	Can also call this at any time to return CODEC to its initial state.
//	Note that the first time this routine is called after reset,
//	it can pause up to 0.6 seconds while CODEC internal amplifiers are powering up.
//	This delay garantees that the CODEC is ready for stable playback immediately upon return.
EX_DLL void		DPxInitAudCodec(void);									// Call this once before other Aud/Mic routines to configure initial audio CODEC state
 
// There's really no point in manually setting audio output values.
// The hardware audio outputs are AC coupled,
// and even the register values slew to 0 when there is no audio schedule running
// (to prevent pop when next audio schedule starts).
EX_DLL void		DPxSetAudLeftValue(int value);						// Set the 16-bit 2's complement signed value for the Left audio output channel
EX_DLL void		DPxSetAudRightValue(int value);						// Set the 16-bit 2's complement signed value for the Right audio output channel
EX_DLL int		DPxGetAudLeftValue(void);							// Get the 16-bit 2's complement signed value for the Left audio output channel
EX_DLL int		DPxGetAudRightValue(void);							// Get the 16-bit 2's complement signed value for the Right audio output channel

// The audio output API has two groups of volume control functions.
// This first group digitally attenuates audio data within the FPGA _before_ sending it to the CODEC.
// As such, volume manipulations here are very precise, and affect both the DATAPixx speaker and Audio OUT ports.
EX_DLL void		DPxSetAudLeftVolume(double volume);					// Set volume for the Left audio output channel, range 0-1
EX_DLL double	DPxGetAudLeftVolume(void);							// Get volume for the Left audio output channel, range 0-1
EX_DLL void		DPxSetAudRightVolume(double volume);				// Set volume for the Right audio output channel, range 0-1
EX_DLL double	DPxGetAudRightVolume(void);							// Get volume for the Right audio output channel, range 0-1
EX_DLL void		DPxSetAudVolume(double volume);						// Set volume for both Left/Right audio channels, range 0-1
EX_DLL double	DPxGetAudVolume(void);								// Get volume for both Left/Right audio channels (or Left channel, if Left/Right are different)
 																	// The floating point volume parameter is in the range 0-1.
 																	// The actual Left/Right volume controls are implemented as 16-bit unsigned FPGA registers.
 																	// The audio samples read from the RAM buffer are multiplied by the volume registers
 																	// (then right-shifted 16 bits) before forwarding the samples to the CODEC.
 																	// Volume value 1.0 is a special case which bypasses this multiplication.
 																	// NOTE: For safety's sake, reset value of L/R volumes = 0, so must set volume before playback.
 
// The audio output API has two groups of volume control functions.
// See the above paragraph for a description of the first group of volume control functions.
// This second group uses CODEC internal registers to independently attenuate L/R audio outputs to the DATAPixx speaker and Audio OUT ports.
// These CODEC volume registers have a precision (step size) of about 0.5 dB (see TI TLV320AIC32 datasheet for details).
// These routines can use either ratio units (0 to 1), or the corresponding dB values (-inf to 0).
// To minimize hiss, it is a good idea to do ball-park attenuation using these CODEC registers.
// Then, for ultra-precise stimulus volume manipulation, use the above SetAudVolume() functions.
// Those functions very precisely attenuate the digital audio data _before_ sending to the CODEC.
EX_DLL void		DPxSetAudCodecOutLeftVolume(double volume, int dBUnits);		// Set volume for the DATAPixx Audio OUT Left channel, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecOutLeftVolume(int dBUnits);						// Get volume for the DATAPixx Audio OUT Left channel, range 0-1 (or dB)
EX_DLL void		DPxSetAudCodecOutRightVolume(double volume, int dBUnits);		// Set volume for the DATAPixx Audio OUT Right channel, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecOutRightVolume(int dBUnits);						// Get volume for the DATAPixx Audio OUT Right channel, range 0-1 (or dB)
EX_DLL void		DPxSetAudCodecOutVolume(double volume, int dBUnits);			// Set volume for the DATAPixx Audio OUT Left and Right channels, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecOutVolume(int dBUnits);							// Get volume for the DATAPixx Audio OUT Left and Right channels (or Left channel, if Left/Right are different)
EX_DLL void		DPxSetAudCodecSpeakerLeftVolume(double volume, int dBUnits);	// Set volume for the DATAPixx Speaker Left channel, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecSpeakerLeftVolume(int dBUnits);					// Get volume for the DATAPixx Speaker Left channel, range 0-1 (or dB)
EX_DLL void		DPxSetAudCodecSpeakerRightVolume(double volume, int dBUnits);	// Set volume for the DATAPixx Speaker Right channel, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecSpeakerRightVolume(int dBUnits);					// Get volume for the DATAPixx Speaker Right channel, range 0-1 (or dB)
EX_DLL void		DPxSetAudCodecSpeakerVolume(double volume, int dBUnits);		// Set volume for the DATAPixx Speaker Left and Right channels, range 0-1 (or dB)
EX_DLL double	DPxGetAudCodecSpeakerVolume(int dBUnits);						// Get volume for the DATAPixx Speaker Left and Right channels (or Left channel, if Left/Right are different)

EX_DLL void		DPxSetAudLRMode(int lrMode);							// Configure how audio Left/Right channels are updated by schedule data
																		// lrMode is one of the following predefined constants:
																		//		DPXREG_AUD_CTRL_LRMODE_MONO		: Each AUD schedule datum goes to left and right channels
																		//		DPXREG_AUD_CTRL_LRMODE_LEFT		: Each AUD schedule datum goes to left channel only
																		//		DPXREG_AUD_CTRL_LRMODE_RIGHT	: Each AUD schedule datum goes to right channel only
																		//		DPXREG_AUD_CTRL_LRMODE_STEREO_1	: Pairs of AUD data are copied to left/right channels
																		//		DPXREG_AUD_CTRL_LRMODE_STEREO_2	: AUD data goes to left channel, AUX data goes to right
EX_DLL int		DPxGetAudLRMode(void);									// Get the audio Left/Right configuration mode
EX_DLL void		DPxSetAudBuffBaseAddr(unsigned buffBaseAddr);			// Set AUD RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetAudBuffBaseAddr(void);							// Get AUD RAM buffer start address
EX_DLL void		DPxSetAudBuffReadAddr(unsigned buffReadAddr);			// Set RAM address from which next AUD datum will be read.  Must be an even value.
EX_DLL unsigned	DPxGetAudBuffReadAddr(void);							// Get RAM address from which next AUD datum will be read
EX_DLL void		DPxSetAudBuffSize(unsigned buffSize);					// Set AUD RAM buffer size in bytes.  Must be an even value.  Buffer wraps to Base after Size.
EX_DLL unsigned	DPxGetAudBuffSize(void);								// Get AUD RAM buffer size in bytes
EX_DLL void		DPxSetAudBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetAuxBuffBaseAddr(unsigned buffBaseAddr);			// Set AUX RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetAuxBuffBaseAddr(void);							// Get AUX RAM buffer start address
EX_DLL void		DPxSetAuxBuffReadAddr(unsigned buffReadAddr);			// Set RAM address from which next AUX datum will be read.  Must be an even value.
EX_DLL unsigned	DPxGetAuxBuffReadAddr(void);							// Get RAM address from which next AUX datum will be read
EX_DLL void		DPxSetAuxBuffSize(unsigned buffSize);					// Set AUX RAM buffer size in bytes.  Must be an even value.  Buffer wraps to Base after Size.
EX_DLL unsigned	DPxGetAuxBuffSize(void);								// Get AUX RAM buffer size in bytes
EX_DLL void		DPxSetAuxBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetAudSchedOnset(unsigned onset);					// Set nanosecond delay between schedule start and first AUD update
EX_DLL unsigned	DPxGetAudSchedOnset(void);								// Get nanosecond delay between schedule start and first AUD update
EX_DLL void		DPxSetAudSchedRate(unsigned rateValue, int rateUnits);	// Set AUD (and AUX) schedule update rate and units
																		// rateUnits is one of the following predefined constants:
																		//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is updates per second, maximum 96 kHz
																		//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is updates per video frame, maximum 96 kHz
																		//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is update period in nanoseconds, minimum 10417 ns
EX_DLL unsigned	DPxGetAudSchedRate(int *rateUnits);						// Get AUD schedule update rate (and optionally get rate units)
EX_DLL void		DPxSetAudSchedCount(unsigned count);					// Set AUD schedule update count
EX_DLL unsigned	DPxGetAudSchedCount(void);								// Get AUD schedule update count
EX_DLL void		DPxEnableAudSchedCountdown(void);						// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableAudSchedCountdown(void);						// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsAudSchedCountdown(void);							// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetAudSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartAudSched(void);									// Start running a AUD schedule
EX_DLL void		DPxStopAudSched(void);									// Stop running a AUD schedule
EX_DLL int		DPxIsAudSchedRunning(void);								// Returns non-0 if AUD schedule is currently running
EX_DLL void		DPxSetAuxSchedOnset(unsigned onset);					// Set nanosecond delay between schedule start and first AUX update
EX_DLL unsigned	DPxGetAuxSchedOnset(void);								// Get nanosecond delay between schedule start and first AUX update
EX_DLL void		DPxSetAuxSchedRate(unsigned rateValue, int rateUnits);	// Set AUX (and AUD) schedule update rate and units
																		// rateUnits is one of the following predefined constants:
																		//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is updates per second, maximum 96 kHz
																		//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is updates per video frame, maximum 96 kHz
																		//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is update period in nanoseconds, minimum 10417 ns
EX_DLL unsigned	DPxGetAuxSchedRate(int *rateUnits);						// Get AUX schedule update rate (and optionally get rate units)
EX_DLL void		DPxSetAuxSchedCount(unsigned count);					// Set AUX schedule update count
EX_DLL unsigned	DPxGetAuxSchedCount(void);								// Get AUX schedule update count
EX_DLL void		DPxEnableAuxSchedCountdown(void);						// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableAuxSchedCountdown(void);						// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsAuxSchedCountdown(void);							// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetAuxSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	 // Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartAuxSched(void);									// Start running a AUX schedule
EX_DLL void		DPxStopAuxSched(void);									// Stop running a AUX schedule
EX_DLL int		DPxIsAuxSchedRunning(void);								// Returns non-0 if AUX schedule is currently running
 
EX_DLL double	DPxGetAudGroupDelay(double sampleRate);					// Returns CODEC Audio OUT group delay in seconds
 
//	MIC (Microphone/Audio Input) subsystem.
//	The DATAPixx has a stereo microphone input system, viewed as two input channels (Left/Right).
//	Data is returned in 16-bit 2's complement format.
//	Typical usage steps for MIC schedules are the same as for ADC schedules.
//	DPxInitAudCodec() must be called once before using any other Aud/Mic routines.
//	MIC data can actually come from two electrical sources.
//	One is a high-impedance microphone input with 2V bias.  This is for unpowered microphones.
//	Second is a line level (1V RMS) audio input for connection to powered microphones,
//	or any standard line out from audio equipment.
//	NOTE: MIC and AUD subsystem share timing resources within CODEC,
//	so MIC and AUD must have the same rate.
//	Calling DPxSetMicSchedRate() or DPxSetMicSched() also indirectly calls DPxSetAudSchedRate().
EX_DLL void		DPxSetMicSource(int source, double gain, int dBUnits);	// Select the source and gain of the microphone input
 																	// source is one of the following predefined constants:
 																	//		DPX_MIC_SRC_MIC_IN	: Microphone level input
 																	//		DPX_MIC_SRC_LINE_IN	: Line level input
EX_DLL int		DPxGetMicSource(double *gain, int dBUnits);			// Get the source (and optionally the gain) of the microphone input
EX_DLL int		DPxGetMicLeftValue(void);							// Get the 16-bit 2's complement signed value for left MIC channel
EX_DLL int		DPxGetMicRightValue(void);							// Get the 16-bit 2's complement signed value for right MIC channel
EX_DLL void		DPxSetMicLRMode(int lrMode);						// Configure how microphone Left/Right channels are stored to schedule buffer
 																	// lrMode is one of the following predefined constants:
 																	//		DPXREG_MIC_CTRL_LRMODE_MONO		: Mono data is written to schedule buffer (average of Left/Right CODEC data)
 																	//		DPXREG_MIC_CTRL_LRMODE_LEFT		: Left data is written to schedule buffer
 																	//		DPXREG_MIC_CTRL_LRMODE_RIGHT	: Right data is written to schedule buffer
 																	//		DPXREG_MIC_CTRL_LRMODE_STEREO	: Left and Right data are both written to schedule buffer
EX_DLL int		DPxGetMicLRMode(void);									// Get the microphone Left/Right configuration mode
EX_DLL void		DPxEnableAudMicLoopback(void);							// Enable loopback between audio outputs and microphone inputs
EX_DLL void		DPxDisableAudMicLoopback(void);							// Disable loopback between audio outputs and microphone inputs
EX_DLL int		DPxIsAudMicLoopback(void);								// Returns non-0 if microphone inputs are driven by audio outputs
EX_DLL void		DPxSetMicBuffBaseAddr(unsigned buffBaseAddr);			// Set MIC RAM buffer start address.  Must be an even value.
EX_DLL unsigned	DPxGetMicBuffBaseAddr(void);							// Get MIC RAM buffer start address
EX_DLL void		DPxSetMicBuffWriteAddr(unsigned buffWriteAddr);			// Set RAM address to which next MIC datum will be written.  Must be an even value.
EX_DLL unsigned	DPxGetMicBuffWriteAddr(void);							// Get RAM address to which next MIC datum will be written
EX_DLL void		DPxSetMicBuffSize(unsigned buffSize);					// Set MIC RAM buffer size in bytes.  Must be an even value.  Buffer wraps after Size.
EX_DLL unsigned	DPxGetMicBuffSize(void);								// Get MIC RAM buffer size in bytes
EX_DLL void		DPxSetMicBuff(unsigned buffAddr, unsigned buffSize);	// Shortcut which assigns Size/BaseAddr/ReadAddr
EX_DLL void		DPxSetMicSchedOnset(unsigned onset);					// Set nanosecond delay between schedule start and first MIC sample
EX_DLL unsigned	DPxGetMicSchedOnset(void);								// Get nanosecond delay between schedule start and first MIC sample
EX_DLL void		DPxSetMicSchedRate(unsigned rateValue, int rateUnits);	// Set MIC schedule sample rate and units
																		// rateUnits is one of the following predefined constants:
																		//		DPXREG_SCHED_CTRL_RATE_HZ		: rateValue is samples per second, maximum 102.4 kHz
																		//		DPXREG_SCHED_CTRL_RATE_XVID		: rateValue is samples per video frame, maximum 102.4 kHz
																		//		DPXREG_SCHED_CTRL_RATE_NANO		: rateValue is sample period in nanoseconds, minimum 9750 ns
EX_DLL unsigned	DPxGetMicSchedRate(int *rateUnits);						// Get MIC schedule sample rate (and optionally get rate units)
EX_DLL void		DPxSetMicSchedCount(unsigned count);					// Set MIC schedule sample count
EX_DLL unsigned	DPxGetMicSchedCount(void);								// Get MIC schedule sample count
EX_DLL void		DPxEnableMicSchedCountdown(void);						// SchedCount decrements at SchedRate, and schedule stops automatically when count hits 0
EX_DLL void		DPxDisableMicSchedCountdown(void);						// SchedCount increments at SchedRate, and schedule is stopped by calling SchedStop
EX_DLL int		DPxIsMicSchedCountdown(void);							// Returns non-0 if SchedCount decrements to 0 and automatically stops schedule
EX_DLL void		DPxSetMicSched(unsigned onset, unsigned rateValue, int rateUnits, unsigned count);	// Shortcut which assigns Onset/Rate/Count. If Count > 0, enables Countdown mode.
EX_DLL void		DPxStartMicSched(void);									// Start running an MIC schedule
EX_DLL void		DPxStopMicSched(void);									// Stop running an MIC schedule
EX_DLL int		DPxIsMicSchedRunning(void);								// Returns non-0 if MIC schedule is currently running
 
EX_DLL double	DPxGetMicGroupDelay(double sampleRate);					// Returns CODEC MIC IN group delay in seconds
 
//	Video subsystem
EX_DLL void		DPxSetVidMode(int vidMode);							// Set the video processing mode
 																	// vidMode is one of the following predefined constants:
 																	//		DPXREG_VID_CTRL_MODE_C24	Straight passthrough from DVI 8-bit (or HDMI "deep" 10/12-bit) RGB to VGA 8/10/12-bit RGB
 																	//		DPXREG_VID_CTRL_MODE_L48	DVI RED[7:0] is used as an index into a 256-entry 16-bit RGB colour lookup table
 																	//		DPXREG_VID_CTRL_MODE_M16	DVI RED[7:0] & GREEN[7:0] concatenate into a VGA 16-bit value sent to all three RGB components
 																	//		DPXREG_VID_CTRL_MODE_C48	Even/Odd pixel RED/GREEN/BLUE[7:0] concatenate to generate 16-bit RGB components at half the horizontal resolution
 																	//		DPXREG_VID_CTRL_MODE_L48D	DVI RED[7:4] & GREEN[7:4] concatenate to form an 8-bit index into a 256-entry 16-bit RGB colour lookup table
 																	//		DPXREG_VID_CTRL_MODE_M16D	DVI RED[7:3] & GREEN[7:3] & BLUE[7:2] concatenate into a VGA 16-bit value sent to all three RGB components
 																	//		DPXREG_VID_CTRL_MODE_C36D	Even/Odd pixel RED/GREEN/BLUE[7:2] concatenate to generate 12-bit RGB components at half the horizontal resolution
                                                                    //		DPXREG_VID_CTRL_MODE_RB24	DVI RED[7:0] & GREEN[7:4] concatenate to form 12-bit RED value, DVI BLUE[7:0] & GREEN[3:0] concatenate to form 12-bit BLUE value, GREEN is forced to 0 (VIEWPixx Rev >= 21 only)
EX_DLL int		DPxGetVidMode(void);								// Get the video processing mode
EX_DLL void		DPxSetVidClut(UInt16* clutData);					// Pass 256*3 (=768) 16-bit video DAC data, in order R0,G0,B0,R1,G1,B1...
 																	// DPxSetVidClut() returns immediately, and CLUT is implemented at next vertical blanking interval.
EX_DLL void		DPxSetVidClutTransparencyColor(UInt16 red, UInt16 green, UInt16 blue);		// Set 48-bit RGB video CLUT transparency color
EX_DLL void		DPxGetVidClutTransparencyColor(UInt16* red, UInt16* green, UInt16* blue);	// Get 48-bit RGB video CLUT transparency color
EX_DLL void		DPxEnableVidClutTransparencyColorMode(void);		// Enable video CLUT transparency color mode
EX_DLL void		DPxDisableVidClutTransparencyColorMode(void);		// Disable video CLUT transparency color mode
EX_DLL int		DPxIsVidClutTransparencyColorMode(void);			// Returns non-0 if CLUT Transparency Color mode is enabled


EX_DLL void		DPxSetVidCluts(UInt16* clutData);					// Pass 512*3 (=1536) 16-bit video DAC data to fill 2 channel CLUTs with independent data, in order R0,G0,B0,R1,G1,B1...
EX_DLL void		DPxEnableVidHorizSplit(void);						// VGA 1 shows left half of video image, VGA 2 shows right half of video image.  The two VGA outputs are perfectly synchronized.
EX_DLL void		DPxDisableVidHorizSplit(void);						// VGA 1 and VGA 2 both show entire video image (hardware video mirroring)
EX_DLL void		DPxAutoVidHorizSplit(void);							// DATAPixx will automatically split video across the two VGA outputs if the horizontal resolution is at least twice the vertical resolution (default mode)
EX_DLL int		DPxIsVidHorizSplit(void);							// Returns non-0 if video is being split across the two VGA outputs.
EX_DLL void		DPxEnableVidVertStereo(void);						// Top/bottom halves of input image are output in two sequencial video frames.
 																	// VESA L/R output is set to 1 when first frame (left eye) is displayed,
 																	// and set to 0 when second frame (right eye) is displayed.
EX_DLL void		DPxDisableVidVertStereo(void);						// Normal display
EX_DLL void		DPxAutoVidVertStereo(void);							// Vertical stereo is enabled automatically when vertical resolution > horizontal resolution (default mode)
EX_DLL int		DPxIsVidVertStereo(void);							// Returns non-0 if DATAPixx is separating input into sequencial left/right stereo images.
EX_DLL void		DPxEnableVidHorizOverlay(void);						// VGA 1 and VGA 2 both show an overlay composite of the left/right halves of the video image
EX_DLL void		DPxDisableVidHorizOverlay(void);					// Horizontal overlay is disabled
EX_DLL int		DPxIsVidHorizOverlay(void);							// Returns non-0 if the left/right halves of the video image are being overlayed
EX_DLL void		DPxSetVidHorizOverlayBounds(int X1, int Y1, int X2, int Y2); // Set bounding rectangle within left half image whose contents are composited with right half image
EX_DLL void		DPxGetVidHorizOverlayBounds(int* X1, int* Y1, int* X2, int* Y2); // Get bounding rectangle of horizontal overlay window
EX_DLL void		DPxSetVidHorizOverlayAlpha(UInt16* alphaData);		// Set 1024 16-bit video horizontal overlay alpha values, in order X0,X1..X511,Y0,Y1...Y511
EX_DLL void		DPxEnableVidRescanWarning(void);					// Enable the VIEWPixx visual indicator (red flashing square on the top left of the screen) if the incoming video is rescanned and not lockable
EX_DLL void		DPxDisableVidRescanWarning(void);					// Disable the VIEWPixx visual indicator (red flashing square on the top left of the screen ) if the incoming video is rescanned and not lockable
EX_DLL int		DPxIsVidRescanWarning(void);						// Return non-0 if the visual indicator is enabled when incoming video is rescanned and not lockable 
EX_DLL int		DPxGetVidHTotal(void);								// Get number of video dot times in one horizontal scan line (includes horizontal blanking interval)
EX_DLL int		DPxGetVidVTotal(void);								// Get number of video lines in one vertical frame (includes vertical blanking interval)
EX_DLL int		DPxGetVidHActive(void);								// Get number of visible pixels in one horizontal scan line
EX_DLL int		DPxGetVidVActive(void);								// Get number of visible lines in one vertical frame
EX_DLL unsigned	DPxGetVidVPeriod(void);								// Get video vertical frame period in nanoseconds
EX_DLL double	DPxGetVidVFreq(void);								// Get video vertical frame rate in Hz
EX_DLL double	DPxGetVidHFreq(void);								// Get video horizontal line rate in Hz
EX_DLL double	DPxGetVidDotFreq(void);								// Get video dot frequency in Hz
EX_DLL int		DPxIsVidDviActive(void);							// Returns non-0 if DATAPixx is currently receiving video data over DVI link
EX_DLL int		DPxIsVidDviActiveDual(void);						// Returns non-0 if DATAPixx is currently receiving video data over dual-link DVI
EX_DLL int      DPxIsVidDviLockable(void);                          // Returns non-0 if VIEWPixx is currently receiving video whose timing can directly drive display.
EX_DLL int		DPxIsVidOverClocked(void);							// Returns non-0 if DATAPixx is receiving video at too high a clock frequency
EX_DLL void		DPxSetVidVesaLeft(void);							// VESA connector outputs left-eye signal
EX_DLL void		DPxSetVidVesaRight(void);							// VESA connector outputs right-eye signal
EX_DLL int		DPxIsVidVesaLeft(void);								// Returns non-0 if VESA connector has left-eye signal
EX_DLL void		DPxEnableVidVesaBlueline(void);                     // VESA 3D output interprets middle pixel on last raster line as a blueline code
EX_DLL void		DPxDisableVidVesaBlueline(void);					// VESA 3D output is not dependent on video content
EX_DLL int		DPxIsVidVesaBlueline(void);							// Returns non-0 if VESA 3D output is dependent on video blueline codes
EX_DLL int		DPxIsVidVesaNv3dSynced(void);						// Returns non-0 if VESA 3D output is dependent on NVIDIA 3D Sync
EX_DLL void		DPxEnableVidVesaFreeRun(void);						// Enable PROPixx 3D VESA output freeRun enable bit -- PROPixx Rev >= 7 only
EX_DLL void		DPxDisableVidVesaFreeRun(void);						// Disable PROPixx 3D VESA output freeRun enable bit -- PROPixx Rev >= 7 only
EX_DLL int		DPxIsVidVesaFreeRun(void);							// Returns non-0 if PROPixx VESA 3D output enabled by the freeRun register bit -- PROPixx Rev >= 7 only
EX_DLL void     DPxSetVidVesaWaveform(int waveform);                // Set the waveform which will be sent to the DATAPixx VESA 3D connector
                                                                    // waveform is one of the following predefined constants:
                                                                    //      DPXREG_VID_VESA_WAVEFORM_LR             : VESA port drives straight L/R squarewave for 3rd party emitter
                                                                    //      DPXREG_VID_VESA_WAVEFORM_CRYSTALEYES    : VESA port drives 3DPixx IR emitter for CrystalEyes 3D goggles
                                                                    //      DPXREG_VID_VESA_WAVEFORM_NVIDIA         : VESA port drives 3DPixx IR emitter for NVIDIA 3D goggles
EX_DLL int      DPxGetVidVesaWaveform(void);                        // Get the waveform which is being sent to the DATAPixx VESA 3D connector
EX_DLL void     DPxSetVidVesaPhase(int phase);                      // Set the 8-bit unsigned phase of the VESA 3D waveform
                                                                    // Varying this phase from 0-255 will fine tune phase relationship between stereo video and 3D goggle switching
                                                                    // The following combinations have been found to work well:
                                                                    // Waveform=DPXREG_VID_VESA_WAVEFORM_NVIDIA, Phase=100, for VIEWPixx/3D + scanning backlight + 3DPixx IR emitter + NVIDIA 3D Vision glasses
                                                                    // Waveform=DPXREG_VID_VESA_WAVEFORM_NVIDIA, Phase=245, for DATAPixx + CRT + 3DPixx IR emitter + NVIDIA 3D Vision glasses
                                                                    // Waveform=DPXREG_VID_VESA_WAVEFORM_CRYSTALEYES, Phase=100, for VIEWPixx/3D + scanning backlight + 3DPixx  IR emitter + CrystalEyes glasses
 
EX_DLL int      DPxGetVidVesaPhase(void);                           // Get the 8-bit unsigned phase of the VESA 3D waveform
EX_DLL UInt16*	DPxGetVidLine(void);								// Read pixels from the DATAPixx line buffer, and return a pointer to the data.
 																	// For each pixel, the buffer contains 16 bit R/G/B/U (where U is unused).
 																	// The returned data could be overwritten by the next DPx* call, so copy data if needed.
EX_DLL void		DPxSetVidPsyncRasterLine(int line);					// Set the raster line on which pixel sync sequence is expected
EX_DLL int		DPxGetVidPsyncRasterLine(void);						// Get the raster line on which pixel sync sequence is expected
EX_DLL void		DPxEnableVidPsyncSingleLine(void);					// Pixel sync is only recognized on a single raster line
EX_DLL void		DPxDisableVidPsyncSingleLine(void);					// Pixel sync is recognized on any raster line
EX_DLL int		DPxIsVidPsyncSingleLine(void);						// Returns non-0 if pixel sync is only recognized on a single raster line
EX_DLL void		DPxEnableVidPsyncBlankLine(void);					// Pixel sync raster line is always displayed black
EX_DLL void		DPxDisableVidPsyncBlankLine(void);					// Pixel sync raster line is displayed normally
EX_DLL int		DPxIsVidPsyncBlankLine(void);						// Returns non-0 if pixel sync raster line is always displayed black
EX_DLL void     DPxEnableVidScanningBacklight(void);                // Enable VIEWPixx scanning backlight
EX_DLL void     DPxDisableVidScanningBacklight(void);               // Disable VIEWPixx scanning backlight
EX_DLL int      DPxIsVidScanningBacklight(void);                    // Returns non-0 if VIEWPixx scanning backlight is enabled
EX_DLL void     DPxVideoScope(int toFile);                          // DVI Input video source analysis

EX_DLL void		DPxSetVidBacklightIntensity(int intensity);			// Set the back light intensity (0-255) -- VIEWPixx Rev >= 31 only
EX_DLL int		DPxGetVidBacklightIntensity(void);                  // Get the back light intensity			-- VIEWPixx Rev >= 31 only

EX_DLL void		DPxSetVidGreyscaleMode(int greyscaleMode);			// Set greyscale video output mode -- VIEWPixx Rev >= 40 only / PROPixx Rev >= 27 only
                                                                    //      DPXREG_VID_CTRL3_GREY_MODE_RED			: Greyscale based on the Red 8-bit input
																	//      DPXREG_VID_CTRL3_GREY_MODE_GRN			: Greyscale based on the Green 8-bit input
																	//      DPXREG_VID_CTRL3_GREY_MODE_BLU			: Greyscale based on the Blue 8-bit input
																	//      DPXREG_VID_CTRL3_GREY_MODE_DISABLE		: Normal RGB Mode
EX_DLL int		DPxGetVidGreyscaleMode(void);						// Get greyscale video output mode

// Liquid crystal displays can exhibit an artifact when presenting 2 static images on alternating video frames, such as with frame-sequencial 3D.
// The origin of this artifact is related to LCD pixel polarity inversion.
// The optical transmission of a liquid crystal cell varies with the magnitude of the voltage applied to the cell.
// Liquid crystal cells are designed to be driven by an AC voltage with little or no DC component.
// As such, the cell drivers alternate the polarity of the cell's driving voltage on alternate video frames.
// The cell will see no net DC driving voltage, as long as the pixel is programmed to the same intensity on even and odd video frames.
// Small differences in a pixel's even and odd frame luminance tend to leave the cell unaffected,
// and large differences in even and odd frame luminance for short periods of time (10-20 frames?) also do not seem to affect the cell;
// however, large differences in luminance for a longer period of time will cause a DC buildup in the pixel's liquid crystal cell.
// This can result in the pixel not showing the programmed luminance correctly,
// and can also cause the pixel to "stick" for several seconds after the image has been removed, causing an after-image on the display.
// VPixx Technologies has developed a strategy for keeping the pixel cells DC balanced.
// Instead of alternating the cell driving voltage on every video frame, we can alternate the voltage only on every second frame.
// This feature is enabled by calling the routine DPxEnableVidLcd3D60Hz().
// Call this routine before presenting static or slowly-moving 3D images, or when presenting 60Hz flickering stimuli.
// Be sure to call DPxDisableVidLcd3D60Hz() afterwards to return to normal pixel driving.
// Note that this feature is only supported on the VIEWPixx/3D when running with a refresh rate of 120Hz.
EX_DLL void     DPxEnableVidLcd3D60Hz(void);                // Enable 3D pixel polarity inversion
EX_DLL void     DPxDisableVidLcd3D60Hz(void);               // Return to normal pixel polarity inversion
EX_DLL int      DPxIsVidLcd3D60Hz(void);                    // Returns non-0 if 3D pixel polarity inversion is enabled


//	TRACKPixx Bridge
EX_DLL void		DPxEnableTPBridgeLoopback(void);
EX_DLL void		DPxDisableTPBridgeLoopback(void);
EX_DLL int		DPxIsTPBridgeLoopback(void);

//	-If an API function detects an error, it will assign a unique error code to a global error variable.
//	This strategy permits DPxGet*() functions to conveniently return requested values directly,
//	and still make available a global error code which can be checked when desired.
EX_DLL void			DPxSetError(int error);
EX_DLL void			DPxClearError(void);
EX_DLL int			DPxGetError(void);
EX_DLL const char*	DPxGetErrorString(void);


//	-Debugging level controls verbosity of debug and trace messages.
//	Set to:
//		0 to disabled messages
//		1 to print libdpx debug messages
//		2 to also print libusb debug messages
EX_DLL void		DPxSetDebug(int level);
EX_DLL int		DPxGetDebug(void);


// Save/RestoreRegs can be used to save and restore the DATAPixx register state.
// Note that these routines use a single local copy (not a stack), so only do 1-deep save/restore.
EX_DLL void		DPxSaveRegs(void);						// Get all DATAPixx registers, and save them in a local copy
EX_DLL void		DPxRestoreRegs(void);					// Write the local copy back to the DATAPixx


// Miscellaneous routines
EX_DLL void		DPxStopAllScheds(void);					// Shortcut to stop running all DAC/ADC/DOUT/DIN/AUD/AUX/MIC schedules


//	-API global error codes
//	Pretty much each API function usage error sets a unique global error code for easier debugging of user apps
#define DPX_SUCCESS								0		// Function executed successfully
#define DPX_FAIL								-1		// Generic failure code

#define DPX_ERR_USB_NO_DATAPIXX					-1000	// No DATAPixx was found
#define DPX_ERR_USB_RAW_EZUSB					-1001	// EZ-USB appears to have no firmware
#define DPX_ERR_USB_RAW_FPGA					-1002	// FPGA appears to be unconfigured
#define DPX_ERR_USB_OPEN						-1003	// An error occurred while opening a USB channel
#define DPX_ERR_USB_OPEN_FPGA					-1004	// An FPGA detection error occurred while opening DATAPixx
#define DPX_ERR_USB_SET_CONFIG					-1005	// Could not set the USB configuration
#define DPX_ERR_USB_CLAIM_INTERFACE				-1006	// Could not claim the USB interface
#define DPX_ERR_USB_ALT_INTERFACE				-1007	// Could not set the USB alternate interface
#define DPX_ERR_USB_UNKNOWN_DPID				-1008	// Unrecognized DATAPixx ID register value
#define DPX_ERR_USB_REG_BULK_WRITE				-1009	// USB error while writing register set
#define DPX_ERR_USB_REG_BULK_READ				-1010	// USB error while reading register set
#define DPX_ERR_USB_DEVSEL_INDEX				-1011	// Illegal device index
#define DPX_ERR_USB_SYSDEVSEL_INDEX				-1012	// Illegal system device index

#define DPX_ERR_SPI_START						-1100	// SPI communication startup error
#define DPX_ERR_SPI_STOP						-1101	// SPI communication termination error
#define DPX_ERR_SPI_READ						-1102	// SPI communication read error
#define DPX_ERR_SPI_WRITE						-1103	// SPI communication write error
#define DPX_ERR_SPI_ERASE						-1104	// SPI communication erase error
#define DPX_ERR_SPI_WAIT_DONE					-1105	// SPI communication error while waiting for SPI write to complete
#define DPX_ERR_SPI_FIRST_CONFIG				-1106	// SPI communication first config

#define DPX_ERR_SETREG16_ADDR_ODD				-1200	// DPxSetReg16 passed an odd address
#define DPX_ERR_SETREG16_ADDR_RANGE				-1201	// DPxSetReg16 passed an address which was out of range
#define DPX_ERR_SETREG16_DATA_RANGE				-1202	// DPxSetReg16 passed a datum which was out of range
#define DPX_ERR_GETREG16_ADDR_ODD				-1203	// DPxGetReg16 passed an odd address
#define DPX_ERR_GETREG16_ADDR_RANGE				-1204	// DPxGetReg16 passed an address which was out of range
#define DPX_ERR_SETREG32_ADDR_ALIGN				-1205	// DPxSetReg32 passed an address which was not 32-bit aligned
#define DPX_ERR_SETREG32_ADDR_RANGE				-1206	// DPxSetReg32 passed an address which was out of range
#define DPX_ERR_GETREG32_ADDR_ALIGN				-1207	// DPxGetReg32 passed an address which was not 32-bit aligned
#define DPX_ERR_GETREG32_ADDR_RANGE				-1208	// DPxGetReg32 passed an address which was out of range

#define DPX_ERR_NANO_TIME_NULL_PTR				-1300	// A pointer argument was null
#define DPX_ERR_NANO_MARK_NULL_PTR				-1301	// A pointer argument was null
#define DPX_ERR_UNKNOWN_PART_NUMBER				-1302	// Unrecognized part number

#define DPX_ERR_RAM_UNKNOWN_SIZE				-1400	// Unrecognized RAM configuration
#define DPX_ERR_RAM_WRITEREAD_FAIL				-1401	// RAM read did not return same value written
#define DPX_ERR_RAM_WRITE_ADDR_ODD				-1402	// RAM write buffer address must be even
#define DPX_ERR_RAM_WRITE_LEN_ODD				-1403	// RAM write buffer length must be even
#define DPX_ERR_RAM_WRITE_TOO_HIGH				-1404	// RAM write block exceeds end of DATAPixx memory
#define DPX_ERR_RAM_WRITE_BUFFER_NULL			-1405	// RAM write source buffer pointer is null
#define DPX_ERR_RAM_WRITE_USB_ERROR				-1406	// A USB error occurred while writing the RAM buffer
#define DPX_ERR_RAM_READ_ADDR_ODD				-1407	// RAM read buffer address must be even
#define DPX_ERR_RAM_READ_LEN_ODD				-1408	// RAM read buffer length must be even
#define DPX_ERR_RAM_READ_TOO_HIGH				-1409	// RAM read block exceeds end of DATAPixx memory
#define DPX_ERR_RAM_READ_BUFFER_NULL			-1410	// RAM read destination buffer pointer is null
#define DPX_ERR_RAM_READ_USB_ERROR				-1411	// A USB error occurred while reading the RAM buffer

#define DPX_ERR_DAC_SET_BAD_CHANNEL				-1500	// Valid channels are 0-3
#define DPX_ERR_DAC_SET_BAD_VALUE				-1501	// Value falls outside DAC's output range
#define DPX_ERR_DAC_GET_BAD_CHANNEL				-1502	// Valid channels are 0-3
#define DPX_ERR_DAC_RANGE_NULL_PTR				-1503	// A pointer argument was null
#define DPX_ERR_DAC_RANGE_BAD_CHANNEL			-1504	// Valid channels are 0-3
#define DPX_ERR_DAC_BUFF_BAD_CHANNEL			-1505	// Valid channels are 0-3
#define DPX_ERR_DAC_BUFF_ODD_BASEADDR			-1506	// An odd buffer base was requested
#define DPX_ERR_DAC_BUFF_BASEADDR_TOO_HIGH		-1507	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DAC_BUFF_ODD_READADDR			-1508	// An odd buffer read address was requested
#define DPX_ERR_DAC_BUFF_READADDR_TOO_HIGH		-1509	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_DAC_BUFF_ODD_WRITEADDR			-1510	// An odd buffer write address was requested
#define DPX_ERR_DAC_BUFF_WRITEADDR_TOO_HIGH		-1511	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_DAC_BUFF_ODD_SIZE				-1512	// An odd buffer size was requested
#define DPX_ERR_DAC_BUFF_TOO_BIG				-1513	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DAC_SCHED_TOO_FAST				-1514	// The requested schedule rate is too fast
#define DPX_ERR_DAC_SCHED_BAD_RATE_UNITS		-1515	// Unnrecognized schedule rate units parameter

#define DPX_ERR_ADC_GET_BAD_CHANNEL				-1600	// Valid channels are 0-17
#define DPX_ERR_ADC_RANGE_NULL_PTR				-1601	// A pointer argument was null
#define DPX_ERR_ADC_RANGE_BAD_CHANNEL			-1602	// Valid channels are 0-17
#define DPX_ERR_ADC_REF_BAD_CHANNEL				-1603	// Valid channels are 0-15
#define DPX_ERR_ADC_BAD_CHAN_REF				-1604	// Unrecognized channel reference parameter
#define DPX_ERR_ADC_BUFF_BAD_CHANNEL			-1605	// Valid channels are 0-15
#define DPX_ERR_ADC_BUFF_ODD_BASEADDR			-1606	// An odd buffer base was requested
#define DPX_ERR_ADC_BUFF_BASEADDR_TOO_HIGH		-1607	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_ADC_BUFF_ODD_READADDR			-1608	// An odd buffer read address was requested
#define DPX_ERR_ADC_BUFF_READADDR_TOO_HIGH		-1609	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_ADC_BUFF_ODD_WRITEADDR			-1610	// An odd buffer write address was requested
#define DPX_ERR_ADC_BUFF_WRITEADDR_TOO_HIGH		-1611	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_ADC_BUFF_ODD_SIZE				-1612	// An odd buffer size was requested
#define DPX_ERR_ADC_BUFF_TOO_BIG				-1613	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_ADC_SCHED_TOO_FAST				-1614	// The requested schedule rate is too fast
#define DPX_ERR_ADC_SCHED_BAD_RATE_UNITS		-1615	// Unnrecognized schedule rate units parameter

#define DPX_ERR_DOUT_SET_BAD_MASK				-1700	// Valid masks set bits 23 downto 0
#define DPX_ERR_DOUT_BUFF_ODD_BASEADDR			-1701	// An odd buffer base was requested
#define DPX_ERR_DOUT_BUFF_BASEADDR_TOO_HIGH		-1702	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DOUT_BUFF_ODD_READADDR			-1703	// An odd buffer read address was requested
#define DPX_ERR_DOUT_BUFF_READADDR_TOO_HIGH		-1704	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_DOUT_BUFF_ODD_WRITEADDR			-1705	// An odd buffer write address was requested
#define DPX_ERR_DOUT_BUFF_WRITEADDR_TOO_HIGH	-1706	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_DOUT_BUFF_ODD_SIZE				-1707	// An odd buffer size was requested
#define DPX_ERR_DOUT_BUFF_TOO_BIG				-1708	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DOUT_SCHED_TOO_FAST				-1709	// The requested schedule rate is too fast
#define DPX_ERR_DOUT_SCHED_BAD_RATE_UNITS		-1710	// Unnrecognized schedule rate units parameter

#define DPX_ERR_DIN_SET_BAD_MASK				-1800	// Valid masks set bits 23 downto 0
#define DPX_ERR_DIN_BUFF_ODD_BASEADDR			-1801	// An odd buffer base was requested
#define DPX_ERR_DIN_BUFF_BASEADDR_TOO_HIGH		-1802	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DIN_BUFF_ODD_READADDR			-1803	// An odd buffer read address was requested
#define DPX_ERR_DIN_BUFF_READADDR_TOO_HIGH		-1804	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_DIN_BUFF_ODD_WRITEADDR			-1805	// An odd buffer write address was requested
#define DPX_ERR_DIN_BUFF_WRITEADDR_TOO_HIGH		-1806	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_DIN_BUFF_ODD_SIZE				-1807	// An odd buffer size was requested
#define DPX_ERR_DIN_BUFF_TOO_BIG				-1808	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_DIN_SCHED_TOO_FAST				-1809	// The requested schedule rate is too fast
#define DPX_ERR_DIN_SCHED_BAD_RATE_UNITS		-1810	// Unnrecognized schedule rate units parameter
#define DPX_ERR_DIN_BAD_STRENGTH				-1811	// Strength is in the range 0-1

#define DPX_ERR_AUD_SET_BAD_VALUE				-1900	// Value falls outside AUD's output range
#define DPX_ERR_AUD_SET_BAD_VOLUME				-1901	// Valid volumes are in the range 0-1
#define DPX_ERR_AUD_SET_BAD_LRMODE				-1902	// See DPxSetAudLRMode() for valid values
#define DPX_ERR_AUD_BUFF_ODD_BASEADDR			-1903	// An odd buffer base was requested
#define DPX_ERR_AUD_BUFF_BASEADDR_TOO_HIGH		-1904	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_AUD_BUFF_ODD_READADDR			-1905	// An odd buffer read address was requested
#define DPX_ERR_AUD_BUFF_READADDR_TOO_HIGH		-1906	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_AUD_BUFF_ODD_WRITEADDR			-1907	// An odd buffer write address was requested
#define DPX_ERR_AUD_BUFF_WRITEADDR_TOO_HIGH		-1908	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_AUD_BUFF_ODD_SIZE				-1909	// An odd buffer size was requested
#define DPX_ERR_AUD_BUFF_TOO_BIG				-1910	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_AUX_BUFF_ODD_BASEADDR			-1911	// An odd buffer base was requested
#define DPX_ERR_AUX_BUFF_BASEADDR_TOO_HIGH		-1912	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_AUX_BUFF_ODD_READADDR			-1913	// An odd buffer read address was requested
#define DPX_ERR_AUX_BUFF_READADDR_TOO_HIGH		-1914	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_AUX_BUFF_ODD_WRITEADDR			-1915	// An odd buffer write address was requested
#define DPX_ERR_AUX_BUFF_WRITEADDR_TOO_HIGH		-1916	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_AUX_BUFF_ODD_SIZE				-1917	// An odd buffer size was requested
#define DPX_ERR_AUX_BUFF_TOO_BIG				-1918	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_AUD_SCHED_TOO_FAST				-1919	// The requested schedule rate is too fast
#define DPX_ERR_AUD_SCHED_TOO_SLOW				-1920	// The requested schedule rate is too slow
#define DPX_ERR_AUD_SCHED_BAD_RATE_UNITS		-1921	// Unnrecognized schedule rate units parameter
#define DPX_ERR_AUD_CODEC_POWERUP				-1922	// The CODEC didn't set its internal powerup bits.

#define DPX_ERR_MIC_SET_GAIN_TOO_LOW			-2000	// See DPxSetMicSource() for valid values
#define DPX_ERR_MIC_SET_GAIN_TOO_HIGH			-2001	// See DPxSetMicSource() for valid values
#define DPX_ERR_MIC_SET_BAD_SOURCE				-2002	// See DPxSetMicSource() for valid values
#define DPX_ERR_MIC_SET_BAD_LRMODE				-2003	// See DPxSetMicLRMode() for valid values
#define DPX_ERR_MIC_BUFF_ODD_BASEADDR			-2004	// An odd buffer base was requested
#define DPX_ERR_MIC_BUFF_BASEADDR_TOO_HIGH		-2005	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_MIC_BUFF_ODD_READADDR			-2006	// An odd buffer read address was requested
#define DPX_ERR_MIC_BUFF_READADDR_TOO_HIGH		-2007	// The requested read address exceeds the DATAPixx RAM
#define DPX_ERR_MIC_BUFF_ODD_WRITEADDR			-2008	// An odd buffer write address was requested
#define DPX_ERR_MIC_BUFF_WRITEADDR_TOO_HIGH		-2009	// The requested write address exceeds the DATAPixx RAM
#define DPX_ERR_MIC_BUFF_ODD_SIZE				-2010	// An odd buffer size was requested
#define DPX_ERR_MIC_BUFF_TOO_BIG				-2011	// The requested buffer is larger than the DATAPixx RAM
#define DPX_ERR_MIC_SCHED_TOO_FAST				-2012	// The requested schedule rate is too fast
#define DPX_ERR_MIC_SCHED_BAD_RATE_UNITS		-2013	// Unnrecognized schedule rate units parameter

#define DPX_ERR_VID_SET_BAD_MODE				-2100	// See DPxSetVidMode() for valid values
#define DPX_ERR_VID_CLUT_WRITE_USB_ERROR		-2101	// A USB error occurred while writing a video CLUT
#define DPX_ERR_VID_VSYNC_USB_ERROR				-2102	// A USB error occurred while waiting for vertical sync
#define DPX_ERR_VID_EDID_WRITE_USB_ERROR		-2103	// A USB error occurred while writing f data
#define DPX_ERR_VID_LINE_READ_USB_ERROR			-2104	// A USB error occurred while reading the video line buffer
#define DPX_ERR_VID_PSYNC_NPIXELS_ARG_ERROR		-2105	// Pixel sync nPixels argument must be in the range 1-8
#define DPX_ERR_VID_PSYNC_TIMEOUT_ARG_ERROR		-2106	// Pixel sync timeout argument must be in the range 0-65535
#define DPX_ERR_VID_PSYNC_LINE_ARG_ERROR		-2107	// Pixel sync raster line argument must be in the range 0-4095
#define DPX_ERR_VID_ALPHA_WRITE_USB_ERROR		-2108	// A USB error occurred while writing video horizontal overlay alpha data
#define DPX_ERR_VID_BASEADDR_ALIGN_ERROR		-2109	// The requested base address was not aligned on a 64kB boundary
#define DPX_ERR_VID_BASEADDR_TOO_HIGH           -2110	// The requested base address exceeds the DATAPixx RAM
#define DPX_ERR_VID_VSYNC_WITHOUT_VIDEO         -2111   // The API was told to block until VSYNC; but DATAPixx is not receiving any video
#define DPX_ERR_VID_BL_INTENSITY_ARG_ERROR      -2112   // Backlight intensity argument must be in the range 0-255
#define DPX_ERR_VID_GREY_MODE_ARG_ERROR         -2113   // Invalid greyscale mode
#define DPX_ERR_VID_LED_MASK_ARG_ERROR          -2114   // Invalid RGB LED Mask

#define DPX_ERR_PPX_BAD_VOLTAGE                 -3000   // Unnrecognized voltage monitor. Argument is not in the range 0 to 6
#define DPX_ERR_PPX_BAD_TEMP                    -3001   // Unnrecognized temperature monitor. Argument is not in the range 0 to 9
#define DPX_ERR_PPX_BAD_LED                     -3002   // Unnrecognized LED. Argument is not in the range 0 to 7
#define DPX_ERR_PPX_BAD_FAN                     -3003   // Unnrecognized fan. Argument is not in the range 0 to 5
#define DPX_ERR_PPX_BAD_LED_CURRENT             -3004   // Invalid LED current. Argument is not in the range 0 to 50
#define DPX_ERR_PPX_SEQ_WRITE_USB_ERROR         -3005	// A USB error occurred while writing a video sequence
#define DPX_ERR_PPX_SWTP_LOAD_PAGE_BAD_VALUE	-3006	// Invalid page number current. Argument is not in the range 0 to 127

#define DPX_ERR_PPX_NO_HIGH_BIT_DEPTH_CAL_FOUND -3050	// No High Bit Depth Calibration found
#define DPX_ERR_PPX_UNSUPPORTED_DLP_SEQ_PGRM    -3051	// Sequencer Program not supported this PROPixx

#define DPX_ERR_PPX_TSCOPE_SCHED_TOO_FAST		-3100	// The requested schedule rate is too fast
#define DPX_ERR_PPX_TSCOPE_SCHED_BAD_RATE_UNITS	-3101	// Unnrecognized schedule rate units parameter

#define DPX_ERR_TRK_BUFF_ODD_BASEADDR			-3200	// An odd buffer base was requested
#define DPX_ERR_TRK_BUFF_BASEADDR_TOO_HIGH		-3201   // The requested buffer is larger than the TRACKPixx RAM
#define DPX_ERR_TRK_BUFF_ODD_READADDR			-3202	// An odd buffer read address was requested
#define DPX_ERR_TRK_BUFF_READADDR_TOO_HIGH		-3203	// The requested read address exceeds the TRACKPixx RAM
#define DPX_ERR_TRK_BUFF_ODD_WRITEADDR			-3204	// An odd buffer write address was requested
#define DPX_ERR_TRK_BUFF_WRITEADDR_TOO_HIGH		-3205	// The requested write address exceeds the TRACKPixx RAM
#define DPX_ERR_TRK_BUFF_ODD_SIZE				-3206	// An odd buffer size was requested
#define DPX_ERR_TRK_BUFF_TOO_BIG				-3207	// The requested buffer is larger than the TRACKPixx RAM
#define DPX_ERR_TRK_BUFF_BASEADDR_TOO_LOW		-3210   // The requested buffer is too low. Address should be higher than 0x140000



#ifdef __cplusplus
}
#endif

#endif

