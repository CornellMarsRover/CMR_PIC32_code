#include <GenericTypeDefs.h>
#include "tft_gfx.h"
#include "tft_master.h"
#include "MDD_includes/FSIO.h"
#define debugbmp  0

#define CMR "CMR_logo.bmp"


static UINT32 spicon_fs, spicon_tft;
static UINT32 spicon2_fs, spicon2_tft;

INT8 readBMP(char image[], unsigned short x, unsigned short y);
void SD_Config();
inline UINT16 im8to16(UINT8 c);
void readBMP24(FSFILE * pointer, unsigned short x, unsigned short y);
void readBMP_not24(FSFILE * pointer);