#include "SD_Card.h"
#include <GenericTypeDefs.h>
#include "MDD_includes/FSIO.h"
#include "tft_gfx.h"
#include "tft_master.h"




void SD_Config() {//resets config and status registers so that SD card transaction can occur
    SPI1CON = spicon_fs;
    SPI1CON2 = spicon2_fs;
    SPISTAT = 0;
}

inline UINT16 im8to16(UINT8 c) {
    // 8-bit format: R R R G G G B B
    // 16-bit format: 5R, 6G, 5B
    return ( (c & 0xE0) << 8
            | (c & 0x1C) << 6
            | (c & 0x03) << 3);
}


UINT32 HRES, VRES; // resolution, horizontal and vertical
UINT32 imgdataSize;
UINT32 BPP; // bits per pixel
UINT8 compression; // 0 = no compression, 1 = RLE-8, 2 = RLE-4
UINT32 start_offset;

void readBMP24(FSFILE * pointer, unsigned short x, unsigned short y) {
    UINT8 dataStream[720];
    UINT32 nr, nc;
    UINT32 rowSize = 4 - ((HRES * 3) % 4);
    if (rowSize == 4) rowSize = HRES * 3;
    else rowSize = (HRES * 3) + rowSize;

    // for now assume compression = 0

    tft_setAddrWindow(x, y, x + HRES - 1, y + VRES - 1);

    for (nr = 0; nr < VRES; nr++) {
        SD_Config();
        Mode8();
        //        if (FSfread(dataStream, 1, rowSize, pointer) != rowSize)
        if (FSfread(dataStream, rowSize, 1, pointer) != 1)
            break;
        for (nc = 0; nc < HRES * 3; nc += 3) {
            tft_pushColor(tft_Color565(dataStream[nc + 2], // R
                    dataStream[nc + 1], // G
                    dataStream[nc])); // B
        }
    }
}

void readBMP_not24(FSFILE * pointer) {

}

INT8 readBMP(char image[], unsigned short x, unsigned short y) {
    UINT8 bmpStream[200];
    char textBuffer[50];
    UINT32 temp32;
    FSFILE * pointer = NULL;

    if (debugbmp) {
        tft_setCursor(0, 0);
        tft_setTextSize(1);
        tft_setTextColor(GREEN);
        tft_writeString("In readBMP_header\n");
        tft_writeString("Opening \"img.bmp\"\n");


    }
    SD_Config();
    Mode8();
    pointer = FSfopen(image, "r");
    if (pointer == NULL) {
        if (debugbmp) {
            tft_writeString("Failed to open \"img.bmp\"\n");
        }
        return -1;
    }
    if (debugbmp) {
        tft_writeString("Opened \"img.bmp\"\n");
        SD_Config();
        Mode8();
    }
    if (FSfread(bmpStream, 1, 14, pointer) != 14) {
        FSfclose(pointer);
        return -1;
    }
    if (debugbmp) {
        tft_writeString("Read 14 bytes\n");
        SD_Config();
    }

    if ((bmpStream[0] != 'B') & (bmpStream[1] != 'M')) {
        Mode8();
        FSfclose(pointer);
        tft_writeString("Not BMP file\n");
        return -1;
    }
    if (debugbmp) {
        tft_writeString("BMP file confirmed\n");
        SD_Config();
    }

    temp32 = bmpStream[5] << 24 | bmpStream[4] << 16 |
            bmpStream[3] << 8 | bmpStream[2];

    if (debugbmp) {
        sprintf(textBuffer, "Size = %u bytes\n", temp32);
        tft_writeString(textBuffer);
    }

    temp32 = bmpStream[13] << 24 | bmpStream[12] << 16 |
            bmpStream[11] << 8 | bmpStream[10];
    if (debugbmp) {
        sprintf(textBuffer, "Start offset = %u\n", temp32);
        tft_writeString(textBuffer);
    }
    start_offset = temp32;


    SD_Config();
    Mode8();
    FSfseek(pointer, 0, SEEK_SET);
    if (FSfread(bmpStream, 1, start_offset, pointer) != start_offset)
        return -1;

    temp32 = bmpStream[17] << 24 | bmpStream[16] << 16 |
            bmpStream[15] << 8 | bmpStream[14];
    if (debugbmp) {
        sprintf(textBuffer, "Bitmap info header size = %u\n", temp32);
        tft_writeString(textBuffer);
    }
    temp32 = bmpStream[21] << 24 | bmpStream[20] << 16 |
            bmpStream[19] << 8 | bmpStream[18];
    if (debugbmp) {
        sprintf(textBuffer, "Image width = %u pixels\n", temp32);
        tft_writeString(textBuffer);
    }
    HRES = temp32;

    temp32 = bmpStream[25] << 24 | bmpStream[24] << 16 |
            bmpStream[23] << 8 | bmpStream[22];
    if (debugbmp) {
        sprintf(textBuffer, "Image height = %u pixels\n", temp32);
        tft_writeString(textBuffer);
    }
    VRES = temp32;

    temp32 = bmpStream[27] << 8 | bmpStream[26];
    if (debugbmp) {
        sprintf(textBuffer, "Number of planes in image = %u\n", temp32);
        tft_writeString(textBuffer);
    }

    temp32 = bmpStream[29] << 8 | bmpStream[28];
    if (debugbmp) {
        sprintf(textBuffer, "Bits per pixel = %u\n", temp32);
        tft_writeString(textBuffer);
    }
    BPP = temp32;

    temp32 = bmpStream[33] << 24 | bmpStream[32] << 16 |
            bmpStream[31] << 8 | bmpStream[30];
    if (debugbmp) {
        sprintf(textBuffer, "Compression type = %u\n", temp32);
        // 0 = no compression, 1 = RLE-8, 2 = RLE-4
        tft_writeString(textBuffer);
    }
    compression = temp32;

    temp32 = bmpStream[37] << 24 | bmpStream[36] << 16 |
            bmpStream[35] << 8 | bmpStream[34];
    if (debugbmp) {
        sprintf(textBuffer, "Data Size (with padding) = %u\n", temp32);
        tft_writeString(textBuffer);
    }
    imgdataSize = temp32;

    temp32 = bmpStream[49] << 24 | bmpStream[48] << 16 |
            bmpStream[47] << 8 | bmpStream[46];
    if (debugbmp) {
        sprintf(textBuffer, "Number of colors in image = %u\n", temp32);
        tft_writeString(textBuffer);
    }

    temp32 = bmpStream[53] << 24 | bmpStream[52] << 16 |
            bmpStream[51] << 8 | bmpStream[50];
    if (debugbmp) {
        sprintf(textBuffer, "Number of important colors = %u\n", temp32);
        tft_writeString(textBuffer);
    }
    //    delay_ms(1000);


    SD_Config();
    Mode8();
    FSfseek(pointer, start_offset, SEEK_SET); // go to beginning of data

    if (BPP == 24) {
        //        tft_fillScreen(ILI9340_BLACK);
        readBMP24(pointer, x, y);
    } else {
        //        tft_fillScreen(ILI9340_BLACK);
        readBMP_not24(pointer);
    }

    SPISTAT = 0;
    Mode8();
    FSfclose(pointer);
    return 0;
}
