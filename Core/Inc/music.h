#ifndef __MUSIC_H
#define __MUSIC_H

#include <stdint.h>

#define MAX_TONEGENS 6
#define MAX_CHANNELS 2

typedef enum returnCodes {
    OK,
    STOPPED,
    WRONG_HEADER,
    OUT_OF_LOOP,
    WRONG_BYTE
} RET_CODE;

typedef struct {
	uint8_t *begin;
	uint32_t size;
} Track;

RET_CODE MusicPlay(Track *track);
void MusicStop(void);
#endif