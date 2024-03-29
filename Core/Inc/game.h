#ifndef __GAME_H
#define __GAME_H
#define MAX_PLAYERS 6
#define TIMER_MAX 180
#define TIMER_STEP 10

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool countScores;
	uint8_t activePlayers;
	uint8_t currentPlayer;
	uint32_t turnTime;
	int32_t timerValue;
	uint32_t scores[MAX_PLAYERS+1];
} GameEngine;

void InitGameEngine();
void IncrementTurnTime();
void DecrementTurnTime();
void AddPlayer();
void RemovePlayer();
uint32_t GetCurrentPlayer();
uint32_t GetTimerValue();
void ChangeScore(int8_t delta);
void ResetTurnTimer();
void NextPlayer();
#endif