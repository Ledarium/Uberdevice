#include "game.h"

extern GameEngine game;

void InitGameEngine() {
    game.timerValue = game.turnTime;
	game.countScores = true;
	game.activePlayers = 0;
	game.currentPlayer = 1;
	game.turnTime = TIMER_STEP;
	game.timerValue = game.turnTime;
    for (int i=0; i<MAX_PLAYERS+1; i++)
        game.scores[i] = 999;
}

void DecrementTurnTime() {
    if (game.turnTime > TIMER_STEP)
        game.turnTime -= TIMER_STEP;
    else 
        game.turnTime = TIMER_MAX;
    game.timerValue = game.turnTime;
}

void IncrementTurnTime() {
    if (game.turnTime + TIMER_STEP <= TIMER_MAX)
        game.turnTime += TIMER_STEP;
    else
        game.turnTime = TIMER_STEP;
    game.timerValue = game.turnTime;
}

void AddPlayer() {
    if (game.activePlayers < MAX_PLAYERS) {
        game.scores[(1+game.activePlayers++)] = 0;
    }
}	

void RemovePlayer() {
    if (game.activePlayers > 1) {
        game.scores[1+(--game.activePlayers)] = 999;
    }
}

void ChangeScore(int8_t delta) {
    game.scores[game.currentPlayer] += delta;
}

void ResetTurnTimer() {
    game.timerValue = game.turnTime;
}

void NextPlayer() {
    game.currentPlayer--;
    game.currentPlayer = ((game.currentPlayer + 1) % game.activePlayers);
    game.currentPlayer++;
    ResetTurnTimer();
}