#pragma once
#include <stdint.h>

void flipout();
void in();
void stopRollers();
void shootClean(uint8_t balls);
void shootStaggered(uint8_t balls);
void shootStaggeredIntake(uint8_t balls);

extern bool suspendDrive;
