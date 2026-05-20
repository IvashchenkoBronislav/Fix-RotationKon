#pragma once

void azimuthInit();
void azimuthUpdate();
int getAzimuthDegrees();
int getAzimuthHalfDegrees();
void setAzimuthDegrees(int value);
void adjustAzimuthDegrees(int delta);
void resetAzimuthSystem();
void startAzimuthMove(int targetDegrees);
void cancelAzimuthMove();
bool isAzimuthMoveActive();
bool isAzimuthMoveLocked();
int getTargetAzimuthDegrees();
int getTargetAzimuthHalfDegrees();
