#include "zf_common_headfile.h"


/***********************************ÏÞ·ùº¯Êý*************************************/
uint8 Limit_uint8(uint8 min, uint8 input, uint8 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

uint16 Limit_uint16(uint16 min, uint16 input, uint16 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

uint32 Limit_uint32(uint32 min, uint32 input, uint32 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

int32 Limit_int32(int32 min, int32 input, int32 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

float Limit_float(float min, float input, float max)
{
  return (input > max) ? max : (input < min) ? min : input;
}

double Limit_double(double min, double input, double max)
{
    return (input > max) ? max : (input < min) ? min : input;
}