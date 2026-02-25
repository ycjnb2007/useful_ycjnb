#ifndef _PARAMS_SAVE_H_
#define _PARAMS_SAVE_H_

#include "zf_common_headfile.h"

void save_pid_to_flash(float params1, float params2, int16 params3);
void read_pid_from_flash(float *params1, float *params2, int16 *params3);
void load_params_from_flash(void);
#endif