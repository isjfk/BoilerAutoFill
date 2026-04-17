#ifndef BoilerAutoFillUI_H
#define BoilerAutoFillUI_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void lvglSetPressureRange(float low, float high);
void lvglSetPressure(float newPressure);
void lvgl_clean_screen();
void lvgl_baf_ui(lv_disp_t *disp);

#ifdef __cplusplus
}
#endif

#endif // BoilerAutoFillUI_H