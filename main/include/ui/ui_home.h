#ifndef UI_HOME_H
#define UI_HOME_H

/*********************
 *      INCLUDES
 *********************/
#include "esp_lvgl_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**********************
     * GLOBAL PROTOTYPES
     **********************/
    void ui_home_init(void);
    void ui_home_destroy(void);

#ifdef __cplusplus
    extern "C"
}
#endif

#endif /* UI_HOME_H */