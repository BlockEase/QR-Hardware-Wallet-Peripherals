/*********************
 *      INCLUDES
 *********************/
#include "ui/ui_home.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "esp_log.h"
#include "ui/ui_toast.h"
#include "ui/ui_loading.h"
#include "app.h"
#include "freertos/FreeRTOS.h"
#include "esp_code_scanner.h"
#include "littlefs_utils.h"
#include "app_peripherals.h"
#include "app_oem.h"
#include "esp_camera.h"
#include "ui/ui_style.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "UI_HOME"
#define OEM_PARTITION_LABEL "oem"

/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
    STEP_OEM_IMAGE = 0,
    STEP_SCAN = 1,
    STEP_CONFIRM = 2,
} ui_home_step_t;

/* logo declare */
LV_IMG_DECLARE(oem)

/**********************
 *  STATIC VARIABLES
 **********************/
static ui_home_step_t step = STEP_SCAN;
static lv_obj_t *current_page = NULL;
static lv_obj_t *save_btn = NULL;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ui_event_handler(lv_event_t *e);
static void qrScannerTask(void *parameters);
static void saveConfigTask(void *parameters);
static void show_oem_image(void);
static void show_scan_page(void);
static void show_confirm_page(void);
static void save_config(void);
static void show_final_page(void);

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void ui_home_init(void);
void ui_home_destroy(void);

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void ui_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        if (step == STEP_OEM_IMAGE)
        {
            show_scan_page();
        }
        else if (step == STEP_SCAN)
        {
            show_confirm_page();
        }
        else if (step == STEP_CONFIRM)
        {
            save_config();
        }
    }
}
static void qrScannerTask(void *parameters)
{
    int width = 240;
    lv_obj_t *image = (lv_obj_t *)parameters;
    if (ESP_OK != app_camera_init())
    {
        return;
    }

    lv_img_dsc_t img_buffer = {
        .header.w = 0,
        .header.h = 0,
        .data_size = 0,
        .header.cf = LV_COLOR_FORMAT_RGB565,
        .data = NULL,
    };
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL)
    {
        ESP_LOGE(TAG, "camera get failed");
        vTaskDelete(NULL);
        return;
    }
    if (fb->width != width || fb->height != width)
    {
        ESP_LOGE(TAG, "camera not in 240x240");
        vTaskDelete(NULL);
        return;
    }
    uint8_t *swap_buf = NULL;
    uint8_t *line_buf = NULL;
    int line_size = width * 2; // RGB565 2 bytes per pixel
    if (CAMERA_SWAP_X || CAMERA_SWAP_Y)
    {
        swap_buf = (uint8_t *)malloc(line_size * width);
        if (!CAMERA_SWAP_Y)
        {
            line_buf = (uint8_t *)malloc(line_size);
        }
    }

    img_buffer.header.w = fb->width;
    img_buffer.header.h = fb->height;
    img_buffer.data_size = line_size * width; // fb->len;

    esp_code_scanner_config_t config = {ESP_CODE_SCANNER_MODE_FAST, ESP_CODE_SCANNER_IMAGE_RGB565, fb->width, fb->height};
    esp_camera_fb_return(fb);

    bool scan_success = false;
    while (!scan_success)
    {
        fb = esp_camera_fb_get();
        if (fb == NULL)
        {
            ESP_LOGE(TAG, "camera get failed");
            continue;
        }
        if (CAMERA_SWAP_X || CAMERA_SWAP_Y)
        {
            int new_x;
            int new_y;
            if (CAMERA_SWAP_X && CAMERA_SWAP_Y)
            {
                /*
                 (0,0) -> (239,239)
                 (0,1) -> (239,238)
                 ...
                 (239,238) -> (0,1)
                 (239,239) -> (0,0)
                 */
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < width; y++)
                    {
                        new_x = width - x - 1;
                        new_y = width - y - 1;
                        memcpy(swap_buf + new_x * line_size + new_y, fb->buf + x * line_size + y, 2);
                    }
                }
            }
            else if (CAMERA_SWAP_X)
            {
                /*
                 (0,0) -> (239,0)
                 (0,1) -> (239,1)
                 ...
                 (239,238) -> (0,238)
                 (239,239) -> (0,239)
                 */
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < width; y++)
                    {
                        new_x = width - x - 1;
                        memcpy(swap_buf + new_x * line_size + y, fb->buf + x * line_size + y, 2);
                    }
                }
            }
            else if (CAMERA_SWAP_Y)
            {
                /*
               (0,0) -> (0,239)
               (0,1) -> (0,238)
               ...
               (239,238) -> (239,1)
               (239,239) -> (239,0)
               */
                for (int y = 0; y < width; y++)
                {
                    memcpy(swap_buf + y * line_size, fb->buf + (width - y - 1) * line_size, line_size);
                }
            }
            img_buffer.data = swap_buf;
        }
        else
        {
            img_buffer.data = fb->buf;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        if (lvgl_port_lock(0))
        {
            lv_img_set_src(image, &img_buffer);
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        // Decode Progress
        esp_image_scanner_t *esp_scn = esp_code_scanner_create();
        esp_code_scanner_set_config(esp_scn, config);
        int decoded_num = esp_code_scanner_scan_image(esp_scn, img_buffer.data);
        if (decoded_num)
        {
            esp_code_scanner_symbol_t result = esp_code_scanner_result(esp_scn);
            if (result.data != NULL && strlen(result.data) > 0)
            {
                scan_success = true;
                ESP_LOGI(TAG, "scan result:%s", result.data);
            }
        }
        /* esp_code_scanner_symbol_t unavailable after esp_code_scanner_destroy */
        esp_code_scanner_destroy(esp_scn);
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    show_confirm_page();

    vTaskDelete(NULL);
}
static void saveConfigTask(void *parameters)
{
    if (!littlefs_format(OEM_PARTITION_LABEL))
    {
        ui_toast_show("Failed to format LittleFS", 0);
        goto done;
    }
    bool ret = app_peripherals_write(OEM_PARTITION_LABEL);
    if (ret)
    {
        // save OEM image
        if (sizeof(oem.data) > 1024 * 512)
        {
            ui_toast_show("OEM image is too large(max 512KB)", 0);
            goto done;
        }
        oem_image_t *oem_image = (oem_image_t *)malloc(sizeof(oem_image_t));
        memset(oem_image, 0, sizeof(oem_image_t));
        oem_image->header = oem.header;
        oem_image->data_size = oem.data_size;
        memcpy(oem_image->data, oem.data, oem_image->data_size);

        bool ret = app_oem_write(OEM_PARTITION_LABEL, oem_image);
        free(oem_image);
        oem_image = NULL;

        if (!ret)
        {
            ui_toast_show("Failed to save oem image", 0);
            goto done;
        }
        show_final_page();
        goto done;
    }
    else
    {
        ui_toast_show("failed to save current config", 0);
        goto done;
    }

done:
    ui_loading_hide();
    vTaskDelete(NULL);
}
static void show_oem_image(void)
{
    /*
         oem image page
      ┌─────────────────┐
      │                 │
      │ ┌─────────────┐ │
      │ │             │ │
      │ │  OEM image  │ │
      │ │             │ │
      │ └─────────────┘ │
      │   ┌───────┐     │
      │   │button │     │
      │   └───────┘     │
      └─────────────────┘
      */

    step = STEP_OEM_IMAGE;
    if (lvgl_port_lock(0))
    {
        lv_obj_t *screen = lv_scr_act();
        int32_t screen_w = lv_obj_get_width(screen);
        int32_t screen_h = lv_obj_get_height(screen);
        current_page = lv_obj_create(screen);
        NO_BODER_PADDING_STYLE(current_page);
        lv_obj_set_size(current_page, screen_w, screen_h);
        lv_obj_center(current_page);
        lv_obj_set_flex_flow(current_page, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(current_page, LV_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *oem_icon = lv_img_create(current_page);
        lv_img_set_src(oem_icon, &oem);
        lv_obj_set_size(oem_icon, 200, 200);

        lv_obj_t *nextStep = lv_button_create(current_page);
        lv_obj_set_size(nextStep, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_t *label = lv_label_create(nextStep);
        lv_label_set_text(label, "Next");
        lv_obj_add_event_cb(nextStep, ui_event_handler, LV_EVENT_CLICKED, NULL);

        lvgl_port_unlock();
    }
}
static void show_scan_page(void)
{
    /*
        scan page
     ┌──────────────┐
     │              │
     │ xxxxxxxxx    │
     │              │
     │┌────────────┐│
     ││            ││
     ││ QR preview ││
     ││            ││
     │└────────────┘│
     │              │
     └──────────────┘
      */

    step = STEP_SCAN;
    if (lvgl_port_lock(0))
    {
        lv_obj_del(current_page);
        current_page = NULL;

        lv_obj_t *screen = lv_scr_act();
        int32_t screen_w = lv_obj_get_width(screen);
        int32_t screen_h = lv_obj_get_height(screen);
        current_page = lv_obj_create(screen);
        NO_BODER_PADDING_STYLE(current_page);
        lv_obj_set_size(current_page, screen_w, screen_h);
        lv_obj_center(current_page);
        lv_obj_set_flex_flow(current_page, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(current_page, LV_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *label = lv_label_create(current_page);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_label_set_text(label, "Scan any QR code to continue");

        lv_obj_t *preview_image = lv_image_create(current_page);
        lv_obj_set_size(preview_image, 240, 240);
        lv_obj_center(preview_image);

        xTaskCreate(qrScannerTask, "qrScannerTask", 4 * 1024, preview_image, 10, NULL);

        lvgl_port_unlock();
    }
}
static void show_confirm_page(void)
{
    /*
       confirm page
     ┌──────────────┐
     │              │
     │ xxxxxx       │
     │              │
     │              │
     │              │
     │              │
     │  ┌───────┐   │
     │  │confrim│   │
     │  └───────┘   │
     └──────────────┘
      */
    step = STEP_CONFIRM;
    if (lvgl_port_lock(0))
    {
        lv_obj_del(current_page);
        current_page = NULL;

        lv_obj_t *screen = lv_scr_act();
        int32_t screen_w = lv_obj_get_width(screen);
        int32_t screen_h = lv_obj_get_height(screen);
        current_page = lv_obj_create(screen);
        NO_BODER_PADDING_STYLE(current_page);
        lv_obj_set_size(current_page, screen_w, screen_h);
        lv_obj_center(current_page);
        lv_obj_set_flex_flow(current_page, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(current_page, LV_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *list = lv_list_create(current_page);
        NO_BODER_PADDING_STYLE(list);
        lv_obj_set_size(list, screen_w, LV_SIZE_CONTENT);
        lv_list_add_text(list, "Passed!");
        lv_list_add_button(list, LV_SYMBOL_OK, "LCD check");
        lv_list_add_button(list, LV_SYMBOL_OK, "Touch panel check");
        lv_list_add_button(list, LV_SYMBOL_OK, "Camera module check");

        save_btn = lv_button_create(current_page);
        lv_obj_set_size(save_btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_t *label = lv_label_create(save_btn);
        lv_label_set_text(label, "Save");
        lv_obj_add_event_cb(save_btn, ui_event_handler, LV_EVENT_CLICKED, NULL);

        lvgl_port_unlock();
    }
}
static void save_config(void)
{
    if (lvgl_port_lock(0))
    {
        lv_obj_add_state(save_btn, LV_STATE_DISABLED);
        lvgl_port_unlock();
    }
    ui_loading_show();
    xTaskCreate(saveConfigTask, "saveConfigTask", 4 * 1024, NULL, 10, NULL);
}
static void show_final_page(void)
{
    if (lvgl_port_lock(0))
    {
        lv_obj_del(current_page);
        current_page = NULL;

        lv_obj_t *screen = lv_scr_act();
        int32_t screen_w = lv_obj_get_width(screen);
        int32_t screen_h = lv_obj_get_height(screen);
        current_page = lv_obj_create(screen);
        NO_BODER_PADDING_STYLE(current_page);
        lv_obj_set_style_pad_all(current_page, 10, 0);
        lv_obj_set_size(current_page, screen_w, screen_h);
        lv_obj_center(current_page);
        lv_obj_set_flex_flow(current_page, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(current_page, LV_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        oem_image_t *oem_image = NULL;
        if (!app_oem_read(OEM_PARTITION_LABEL, &oem_image))
        {
            ui_toast_show("Failed to read oem image", 0);
            return;
        }
        lv_image_dsc_t oem_cpy;
        oem_cpy.header = oem_image->header;
        oem_cpy.data_size = oem_image->data_size;
        uint8_t *img_data = (uint8_t *)malloc(oem_cpy.data_size);
        memcpy(img_data, oem_image->data, oem_cpy.data_size);
        oem_cpy.data = img_data;
        free(oem_image);
        oem_image = NULL;

        {
            /*
            const lv_image_dsc_t oem = {
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.cf = LV_COLOR_FORMAT_RGB565A8,
  .header.flags = 0,
  .header.w = 115,
  .header.h = 133,
  .header.stride = 230,
  .data_size = sizeof(oem_map),
  .data = oem_map,
};
             */
            ESP_LOGI(TAG, "oem_cpy.header.magic: %d", (int)oem_cpy.header.magic);
            ESP_LOGI(TAG, "oem_cpy.header.cf: %d", (int)oem_cpy.header.cf);
            ESP_LOGI(TAG, "oem_cpy.header.flags: %d", (int)oem_cpy.header.flags);
            ESP_LOGI(TAG, "oem_cpy.header.w: %d", (int)oem_cpy.header.w);
            ESP_LOGI(TAG, "oem_cpy.header.h: %d", (int)oem_cpy.header.h);
            ESP_LOGI(TAG, "oem_cpy.header.stride: %d", (int)oem_cpy.header.stride);
            ESP_LOGI(TAG, "oem_cpy.data_size: %d", (int)oem_cpy.data_size);
        }

        lv_obj_t *oem_icon = lv_img_create(current_page);
        lv_img_set_src(oem_icon, &oem_cpy);
        lv_obj_set_size(oem_icon, 200, 200);

        lv_obj_t *label = lv_label_create(current_page);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_label_set_text(label, "Success!\n\n");

        label = lv_label_create(current_page);
        lv_obj_set_size(label, screen_w - 20, LV_SIZE_CONTENT);
        lv_label_set_text(label, "The OEM and peripherals configuration has been saved!\nYou can now disconnect the power and flash the firmware onto the dev board.");
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);

        lvgl_port_unlock();
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void ui_home_init(void)
{
    show_oem_image();
}
void ui_home_destroy(void)
{
}