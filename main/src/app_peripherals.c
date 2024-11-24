/*********************
 *      INCLUDES
 *********************/
#include "esp_log.h"
#include "esp_system.h"
#include "esp_camera.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "app_peripherals.h"
#include "peripherals.h"
#include "crc32.h"
#include "string.h"
#include "littlefs_utils.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "app_peripherals"

#define VERSION 1
#define OEM_CONFIG_VERSION_FILE "version.txt"
#define OEM_CONFIG_PERIPHERALS_FILE "peripherals.bin"

#if CONFIG_LCD_ST7735
#include "esp_lcd_panel_st7735.h"
#elif CONFIG_LCD_ST7789
#include "esp_lcd_panel_st7789.h"
#elif CONFIG_LCD_ST7796
#include "esp_lcd_panel_st7796.h"
#elif CONFIG_LCD_ILI9341
#include "esp_lcd_panel_ili9341.h"
#elif CONFIG_LCD_GC9A01
#include "esp_lcd_gc9a01.h"
#endif

#if CONFIG_TOUCH_CST816S
#include "esp_lcd_touch_cst816s.h"
#elif CONFIG_TOUCH_FT5X06
#include "esp_lcd_touch_ft5x06.h"
#elif CONFIG_TOUCH_FT6X36
#include "esp_lcd_touch_ft5x06.h"
#elif CONFIG_TOUCH_GT1151
#include "esp_lcd_touch_gt1151.h"
#elif CONFIG_TOUCH_GT911
#include "esp_lcd_touch_gt911.h"
#elif CONFIG_TOUCH_TT21100
#include "esp_lcd_touch_tt21100.h"
#endif

/*********************
 *      Camera Pins
 *********************/
#if CONFIG_CAMERA_MODULE_WROVER_KIT
#define CAMERA_MODULE_NAME "Wrover Kit"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 21
#define CAMERA_PIN_SIOD 26
#define CAMERA_PIN_SIOC 27

#define CAMERA_PIN_D7 35
#define CAMERA_PIN_D6 34
#define CAMERA_PIN_D5 39
#define CAMERA_PIN_D4 36
#define CAMERA_PIN_D3 19
#define CAMERA_PIN_D2 18
#define CAMERA_PIN_D1 5
#define CAMERA_PIN_D0 4
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 23
#define CAMERA_PIN_PCLK 22

#elif CONFIG_CAMERA_MODULE_ESP_EYE
#define CAMERA_MODULE_NAME "ESP-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 4
#define CAMERA_PIN_SIOD 18
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 36
#define CAMERA_PIN_D6 37
#define CAMERA_PIN_D5 38
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 35
#define CAMERA_PIN_D2 14
#define CAMERA_PIN_D1 13
#define CAMERA_PIN_D0 34
#define CAMERA_PIN_VSYNC 5
#define CAMERA_PIN_HREF 27
#define CAMERA_PIN_PCLK 25

#elif CONFIG_CAMERA_MODULE_ESP_S2_KALUGA
#define CAMERA_MODULE_NAME "ESP-S2-KALUGA"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 1
#define CAMERA_PIN_SIOD 8
#define CAMERA_PIN_SIOC 7

#define CAMERA_PIN_D7 38
#define CAMERA_PIN_D6 21
#define CAMERA_PIN_D5 40
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 42
#define CAMERA_PIN_D2 41
#define CAMERA_PIN_D1 37
#define CAMERA_PIN_D0 36
#define CAMERA_PIN_VSYNC 2
#define CAMERA_PIN_HREF 3
#define CAMERA_PIN_PCLK 33

#elif CONFIG_CAMERA_MODULE_ESP_S3_EYE
#define CAMERA_MODULE_NAME "ESP-S3-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1

#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 15

#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5

#define CAMERA_PIN_D0 11
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D7 16

#elif CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
#define CAMERA_MODULE_NAME "ESP-DEVCAM"
#define CAMERA_PIN_PWDN 32
#define CAMERA_PIN_RESET 33

#define CAMERA_PIN_XCLK 4
#define CAMERA_PIN_SIOD 18
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 36
#define CAMERA_PIN_D6 19
#define CAMERA_PIN_D5 21
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 35
#define CAMERA_PIN_D2 14
#define CAMERA_PIN_D1 13
#define CAMERA_PIN_D0 34
#define CAMERA_PIN_VSYNC 5
#define CAMERA_PIN_HREF 27
#define CAMERA_PIN_PCLK 25

#elif CONFIG_CAMERA_MODULE_M5STACK_PSRAM
#define CAMERA_MODULE_NAME "M5STACK-PSRAM"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET 15

#define CAMERA_PIN_XCLK 27
#define CAMERA_PIN_SIOD 25
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 19
#define CAMERA_PIN_D6 36
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 5
#define CAMERA_PIN_D2 34
#define CAMERA_PIN_D1 35
#define CAMERA_PIN_D0 32
#define CAMERA_PIN_VSYNC 22
#define CAMERA_PIN_HREF 26
#define CAMERA_PIN_PCLK 21

#elif CONFIG_CAMERA_MODULE_M5STACK_WIDE
#define CAMERA_MODULE_NAME "M5STACK-WIDE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET 15
#define CAMERA_PIN_XCLK 27
#define CAMERA_PIN_SIOD 22
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 19
#define CAMERA_PIN_D6 36
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 5
#define CAMERA_PIN_D2 34
#define CAMERA_PIN_D1 35
#define CAMERA_PIN_D0 32
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 26
#define CAMERA_PIN_PCLK 21

#elif CONFIG_CAMERA_MODULE_AI_THINKER
#define CAMERA_MODULE_NAME "AI-THINKER"
#define CAMERA_PIN_PWDN 32
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 0
#define CAMERA_PIN_SIOD 26
#define CAMERA_PIN_SIOC 27

#define CAMERA_PIN_D7 35
#define CAMERA_PIN_D6 34
#define CAMERA_PIN_D5 39
#define CAMERA_PIN_D4 36
#define CAMERA_PIN_D3 21
#define CAMERA_PIN_D2 19
#define CAMERA_PIN_D1 18
#define CAMERA_PIN_D0 5
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 23
#define CAMERA_PIN_PCLK 22

#elif CONFIG_CAMERA_MODULE_CUSTOM
#define CAMERA_MODULE_NAME CAMERA_MODULE_CUSTOM_NAME
#define CAMERA_PIN_PWDN CAMERA_MODULE_CUSTOM_PIN_PWDN
#define CAMERA_PIN_RESET CAMERA_MODULE_CUSTOM_PIN_RESET
#define CAMERA_PIN_XCLK CAMERA_MODULE_CUSTOM_PIN_XCLK
#define CAMERA_PIN_SIOD CAMERA_MODULE_CUSTOM_PIN_SIOD
#define CAMERA_PIN_SIOC CAMERA_MODULE_CUSTOM_PIN_SIOC

#define CAMERA_PIN_D7 CAMERA_MODULE_CUSTOM_PIN_Y9
#define CAMERA_PIN_D6 CAMERA_MODULE_CUSTOM_PIN_Y8
#define CAMERA_PIN_D5 CAMERA_MODULE_CUSTOM_PIN_Y7
#define CAMERA_PIN_D4 CAMERA_MODULE_CUSTOM_PIN_Y6
#define CAMERA_PIN_D3 CAMERA_MODULE_CUSTOM_PIN_Y5
#define CAMERA_PIN_D2 CAMERA_MODULE_CUSTOM_PIN_Y4
#define CAMERA_PIN_D1 CAMERA_MODULE_CUSTOM_PIN_Y3
#define CAMERA_PIN_D0 CAMERA_MODULE_CUSTOM_PIN_Y2
#define CAMERA_PIN_VSYNC CAMERA_MODULE_CUSTOM_PIN_VSYNC
#define CAMERA_PIN_HREF CAMERA_MODULE_CUSTOM_PIN_HREF
#define CAMERA_PIN_PCLK CAMERA_MODULE_CUSTOM_PIN_PCLK
#endif

/**********************
 *  STATIC VARIABLES
 **********************/

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t lcd_init(void);
static esp_err_t touch_init(void);
static esp_err_t lvgl_init(void);
static uint32_t checksum(peripherals_config_t *peripherals_config);
static bool write_peripherals_config(char *oem_partition_label, peripherals_config_t *config);

/**********************
 * GLOBAL PROTOTYPES
 **********************/
esp_err_t app_camera_init(void);
esp_err_t app_lvgl_init(void);
bool app_peripherals_read(char *oem_partition_label, int *version, peripherals_config_t **config);
bool app_peripherals_write(char *oem_partition_label);

/**********************
 *   STATIC FUNCTIONS
 **********************/
static esp_err_t lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << CONFIG_LCD_GPIO_BL};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = CONFIG_LCD_GPIO_SCLK,
        .mosi_io_num = CONFIG_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = CONFIG_LCD_H_RES * CONFIG_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CONFIG_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = CONFIG_LCD_GPIO_DC,
        .cs_gpio_num = CONFIG_LCD_GPIO_CS,
        .pclk_hz = CONFIG_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = CONFIG_LCD_CMD_BITS,
        .lcd_param_bits = CONFIG_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CONFIG_LCD_GPIO_RST,
        .color_space = CONFIG_LCD_COLOR_SPACE,
        .bits_per_pixel = CONFIG_LCD_BITS_PER_PIXEL,
    };
#if CONFIG_LCD_ST7735
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7735(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");
#elif CONFIG_LCD_ST7789
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");
#elif CONFIG_LCD_ST7796
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7796(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");
#elif CONFIG_LCD_ILI9341
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");
#elif CONFIG_LCD_GC9A01
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_gc9a01(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");
#endif

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, false, false);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_LCD_GPIO_BL, CONFIG_LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel)
    {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io)
    {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(CONFIG_LCD_SPI_NUM);
    return ret;
}
static esp_err_t touch_init(void)
{
    /* Initilize I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_TOUCH_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = CONFIG_TOUCH_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_TOUCH_I2C_CLK_HZ};
    ESP_RETURN_ON_ERROR(i2c_param_config(CONFIG_TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(CONFIG_TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_H_RES,
        .y_max = CONFIG_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
        .int_gpio_num = CONFIG_TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = CONFIG_TOUCH_SWAP_XY,
            .mirror_x = CONFIG_TOUCH_MIRROR_X,
            .mirror_y = CONFIG_TOUCH_MIRROR_Y,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

#if CONFIG_TOUCH_CST816S
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle);
#elif CONFIG_TOUCH_FT5X06
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &touch_handle);
#elif CONFIG_TOUCH_FT6X36
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &touch_handle);
#elif CONFIG_TOUCH_GT1151
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT1151_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt1151(tp_io_handle, &tp_cfg, &touch_handle);
#elif CONFIG_TOUCH_GT911
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &touch_handle);
#elif CONFIG_TOUCH_TT21100
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_TT21100_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_tt21100(tp_io_handle, &tp_cfg, &touch_handle);
#endif
}
static esp_err_t lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 1024 * 12,  /* LVGL task stack size, 12KB to avoid `stack overflow in task LVGL task` */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = CONFIG_LCD_H_RES * CONFIG_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = CONFIG_LCD_DRAW_BUFF_DOUBLE,
        .hres = CONFIG_LCD_H_RES,
        .vres = CONFIG_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = CONFIG_LCD_SWAP_XY,
            .mirror_x = CONFIG_LCD_MIRROR_X,
            .mirror_y = CONFIG_LCD_MIRROR_Y,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = true,
            .swap_bytes = true,
        }};
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}
static uint32_t checksum(peripherals_config_t *peripherals_config)
{
    size_t size = sizeof(peripherals_config_t) - sizeof(uint32_t);
    unsigned char *m = (unsigned char *)malloc(size);
    memcpy(m, peripherals_config, size);
    uint32_t c = crc32(0, m, size);
    free(m);
    return c;
}
static bool write_peripherals_config(char *oem_partition_label, peripherals_config_t *config)
{
    bool ret = true;
    {
        // version
        uint8_t version_buffer[10];
        sprintf((char *)version_buffer, "%d", VERSION);
        ret = littlefs_write_file(oem_partition_label, OEM_CONFIG_VERSION_FILE, version_buffer, 10);
        if (!ret)
        {
            ESP_LOGE(TAG, "failed to write version file");
            return false;
        }

        // peripherals
        int checksum_value = checksum(config);
        config->check_sum = checksum_value;
        uint8_t *data = malloc(sizeof(peripherals_config_t));
        memcpy(data, config, sizeof(peripherals_config_t));
        ret = littlefs_write_file(oem_partition_label, OEM_CONFIG_PERIPHERALS_FILE, data, sizeof(peripherals_config_t));
        free(data);
        data = NULL;
        if (!ret)
        {
            ESP_LOGE(TAG, "failed to write peripherals file");
            return false;
        }
    }

    {
        // check version file
        uint8_t *version_buffer = NULL;
        size_t version_size = 0;
        ret = littlefs_read_file(oem_partition_label, OEM_CONFIG_VERSION_FILE, &version_buffer, &version_size);
        if (!ret)
        {
            ESP_LOGE(TAG, "failed to read version file");
            return false;
        }
        if (version_size != 10)
        {
            free(version_buffer);
            version_buffer = NULL;
            return false;
        }
        int version = atoi((char *)version_buffer);
        if (version != VERSION)
        {
            ESP_LOGE(TAG, "version mismatch");
            return false;
        }
        free(version_buffer);
        version_buffer = NULL;

        // check peripherals file
        uint8_t *peripherals_buffer = NULL;
        size_t peripherals_size = 0;
        ret = littlefs_read_file(oem_partition_label, OEM_CONFIG_PERIPHERALS_FILE, &peripherals_buffer, &peripherals_size);
        if (!ret)
        {
            ESP_LOGE(TAG, "failed to read peripherals file");
            return false;
        }
        if (peripherals_size != sizeof(peripherals_config_t))
        {
            free(peripherals_buffer);
            peripherals_buffer = NULL;
            ESP_LOGE(TAG, "peripherals size mismatch");
            return false;
        }

        int checksum_value = checksum(config);
        config->check_sum = checksum_value;
        uint8_t *data = malloc(sizeof(peripherals_config_t));
        memcpy(data, config, sizeof(peripherals_config_t));
        ret = memcmp(data, peripherals_buffer, peripherals_size) == 0;
        free(data);
        data = NULL;
        free(peripherals_buffer);
        peripherals_buffer = NULL;

        return ret;
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t app_camera_init(void)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sccb_sda = CAMERA_PIN_SIOD;
    config.pin_sccb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = CAMERA_PIXFORMAT;
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = 10;
    config.fb_count = CAMERA_FB_COUNT;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID)
        s->set_vflip(s, 1); // flip it back
    else if (s->id.PID == GC0308_PID)
    {
        s->set_hmirror(s, 0);
    }
    else if (s->id.PID == GC032A_PID)
    {
        s->set_vflip(s, 1);
        // s->set_hmirror(s, 0); //something wrong
    }

    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 2);
        s->set_contrast(s, 3);
    }

    return ESP_OK;
}
esp_err_t app_lvgl_init(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(lcd_init());

    /* Touch initialization */
    ESP_ERROR_CHECK(touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(lvgl_init());

    return ESP_OK;
}
bool app_peripherals_read(char *oem_partition_label, int *version, peripherals_config_t **config)
{
    // read version
    uint8_t *version_buffer = NULL;
    size_t version_size = 0;
    bool ret = littlefs_read_file(oem_partition_label, OEM_CONFIG_VERSION_FILE, &version_buffer, &version_size);
    if (!ret)
    {
        return false;
    }
    if (version_size != 10)
    {
        free(version_buffer);
        version_buffer = NULL;
        return false;
    }
    *version = atoi((char *)version_buffer);
    free(version_buffer);
    version_buffer = NULL;

    // read peripherals
    uint8_t *config_buffer = NULL;
    size_t size = 0;
    ret = littlefs_read_file(oem_partition_label, OEM_CONFIG_PERIPHERALS_FILE, &config_buffer, &size);
    if (!ret)
    {
        return false;
    }
    if (size != sizeof(peripherals_config_t))
    {
        free(config_buffer);
        config_buffer = NULL;
        return false;
    }
    *config = malloc(sizeof(peripherals_config_t));
    memcpy(*config, config_buffer, size);
    free(config_buffer);
    config_buffer = NULL;
    return true;
}
bool app_peripherals_write(char *oem_partition_label)
{
    peripherals_config_t *config = malloc(sizeof(peripherals_config_t));
    memset(config, 0, sizeof(peripherals_config_t));
    {
        /* config->camera_module,config->camera_module_custom_config */
        {
#if CONFIG_CAMERA_MODULE_WROVER_KIT
            config->camera_module = CAMERA_MODULE_WROVER_KIT;
#elif CONFIG_CAMERA_MODULE_ESP_EYE
            config->camera_module = CAMERA_MODULE_ESP_EYE;
#elif CONFIG_CAMERA_MODULE_ESP_S2_KALUGA
            config->camera_module = CAMERA_MODULE_ESP_S2_KALUGA;
#elif CONFIG_CAMERA_MODULE_ESP_S3_EYE
            config->camera_module = CAMERA_MODULE_ESP_S3_EYE;
#elif CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
            config->camera_module = CAMERA_MODULE_ESP32_CAM_BOARD;
#elif CONFIG_CAMERA_MODULE_M5STACK_PSRAM
            config->camera_module = CAMERA_MODULE_M5STACK_PSRAM;
#elif CONFIG_CAMERA_MODULE_M5STACK_WIDE
            config->camera_module = CAMERA_MODULE_M5STACK_WIDE;
#elif CONFIG_CAMERA_MODULE_AI_THINKER
            config->camera_module = CAMERA_MODULE_AI_THINKER;
#elif CONFIG_CAMERA_MODULE_CUSTOM
            config->camera_module = CAMERA_MODULE_CUSTOM;
            {
                config->camera_module_custom_config.pin_pwdn = CAMERA_MODULE_CUSTOM_PIN_PWDN;
                config->camera_module_custom_config.pin_reset = CAMERA_MODULE_CUSTOM_PIN_RESET;
                config->camera_module_custom_config.pin_xclk = CAMERA_MODULE_CUSTOM_PIN_XCLK;
                config->camera_module_custom_config.pin_sioc = CAMERA_MODULE_CUSTOM_PIN_SIOC;
                config->camera_module_custom_config.pin_siod = CAMERA_MODULE_CUSTOM_PIN_SIOD;
                config->camera_module_custom_config.pin_d0 = CAMERA_MODULE_CUSTOM_PIN_D0;
                config->camera_module_custom_config.pin_d1 = CAMERA_MODULE_CUSTOM_PIN_D1;
                config->camera_module_custom_config.pin_d2 = CAMERA_MODULE_CUSTOM_PIN_D2;
                config->camera_module_custom_config.pin_d3 = CAMERA_MODULE_CUSTOM_PIN_D3;
                config->camera_module_custom_config.pin_d4 = CAMERA_MODULE_CUSTOM_PIN_D4;
                config->camera_module_custom_config.pin_d5 = CAMERA_MODULE_CUSTOM_PIN_D5;
                config->camera_module_custom_config.pin_d6 = CAMERA_MODULE_CUSTOM_PIN_D6;
                config->camera_module_custom_config.pin_d7 = CAMERA_MODULE_CUSTOM_PIN_D7;
                config->camera_module_custom_config.pin_vsync = CAMERA_MODULE_CUSTOM_PIN_VSYNC;
                config->camera_module_custom_config.pin_href = CAMERA_MODULE_CUSTOM_PIN_HREF;
                config->camera_module_custom_config.pin_pclk = CAMERA_MODULE_CUSTOM_PIN_PCLK;
            }
#else
#error "camera module not configured"
#endif
        }
        /* camera_module_config */
        {
            config->camera_module_config.xclk_freq_hz = XCLK_FREQ_HZ;
            config->camera_module_config.pixelformat = CAMERA_PIXFORMAT;
            config->camera_module_config.framesize = CAMERA_FRAME_SIZE;
            config->camera_module_config.fb_count = CAMERA_FB_COUNT;
            config->camera_module_config.swap_x = CAMERA_SWAP_X;
            config->camera_module_config.swap_y = CAMERA_SWAP_Y;
        }
        /* lcd_module */
        {
#if CONFIG_LCD_ST7735
            config->lcd_module = LCD_MODULE_ST7735;
#elif CONFIG_LCD_ST7789
            config->lcd_module = LCD_MODULE_ST7789;
#elif CONFIG_LCD_ST7796
            config->lcd_module = LCD_MODULE_ST7796;
#elif CONFIG_LCD_ILI9341
            config->lcd_module = LCD_MODULE_ILI9341;
#elif CONFIG_LCD_GC9A01
            config->lcd_module = LCD_MODULE_GC9A01;
#else
#error "lcd module not configured"
#endif
        }
        /* lcd_module_pin_config */
        {
            config->lcd_module_pin_config.pin_sclk = CONFIG_LCD_GPIO_SCLK;
            config->lcd_module_pin_config.pin_mosi = CONFIG_LCD_GPIO_MOSI;
            config->lcd_module_pin_config.pin_rst = CONFIG_LCD_GPIO_RST;
            config->lcd_module_pin_config.pin_dc = CONFIG_LCD_GPIO_DC;
            config->lcd_module_pin_config.pin_cs = CONFIG_LCD_GPIO_CS;
            config->lcd_module_pin_config.pin_bl = CONFIG_LCD_GPIO_BL;
        }
        /* lcd_module_config */
        {
            config->lcd_module_config.h_res = CONFIG_LCD_H_RES;
            config->lcd_module_config.v_res = CONFIG_LCD_V_RES;
            config->lcd_module_config.swap_xy = CONFIG_LCD_SWAP_XY;
            config->lcd_module_config.mirror_x = CONFIG_LCD_MIRROR_X;
            config->lcd_module_config.mirror_y = CONFIG_LCD_MIRROR_Y;
            config->lcd_module_config.gpio_bl_pwm = CONFIG_LCD_GPIO_BL_PWM;
            config->lcd_module_config.spi_num = CONFIG_LCD_SPI_NUM;
            config->lcd_module_config.pixel_clk_hz = CONFIG_LCD_PIXEL_CLK_HZ;
            config->lcd_module_config.cmd_bits = CONFIG_LCD_CMD_BITS;
            config->lcd_module_config.param_bits = CONFIG_LCD_PARAM_BITS;
            config->lcd_module_config.color_space = CONFIG_LCD_COLOR_SPACE;
            config->lcd_module_config.bits_per_pixel = CONFIG_LCD_BITS_PER_PIXEL;
            config->lcd_module_config.draw_buff_double = CONFIG_LCD_DRAW_BUFF_DOUBLE;
            config->lcd_module_config.draw_buff_height = CONFIG_LCD_DRAW_BUFF_HEIGHT;
            config->lcd_module_config.bl_on_level = CONFIG_LCD_BL_ON_LEVEL;
        }
        /* touch_module */
        {
#if CONFIG_TOUCH_CST816S
            config->touch_module = TOUCH_MODULE_CST816S;
#elif CONFIG_TOUCH_FT5X06
            config->touch_module = TOUCH_MODULE_FT5X06;
#elif CONFIG_TOUCH_FT6X36
            config->touch_module = TOUCH_MODULE_FT6X36;
#elif CONFIG_TOUCH_GT1151
            config->touch_module = TOUCH_MODULE_GT1151;
#elif CONFIG_TOUCH_GT911
            config->touch_module = TOUCH_MODULE_GT911;
#elif CONFIG_TOUCH_TT21100
            config->touch_module = TOUCH_MODULE_TT21100;
#else
#error "touch module not configured"
#endif
        }
        /* touch_module_config */
        {
            config->touch_module_config.i2c_scl = CONFIG_TOUCH_I2C_SCL;
            config->touch_module_config.i2c_sda = CONFIG_TOUCH_I2C_SDA;
            config->touch_module_config.gpio_int = CONFIG_TOUCH_GPIO_INT;
            config->touch_module_config.i2c_clk_hz = CONFIG_TOUCH_I2C_CLK_HZ;
            config->touch_module_config.swap_xy = CONFIG_TOUCH_SWAP_XY;
            config->touch_module_config.mirror_x = CONFIG_TOUCH_MIRROR_X;
            config->touch_module_config.mirror_y = CONFIG_TOUCH_MIRROR_Y;
            config->touch_module_config.i2c_num = CONFIG_TOUCH_I2C_NUM;
        }
    }
    bool ret = write_peripherals_config(oem_partition_label, config);
    free(config);
    config = NULL;
    return ret;
}
