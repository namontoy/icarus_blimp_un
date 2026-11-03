/*
 * ESP32-CAM pin maps.
 *
 * AI-Thinker is the board nearly everyone means by "ESP32-CAM": the black
 * module with the microSD slot and the 1 W white LED. The other maps are here
 * so a different board needs a one-line change, not an edit of the main file.
 */
#pragma once

#if defined(CONFIG_OPTFLOW_BOARD_AI_THINKER)

#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1      /* not broken out; the driver soft-resets */
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

/* The 1 W flash LED. It is a serious heat source and we never want it on, but
   GPIO4 also floats bright on some boards, so we drive it low explicitly. */
#define CAM_PIN_FLASH_LED 4

/* UART1 to the flight board (S3), the wiring target_cam uses
   (firmware/projects/target_cam/main/camera_pins.h): the camera transmits on
   IO14 and receives on IO13, the microSD slot's CLK and DAT3, so the slot
   stays empty. The same module can run either firmware on the same wires. */
#define S3_LINK_PIN_RX  13
#define S3_LINK_PIN_TX  14

#elif defined(CONFIG_OPTFLOW_BOARD_S3_WROOM_CAM)

/* ESP32-S3-WROOM CAM, Freenove pin map (the same as Espressif's ESP32-S3-EYE
   and Arduino's CAMERA_MODEL_ESP32S3_EYE), from
   docs/s3-wroom-cam-wiring-plan.md. PWDN and RESET are not wired to the S3. */
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5
#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2       8
#define CAM_PIN_D1       9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13
/* No flash LED: the board's light is a WS2812 on GPIO48, dark unless driven. */
#define CAM_PIN_FLASH_LED -1

/* UART1 to the flight board. TX stays on IO14, as on the AI-Thinker, so the
   S3-Zero's UART2 (RX GPIO18) needs no change. RX cannot stay on IO13: on this
   board IO13 is the camera's PCLK. IO21 is free (not camera, PSRAM, USB,
   UART0, LED or microSD) and nothing is wired to it today; the S3-Zero never
   transmits to this camera. Check both pins on the board's silk before wiring:
   clones of this board differ in what they break out. */
#define S3_LINK_PIN_RX  21
#define S3_LINK_PIN_TX  14

#elif defined(CONFIG_OPTFLOW_BOARD_ESP_EYE)

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     4
#define CAM_PIN_SIOD    18
#define CAM_PIN_SIOC    23
#define CAM_PIN_D7      36
#define CAM_PIN_D6      37
#define CAM_PIN_D5      38
#define CAM_PIN_D4      39
#define CAM_PIN_D3      35
#define CAM_PIN_D2      14
#define CAM_PIN_D1      13
#define CAM_PIN_D0      34
#define CAM_PIN_VSYNC    5
#define CAM_PIN_HREF    27
#define CAM_PIN_PCLK    25
#define CAM_PIN_FLASH_LED -1

#elif defined(CONFIG_OPTFLOW_BOARD_XIAO_S3)

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    10
#define CAM_PIN_SIOD    40
#define CAM_PIN_SIOC    39
#define CAM_PIN_D7      48
#define CAM_PIN_D6      11
#define CAM_PIN_D5      12
#define CAM_PIN_D4      14
#define CAM_PIN_D3      16
#define CAM_PIN_D2      18
#define CAM_PIN_D1      17
#define CAM_PIN_D0      15
#define CAM_PIN_VSYNC   38
#define CAM_PIN_HREF    47
#define CAM_PIN_PCLK    13
#define CAM_PIN_FLASH_LED -1

#else
#error "Select a camera board in menuconfig (Optical Flow Configuration)"
#endif
