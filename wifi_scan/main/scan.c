/**
  C:\tools\Espressif\frameworks\esp-idf-v5.3.1\examples\wifi

  Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This example shows how to scan for available set of APs.
*/
#include <string.h>
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <esp_sleep.h>

#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "driver/gpio.h"

#define DEFAULT_SCAN_LIST_SIZE 10 // Number of APs to scan for

static const char *TAG = "scan";

#define TEST_FLASH_MS 100
#define LED_RED GPIO_NUM_0
#define LED_YELLOW GPIO_NUM_19
#define LED_GREEN GPIO_NUM_18

#define BUTTON GPIO_NUM_2 // GPIO pin for the button
#define WAKE_HOLD GPIO_NUM_1 // GPIO pin held high for button press power
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  

// From the gpio example:
#define CONFIG_GPIO_OUTPUT_0    GPIO_NUM_18
#define CONFIG_GPIO_OUTPUT_1    GPIO_NUM_19
#define CONFIG_GPIO_OUTPUT_2    GPIO_NUM_0

#define GPIO_OUTPUT_IO_0    CONFIG_GPIO_OUTPUT_0
#define GPIO_OUTPUT_IO_1    CONFIG_GPIO_OUTPUT_1
#define GPIO_OUTPUT_IO_2    CONFIG_GPIO_OUTPUT_2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2))

// Description: This code demonstrates how to put an ESP32 into deep sleep mode and wake it up using a GPIO pin. It uses the Adafruit NeoPixel library to control an RGB LED, which changes color every second for 20 seconds before entering deep sleep.
int prep_sleep(void)
{
    // Wake button power hold
    if (gpio_hold_en(WAKE_HOLD) != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(BUTTON), ESP_EXT1_WAKEUP_ANY_HIGH) != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON) != ESP_OK)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void post_sleep(void)
{
    // Prepare the system for sleep mode (this powers down all RTC domains)

    // Wake button power hold
    gpio_hold_dis(WAKE_HOLD);
    esp_sleep_disable_ext1_wakeup_io(BUTTON);
}

void enter_sleep(void)
{
    // Enter deep sleep mode
    // esp_light_sleep_start(); // This will put the chip into light sleep mode
    esp_deep_sleep_start();
}

int setup_led_pin(int gpio_num)
{
    if (gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (gpio_set_level(gpio_num, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (gpio_set_drive_capability(gpio_num, GPIO_DRIVE_CAP_1) != ESP_OK)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Sets up output LED and input Button.
int setup_io_pins(void)
{
    // FROM gpio example:
    //zero-initialize the config structure.
    // gpio_config_t io_conf = {};
    // //disable interrupt
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // //set as output mode
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //disable pull-down mode
    // io_conf.pull_down_en = 0;
    // //disable pull-up mode
    // io_conf.pull_up_en = 0;
    // //configure GPIO with the given settings
    // gpio_config(&io_conf);

    esp_rom_gpio_pad_select_gpio(LED_RED);
    esp_rom_gpio_pad_select_gpio(LED_YELLOW);
    esp_rom_gpio_pad_select_gpio(LED_GREEN);

    // gpio_set_drive_capability(LED_RED, GPIO_DRIVE_CAP_3);
    // gpio_set_drive_capability(LED_YELLOW, GPIO_DRIVE_CAP_3);
    // gpio_set_drive_capability(LED_GREEN, GPIO_DRIVE_CAP_3);

    // LED:Red
    if (setup_led_pin(LED_RED) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (setup_led_pin(LED_YELLOW) != ESP_OK)
    {
        //gpio_pad_select_gpio(LED_YELLOW);
        return ESP_FAIL;
    }
    if (setup_led_pin(LED_GREEN) != ESP_OK)
    {
        //gpio_pad_select_gpio(LED_GREEN);
        return ESP_FAIL;
    }

    // Button
    if (gpio_set_direction(BUTTON, GPIO_MODE_INPUT) != ESP_OK)
    {
        return ESP_FAIL;
    }

    // Wake-hold for button press
    if (gpio_set_direction(WAKE_HOLD, GPIO_MODE_OUTPUT) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (gpio_set_level(WAKE_HOLD, 1) != ESP_OK)
    {
        return ESP_FAIL;
    }

    // NOTE: using external physical pulldown resistor on button
    gpio_pulldown_dis(BUTTON);
    gpio_pullup_dis(BUTTON);

    return ESP_OK;
}

int flash_led(int led, int seconds)
{
    int ms = seconds * 1000; // Convert seconds to milliseconds
    if (gpio_set_level(led, 1) != ESP_OK)
    {
        return ESP_FAIL;
    }

    vTaskDelay(ms / portTICK_PERIOD_MS);

    if (gpio_set_level(led, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
    return ESP_OK;
}

int flash_leds(int led1, int led2, int seconds)
{
    int ms = seconds * 1000; // Convert seconds to milliseconds
    if (gpio_set_level(led1, 1) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (gpio_set_level(led2, 1) != ESP_OK)
    {
        return ESP_FAIL;
    }

    vTaskDelay(ms / portTICK_PERIOD_MS);

    if (gpio_set_level(led1, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (gpio_set_level(led2, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
    return ESP_OK;
}

void testFlash(uint8_t led)
{
    gpio_set_level(led, 1);
    vTaskDelay(TEST_FLASH_MS / portTICK_PERIOD_MS);
    gpio_set_level(led, 0);
}

void errflash(void)
{
    for (int i = 0; i < 10; i++)
    {
        flash_led(LED_RED,1); // Flash LED to indicate error
    }
    for (;;) {}

}

// Not printing auth mode.
// Not printing cipher type

typedef enum {
    RSSI_SCORE_RED = 1,
    RSSI_SCORE_RED_YELLOW = 2,
    RSSI_SCORE_YELLOW = 3,
    RSSI_SCORE_YELLOW_GREEN = 4,
    RSSI_SCORE_GREEN = 5,
} rssi_score;

static rssi_score get_score(int rssi)
{
    if (rssi <= -90)
    {
        return RSSI_SCORE_RED;
    }
    else if (rssi <= -80)
    {
        return RSSI_SCORE_RED_YELLOW;
    }
    else if (rssi <= -70)
    {
        return RSSI_SCORE_YELLOW;
    }
    else if (rssi <= -68)
    {
        return RSSI_SCORE_YELLOW_GREEN;
    }
    else
    {
        return RSSI_SCORE_GREEN;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static void wifi_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Not using scanning channel bitmap
    esp_wifi_scan_start(NULL, true);

    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    for (int i = 0; i < number; i++) {
        if (strlen((const char *) ap_info[i].ssid) >= 11 && strncmp("YOURACCESSPOINT", (const char*)ap_info[i].ssid, 11) == 0)
        {
            ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
            int rssi = ap_info[i].rssi;
            rssi_score score = get_score(rssi);
            switch (score)
            {
                case RSSI_SCORE_RED:
                    ESP_LOGI(TAG, "Signal Strength \tRED");
                    flash_led(LED_RED, 5);
                    break;
                case RSSI_SCORE_RED_YELLOW:
                    ESP_LOGI(TAG, "Signal Strength \tRED-YELLOW");
                    flash_leds(LED_RED, LED_YELLOW, 5);
                    break;
                case RSSI_SCORE_YELLOW:
                    ESP_LOGI(TAG, "Signal Strength \tYELLOW");
                    flash_led(LED_YELLOW, 5);
                    break;
                case RSSI_SCORE_YELLOW_GREEN:
                    ESP_LOGI(TAG, "Signal Strength \tYELLOW-GREEN");
                    flash_leds(LED_YELLOW, LED_GREEN, 5);
                    break;
                case RSSI_SCORE_GREEN:
                    ESP_LOGI(TAG, "Signal Strength \tGREEN");
                    flash_led(LED_GREEN, 5);
                    break;
            }
            ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
            ESP_LOGI(TAG, "Channel \t\t%d", ap_info[i].primary);
            break; // Expect only one match
        }
    }
}

void app_main(void)
{
    // Warm up time for the ESP32 to stabilize
    // May make it easier to re-flash the ESP32    
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    if (setup_io_pins() != ESP_OK)
    {
        // Flash error on setup pins.
        errflash();
    }

    testFlash(LED_RED);
    testFlash(LED_YELLOW);
    testFlash(LED_GREEN);

    for (;;) 
    {
        wifi_scan();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (prep_sleep() != ESP_OK)
        {
            // Flash error on sleep preparation.
            errflash();
        }
        enter_sleep(); // Enter deep sleep mode
        post_sleep(); // Prepare for wake up on button press

    }
}
