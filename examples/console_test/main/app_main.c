/* Wi-Fi CSI Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "esp_console.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "esp_radar.h"
#include "app_priv.h"

#define CONFIG_CSI_BUF_SIZE          50
#define CONFIG_GPIO_LED_MOVE_STATUS  GPIO_NUM_18

#define LEN_MAC_ADDR 20

static const char *TAG  = "app_main";

float g_move_absolute_threshold = 0.3;
float g_move_relative_threshold = 1.5;

static void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
}

int j = 0;

void wifi_csi_raw_cb(void *ctx, wifi_csi_info_t *data)
{
	wifi_csi_info_t received = data[0];

	char senddMacChr[LEN_MAC_ADDR] = {0}; // sender
	sprintf(senddMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", received.mac[0], received.mac[1], received.mac[2], received.mac[3], received.mac[4], received.mac[5]);
	
//	"A2:B7:93:ED:39:CD" ------ CreatComm
//	"D4:6E:0E:A7:9B:2B" ------ TP-LINK
//	"30:23:03:2D:E7:F4" ------ wemotest
//	"34:97:F6:DE:5E:B0" ------ Lyra Mini
//	"00:03:7F:12:8C:8C" / "00:03:7F:12:EE:EE" ------ Jalapeno
//	"40:B0:76:35:50:10" ------ asus router
//	"78:D2:94:C0:74:22" ------ netgear
//	"2C:FD:A1:90:65:2A" ------ Lyra Mini AP
//	"00:03:7F:12:BA:BA" ------ Jalapeno
	if( received.rx_ctrl.sig_mode == 1 ){
		j++;
		printf("Number %d CSI callback func printout.\n", j);		

		printf("rate: %d\n", received.rx_ctrl.rate); // PHY rate encoding of the packet, only valid for non HT(11bg) packet
		printf("signal mode: %d -> ", received.rx_ctrl.sig_mode);
		switch(received.rx_ctrl.sig_mode){
			case 0: printf("Non HT(11bg)\n"); break;
			case 1: printf("HT(11n)\n"); break;
			case 3: printf("VHT(11ac)\n"); break;
			default: printf("unknown");		
		}
		
		printf("HT20 (0) or HT40 (1) (0 - 20MHz, 1 - 40MHz): %d\n", received.rx_ctrl.cwb);
		printf("Space time block present: %d\n", received.rx_ctrl.stbc);
		printf("Secondary channel (0 - none, 1 - above, 2 - below): %d\n", received.rx_ctrl.secondary_channel);
		printf("Length: %d\n", received.len);
		printf("RSSI (Received Signal Strength Indicator of packet): %d\n", received.rx_ctrl.rssi);
		printf("WiFi antenna number: %d\n", received.rx_ctrl.ant);
		printf("Error state: %d\n", received.rx_ctrl.rx_state);
		printf("First word invalid: %d\n", received.first_word_invalid);
		printf("Timestamp:%d\n", received.rx_ctrl.timestamp); // unit: microseconds


		// may display other information
		
		// print data
		printf("CSI address: %s, len: %d\n", senddMacChr, data->len);
		int8_t* my_ptr = data->buf;
		printf("*****"); 
		for(int i = 0; i < data->len; i++){ 
			printf("%02X ", my_ptr[i]); 	
		}
		printf("!!!!!");
		printf("\n\n");

	} 
}


static void wifi_radar_cb(const wifi_radar_info_t *info, void *ctx)
{
    static int s_count = 0;
    wifi_radar_config_t radar_config = {0};
    static float s_amplitude_std_list[CONFIG_CSI_BUF_SIZE];
    bool trigger_relative_flag = false;

    esp_wifi_radar_get_config(&radar_config);

    float amplitude_std  = avg(info->amplitude_std, radar_config.filter_len / 128);
    // float amplitude_corr = avg(info->amplitude_corr, radar_config.filter_len / 128);
    float amplitude_std_max = 0;
    float amplitude_std_avg = 0;
    s_amplitude_std_list[s_count % CONFIG_CSI_BUF_SIZE] = amplitude_std;

    s_count++;

    if (s_count > CONFIG_CSI_BUF_SIZE) {
        amplitude_std_max = max(s_amplitude_std_list, CONFIG_CSI_BUF_SIZE, 0.10);
        amplitude_std_avg = trimmean(s_amplitude_std_list, CONFIG_CSI_BUF_SIZE, 0.10);

        for (int i = 1, count = 0; i < 6; ++i) {
            if (s_amplitude_std_list[(s_count - i) % CONFIG_CSI_BUF_SIZE] > amplitude_std_avg * g_move_relative_threshold
                    || s_amplitude_std_list[(s_count - i) % CONFIG_CSI_BUF_SIZE] > amplitude_std_max) {
                if (++count > 2) {
                    trigger_relative_flag = true;
                    break;
                }
            }
        }
    }

    static uint32_t s_last_move_time = 0;

    if (amplitude_std > g_move_absolute_threshold || trigger_relative_flag) {
        gpio_set_level(CONFIG_GPIO_LED_MOVE_STATUS, 1);
        s_last_move_time = xTaskGetTickCount() * (1000 / configTICK_RATE_HZ);;
        ESP_LOGW(TAG, "Someone is moving");
    }

    if (s_last_move_time && xTaskGetTickCount() * (1000 / configTICK_RATE_HZ) - s_last_move_time > 3 * 1000){
        s_last_move_time  = 0;
        gpio_set_level(CONFIG_GPIO_LED_MOVE_STATUS, 0);
    }
    /*
    ESP_LOGI(TAG, "<%d> time: %u ms, rssi: %d, corr: %.3f, std: %.3f, std_avg: %.3f, std_max: %.3f, threshold: %.3f/%.3f, trigger: %d/%d, free_heap: %u/%u",
             s_count, info->time_end - info->time_start, info->rssi_avg,
             amplitude_corr, amplitude_std, amplitude_std_avg, amplitude_std_max,
             g_move_absolute_threshold, g_move_relative_threshold,
             amplitude_std > g_move_absolute_threshold, trigger_relative_flag,
             esp_get_minimum_free_heap_size(), esp_get_free_heap_size());
    */
}

void app_main(void)
{
    int a = 10;
    int *cmd_ret = &a;
    wifi_radar_config_t radar_config = {
        .wifi_radar_cb = wifi_radar_cb,
        // .filter_len = 384,
        // .filter_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        // .wifi_csi_raw_cb = wifi_csi_raw_cb,
    };

    /**
     * @brief Used to show whether someone is moving
     */
    gpio_reset_pin(CONFIG_GPIO_LED_MOVE_STATUS);
    gpio_set_direction(CONFIG_GPIO_LED_MOVE_STATUS, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_GPIO_LED_MOVE_STATUS, 0);

    /**
     * @brief Initialize Wi-Fi radar
     */
    wifi_init();
    esp_wifi_radar_init();
    esp_wifi_radar_set_config(&radar_config);
    esp_wifi_radar_start();

    /**
     * @brief Register serial command
     */
    // esp_console_repl_t *repl = NULL;
    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    // esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    // esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    // repl_config.prompt = "csi>";
    // ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_init(&console_config));

    cmd_register_system();
    cmd_register_wifi();
    cmd_register_ping();
    cmd_register_csi();
    cmd_register_detect();

    /*
    printf("\n");
    printf(" =======================================================================\n");
    printf(" |                    Steps to test CSI                                |\n");
    printf(" |                                                                     |\n");
    printf(" |  1. Print 'help' to gain overview of commands                       |\n");
    printf(" |     "LOG_COLOR_E"csi>"LOG_RESET_COLOR" help                                                       |\n");
    printf(" |  2. Start SoftAP or Sta on another ESP32/ESP32S2/ESP32C3            |\n");
    printf(" |     "LOG_COLOR_W"csi>"LOG_RESET_COLOR" sta <ssid> <pwd>                                           |\n");
    printf(" |     "LOG_COLOR_I"csi>"LOG_RESET_COLOR" ap <ssid> <pwd>                                            |\n");
    printf(" |  3. Run ping to test                                                |\n");
    printf(" |     "LOG_COLOR_E"csi>"LOG_RESET_COLOR" ping <ip> -c <count> -i <interval>                         |\n");
    printf(" |  4. Configure CSI parameters                                        |\n");
    printf(" |     "LOG_COLOR_W"csi>"LOG_RESET_COLOR" csi -m <mac(xx:xx:xx:xx:xx:xx)> -l <len>                   |\n");
    printf(" |  5. Configure CSI parameters                                        |\n");
    printf(" |     "LOG_COLOR_I"csi>"LOG_RESET_COLOR" detect -a <absolute_threshold> -r <relative_threshold>     |\n");
    printf(" |                                                                     |\n");
    printf(" ======================================================================\n\n");
    */

    // ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_ERROR_CHECK(esp_console_run("sta esp32_Access_Point twoorigin", cmd_ret));
    ESP_ERROR_CHECK(esp_console_run("ping 192.168.4.1", cmd_ret));
    ESP_ERROR_CHECK(esp_console_run("csi -m 7c:9e:bd:34:cb:39 -l 384", cmd_ret));
    ESP_ERROR_CHECK(esp_console_run("csi -o", cmd_ret));

}
