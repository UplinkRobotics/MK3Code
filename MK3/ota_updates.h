//ota_updates.h
#ifndef OTA_UPDATES
#define OTA_UPDATES
#include <WiFi.h>
#include "Arduino.h"
/***************************************************************************************************/
// WiFI variables
#define DATA_ARRAY_SIZE 2048
#define FW_VERSION  "1.1"
/***************************************************************************************************/

// class to contain all of the wifi update functions and objects
class Wifi{
	public:
    WiFiClient client;
    String getbinname();
    void beginwifi();
    void endwifi();
    void update_setup();
    void update_crawler();
    bool check_for_connection();
    void run_update_funct();
    bool get_binary_data();
    void start_transmission();
    void send_fw_version();
    void send_crawler_info();
    void send_okay();
    void send_err();
    int send_buffer(const char *buffer, int len);
};
#endif
