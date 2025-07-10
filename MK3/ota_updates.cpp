//ota_updates.cpp
#include "ota_updates.h"
#include <WiFi.h>
#include <Update.h>
#include <cstdio>
#include "FFat.h"
#include "FS.h"
#include "mk3.h"

// WiFI variables

char *ssid = CRAWLER_VERSION;
char *temp = CRAWLER_NUM;
const char *password = "1234567890"; 
const int port = 11100;
WiFiServer server(port);
//WiFiClient client;
String bin_file_name = "/OTA_binary.bin"; // filename for the firmware

/**************************************************************************************************/
// Over-The-Air Update functions
/**************************************************************************************************/

// get the firmware file name
String Wifi::getbinname(){
  return bin_file_name;
}

// turn on wifi functionality 
void Wifi::beginwifi(){
// start the WiFi access point to be ready for connection
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPIP();
  server.begin();
}

// turn off wifi
void Wifi::endwifi(){
  server.close();
  WiFi.mode(WIFI_OFF);
}

// test if a connection has been established
bool Wifi::check_for_connection() {
  if (!client.connected()) client = server.accept();

  if (client.connected()){
    return true;
  } else {
    return false;
  }
}

int Wifi::send_buffer(const char *buffer, int len) {
  int ret = 0;
  //Do not attempt to write to the client if it is not connected.
  if (!client.connected()){
    Serial.println("fail in send buffer");
    return -1;
  }
  ret = client.write(buffer, len);
  return ret;
}

// send the firmware version across to the app
void Wifi::send_fw_version() {
  String version = FW_VERSION;
  int len = version.length();
  const char *buff = version.c_str();
  int ret = 0;

  //send buffer to the app.
  ret = send_buffer(buff, len);

  if (ret == -1) Serial.println("Failed to write firmware version to the client.");
}

// send crawler data across to the app
void Wifi::send_crawler_info() {
  String send_data = String(CRAWLER_VERSION) + " " + String(FW_VERSION) + " " + String(CRAWLER_NUM) + " " + String(MANUFACTURE_DATE) + "\n";
  int len = send_data.length();
  const char *buff = send_data.c_str();
  int ret = 0;

  //send buffer to the app.
  ret = send_buffer(buff, len);

  if (ret == -1) Serial.println("Failed to write firmware version to the client.");
}

// Send an OK ping to the app
void Wifi::send_okay() {
  String okay = "OK\n";
  int len = okay.length();
  const char *buff = okay.c_str();
  int ret = 0;

  //send buffer to the app.
  ret = send_buffer(buff, len);

  if (ret == -1) Serial.println("Failed to send verification to the client.");
}

// send an error ping to the app
void Wifi::send_err() {
  String err = "ERROR";
  int len = err.length();
  const char *buff = err.c_str();
  int ret = 0;

  //send buffer to the app.
  ret = send_buffer(buff, len);

  if (ret == -1) Serial.println("Failed to send error message to the client.");
}

// perform the update
void Wifi::run_update_funct() {
  size_t update_file_size = 0;
  size_t written = 0;

  //Create file to store the Binary contents.
  File file = FFat.open(bin_file_name, FILE_READ);
  if (!file) {
    // Serial.println("Error in Opening Binary file for read.");
    return;
  }

  update_file_size = file.size();

  if (update_file_size <= 0) {
    send_err();
    Serial.println("Invalid file size.");
    goto error;
  }
  Serial.println(update_file_size);
  if (Update.begin(update_file_size)) {
    Serial.println("There is sufficient space for the update");
  } else {
    send_err();
    Serial.println("There is not sufficient space for the update");
    goto error;
  }

  written = Update.writeStream(file);

  if(written == update_file_size) {
    Serial.println("Successfully wrote binary to flash");
  } else {
    send_err();
    Serial.println("Failed to write Binary to flash");
    Serial.println(String(written));
    goto error;
  }

  if(Update.end()) {
    Serial.println("OTA Updates have been completed");
  }  else {
    send_err();
    Serial.println("Update is not finished! Something might have gone wrong");
    goto error;
  }

  if (Update.isFinished()) {
    send_okay();
    Serial.println("Update is successfully completed! Rebooting ESP32 now");
  } else {
    send_err();
    Serial.println("isFinished() returned false! Something might have gone wrong");
    goto error;
  }

  delay(500);
  ESP.restart();
  error:
  // Serial.println(Update.errorString());
  file.close();
}

// fetch the firmware file
bool Wifi::get_binary_data() {
  uint8_t binary_data_array[DATA_ARRAY_SIZE];
  uint8_t *bin = binary_data_array;
  int progress = 0;
  int total_transmission_size = 0;
  size_t ret = 0;
  if(!FFat.begin(true)){
    Serial.println("An Error has occurred while mounting FFat");
    return false;
  }
  if (FFat.exists(bin_file_name)) {
    Serial.println("Deleting old bin file!");
    FFat.remove(bin_file_name);
  }
  if(FFat.remove(bin_file_name)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed (no file?)");
  }

  //Create file to store the Binary contents.
  File file1 = FFat.open(bin_file_name, FILE_WRITE);

  if (!file1) {
    Serial.println("Error in creating file for the Binary data.");
    return false;
  }

  String file_size = client.readStringUntil('\n');
  total_transmission_size = file_size.toInt();

  while (!client.available() && client.connected()){
    //wait for the data to start being sent.
  }

  Serial.println("Collecting new Binary data.");
  while(client && client.connected()) {
    if (client.available()) {
      ret = client.read(bin, DATA_ARRAY_SIZE);
      file1.write(bin, ret);
      progress += ret;
    }

    //Timeout used to indicate end of transmission.
    if (progress == total_transmission_size) {
      Serial.println("Done receiving Update binary");
      break;
    }
    delay(0.1);
  }

  Serial.println("Done recieving Update Binary data!");
  file1.close();
  return true;
}

// Top level function to perform the whole update process
void Wifi::update_crawler() {
  if (get_binary_data()) {
    Serial.println("Found Firmware Update! Installing updates now...");
    run_update_funct();
  } else {
    Serial.println("Failed to collect binary data for new firmware");
  }
}

void Wifi::start_transmission() {
  //check for connection with the App.
  if(!check_for_connection()) return;
  
  Serial.println("Established connection with Mobile device!");

  while (!client.available() && client.connected()) {
    //wait to receive transmission protocol from the sender.
  }

  //return from function if connection is broken.
  if (!client.connected()){
    Serial.println("fail");
    return;
  }
  String message = client.readStringUntil('\n');
  char key = message[0]; //convert the String to a char
  Serial.println("key: ");
  Serial.println(key);
  switch(key) {
    case '+' :
      Serial.println("Start updating the crawler");
      update_crawler();
      break;
    case '=' :
      Serial.println("Checking for update.");
      send_fw_version();
      break;
    case '-' :
      Serial.println("Fetching crawler data.");
      send_crawler_info();
      break;
    default:
      Serial.println("Invalid Transmission protocol.");
      client.stop();
  }
}
