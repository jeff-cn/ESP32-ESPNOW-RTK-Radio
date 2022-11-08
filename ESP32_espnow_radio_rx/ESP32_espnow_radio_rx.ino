// ESP-NOW library
#include <esp_now.h>

// ESP32 WiFi library
#include <WiFi.h>

// ESP32 software serial impementation
#include <SoftwareSerial.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcast_address[] = {0x94, 0xB5, 0x55, 0x2B, 0x3B, 0x3C};

// Max line length
#define PACKET_LENGTH 250

// Max message length
#define MAX_MSG_LENGTH 2500

bool data_available = false;
bool last_char_start_of_msg = false;

char rtk_data[PACKET_LENGTH];

// Software serial pins for communicating with GNSS receiver
#define TX 26
#define RX 25
SoftwareSerial swSerial;

esp_now_peer_info_t peerInfo;

void setup() 
{
  // USB serial
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

  // Register call back for when data is sent
  esp_now_register_send_cb(onDataSend);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcast_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(onDataReceive);
}

unsigned int msg_indx = 0;
static char rtcm_msg[MAX_MSG_LENGTH];
char last_byte = 0;
unsigned int msg_length = 99999;
bool in_message = false;
 
void loop()
{

    // Wait for data to be received, received data triggers the call back
}



// Callback when data is sent
void onDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    Serial.print("\r\nLast Packet Send Status:\t");
}

// Callback when data is received
void onDataReceive(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    // Receive correction data
    memcpy(&rtk_data, incomingData, PACKET_LENGTH);
    Serial.print("Bytes received: ");
    Serial.println(len);
    
    // Loop through received message
    for(uint8_t i=0;i<PACKET_LENGTH;i++)
    {

        // Read the next character in the message
        char in_byte = rtk_data[i];
        Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
        Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
        Serial.print("i: ");Serial.print(i);Serial.print(" ");Serial.println(in_byte, HEX);

        // Look for the start of a message
        if (!in_message && last_byte == 0xD3 && (in_byte & 0xFC) == 0x00)
        {

            rtcm_msg[0] = last_byte;
            rtcm_msg[1] = in_byte;
            rtcm_msg[2] = rtk_data[i+1];
            Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
            Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
            Serial.print("i: ");Serial.print(i+1);Serial.print(" ");Serial.println(rtk_data[2], HEX);
            rtcm_msg[3] = rtk_data[i+2];
            Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
            Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
            Serial.print("i: ");Serial.print(i+2);Serial.print(" ");Serial.println(rtk_data[3], HEX);
            rtcm_msg[4] = rtk_data[i+3];
            Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
            Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
            Serial.print("i: ");Serial.print(i+3);Serial.print(" ");Serial.println(rtk_data[4], HEX);

            msg_length = (((uint32_t)rtcm_msg[1] & 3) << 8) + ((uint32_t)rtcm_msg[2] << 0) + 6;
            unsigned int msg_num = ((uint32_t)rtcm_msg[3] << 4) + ((uint32_t)rtcm_msg[4] >> 4);

            Serial.print(millis());Serial.print(" Message Length: "); Serial.print(msg_length);
            Serial.print(" Message Number: "); Serial.println(msg_num);

            msg_indx = 5;
            i = i+3;

            in_message = true;
        }

        // End of a message (inside a packet)
        else if (msg_indx == msg_length-1)
        {

            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;
        
            // Send message to GPS receiver
            swSerial.write(rtcm_msg, msg_length);

            Serial.print(Crc24Quick(0x000000, msg_length, rtcm_msg));

            Serial.println(" Sending message to GPS");

            // Done with message
            in_message = false;

            msg_length = 0;

            msg_indx = 0;

        }

        // Reading a message
        else if (in_message)
        {
            
            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;

            msg_indx++;
        }
        
        last_byte = in_byte;
    }
}
 
uint32_t Crc24Quick(uint32_t Crc, uint32_t Size, char *Buffer)
{
  static const uint32_t crctab[] = {
    0x00000000,0x01864CFB,0x038AD50D,0x020C99F6,0x0793E6E1,0x0615AA1A,0x041933EC,0x059F7F17,
    0x0FA18139,0x0E27CDC2,0x0C2B5434,0x0DAD18CF,0x083267D8,0x09B42B23,0x0BB8B2D5,0x0A3EFE2E };

  while(Size--)
  {
    Crc ^= (uint32_t)*Buffer++ << 16;
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
  }

  return(Crc & 0xFFFFFF);
}
