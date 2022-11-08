// ESP-NOW library
#include <esp_now.h>

// ESP32 WiFi library
#include <WiFi.h>

// ESP32 software serial impementation
#include <SoftwareSerial.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcast_address[] = {0x94, 0xB5, 0x55, 0x25, 0xCF, 0xC4};

// Max line length
#define MAX_PACKET_LENGTH 250

// Max number of packets in a burst
#define MAX_NUM_PACKETS 4

// Software serial pins for communicating with GNSS receiver
#define TX 26
#define RX 25
SoftwareSerial swSerial;

// Indicates that data is available to add
bool data_to_add = false;

// A character array for one GPS message
typedef struct char_array {char x[MAX_PACKET_LENGTH];} char_array;

// Invalid array, returned when out of bound
char_array default_invalid_array;

typedef struct gps_data_and_length
{
    char_array gps_data;
    uint8_t packet_length;
} gps_data_and_length;

// Structure to contain stored GPS messages
typedef struct circular_buffer
{
    char_array gps_data[MAX_NUM_PACKETS];
    uint8_t packet_length[MAX_NUM_PACKETS];
    uint8_t head;
    uint8_t tail;
} circular_buffer;

circular_buffer cb;

// Routine to initialize the circular buffer
void cb_init(circular_buffer *cb)
{
    Serial.println("init buffer");
    cb->head = 0;
    cb->tail = 0;
}

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

    // Initialize buffer and invalid array
    cb_init(&cb);
    for (uint8_t i=0; i<MAX_PACKET_LENGTH; i++)
    {
        default_invalid_array.x[i]=0;
    }
  
    // Register peer
    memcpy(peerInfo.peer_addr, broadcast_address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(onDataReceive);
}

unsigned long i;
void loop()
{

    // Read serial buffer and push LoRa sized (250 byte) packets to cicular buffer
    readSerialBuffer();
  
    // If there is data to transmit and the serial data from GPS is all received, then transmit data over radio
    if (cb.head != cb.tail)
    {
        transmit_data();
    }

}

// Callback when data is sent
void onDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    if (status ==0)
    {
        Serial.println("Data sent succesfully");
    }
    else
    {
        Serial.println("Data send failed");
    }
}

// Callback when data is received
void onDataReceive(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  Serial.print("Bytes received: ");
  Serial.println(len);
}

// Read serial buffer and pack data into LoRa sized (250 byte) packets
void readSerialBuffer ()
{

    // Packet data to be filled and pushed to buffer
    static char_array packet;

    // Position through current packet
    static uint8_t input_pos = 0;

    // Serial character
    byte in_byte;

    // If serial data is still available
    while (swSerial.available ())
    {
        // Fill the packet with characters
        if (input_pos < MAX_PACKET_LENGTH)
        {
            // Read data from serial
            in_byte = swSerial.read();

            packet.x[input_pos] = in_byte;
            data_to_add = true;
            //Serial.println(in_byte, HEX);
            input_pos++;

        }
        // If the packet is full, then add it to the circular buffer
        else if (input_pos == MAX_PACKET_LENGTH)
        {
 
            // Push packet and packet length into buffer
            cb_push_front(&cb, packet, input_pos);
            
            Serial.print(millis());Serial.print(" Saving length: ");Serial.println(input_pos);
            
            // Return length counter to zero
            input_pos = 0;

            data_to_add = false;
        }   

    }

}

// Transmit available data over radio
void transmit_data ()
{

    // Pointer to first element of current row of burst array
    gps_data_and_length single_packet = cb_pop_back(&cb);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcast_address, (uint8_t *) single_packet.gps_data.x, single_packet.packet_length);
    Serial.print(millis());Serial.print(" Sending - Length: ");Serial.print(single_packet.packet_length);

    if (result == ESP_OK)
    {
        Serial.println("Sent with success");
    }
    else
    {
        Serial.println("Error sending the data");
    }
    
}

// Push new data onto cicular buffer
void cb_push_front(circular_buffer *cb, char_array item, uint8_t packet_length)
{
    int next_head = cb->head + 1;
    if(next_head >= MAX_NUM_PACKETS)
        next_head = 0;
    if(next_head == cb->tail)
    {
        Serial.println("buffer overflow");
        return;
    }
    cb->gps_data[cb->head] = item;
    cb->packet_length[cb->head] = packet_length;
    cb->head = next_head;
}

// Pop data off circular buffer for use
gps_data_and_length cb_pop_back(circular_buffer *cb)
{
    gps_data_and_length gp;
    
    if(cb->tail == cb->head)
    {
        Serial.println("buffer underflow");
        gp.packet_length = 0;
        gp.gps_data = default_invalid_array;
        return gp;
    }
    gp.packet_length = cb->packet_length[cb->tail];
    gp.gps_data = cb->gps_data[cb->tail];
    cb->tail++;
    if(cb->tail >= MAX_NUM_PACKETS)
        cb->tail = 0;
    return gp;
}
 
