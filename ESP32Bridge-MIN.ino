#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include <stdlib.h>

// ---- Wi-Fi Parameters ----
const char* ssid = "TESLATERM";    // Access Point SSID
const char* password = "12345678"; // Access Point Password

// ---- UDP Parameters ----
const int udp_port = 1337;   // UDP listening port
const int local_port = 1337; // Local UDP port

// ---- Serial Parameters ----
#define RX_PIN RX          // ESP32-S3 RX pin (UART0)
#define TX_PIN TX          // ESP32-S3 TX pin (UART0)
#define BAUDRATE 460800    // Serial communication speed

// Define the maximum MIN frame size
#define MAX_PAYLOAD 512

// Access Point network configuration
IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Global objects
WiFiUDP udp;
HardwareSerial SerialPort(1);

// Store the last known UDP sender IP/port
IPAddress remoteIP;
uint16_t remotePort = 0;

// -----------------------------------------------------------------------------
//  PART 1: Set up the Access Point
// -----------------------------------------------------------------------------
void startAccessPoint() {
  WiFi.softAPConfig(apIP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("Wi-Fi AP launched, IP: ");
  Serial.println(WiFi.softAPIP());
}

// -----------------------------------------------------------------------------
//  PART 2: MIN protocol essentials (state machine definitions)
// -----------------------------------------------------------------------------

// Constants for the MIN protocol
enum {
  HEADER_BYTE = 0xaaU,
  STUFF_BYTE  = 0x55U, 
  EOF_BYTE    = 0x55U,
};

// Receiver state machine states
enum {
  SEARCHING_FOR_SOF,
  RECEIVING_ID_CONTROL,
  RECEIVING_SEQ_3,
  RECEIVING_SEQ_2,
  RECEIVING_SEQ_1,
  RECEIVING_SEQ_0,
  RECEIVING_LENGTH,
  RECEIVING_PAYLOAD,
  RECEIVING_CHECKSUM_3,
  RECEIVING_CHECKSUM_2,
  RECEIVING_CHECKSUM_1,
  RECEIVING_CHECKSUM_0,
  RECEIVING_EOF,
};

// Context structure for each direction of MIN
struct min_context {
  // To detect three consecutive 0xAA bytes
  uint8_t rx_header_bytes_seen;

  // Current state of the state machine
  uint8_t rx_frame_state;

  // Payload length fields
  uint8_t rx_frame_payload_bytes;
  uint8_t rx_frame_length;

  // ID/control info
  uint8_t rx_frame_id_control;
  uint8_t rx_control;

  // Position in the forwarding buffer
  uint16_t rx_data_position;

  // Buffer holding the entire MIN frame
  uint8_t* rx_forward_buffer;

  // Indicates if this context is from the Serial side or from the UDP side
  bool fromSerial;
};

// -----------------------------------------------------------------------------
//  FUNCTION: Called when a valid MIN frame has been received
// -----------------------------------------------------------------------------
void valid_frame_received(struct min_context *self) {
  // The total size of the frame is from index 0 up to rx_data_position - 1
  int frame_size = self->rx_data_position;

  if (self->fromSerial) {
    // If it came from Serial, forward it to the UDP side (if remote port is known)
    if (remotePort != 0) {
      udp.beginPacket(remoteIP, remotePort);
      udp.write(self->rx_forward_buffer, frame_size);
      udp.endPacket();
    }
  } else {
    // If it came from UDP, forward it to the Serial side
    SerialPort.write(self->rx_forward_buffer, frame_size);
  }
}

// -----------------------------------------------------------------------------
//  FUNCTION: Process incoming bytes through the MIN state machine
// -----------------------------------------------------------------------------
void min_rx_byte(struct min_context *self, uint8_t byte) {
  if (self->rx_forward_buffer == NULL) return;

  // If we've already seen "aa aa", then the next byte may trigger the start of frame
  if (self->rx_header_bytes_seen == 2) {
    self->rx_header_bytes_seen = 0;

    // If we see a third "aa", then it's the start of a new frame
    if (byte == HEADER_BYTE) {
      self->rx_frame_state = RECEIVING_ID_CONTROL;
      // Reset the buffer
      self->rx_data_position = 3;
      self->rx_forward_buffer[0] = 0xaa;
      self->rx_forward_buffer[1] = 0xaa;
      self->rx_forward_buffer[2] = 0xaa;
      return;
    }

    // If it's a stuffing byte, store it and continue
    if (byte == STUFF_BYTE) {
      self->rx_forward_buffer[self->rx_data_position++] = byte;
      return;
    }

    // Otherwise, this is not valid, go back to searching
    self->rx_frame_state = SEARCHING_FOR_SOF;
    return;
  }

  // Check if we have an 0xaa byte
  if (byte == HEADER_BYTE) {
    self->rx_header_bytes_seen++;
  } else {
    self->rx_header_bytes_seen = 0;
  }

  // Store this byte into the buffer (we're preserving the frame in its entirety)
  if (self->rx_data_position < MAX_PAYLOAD) {
    self->rx_forward_buffer[self->rx_data_position++] = byte;
  } else {
    // If we exceed MAX_PAYLOAD, we reset to avoid overflow
    self->rx_frame_state = SEARCHING_FOR_SOF;
    return;
  }

  // Advance the state machine
  switch(self->rx_frame_state) {
    case SEARCHING_FOR_SOF:
      // While searching, we don't accumulate anything unless 3 x 0xAA appear
      self->rx_data_position = 0;
      break;

    case RECEIVING_ID_CONTROL:
      self->rx_frame_id_control = byte;
      self->rx_frame_payload_bytes = 0;
      if (byte & 0x80U) {
        self->rx_frame_state = RECEIVING_SEQ_3;
      } else {
        self->rx_frame_state = RECEIVING_LENGTH;
      }
      break;

    case RECEIVING_SEQ_3:
      self->rx_frame_state = RECEIVING_SEQ_2;
      break;

    case RECEIVING_SEQ_2:
      self->rx_frame_state = RECEIVING_SEQ_1;
      break;

    case RECEIVING_SEQ_1:
      self->rx_frame_state = RECEIVING_SEQ_0;
      break;

    case RECEIVING_SEQ_0:
      self->rx_frame_state = RECEIVING_LENGTH;
      break;

    case RECEIVING_LENGTH:
      self->rx_frame_length = byte;
      self->rx_control = byte;
      if (self->rx_frame_length > 0) {
        if (self->rx_frame_length <= MAX_PAYLOAD) {
          self->rx_frame_state = RECEIVING_PAYLOAD;
        } else {
          // Frame is too long, reset
          self->rx_frame_state = SEARCHING_FOR_SOF;
        }
      } else {
        // Length = 0 -> go straight to checksum
        self->rx_frame_state = RECEIVING_CHECKSUM_3;
      }
      break;

    case RECEIVING_PAYLOAD:
      if (--self->rx_frame_length == 0) {
        self->rx_frame_state = RECEIVING_CHECKSUM_3;
      }
      break;

    case RECEIVING_CHECKSUM_3:
      self->rx_frame_state = RECEIVING_CHECKSUM_2;
      break;

    case RECEIVING_CHECKSUM_2:
      self->rx_frame_state = RECEIVING_CHECKSUM_1;
      break;

    case RECEIVING_CHECKSUM_1:
      self->rx_frame_state = RECEIVING_CHECKSUM_0;
      break;

    case RECEIVING_CHECKSUM_0:
      self->rx_frame_state = RECEIVING_EOF;
      break;

    case RECEIVING_EOF:
      // Must see 0x55 at the end
      if (byte == EOF_BYTE) {
        // A valid frame has been received
        valid_frame_received(self);
      }
      // In any case, go back to searching for new frames
      self->rx_frame_state = SEARCHING_FOR_SOF;
      break;

    default:
      self->rx_frame_state = SEARCHING_FOR_SOF;
      break;
  }
}

// -----------------------------------------------------------------------------
//  PART 3: FreeRTOS tasks
// -----------------------------------------------------------------------------

// 3.1 - Task reading from Serial and parsing MIN, then sending to UDP
void serialToMin(void* parameter) {
  // Create a MIN context for the Serial->UDP direction
  struct min_context min_ctx_serial;
  min_ctx_serial.rx_forward_buffer = (uint8_t*)malloc(MAX_PAYLOAD);
  min_ctx_serial.rx_header_bytes_seen = 0;
  min_ctx_serial.rx_frame_state = SEARCHING_FOR_SOF;
  min_ctx_serial.fromSerial = true;

  while(true) {
    int len = SerialPort.available();
    if (len > 0) {
      for (int i = 0; i < len; i++) {
        uint8_t b = SerialPort.read();
        min_rx_byte(&min_ctx_serial, b);
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// 3.2 - Task reading from UDP and parsing MIN, then sending to Serial
void udpToMin(void* parameter) {
  // Create a MIN context for the UDP->Serial direction
  struct min_context min_ctx_udp;
  min_ctx_udp.rx_forward_buffer = (uint8_t*)malloc(MAX_PAYLOAD);
  min_ctx_udp.rx_header_bytes_seen = 0;
  min_ctx_udp.rx_frame_state = SEARCHING_FOR_SOF;
  min_ctx_udp.fromSerial = false;

  // Temporary buffer for reading incoming UDP packets in bulk
  uint8_t tempBuf[512];

  while(true) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      remoteIP = udp.remoteIP();     // Remember the sender's IP
      remotePort = udp.remotePort(); // Remember the sender's port

      // Read the incoming packet into tempBuf
      int toRead = (packetSize > 512) ? 512 : packetSize;
      int len = udp.read(tempBuf, toRead);

      // Feed each byte into the MIN state machine
      for (int i = 0; i < len; i++) {
        min_rx_byte(&min_ctx_udp, tempBuf[i]);
      }

      // If packetSize > 512, any extra data is not processed in this example,
      // but you could handle that in a loop if needed.
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// -----------------------------------------------------------------------------
//  PART 4: setup() and loop()
// -----------------------------------------------------------------------------
void setup() {
  // Initialize the USB serial (for debug)
  Serial.begin(BAUDRATE);

  // Start Wi-Fi Access Point
  startAccessPoint();

  // Initialize the hardware serial (bridge)
  SerialPort.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start listening for UDP packets
  udp.begin(local_port);

  // Create FreeRTOS tasks (assigned to different cores)
  xTaskCreatePinnedToCore(serialToMin, "Serial_to_MIN", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udpToMin,    "UDP_to_MIN",    4096, NULL, 1, NULL, 0);
}

void loop() {
  // Everything is handled by FreeRTOS tasks
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
