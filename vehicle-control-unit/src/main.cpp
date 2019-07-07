#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.


// Pin assignments.
constexpr uint8_t rf24_ce_pin = 10;
constexpr uint8_t spi_sck_pin = 14;
constexpr uint8_t rf24_csn_pin = 15;

RF24 nrf24l01(rf24_ce_pin, rf24_csn_pin);
byte nrf24l01_addresses[][5] = {"HMI0", "VCU0"};


void setup()
{
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Setup SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Setup nRF24L01+.
  nrf24l01.begin();
  nrf24l01.setPALevel(RF24_PA_LOW);
  nrf24l01.setDataRate(RF24_250KBPS);
  nrf24l01.setChannel(0x01);
  nrf24l01.setRetries(4, 10);
  nrf24l01.setAddressWidth(4);
  nrf24l01.setPayloadSize(8);
  nrf24l01.openWritingPipe(nrf24l01_addresses[1]);
  nrf24l01.openReadingPipe(1, nrf24l01_addresses[0]);
  nrf24l01.startListening();
  // Setup debug console.
  Serial.begin(9600);
  // DEBUG.
  while(!Serial)
  {
    ;
  }
  nrf24l01.printDetails();
  // END DEBUG.
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


struct ControlMessage
{
  uint8_t message_type;
  uint16_t input_steering;
  uint16_t input_throttle;
  uint8_t padding[3];
};


void loop()
{
  if(nrf24l01.available())
  {
    ControlMessage control_message;
    nrf24l01.read(&control_message, 8);
    Serial.printf("received: message type: %2x steering: %4u , throttle = %4u.\n",
      control_message.message_type, control_message.input_steering, control_message.input_throttle);
  }
}