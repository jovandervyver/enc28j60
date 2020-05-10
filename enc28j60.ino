#include <SPI.h>

/**
 * Instead of using a function pointer, rely on extern functions
 * to save a bit of memory on Uno
 */
#define CHIP_SELECT_FUNCTION(_dev_, _chip_select_state_) \
      enc28j60ChipSelect(_chip_select_state_)

#define SPI_TXRX_FUNCTION(_dev_, _spi_data_) \
      enc28j60SpiTransfer(_spi_data_)

#include "enc28j60.h"

#define ENC28J60_CHIP_SELECT_PIN 10

const mac_addr_t mac = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
enc28j60_dev_t dev;

extern void enc28j60ChipSelect(const enc28j60_chip_select_t chipSelect) {
  if (chipSelect == ENC28J60_CHIP_SELECT_LOW) {
    digitalWrite(ENC28J60_CHIP_SELECT_PIN, LOW);
  } else if (chipSelect == ENC28J60_CHIP_SELECT_HIGH) {
    digitalWrite(ENC28J60_CHIP_SELECT_PIN, HIGH);
  }
}

extern uint8_t enc28j60SpiTransfer(const uint8_t input) {
  return SPI.transfer(input);
}

static void initENC28J60() {
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  enc28j60_init_status_t status = enc28j60_initialize(&dev, mac, ENC28J60_FULL_DUPLEX);

  Serial.print(F("Init status: "));
  switch(status) {
    case ENC28J60_INIT_SUCCESS: { Serial.print(F("Init success")); break; }
    case ENC28J60_INVALID_CONFIGURATION: { Serial.print(F("Init failure: Invalid input")); return; }
    case ENC28J60_SELF_TEST_FAILURE: { Serial.print(F("Init failure: BIST failed")); return; }
    case ENC28J60_INVALID_REVISION: { Serial.print(F("Init failure: Unexpected revision")); return; }
    case ENC28J60_CONFIGURE_ERROR: { Serial.print(F("Init failure: Device configuration failed")); return; }
  }
  Serial.print(F("\n"));

  Serial.print(F("Device revision:"));
  Serial.print(enc28j60_hardware_revision(&dev), DEC);
  Serial.print(F("\n"));

}

void setup() {
  Serial.begin(115200);
  while(!Serial) {};

  initENC28J60();
  Serial.print("Init complete\n");
}

void printMacAddress(const mac_addr_t macAddress) {
  Serial.print(macAddress[0], HEX);
  for(uint8_t index = 1; index < sizeof(mac_addr_t); ++index) {
    Serial.print(F(":"));
    Serial.print(macAddress[index], HEX);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print(F("Link status:"));
  switch(enc28j60_link_status(&dev)) {
    case ENC28J60_LINK_DOWN: { Serial.print(F("Link down")); break; }
    case ENC28J60_10M_HALF_DUPLEX: { Serial.print(F("Up 10Mb/s Half Duplex")); break; }
    case ENC28J60_10M_FULL_DUPLEX: { Serial.print(F("Up 10Mb/s Full Duplex")); break; }
  }
  Serial.print(F("\n"));

  eth_frm_len_t next_frame_length = 0;
  do {
    next_frame_length = enc28j60_next_rx_packet(&dev);
    if (next_frame_length > 0) {
      Serial.print(F("New packet received\n"));

      char* buffer = malloc(next_frame_length + 1);
      enc28j60_read_rx_packet(&dev, (uint8_t*) buffer, next_frame_length);
      buffer[next_frame_length] = 0;
      Serial.print(buffer);
      free(buffer);
      Serial.print(F("\n"));
    }
  } while(next_frame_length > 0);

  /**
   * Invalid packet but just excercise the code
   */
  uint8_t buffer[16];
  enc28j60_write_tx_buffer(&dev, buffer, 16);
  enc28j60_execute_tx(&dev);

  delay(10000);
}
