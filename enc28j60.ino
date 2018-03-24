#include "enc28j60.h"
#include <SPI.h>

#define ENC28J60_CHIP_SELECT_PIN 10

const mac_addr_t mac = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
eth_driver_t driver;

static void enc28j60ChipSelect(const enc28j60_chip_select_t chipSelect) {
  if (chipSelect == ENC28J60_CHIP_SELECT_LOW) {
    digitalWrite(ENC28J60_CHIP_SELECT_PIN, LOW);
  } else if (chipSelect == ENC28J60_CHIP_SELECT_HIGH) {
    digitalWrite(ENC28J60_CHIP_SELECT_PIN, HIGH);
  }
}

static uint8_t enc28j60SpiTransfer(const uint8_t input) {
  return SPI.transfer(input);
}

static void initENC28J60() {
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  enc28j60_init_status_t status = enc28j60_eth_driver(&driver, &enc28j60ChipSelect, &enc28j60SpiTransfer, mac, ENC28J60_FULL_DUPLEX);

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
  Serial.print(enc28j60_hardware_revision(&driver), DEC);
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
  switch(enc28j60_link_status(&driver)) {
    case ETH_LINK_DOWN: { Serial.print(F("Link down")); break; }
    case ETH_10M_HALF_DUPLEX: { Serial.print(F("Up 10Mb/s Half Duplex")); break; }
    case ETH_10M_FULL_DUPLEX: { Serial.print(F("Up 10Mb/s Full Duplex")); break; }
  }
  Serial.print(F("\n"));

  delay(10000);
}
