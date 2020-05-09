/**
 * Provide hardware specific implementation using Arduino
 */

#ifdef ARDUINO

  #include <Arduino.h>
  #include "hardware_platform.h"

  void* hardware_platform_malloc(const int8_t size) {
    return (void*) malloc(((size_t) size));
  }

  void hardware_platform_free(void* ptr) {
    free(ptr);
  }

  void hardware_platform_sleep_us(const int8_t value) {
    delayMicroseconds(value);
  }

  void hardware_platform_sleep_ms(const int8_t value) {
    delay(value);
  }

  millisecond_timestamp_t hardware_platform_milliseconds() {
    return (millisecond_timestamp_t) millis();
  }

  void hardware_platform_print_cstring(const char* cstring) {
    Serial.print(cstring);
  }

#endif
