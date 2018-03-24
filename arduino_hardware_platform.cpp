/**
 * Provide hardware specific implementation using Arduino
 */

#ifdef ARDUINO

  #include <Arduino.h>
  #include "hardware_platform.h"

  void* hardware_platform_malloc(const int8_t size) {
    void* result = (void*) malloc((size_t) size);
    return result;
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

  unsigned long hardware_platform_milliseconds() {
    return millis();
  }


  void hardware_platform_print_cstring(const char* cstring) {
    Serial.print(cstring);
  }

#endif
