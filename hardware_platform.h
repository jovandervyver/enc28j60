#ifndef __HARDWARE__PLATFORM__H__
#define __HARDWARE__PLATFORM__H__

#include "hardware_platform_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

  #include <stdint.h>

  /**
   * Function to allocate & release memory
   */
  void* hardware_platform_malloc(const int8_t size);
  void hardware_platform_free(void* ptr);

  /**
   * Sleep the given amount of time
   * 
   * Value will always be > 0 and less than INT8_MAX
   */
  void hardware_platform_sleep_us(const int8_t value);
  void hardware_platform_sleep_ms(const int8_t value);

  /**
   * Get some type of elapsed milliseconds from the platform
   */
  uint_least32_t hardware_platform_milliseconds();

  /**
   * If the platform supports any kind of output (serial, monitor, etc.)
   *
   * This function should print the cstrong to that output
   */
  void hardware_platform_print_cstring(const char* cstring);

#ifdef __cplusplus
}
#endif

#endif
