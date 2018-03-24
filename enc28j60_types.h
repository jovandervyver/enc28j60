#ifndef __ENC28J60__TYPES__H__
#define __ENC28J60__TYPES__H__
#ifdef __cplusplus
extern "C" {
#endif

  #include <stdint.h>
  #include "eth_types.h"

  /**
   * Enum defining the ENC28J60 Chip Select current state
   */
    typedef enum {
      ENC28J60_CHIP_SELECT_LOW,
      ENC28J60_CHIP_SELECT_HIGH
    } enc28j60_chip_select_t;

  /**
   * Set the ENC28J60 device Chip Select to the value provided
   */
  typedef void (*enc28j60_chip_select_f)(const enc28j60_chip_select_t new_chip_select_state);

  /**
   * Transfer the provided byte to the ENC28J60 followed by reading the
   * response SPI byte
   */
  typedef uint8_t (*enc28j60_spi_txrx_f)(const uint8_t spi_byte);

  /**
   * Device hardware revision
   */
  typedef uint8_t enc28j60_rev_t;

  typedef enum {
    ENC28J60_SKIP_REVISION_CHECK   = 1<<1,
    ENC28J60_SKIP_SELF_TEST        = 1<<2,
    ENC28J60_FULL_DUPLEX           = 1<<3
  } enc28j60_init_options_t;

  /**
   * Status for init function call
   */
  typedef enum {
    ENC28J60_INIT_SUCCESS,
    ENC28J60_INVALID_CONFIGURATION,
    ENC28J60_MALLOC_FAILURE,
    ENC28J60_SELF_TEST_FAILURE,
    ENC28J60_INVALID_REVISION,
    ENC28J60_CONFIGURE_ERROR,
  } enc28j60_init_status_t;

#ifdef __cplusplus
}
#endif
#endif
