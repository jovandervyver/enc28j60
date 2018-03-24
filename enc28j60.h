#ifndef __ENC28J60__H__
#define __ENC28J60__H__
#ifdef __cplusplus
extern "C" {
#endif

  #include <stdint.h>
  #include "enc28j60_types.h"
  #include "eth_driver.h"

  enc28j60_init_status_t enc28j60_eth_driver(
    eth_driver_t* driver,
    enc28j60_chip_select_f chip_select,
    enc28j60_spi_txrx_f spi_txrx,
    const mac_addr_t mac_address,
    const enc28j60_init_options_t options);

  void enc28j60_close(eth_driver_t* driver);

  enc28j60_rev_t enc28j60_hardware_revision(eth_driver_t* driver);
  eth_link_status_t enc28j60_link_status(eth_driver_t* driver);

#ifdef __cplusplus
}
#endif
#endif
