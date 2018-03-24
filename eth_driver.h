#ifndef __ETH__DRIVER__H__
#define __ETH__DRIVER__H__

  #include <stdint.h>
  #include "eth_types.h"

  /**
   * Transmit status
   */
  typedef enum {
    ETH_TX_BUFFER_EMPTY,
    ETH_TX_TIMEOUT,
    ETH_TX_ERROR,
    ETH_TX_SUCCESS
  } tx_status_t;
  
  typedef struct _eth_driver_t eth_driver_t;

  /**
   * Receive the next packet into the packet buffer
   * and return the size of the packet
   *
   * If a packet fails validation the packet is discarded
   *
   * Returns -1 if no valid packets are available
   */
  typedef eth_frm_len_t (*eth_next_rx_packet_f)(void* dev);
  #define ETH_NEXT_RX_PACKET(_driver) (_driver)->eth_next_rx_packet((_driver)->dev)

  /**
   * Read bytes from the packet buffer
   *
   * Reading beyond the size of eth_next_rx_packet_f is undefined
   */
  typedef void (*eth_read_rx_packet_f)(void* dev, uint8_t* output_buffer, eth_frm_len_t buffer_size);
  #define ETH_READ_RX_PACKET(_driver,  _output_buffer, _buffer_size) ((void) (_driver)->eth_read_rx_packet((_driver)->dev, _output_buffer, _buffer_size))

  /**
   * Write to the packet transmit buffer, responds with the number of bytes left in the buffer
   *
   * Returns -1 if attempting to write beyond the buffer
   */
  typedef eth_frm_len_t (*eth_write_tx_buffer_f)(void* dev, const uint8_t* input_buffer, eth_frm_len_t buffer_size);
  #define ETH_WRITE_TX_BUFFER(_driver, _input_buffer, _buffer_size) (_driver)->eth_write_tx_buffer((_driver)->dev, input_buffer, buffer_size)

  /**
   * Send the packet currently in the transmit buffer
   * and reset the transmit buffer so it is empty
   */
  typedef tx_status_t (*eth_tx_start_f)(void* dev);
  #define ETH_TX_START(_driver) (_driver)->eth_tx_start((_driver)->dev)

  struct _eth_driver_t {
    void* dev;

    /**
     * Driver will set these function pointers to implementation
     */
    eth_next_rx_packet_f eth_next_rx_packet;
    eth_read_rx_packet_f eth_read_rx_packet;
    eth_write_tx_buffer_f eth_write_tx_buffer;
    eth_tx_start_f eth_tx_start;
  };

#endif
