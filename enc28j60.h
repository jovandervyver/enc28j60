#ifndef __ENC28J60__H__
#define __ENC28J60__H__
#ifdef __cplusplus
extern "C" {
#endif

  #include <stdint.h>

  typedef uint8_t mac_addr_t[6];
  typedef uint8_t ipv4_t[4];
  typedef int16_t eth_frm_len_t;
  typedef uint16_t ethertype_t;

  #ifndef FALSE
    #define FALSE  0
    #define TRUE   1
  #endif

  #define ETHERTYPE_IPV4          0x0800
  #define ETHERTYPE_ARP           0x0806
  #define ETHERTYPE_WOL           0x0842
  #define ETHERTYPE_REVERSE_ARP   0x8035
  #define ETHERTYPE_VLAN_SINGLE   0x8100
  #define ETHERTYPE_IPV6          0x86DD

  typedef enum {
    ENC28J60_DEVICE_ERROR,
    ENC28J60_LINK_DOWN,
    ENC28J60_10M_HALF_DUPLEX,
    ENC28J60_10M_FULL_DUPLEX,
  } enc28j60_link_status_t;

  /**
   * Enum defining the ENC28J60 Chip Select current state
   */
  typedef enum {
    ENC28J60_CHIP_SELECT_LOW,
    ENC28J60_CHIP_SELECT_HIGH
  } enc28j60_chip_select_t;

  /**
   * Device hardware revision
   */
  typedef uint8_t enc28j60_rev_t;

  /**
   * Initialization options
   */
  typedef enum {
    ENC28J60_SKIP_REVISION_CHECK   = 1 << 1,
    ENC28J60_SKIP_SELF_TEST        = 1 << 2,
    ENC28J60_FULL_DUPLEX           = 1 << 3
  } enc28j60_init_options_t;

  /**
   * Status for init function call
   */
  typedef enum {
    ENC28J60_INIT_SUCCESS,
    ENC28J60_INVALID_CONFIGURATION,
    ENC28J60_SELF_TEST_FAILURE,
    ENC28J60_INVALID_REVISION,
    ENC28J60_CONFIGURE_ERROR,
  } enc28j60_init_status_t;

  /**
   * Device status after sending an ethernet frame
   */
  typedef enum {
    ENC28J60_TX_BUFFER_EMPTY,
    ENC28J60_TX_TIMEOUT,
    ENC28J60_TX_ERROR,
    ENC28J60_TX_SUCCESS
  } enc28j60_tx_status_t;

  /**
   * Do not use the default mechanism of passing a chip select function pointer
   * when creating a dev, intead CHIP_SELECT_FUNCTION implements a custom function
   */
  #ifndef CHIP_SELECT_FUNCTION
    #define CUSTOM_CHIP_SELECT_FUNCTION FALSE

    #define CHIP_SELECT_FUNCTION(_dev_, _chip_select_state_) \
      (_dev_)->chip_select_function(_chip_select_state_)

    /**
     * Set the ENC28J60 device Chip Select to the value provided
     */
    typedef void (*enc28j60_chip_select_f)(const enc28j60_chip_select_t new_chip_select_state);

  #else
    #define CUSTOM_CHIP_SELECT_FUNCTION TRUE
  #endif

  /**
   * Do not use the default mechanism of passing a SPI function pointer
   * when creating a dev, intead SPI_TXRX_FUNCTION implements a custom function
   */
  #ifndef SPI_TXRX_FUNCTION
    #define CUSTOM_SPI_TXRX_FUNCTION    FALSE

    #define SPI_TXRX_FUNCTION(_dev_, _spi_data_) \
      (_dev_)->spi_txrx_function(_spi_data_)

    /**
     * Transfer the provided byte to the ENC28J60 followed by reading the
     * response SPI byte
     */
    typedef uint8_t (*enc28j60_spi_txrx_f)(const uint8_t spi_byte);

  #else
    #define CUSTOM_SPI_TXRX_FUNCTION    TRUE
  #endif

  typedef uint8_t  enc28j60_addr_t;
  typedef uint16_t enc28j60_mem_ptr_t;

  /**
   * ENC28J60 device struct
   */
  typedef struct {
    enc28j60_addr_t bank;
    enc28j60_mem_ptr_t next_packet_pointer;
    eth_frm_len_t tx_size;

    #if (CUSTOM_CHIP_SELECT_FUNCTION == FALSE)
      enc28j60_chip_select_f chip_select_function;
    #endif

    #if (CUSTOM_SPI_TXRX_FUNCTION == FALSE)
      enc28j60_spi_txrx_f spi_txrx_function;
    #endif

  } enc28j60_dev_t;

  enc28j60_init_status_t enc28j60_initialize(
    enc28j60_dev_t* dev,

    #if (CUSTOM_CHIP_SELECT_FUNCTION == FALSE)
      enc28j60_chip_select_f chip_select_function,
    #endif

    #if (CUSTOM_SPI_TXRX_FUNCTION == FALSE)
      enc28j60_spi_txrx_f spi_txrx_function,
    #endif

    const mac_addr_t mac_address,
    const enc28j60_init_options_t options);

  eth_frm_len_t enc28j60_next_rx_packet(enc28j60_dev_t* dev);

  void enc28j60_read_rx_packet(enc28j60_dev_t* dev, uint8_t* buffer, const eth_frm_len_t buffer_size);

  eth_frm_len_t enc28j60_write_tx_buffer(enc28j60_dev_t* dev, const uint8_t* buffer, const eth_frm_len_t buffer_size);

  enc28j60_tx_status_t enc28j60_execute_tx(enc28j60_dev_t* dev);

  void enc28j60_close(enc28j60_dev_t* dev);

  enc28j60_rev_t enc28j60_hardware_revision(enc28j60_dev_t* dev);

  void enc28j60_mac_address(enc28j60_dev_t* dev, mac_addr_t mac_address);

  enc28j60_link_status_t enc28j60_link_status(enc28j60_dev_t* dev);

#ifdef __cplusplus
}
#endif
#endif
