#include <stddef.h>
#include "enc28j60.h"
#include "enc28j60_constants.h"
#include "hardware_platform.h"

typedef uint8_t enc28j60_reg_t;
typedef uint8_t enc28j60_addr_t;
typedef uint16_t enc28j60_mem_ptr_t;

typedef struct {
  enc28j60_addr_t bank;
  enc28j60_mem_ptr_t next_packet_pointer;
  eth_frm_len_t tx_size;
  enc28j60_chip_select_f chip_select_function;
  enc28j60_spi_txrx_f spi_function;
} enc28j60_dev_t;

#define ENC28J60_DEV(eth_driver_ptr) ((enc28j60_dev_t*) eth_driver_ptr->dev)

#define ENC28J60_CS_LOW(dev_ptr) dev_ptr->chip_select_function(ENC28J60_CHIP_SELECT_LOW);
#define ENC28J60_CS_HIGH(dev_ptr) dev_ptr->chip_select_function(ENC28J60_CHIP_SELECT_HIGH);

#define ENC28J60_WRITE_BYTE(dev_ptr, val) (void) dev_ptr->spi_function((uint8_t) val)
#define ENC28J60_READ_BYTE(dev_ptr) dev_ptr->spi_function(((uint8_t) 0x00u))

/**
 * Implementation
 */

static inline uint16_t enc28j60_uint16(const uint8_t low, const uint8_t high) {
  /**
   * ENC28J60 is little endian
   */
   return ((uint16_t) low) | (((uint16_t) high) << 8);
}

static inline uint16_t enc28j60_erxrdpt_errata(uint16_t value) {
  /**
   * From errata:
   *
   *  Module: Memory (Ethernet Buffer)
   *
   * The receive hardware may corrupt the circular receive buffer (including the
   * Next Packet Pointer and receive status vector fields) when an even value
   * is programmed into the ERXRDPTH:ERXRDPTL registers.
   *
   * Work around:
   *
   * Ensure that only odd addresses are written to the ERXRDPT registers.
   * Assuming that ERXND contains an odd value, many applications can derive
   * a suitable value to write to ERXRDPT by subtracting one from the 
   * Next Packet Pointer (a value always ensured to be even because of hardware padding)
   * and then compensating for a potential ERXST to ERXND wraparound.
   *
   * Assuming that the receive buffer area does not span the 1FFFh to 0000h memory 
   * boundary, the logic in Example 1 will ensure that ERXRDPT is programmed with an odd value:
   *
   * EXAMPLE1:
   *
   * if (Next Packet Pointer – 1 < ERXST) or (Next Packet Pointer – 1 > ERXND)
   *  then:
   *    ERXRDPT = ERXND
   *  else:
   *    ERXRDPT = Next Packet Pointer – 1
   */

  #if (0 == (ENC28J60_RXEND % 2))
    #error "ENC28J60_RXEND must be an uneven number"
  #endif

  #if (0 == ENC28J60_RXSTART)
    if (0 == value) return ENC28J60_RXEND;
    --value;
    if (value > ENC28J60_RXEND) return ENC28J60_RXEND;
    return value;
  #else
    --value;
    if ((value < ENC28J60_RXSTART) || (value > ENC28J60_RXEND)) {
      return ENC28J60_RXEND;
    } else {
      return value;
    }
  #endif
}

static void enc28j60_write_op(enc28j60_dev_t* device, const enc28j60_reg_t op, const enc28j60_addr_t address, const uint8_t value) {
  ENC28J60_CS_LOW(device);

  ENC28J60_WRITE_BYTE(device, (op | (address & ADDR_MASK)));
  ENC28J60_WRITE_BYTE(device, value);

  ENC28J60_CS_HIGH(device);
}

static uint8_t enc28j60_read_op(enc28j60_dev_t* device, const enc28j60_reg_t op, const enc28j60_addr_t address) {
  ENC28J60_CS_LOW(device);

  ENC28J60_WRITE_BYTE(device, (op | (address & ADDR_MASK)));
  if (address & SPRD_MASK) {
    ENC28J60_WRITE_BYTE(device, ((uint8_t) 0x00u));
  }
  const uint8_t value = ENC28J60_READ_BYTE(device);

  ENC28J60_CS_HIGH(device);

  return value;
}

static void enc28j60_set_bank(enc28j60_dev_t* device, enc28j60_addr_t address) {
  /**
   * Registers EIE, EIR, ESTAT, ECON2, ECON1
   * are present in all banks, no need to switch bank
   */
  address = address & BANK_MASK;

  if (address == ALL_BANKS) return;
  if (address == device->bank) return;

  device->bank = address;

  ENC28J60_CS_LOW(device);
  ENC28J60_WRITE_BYTE(device, (ENC28J60_BIT_FIELD_CLR | ADDR_MASK));
  ENC28J60_WRITE_BYTE(device, (ECON1_BSEL1 | ECON1_BSEL0));
  ENC28J60_CS_HIGH(device);

  ENC28J60_CS_LOW(device);
  ENC28J60_WRITE_BYTE(device, (ENC28J60_BIT_FIELD_SET | ADDR_MASK));
  ENC28J60_WRITE_BYTE(device, (address >> 5));
  ENC28J60_CS_HIGH(device);
}

static void enc28j60_set_register_bit(enc28j60_dev_t* device, const enc28j60_addr_t address, const uint8_t mask) {
  enc28j60_set_bank(device, address);
  enc28j60_write_op(device, ENC28J60_BIT_FIELD_SET, address, mask);
}

static void enc28j60_clear_register_bit(enc28j60_dev_t* device, const enc28j60_addr_t address, const uint8_t mask) {
  enc28j60_set_bank(device, address);
  enc28j60_write_op(device, ENC28J60_BIT_FIELD_CLR, address, mask);
}

static void enc28j60_write_register_byte(enc28j60_dev_t* device, const enc28j60_addr_t address, const uint8_t value) {
  enc28j60_set_bank(device, address);
  enc28j60_write_op(device, ENC28J60_WRITE_CTRL_REG, address, value);
}

static uint8_t enc28j60_read_register_byte(enc28j60_dev_t* device, const enc28j60_addr_t address) {
  enc28j60_set_bank(device, address);
  return enc28j60_read_op(device, ENC28J60_READ_CTRL_REG, address);
}

static void enc28j60_write_register_word(enc28j60_dev_t* device, const enc28j60_addr_t address, const uint16_t value) {
  enc28j60_set_bank(device, address);

  enc28j60_write_op(device, ENC28J60_WRITE_CTRL_REG, address, (uint8_t) (value & 0xFF));
  enc28j60_write_op(device, ENC28J60_WRITE_CTRL_REG, address + 1, (uint8_t) ((value >> 8) & 0xFF));
}

static uint16_t enc28j60_read_register_word(enc28j60_dev_t* device, const enc28j60_addr_t address) {
  enc28j60_set_bank(device, address);

  const uint8_t low = enc28j60_read_op(device, ENC28J60_READ_CTRL_REG, address);
  const uint8_t high = enc28j60_read_op(device, ENC28J60_READ_CTRL_REG, address + 1);

  return enc28j60_uint16(low, high);
}

static void enc28j60_reset(enc28j60_dev_t* device) {
 /**
  * Rev. B7 Silicon Errata:
  *
  * After sending an SPI Reset command, the PHY clock is stopped
  * but the ESTAT.CLKRDY bit is not cleared. Therefore,
  * polling the CLKRDY bit will not work to detect if
  * the PHY is ready.
  *
  * Additionally, the hardware start-up time of 300 μs
  * may expire before the device is ready to operate.
  *
  * Work around:
  * After issuing the Reset command, wait for at least
  * 1 ms in firmware for the device to be ready.
  */

  ENC28J60_CS_LOW(device);
  ENC28J60_WRITE_BYTE(device, ENC28J60_SOFT_RESET);
  ENC28J60_CS_HIGH(device);

  hardware_platform_sleep_ms(1);
  uint8_t count = 6;
  do { // Sleep 300ms
    hardware_platform_sleep_us(50);
  } while(--count);

  /**
   * Whenever a reset is done, ECON1 will inevitable need to be cleared next
   */
  enc28j60_write_register_byte(device, ECON1, 0x00u);
}

static void enc28j60_buffer_write(enc28j60_dev_t* device, const uint8_t* bytes, eth_frm_len_t length) {
  ENC28J60_CS_LOW(device);
  ENC28J60_WRITE_BYTE(device, ENC28J60_WRITE_BUF_MEM);

  do {
    ENC28J60_WRITE_BYTE(device, *bytes);
    ++bytes;
  } while(--length);

  ENC28J60_CS_HIGH(device);
}

static void enc28j60_buffer_read(enc28j60_dev_t* device, uint8_t* bytes, eth_frm_len_t length) {
  ENC28J60_CS_LOW(device);
  ENC28J60_WRITE_BYTE(device, ENC28J60_READ_BUF_MEM);

  do {
    *bytes = ENC28J60_READ_BYTE(device);
    ++bytes;
  } while(--length);

  ENC28J60_CS_HIGH(device);
}

static inline uint8_t enc28j60_phy_wait(enc28j60_dev_t* device) {
  uint8_t timeout = 4;

  do {
    /*
     * From datasheet: Wait 10.24 μs.
     * Poll the MISTAT.BUSY bit to be certain that the operation is complete.
     *
     * Adding a safety margin
     */

    hardware_platform_sleep_us(14);

    if (likely(!enc28j60_read_register_byte(device, MISTAT) & MISTAT_BUSY)) {
      return 1;
    }

  } while(--timeout);

  return 0;
}

static uint8_t enc28j60_phy_write(enc28j60_dev_t* device, const enc28j60_addr_t address, const uint16_t value) {
  enc28j60_write_register_byte(device, MIREGADR, address);
  enc28j60_write_register_word(device, MIWR, value);
  return enc28j60_phy_wait(device);
}

static uint16_t enc28j60_phy_read(enc28j60_dev_t* device, const enc28j60_addr_t address) {
  enc28j60_write_register_byte(device, MIREGADR, address);
  enc28j60_write_register_byte(device, MICMD, MICMD_MIIRD);

  if (likely(enc28j60_phy_wait(device))) {
    enc28j60_write_register_byte(device, MICMD, 0x00u);
    return enc28j60_read_register_word(device, MIRD);
  }

  return 0;
}

static uint8_t enc28j60_configure(enc28j60_dev_t* device, const mac_addr_t mac_address, const uint8_t full_duplex) {
  enc28j60_reset(device);

  /**
   * Auto increment the buffer pointer
   */
  enc28j60_write_register_byte(device, ECON2, ECON2_AUTOINC);

  /**
   * Set the ENC28J60 receive buffer start and end address
   */
  enc28j60_write_register_word(device, ERXST, ENC28J60_RXSTART);
  enc28j60_write_register_word(device, ERXND, ENC28J60_RXEND);

  /**
   * Set the next packet pointer
   */
  device->next_packet_pointer = ENC28J60_RXSTART;
  enc28j60_write_register_word(device, ERXRDPT, ENC28J60_RXSTART);

  /**
   * Set the ENC28J60 transmit buffer start and end address
   */
  enc28j60_write_register_word(device, ETXST, ENC28J60_TXSTART);
  enc28j60_write_register_word(device, ETXND, ENC28J60_TXEND);

  /**
   * ERXFCON: Receive filter
   * 
   * UCEN: Unicast Filter Enable bit
   *   1 = Packets with a destination address matching the local MAC address will be accepted
   *   0 = Filter disabled
   *
   * ANDOR: AND/OR Filter Select bit
   *   1 = AND: Packets will be rejected unless all enabled filters accept the packet
   *   0 = OR: Packets will be accepted unless all enabled filters reject the packet
   *
   * CRCEN: Post-Filter CRC Check Enable bit
   *   1 = All packets with an invalid CRC will be discarded
   *   0 = The CRC validity will be ignored
   *
   * PMEN: Pattern Match Filter Enable bit
   *   1 = Packets which meet the Pattern Match criteria will be accepted
   *   0 = Filter disabled
   *
   * MPEN: Magic Packetô Filter Enable bit
   *   1 = Magic Packets for the local MAC address will be accepted
   *   0 = Filter disabled
   *
   * HTEN: Hash Table Filter Enable bit
   *   1 = Packets which meet the Hash Table criteria will be accepted
   *   0 = Filter disabled
   *
   * MCEN: Multicast Filter Enable bit
   *   1 = Packets which have the Least Significant bit set in the destination address will be accepted
   *   0 = Filter disabled
   *
   * BCEN: Broadcast Filter Enable bit
   *   1 = Packets which have a destination address of FF-FF-FF-FF-FF-FF will be accepted
   *   0 = Filter disabled
   */
  enc28j60_write_register_byte(device, ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

  /**
   * Initialize MAC
   */

  /**
   * 1. Set the MARXEN bit in MACON1 to enable the MAC to receive frames. If using full duplex, most
   * applications should also set TXPAUS and RXPAUS to allow IEEE defined flow control to
   * function.
   */
  enc28j60_write_register_byte(device, MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

  /**
   * 2. Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3. Most applications
   * should enable automatic padding to at least 60 bytes and always append a valid CRC. For
   * convenience, many applications may wish to set the FRMLNEN bit as well to enable frame length
   * status reporting. The FULDPX bit should be set if the application will be connected to a
   * full-duplex configured remote node; otherwise, it should be left clear.
   */
  if (full_duplex) {
    enc28j60_write_register_byte(device, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX);
  } else {
    enc28j60_write_register_byte(device, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);

    /**
     * 3. Configure the bits in MACON4. For conformance to the IEEE 802.3 standard, set the DEFER bit.
     */
    enc28j60_write_register_byte(device, MACON4, MACON4_DEFER);
  }

  /**
   * 4. Program the MAMXFL registers with the maximum frame length to be permitted to be received
   * or transmitted. Normal network nodes are designed to handle packets that are 1518 bytes
   * or less.
   */
  enc28j60_write_register_word(device, MAMXFLL, ENC28J60_MAX_FRAME_SIZE);

  /**
   * 5. Configure the Back-to-Back Inter-Packet Gap register, MABBIPG. Most applications will program
   * this register with 15h when Full-Duplex mode is used and 12h when Half-Duplex mode is used.
   *
   * 6. Configure the Non-Back-to-Back Inter-Packet Gap register low byte, MAIPGL.
   * Most applications will program this register with 12h.
   *
   * 7. If half duplex is used, the Non-Back-to-Back Inter-Packet Gap register high byte, MAIPGH,
   * should be programmed. Most applications will program this register to 0Ch.
   */
  if (full_duplex) {
    enc28j60_write_register_word(device, MAIPG, 0x0012u);
    enc28j60_write_register_byte(device, MABBIPG, 0x15u);
  } else {
    enc28j60_write_register_word(device, MAIPGL, 0x0C12u);
    enc28j60_write_register_byte(device, MABBIPG, 0x12u);
  }

  /**
   * 8. If Half-Duplex mode is used, program the Retransmission and Collision Window registers,
   * MACLCON1 and MACLCON2. Most applications will not need to change the default Reset values.
   * If the network is spread over exceptionally long cables, the default value of MACLCON2 may
   * need to be increased.
   */
  enc28j60_write_register_byte(device, MACON2, 0x00u);

  /**
   * 9. Program the local MAC address into the MAADR1:MAADR6 registers. 
   */
  enc28j60_write_register_byte(device, MAADR5, mac_address[0]);
  enc28j60_write_register_byte(device, MAADR4, mac_address[1]);
  enc28j60_write_register_byte(device, MAADR3, mac_address[2]);
  enc28j60_write_register_byte(device, MAADR2, mac_address[3]);
  enc28j60_write_register_byte(device, MAADR1, mac_address[4]);
  enc28j60_write_register_byte(device, MAADR0, mac_address[5]);

  do {
    if (!enc28j60_phy_write(device, PHLCON, ENC28J60_LAMPS_MODE)) break;

    if (full_duplex) {
      if (!enc28j60_phy_write(device, PHCON1, PHCON1_PDPXMD)) break;
      if (!enc28j60_phy_write(device, PHCON2, 0x00)) break;
    } else {
      if (!enc28j60_phy_write(device, PHCON1, 0x00)) break;
      if (!enc28j60_phy_write(device, PHCON2, PHCON2_HDLDIS)) break;
    }

    if (!enc28j60_phy_write(device, PHIE, 0)) break;

    enc28j60_clear_register_bit(device, EIR, EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);
    enc28j60_clear_register_bit(device, EIE, EIE_INTIE | EIE_PKTIE | EIE_DMAIE | EIE_LINKIE | EIE_TXIE | EIE_TXERIE | EIE_RXERIE);

    /**
     * RXEN: Receive Enable bit
     *   1 = Packets which pass the current filter configuration will be written into the receive buffer
     *   0 = All packets received will be ignored
     */
    enc28j60_set_register_bit(device, ECON1, ECON1_RXEN);

    /**
     * Set the transmit start pointer and write the initial header byte
     */
    enc28j60_write_register_word(device, EWRPT, ENC28J60_TXSTART);
    enc28j60_write_op(device, ENC28J60_WRITE_BUF_MEM, 0x00u, 0x00u);
    device->tx_size = 0;

    /**
     * Wait time for the link to become active
     */
    hardware_platform_sleep_ms(100);

    return 1;
  } while(0);

  return 0;
}

static eth_frm_len_t enc28j60_next_rx_packet(eth_driver_t* driver) {
  while(enc28j60_read_register_byte(ENC28J60_DEV(driver), EPKTCNT)) {
    /**
     * Advance the packet pointer to the start of the next packet
     */
    enc28j60_write_register_word(ENC28J60_DEV(driver), ERXRDPT, enc28j60_erxrdpt_errata(ENC28J60_DEV(driver)->next_packet_pointer));

    /**
     * Set the read pointer to the start of the next packet
     */
    enc28j60_write_register_word(ENC28J60_DEV(driver), ERDPT, ENC28J60_DEV(driver)->next_packet_pointer);

    /**
     * Parse the receive status vector
     *   bytes 1-2: next packet pointer
     *   bytes 3-4: packet length
     *   bytes 5-6: status bits
     */
    uint8_t bytes[RSV_SIZE];
    enc28j60_buffer_read(ENC28J60_DEV(driver), bytes, RSV_SIZE);

    ENC28J60_DEV(driver)->next_packet_pointer = (enc28j60_mem_ptr_t) enc28j60_uint16(bytes[0], bytes[1]);

    /**
     * Decrement the packet counter by 1
     */
    enc28j60_set_register_bit(ENC28J60_DEV(driver), ECON2, ECON2_PKTDEC);

    if (RSV_RX_OK & bytes[4]) {
      return (eth_frm_len_t) enc28j60_uint16(bytes[2], bytes[3]);
    }
  }

  return -1;
}

static void enc28j60_read_rx_packet(eth_driver_t* driver, uint8_t* output_buffer, eth_frm_len_t buffer_size) {
  if (0 >= buffer_size) return;
  if (unlikely(buffer_size >= ENC28J60_MAX_FRAME_SIZE)) return;

  enc28j60_buffer_read(ENC28J60_DEV(driver), output_buffer, buffer_size);
}

static uint8_t enc28j60_execute_tx(enc28j60_dev_t* device) {
  const unsigned long start_millis = hardware_platform_milliseconds();
  
  /**
    * Start the transmission process by setting ECON1.TXRTS
    */
  enc28j60_set_register_bit(device, ECON1, ECON1_TXRTS);

  /**
    * Wait for transmit to complete
    */
  uint8_t timeout = 1;
  do {
    const uint8_t eir = enc28j60_read_register_byte(device, EIR);
    if (eir & (EIR_TXIF | EIR_TXERIF)) {
      timeout = 0;
      break;
    }
  } while(ENC28J60_TRANSMIT_TIMEOUT_MS > (hardware_platform_milliseconds() - start_millis));

  /**
   * From errata:
   * 
   *  Module: Transmit Logic
   *  
   *  In Half-Duplex mode, a hardware transmission abort – caused by excessive collisions, a late collision
   *  or excessive deferrals – may stall the internal transmit logic. The next packet transmit initiated by
   *  the host controller may never succeed. That is, ECON1.TXRTS could remain set indefinitely.
   *  
   *  Work around
   *  Before attempting to transmit a packet (setting ECON1.TXRTS), reset the internal transmit logic
   *  by setting ECON1.TXRST and then clearing ECON1.TXRST. The host controller may wish to
   *  issue this Reset before any packet is transmitted (for simplicity), or it may wish to conditionally reset
   *  the internal transmit logic based on the Transmit Error Interrupt Flag (EIR.TXERIF), which will
   *  become set whenever a transmit abort occurs.
   *  
   *  Clearing ECON1.TXRST may cause a new transmit error interrupt event (with EIR.TXERIF
   *  becoming set). Therefore, the interrupt flag should be cleared after the Reset is completed.
   */
  enc28j60_set_register_bit(device, ECON1, ECON1_TXRST);
  enc28j60_clear_register_bit(device, ECON1, ECON1_TXRST);
  enc28j60_clear_register_bit(device, EIR, EIR_TXERIF | EIR_TXIF | ECON1_TXRTS);

  if (timeout) {
    return UINT8_MAX;
  }

  enc28j60_write_register_word(device, ERDPT, ENC28J60_TXSTART + 1 + device->tx_size);

  uint8_t tsv[3];
  enc28j60_buffer_write(device, tsv, sizeof(tsv));

  enc28j60_write_register_word(device, ERDPT, device->next_packet_pointer);

  return tsv[2];
}

static eth_frm_len_t enc28j60_write_tx_buffer(eth_driver_t* driver, const uint8_t* buffer, eth_frm_len_t buffer_size) {
  if (0 >= buffer_size) {
    return ENC28J60_MAX_FRAME_SIZE - ENC28J60_DEV(driver)->tx_size;
  }

  const eth_frm_len_t tx_size = buffer_size + ENC28J60_DEV(driver)->tx_size;
  if (unlikely(tx_size >= ENC28J60_MAX_FRAME_SIZE)) {
    return -1;
  }

  enc28j60_buffer_write(ENC28J60_DEV(driver), buffer, buffer_size);
  ENC28J60_DEV(driver)->tx_size = tx_size;

  return ENC28J60_MAX_FRAME_SIZE - tx_size;
}

static tx_status_t enc28j60_tx_start(eth_driver_t* driver) {  
  if (unlikely(0 >= ENC28J60_DEV(driver)->tx_size)) {
    return ETH_TX_BUFFER_EMPTY;
  }

  /**
   *  Appropriately program the ETXND Pointer. It should point to the last byte in the data payload.
   */
  enc28j60_write_register_word(ENC28J60_DEV(driver), ETXND, ENC28J60_TXSTART + 1 + ENC28J60_DEV(driver)->tx_size);

  /**
   * Start transmission
   */
  const uint8_t result = enc28j60_execute_tx(ENC28J60_DEV(driver));

  /**
   * Reset transmit memory for next write
   */
  enc28j60_write_register_word(ENC28J60_DEV(driver), ETXST, ENC28J60_TXSTART);

  /**
   * Set the write pointer
   */
  enc28j60_write_register_word(ENC28J60_DEV(driver), EWRPT, ENC28J60_TXSTART);

  /**
   * Write the per packet control byte
   */
  enc28j60_write_op(ENC28J60_DEV(driver), ENC28J60_WRITE_BUF_MEM, 0x00u, 0x00u);

  ENC28J60_DEV(driver)->tx_size = 0;

  if (result & TSV_TX_DONE) {
    return ETH_TX_SUCCESS;
  } else if (UINT8_MAX == result) {
    return ETH_TX_TIMEOUT;
  } else {
    return ETH_TX_ERROR;
  }
}

/**
 * There is a lot of "magic" here that is blindly implemented from the datasheet.
 * This function however is a really good test to determine if the ENC28J60
 * is working as expected
 */
static uint8_t enc28j60_built_in_self_test(enc28j60_dev_t* device, const uint8_t mode) {
  enc28j60_reset(device);

  /**
   * From the datasheet.
   *
   * To use the BIST:
   */

  /**
   * 1. Program the EDMAST register pair to 0000h.
   */
  enc28j60_write_register_word(device, EDMAST, 0x0000u);

  /**
   * 2. Program EDMAND and ERXND register pairs to 1FFFh.
   */
  enc28j60_write_register_word(device, EDMAND, ENC28J60_MEMORY_SIZE);
  enc28j60_write_register_word(device, ERXND, ENC28J60_MEMORY_SIZE);

  /**
   * 3. Configure the DMA for checksum generation by setting CSUMEN in ECON1.
   */
  enc28j60_set_register_bit(device, ECON1, ECON1_CSUMEN);

  /**
   * 5. Enable Test mode, select the desired test, select the desired port configuration for the test.
   * 6. Start the BIST by setting EBSTCON.BISTST
   *
   * Additionally further down:
   *
   * To ensure full testing, the test should be redone with the Port Select bit, PSEL, altered.
   * When not using Address Fill mode, additional tests may be done with different seed values
   * to gain greater confidence that the memory is working as expected
   */
  if (mode == BIST_ADDRESS_FILL) {
    /**
    * In Address Fill mode, the BIST controller will write the low byte of each memory address
    * into the associated buffer location. As an example, after the BIST is operated,
    * the location 0000h should have 00h in it, location 0001h should have 01h in it,
    * location 0E2Ah should have 2Ah in it and so on. With this fixed memory pattern, the BIST
    * and DMA modules should always generate a checksum of F807h. The host controller
    * may use Address Fill mode to confirm that the BIST and DMA modules themselves are both
    * operating as intended.
    */

    enc28j60_write_register_byte(device, EBSTCON, EBSTCON_TME | EBSTCON_BISTST | mode);
  } else {
    /*
     * 4. Write the seed/initial shift value byte to the EBSTSD register 
     * (this is not necessary if Address Fill mode is used).
     */
    enc28j60_write_register_byte(device, EBSTSD, 0b10101010u);

    if (mode == BIST_RANDOM_FILL) {
      /**
       * In Random Data Fill mode, the BIST controller will write pseudo-random data into the buffer.
       * The random data is generated by a Linear Feedback Shift Register (LFSR) implementation.
       * The random number generator is seeded by the initial contents of the EBSTSD register and
       * the register will have new contents when the BIST is finished.
       *
       * Because of the LFSR implementation, an initial seed of zero will generate a continuous pattern of zeros.
       * As a result, a non-zero seed value will likely perform a more extensive memory test.
       *
       * Selecting the same seed for two separate trials will allow a repeat of the same test. 
       */

      enc28j60_write_register_byte(device, EBSTCON, EBSTCON_TME | EBSTCON_PSEL | EBSTCON_BISTST | mode);
    } else {
      return 0;
    }
  }

  uint8_t count = 8;
  do {
    hardware_platform_sleep_us(100);
    if (0 == --count) break;
  } while(enc28j60_read_op(device, ENC28J60_READ_CTRL_REG, EBSTCON) & EBSTCON_BISTST);

  if (likely(count)) {
    enc28j60_clear_register_bit(device, EBSTCON, EBSTCON_TME);
  } else {
    return 0;
  }

  /** 
   * 7. Start the DMA checksum by setting DMAST in ECON1. The DMA controller will read the
   * memory at the same rate the BIST controller will write to it, so the DMA can be
   * started any time after the BIST is started.
   */
  enc28j60_set_register_bit(device, ECON1, ECON1_DMAST);

  /**
   * 8. Wait for the DMA to complete by polling the DMAST bit or receiving the DMA interrupt (if enabled).
   *
   * But further down:
   *
   * At any time during a test, the test can be canceled by clearing the BISTST, DMAST and TME bits.
   * While the BIST is filling memory, the EBSTSD register should not be accessed, nor should any
   * configuration changes occur.
   *
   * When the BIST completes its memory fill and checksum generation,
   * the BISTST bit will automatically be cleared.
   *
   * The BIST module requires one main clock cycle for each byte that it writes into the RAM.
   * The DMA module's checksum implementation requires the same
   * time but it can be started immediately after the BIST is started. As a result, the minimum time
   * required to do one test pass is slightly greater than 327.68 μs.
   */
  count = 8;
  do {
    hardware_platform_sleep_us(100);
    if (0 == --count) break;
  } while(enc28j60_read_op(device, ENC28J60_READ_CTRL_REG, ECON1) & ECON1_DMAST);

  if (likely(count)) {
    enc28j60_write_register_byte(device, EBSTCON, 0x00u);
    enc28j60_set_register_bit(device, ECON1, 0x00u);
  } else {
    return 0;
  }

  /** 
   * 9. Compare the EDMACS registers with the EBSTCS registers
   */
  const uint16_t edmacs = enc28j60_read_register_word(device, EDMACS);
  const uint16_t ebstcs = enc28j60_read_register_word(device, EBSTCS);

  if (unlikely(edmacs != ebstcs)) return 0;
  if (unlikely((mode == BIST_ADDRESS_FILL) && (0xF807u != edmacs))) return 0;

  return 1;
}

enc28j60_init_status_t enc28j60_eth_driver(
    eth_driver_t* driver,
    enc28j60_chip_select_f chip_select_function,
    enc28j60_spi_txrx_f spi_function,
    const mac_addr_t mac_address,
    const enc28j60_init_options_t options) {

  if ((NULL == driver) || (NULL == chip_select_function) || (NULL == spi_function)) {
    return ENC28J60_INVALID_CONFIGURATION;
  }

  enc28j60_dev_t* device = (enc28j60_dev_t*) hardware_platform_malloc(sizeof(enc28j60_dev_t));
  if (NULL == device) return ENC28J60_MALLOC_FAILURE;

  device->chip_select_function = chip_select_function;
  device->spi_function = spi_function;
  device->bank = 0;
  device->next_packet_pointer = 0;
  device->tx_size = 0;

  if (!(options & ENC28J60_SKIP_SELF_TEST)) {
    /**
     * Use hardware validation to assert the hardware is working and the device is connected
     *
     * This method adds no more than 1 second of latency during init and
     * it is a very good method to determine that the device responds as expected
     */

    if (!enc28j60_built_in_self_test(device, BIST_ADDRESS_FILL)) {
      enc28j60_reset(device);
      return ENC28J60_SELF_TEST_FAILURE;
    }
  }

  /**
   * Use hardware revision as device validation.
   *
   * This method is very unreliable. During testing I was able to pass
   * this test most of the time without a device even being connected.
   *
   * And with another SPI device (not ENC28J60) connected it passed all of the time
   */
  if (!(options & ENC28J60_SKIP_REVISION_CHECK)) {
    const enc28j60_rev_t revision = enc28j60_read_register_byte(device, EREVID);
    if ((0 >= revision) || (revision >= INT8_MAX)) {
      return ENC28J60_INVALID_REVISION;
    }
  }

  const uint8_t full_duplex = (options & ENC28J60_FULL_DUPLEX) ? 1 : 0;
  if (!enc28j60_configure(device, mac_address, full_duplex)) {
    enc28j60_reset(device);
    return ENC28J60_CONFIGURE_ERROR;
  }

  driver->dev = (void*) device;
  driver->eth_next_rx_packet = &enc28j60_next_rx_packet;
  driver->eth_read_rx_packet = &enc28j60_read_rx_packet;
  driver->eth_write_tx_buffer = &enc28j60_write_tx_buffer;
  driver->eth_tx_start = &enc28j60_tx_start;

  return ENC28J60_INIT_SUCCESS;
}


void enc28j60_close(eth_driver_t* driver) {
  if (unlikely(NULL == driver)) {
    return;
  }

  enc28j60_reset(ENC28J60_DEV(driver));

  hardware_platform_free((void*) ENC28J60_DEV(driver));
}

enc28j60_rev_t enc28j60_hardware_revision(eth_driver_t* driver) {
  if (unlikely(NULL == driver))  {
    return 0;
  }

  return enc28j60_read_register_byte(ENC28J60_DEV(driver), EREVID);
}

eth_link_status_t enc28j60_link_status(eth_driver_t* driver) {
  if (unlikely(NULL == driver)) {
    return ETH_DEVICE_ERROR;
  }

  const uint16_t phstat2 = enc28j60_phy_read(ENC28J60_DEV(driver), PHSTAT2);

  if (!(phstat2 & PHSTAT2_LSTAT)) {
    return ETH_LINK_DOWN;
  }

  if (phstat2 & PHSTAT2_DPXSTAT) {
    return ETH_10M_FULL_DUPLEX;
  } else {
    return ETH_10M_HALF_DUPLEX;
  }
}
