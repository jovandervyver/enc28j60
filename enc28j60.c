#include <stddef.h>
#include "enc28j60.h"
#include "enc28j60_constants.h"
#include "hardware_platform.h"

typedef uint8_t  enc28j60_reg_t;

#define ENC28J60_CS_LOW(_dev_) \
  CHIP_SELECT_FUNCTION((_dev_), ENC28J60_CHIP_SELECT_LOW)

#define ENC28J60_CS_HIGH(_dev_) \
  CHIP_SELECT_FUNCTION((_dev_), ENC28J60_CHIP_SELECT_HIGH)

#define ENC28J60_WRITE_BYTE(_dev_, _val_) \
  SPI_TXRX_FUNCTION((_dev_), (_val_))

#define ENC28J60_READ_BYTE(_dev_) \
  ((uint8_t) (SPI_TXRX_FUNCTION((_dev_), 0)))

#define ENC28J60_GET_BANK(_dev_) \
  ((enc28j60_addr_t) ((_dev_)->bank))

#define ENC28J60_SET_BANK(_dev_, _bank_) \
  ((_dev_)->bank = ((enc28j60_addr_t) (_bank_)))

#define ENC28J60_GET_NEXT_PACKET_POINTER(_dev_) \
  ((enc28j60_mem_ptr_t) ((_dev_)->next_packet_pointer))

#define ENC28J60_SET_NEXT_PACKET_POINTER(_dev_, _next_packet_pointer_) \
  ((_dev_)->next_packet_pointer = ((enc28j60_mem_ptr_t) (_next_packet_pointer_)))

#define ENC28J60_GET_TX_SIZE(_dev_) \
  ((eth_frm_len_t) ((_dev_)->tx_size))

#define ENC28J60_SET_TX_SIZE(_dev_, _tx_size_) \
  ((_dev_)->tx_size = ((eth_frm_len_t) (_tx_size_)))

#define ENC28J60_UINT16(_low_, _high_) \
  ((((uint16_t) (_low_))) | ((((uint16_t) (_high_)) << 8)))

#define ENC28J60_MAX_RECEIVE_LOOP_COUNT  32

static inline uint16_t enc28j60_erxrdpt_errata(const uint16_t value) {
  /**
   * From errata:
   *
   * Module: Memory (Ethernet Buffer)
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
   * if (Next Packet Pointer - 1 < ERXST) or (Next Packet Pointer - 1 > ERXND)
   *  then:
   *    ERXRDPT = ERXND
   *  else:
   *    ERXRDPT = Next Packet Pointer - 1
   */
  if ((value <= ENC28J60_RXSTART) || (value >= ENC28J60_RXEND)) {
    return ENC28J60_RXEND;
  } else {
    return value - 1;
  }
}

static void enc28j60_write_op(
  const enc28j60_dev_t* dev,
  const enc28j60_reg_t op,
  const enc28j60_addr_t address,
  const uint8_t value) {

  ENC28J60_CS_LOW(dev);

  ENC28J60_WRITE_BYTE(dev, op | (address & ADDR_MASK));
  ENC28J60_WRITE_BYTE(dev, value);

  ENC28J60_CS_HIGH(dev);
}

static uint8_t enc28j60_read_op(
  const enc28j60_dev_t* dev,
  const enc28j60_reg_t op,
  const enc28j60_addr_t address) {

  ENC28J60_CS_LOW(dev);

  ENC28J60_WRITE_BYTE(dev, (op | (address & ADDR_MASK)));
  if (address & SPRD_MASK) {
    ENC28J60_WRITE_BYTE(dev, 0);
  }
  const uint8_t value = ENC28J60_READ_BYTE(dev);

  ENC28J60_CS_HIGH(dev);

  return value;
}

static void enc28j60_set_bank(enc28j60_dev_t* dev, enc28j60_addr_t address) {
  /**
   * Registers EIE, EIR, ESTAT, ECON2, ECON1
   * are present in all banks, no need to switch bank
   */
  address = address & BANK_MASK;
  if ((address != ALL_BANKS) && (address != ENC28J60_GET_BANK(dev))) {
    ENC28J60_CS_LOW(dev);
    ENC28J60_WRITE_BYTE(dev, (ENC28J60_BIT_FIELD_CLR | ADDR_MASK));
    ENC28J60_WRITE_BYTE(dev, (ECON1_BSEL1 | ECON1_BSEL0));
    ENC28J60_CS_HIGH(dev);

    ENC28J60_CS_LOW(dev);
    ENC28J60_WRITE_BYTE(dev, (ENC28J60_BIT_FIELD_SET | ADDR_MASK));
    ENC28J60_WRITE_BYTE(dev, (address >> 5));
    ENC28J60_CS_HIGH(dev);

    ENC28J60_SET_BANK(dev, address);
  }
}

static void enc28j60_set_register_bit(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address,
  const uint8_t mask) {

  enc28j60_set_bank(dev, address);
  enc28j60_write_op(dev, ENC28J60_BIT_FIELD_SET, address, mask);
}

static void enc28j60_clear_register_bit(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address,
  const uint8_t mask) {

  enc28j60_set_bank(dev, address);
  enc28j60_write_op(dev, ENC28J60_BIT_FIELD_CLR, address, mask);
}

static void enc28j60_write_register_byte(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address,
  const uint8_t value) {

  enc28j60_set_bank(dev, address);
  enc28j60_write_op(dev, ENC28J60_WRITE_CTRL_REG, address, value);
}

static uint8_t enc28j60_read_register_byte(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address) {

  enc28j60_set_bank(dev, address);
  return enc28j60_read_op(dev, ENC28J60_READ_CTRL_REG, address);
}

static void enc28j60_write_register_word(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address,
  const uint16_t value) {

  enc28j60_set_bank(dev, address);

  enc28j60_write_op(
    dev, ENC28J60_WRITE_CTRL_REG, address, ((uint8_t) value));

  enc28j60_write_op(
    dev, ENC28J60_WRITE_CTRL_REG, (address + 1), ((uint8_t) (value >> 8)));
}

static uint16_t enc28j60_read_register_word(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address) {

  enc28j60_set_bank(dev, address);

  const uint8_t low = enc28j60_read_op(
    dev, ENC28J60_READ_CTRL_REG, address);

  const uint8_t high = enc28j60_read_op(
    dev, ENC28J60_READ_CTRL_REG, (address + 1));

  return ENC28J60_UINT16(low, high);
}

static void enc28j60_reset(enc28j60_dev_t* dev) {
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

  ENC28J60_CS_LOW(dev);
  ENC28J60_WRITE_BYTE(dev, ENC28J60_SOFT_RESET);
  ENC28J60_CS_HIGH(dev);

  hardware_platform_sleep_ms(2);

  /**
   * Whenever a reset is done, ECON1 will inevitable need to be cleared next
   */
  enc28j60_write_register_byte(dev, ECON1, 0);
}

static void enc28j60_buffer_write(
  const enc28j60_dev_t* dev,
  const uint8_t* bytes,
  eth_frm_len_t length) {

  ENC28J60_CS_LOW(dev);
  ENC28J60_WRITE_BYTE(dev, ENC28J60_WRITE_BUF_MEM);

  do {
    ENC28J60_WRITE_BYTE(dev, *bytes);
    bytes += sizeof(uint8_t);
  } while(--length);

  ENC28J60_CS_HIGH(dev);
}

static void enc28j60_buffer_read(
  const enc28j60_dev_t* dev,
  uint8_t* bytes,
  eth_frm_len_t length) {

  ENC28J60_CS_LOW(dev);
  ENC28J60_WRITE_BYTE(dev, ENC28J60_READ_BUF_MEM);

  do {
    *bytes = ENC28J60_READ_BYTE(dev);
    bytes += sizeof(uint8_t);
  } while(--length);

  ENC28J60_CS_HIGH(dev);
}

static inline uint8_t enc28j60_phy_wait(enc28j60_dev_t* dev) {
  uint8_t timeout = 3;
  do {
    /*
     * From datasheet: Wait 10.24 μs.
     * Poll the MISTAT.BUSY bit to be certain that the operation is complete.
     *
     * Adding a safety margin
     */
    hardware_platform_sleep_us(14);

    if (likely(!
      enc28j60_read_register_byte(dev, MISTAT) & MISTAT_BUSY)) {
      return 1;
    }
  } while(--timeout);

  return 0;
}

static uint8_t enc28j60_phy_write(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address,
  const uint16_t value) {

  enc28j60_write_register_byte(dev, MIREGADR, address);
  enc28j60_write_register_word(dev, MIWR, value);

  return enc28j60_phy_wait(dev);
}

static uint16_t enc28j60_phy_read(
  enc28j60_dev_t* dev,
  const enc28j60_addr_t address) {

  enc28j60_write_register_byte(dev, MIREGADR, address);
  enc28j60_write_register_byte(dev, MICMD, MICMD_MIIRD);

  if (likely(enc28j60_phy_wait(dev))) {
    enc28j60_write_register_byte(dev, MICMD, 0);
    return enc28j60_read_register_word(dev, MIRD);
  } else {
    return 0;
  }
}

static uint8_t enc28j60_configure(
  enc28j60_dev_t* dev,
  const mac_addr_t mac_address,
  const uint8_t full_duplex) {

  enc28j60_reset(dev);

  /**
   * Auto increment the buffer pointer
   */
  enc28j60_write_register_byte(dev, ECON2, ECON2_AUTOINC);

  #if (ENC28J60_RXSTART < 0)
    #error "ENC28J60_RXSTART < 0"
  #endif

  #if (ENC28J60_RXSTART >= ENC28J60_RXEND)
    #error "ENC28J60_RXSTART >= ENC28J60_RXEND"
  #endif

  #if ((ENC28J60_RXEND % 2) == 0)
    #error "ENC28J60_RXEND must be an uneven number; see ENC28J60 errata"
  #endif

  /**
   * Set the ENC28J60 receive buffer start and end address
   */
  enc28j60_write_register_word(dev, ERXST, ENC28J60_RXSTART);
  enc28j60_write_register_word(dev, ERXND, ENC28J60_RXEND);

  /**
   * Set the next packet pointer
   */
  ENC28J60_SET_NEXT_PACKET_POINTER(dev, ENC28J60_RXSTART);
  enc28j60_write_register_word(dev, ERXRDPT, ENC28J60_RXSTART);

  #if (ENC28J60_TXSTART < 0)
    #error "ENC28J60_TXSTART < 0"
  #endif

  #if (ENC28J60_TXSTART >= ENC28J60_TXEND)
    #error "ENC28J60_TXSTART >= ENC28J60_TXEND"
  #endif

  #if (ENC28J60_TXEND > ENC28J60_MEMORY_SIZE)
    #error "ENC28J60_TXEND > ENC28J60_MEMORY_SIZE"
  #endif

  /**
   * Set the ENC28J60 transmit buffer start and end address
   */
  enc28j60_write_register_word(dev, ETXST, ENC28J60_TXSTART);
  enc28j60_write_register_word(dev, ETXND, ENC28J60_TXEND);

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
  enc28j60_write_register_byte(dev, ERXFCON,
    (ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN));

  /**
   * Initialize MAC
   */

  /**
   * 1. Set the MARXEN bit in MACON1 to enable the MAC to receive frames. If using full duplex, most
   * applications should also set TXPAUS and RXPAUS to allow IEEE defined flow control to
   * function.
   */
  enc28j60_write_register_byte(dev, MACON1,
    (MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS));

  /**
   * 2. Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3. Most applications
   * should enable automatic padding to at least 60 bytes and always append a valid CRC. For
   * convenience, many applications may wish to set the FRMLNEN bit as well to enable frame length
   * status reporting. The FULDPX bit should be set if the application will be connected to a
   * full-duplex configured remote node; otherwise, it should be left clear.
   */
  if (full_duplex) {
    enc28j60_write_register_byte(dev, MACON3,
      (MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX));
  } else {
    enc28j60_write_register_byte(dev, MACON3,
      (MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN));

    /**
     * 3. Configure the bits in MACON4. For conformance to the IEEE 802.3 standard, set the DEFER bit.
     */
    enc28j60_write_register_byte(dev, MACON4, MACON4_DEFER);
  }

  /**
   * 4. Program the MAMXFL registers with the maximum frame length to be permitted to be received
   * or transmitted. Normal network nodes are designed to handle packets that are 1518 bytes
   * or less.
   */
  enc28j60_write_register_word(dev, MAMXFLL, ENC28J60_MAX_FRAME_SIZE);

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
    enc28j60_write_register_word(dev, MAIPG, 0x0012u);
    enc28j60_write_register_byte(dev, MABBIPG, 0x15u);
  } else {
    enc28j60_write_register_word(dev, MAIPGL, 0x0C12u);
    enc28j60_write_register_byte(dev, MABBIPG, 0x12u);
  }

  /**
   * 8. If Half-Duplex mode is used, program the Retransmission and Collision Window registers,
   * MACLCON1 and MACLCON2. Most applications will not need to change the default Reset values.
   * If the network is spread over exceptionally long cables, the default value of MACLCON2 may
   * need to be increased.
   */
  enc28j60_write_register_byte(dev, MACON2, 0);

  /**
   * 9. Program the local MAC address into the MAADR1:MAADR6 registers. 
   */
  enc28j60_write_register_byte(dev, MAADR5, mac_address[0]);
  enc28j60_write_register_byte(dev, MAADR4, mac_address[1]);
  enc28j60_write_register_byte(dev, MAADR3, mac_address[2]);
  enc28j60_write_register_byte(dev, MAADR2, mac_address[3]);
  enc28j60_write_register_byte(dev, MAADR1, mac_address[4]);
  enc28j60_write_register_byte(dev, MAADR0, mac_address[5]);

  do {
    if (unlikely(!enc28j60_phy_write(dev, PHLCON, ENC28J60_LAMPS_MODE))) {
      break;
    }

    if (full_duplex) {
      if (unlikely(!enc28j60_phy_write(dev, PHCON1, PHCON1_PDPXMD))) {
        break;
      }

      if (unlikely(!enc28j60_phy_write(dev, PHCON2, 0))) {
        break;
      }
    } else {
      if (unlikely(!enc28j60_phy_write(dev, PHCON1, 0))) {
        break;
      }

      if (unlikely(!enc28j60_phy_write(dev, PHCON2, PHCON2_HDLDIS))) {
        break;
      }
    }

    if (unlikely(!enc28j60_phy_write(dev, PHIE, 0))) {
      break;
    }

    enc28j60_clear_register_bit(dev, EIR,
      (EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF |
       EIR_RXERIF | EIR_PKTIF));

    enc28j60_clear_register_bit(dev, EIE,
      (EIE_INTIE | EIE_PKTIE | EIE_DMAIE | EIE_LINKIE |
       EIE_TXIE | EIE_TXERIE | EIE_RXERIE));

    /**
     * RXEN: Receive Enable bit
     *   1 = Packets which pass the current filter configuration will be written into the receive buffer
     *   0 = All packets received will be ignored
     */
    enc28j60_set_register_bit(dev, ECON1, ECON1_RXEN);

    /**
     * Set the transmit start pointer and write the initial header byte
     */
    enc28j60_write_register_word(dev, EWRPT, ENC28J60_TXSTART);
    enc28j60_write_op(dev, ENC28J60_WRITE_BUF_MEM, 0, 0);
    ENC28J60_SET_TX_SIZE(dev, 0);

    /**
     * Wait time for the link to become active
     */
    hardware_platform_sleep_ms(100);

    return 1;
  } while(0);

  enc28j60_reset(dev);
  return 0;
}

eth_frm_len_t enc28j60_next_rx_packet(enc28j60_dev_t* dev) {
  #if ENC28J60_MAX_RECEIVE_LOOP_COUNT <= 0
    #error "ENC28J60_MAX_RECEIVE_LOOP_COUNT <= 0"
  #endif

  #if ENC28J60_MAX_RECEIVE_LOOP_COUNT > UINT8_MAX
    #error "ENC28J60_MAX_RECEIVE_LOOP_COUNT > UINT8_MAX"
  #endif

  /**
   * Try process ENC28J60_MAX_RECEIVE_LOOP_COUNT packets,
   * then return -1 to avoid stealing all the system resources
   */
  uint8_t count;
  for (count = 1; count <= ENC28J60_MAX_RECEIVE_LOOP_COUNT; ++count) {
    if (enc28j60_read_register_byte(dev, EPKTCNT)) {
      if (1) {
        const enc28j60_mem_ptr_t next_packet_pointer =
          ENC28J60_GET_NEXT_PACKET_POINTER(dev);

        /**
         * Advance the packet pointer to the start of the next packet
         */
        enc28j60_write_register_word(
          dev, ERXRDPT, enc28j60_erxrdpt_errata(next_packet_pointer));

        /**
         * Set the read pointer to the start of the next packet
         */
        enc28j60_write_register_word(dev, ERDPT, next_packet_pointer);
      }

      /**
       * Parse the receive status vector
       *   bytes 1-2: next packet pointer
       *   bytes 3-4: packet length
       *   bytes 5-6: status bits
       */
      uint8_t bytes[RSV_SIZE];
      enc28j60_buffer_read(dev, bytes, RSV_SIZE);

      ENC28J60_SET_NEXT_PACKET_POINTER(
        dev, ENC28J60_UINT16(bytes[0], bytes[1]));

      /**
       * Decrement the packet counter by 1
       */
      enc28j60_set_register_bit(dev, ECON2, ECON2_PKTDEC);

      if (RSV_RX_OK & bytes[4]) {
        return (eth_frm_len_t) ENC28J60_UINT16(bytes[2], bytes[3]);
      }
    }
  }

  return count != ENC28J60_MAX_RECEIVE_LOOP_COUNT ? -1 : 0;
}

void enc28j60_read_rx_packet(
  enc28j60_dev_t* dev,
  uint8_t* buffer,
  const eth_frm_len_t buffer_size) {

  if (buffer_size > 0) {
    enc28j60_buffer_read(dev, buffer, buffer_size);
  }
}

static void enc28j60_reset_tx_logic(enc28j60_dev_t* dev) {
  enc28j60_set_register_bit(dev, ECON1, ECON1_TXRST);
  hardware_platform_sleep_us(14);

  enc28j60_clear_register_bit(dev, ECON1, ECON1_TXRST);
  enc28j60_clear_register_bit(dev, EIR,
    (EIR_TXERIF | EIR_TXIF | ECON1_TXRTS));
}

static uint8_t enc28j60_tx_start(
  enc28j60_dev_t* dev,
  const eth_frm_len_t tx_size) {

  #if ENC28J60_TRANSMIT_TIMEOUT_MS <= 0
    #error "ENC28J60_TRANSMIT_TIMEOUT_MS <= 0"
  #endif

  const millisecond_timestamp_t start_millis =
    hardware_platform_milliseconds();

  /**
   * Start the transmission process by setting ECON1.TXRTS
   */
  enc28j60_set_register_bit(dev, ECON1, ECON1_TXRTS);

  /**
   * Wait for transmit to complete
   */
  uint8_t eir_status;
  do {
    eir_status = enc28j60_read_register_byte(dev, EIR) &
      (EIR_TXIF | EIR_TXERIF);
  } while((eir_status == 0) &&
    (ENC28J60_TRANSMIT_TIMEOUT_MS >
      (hardware_platform_milliseconds() - start_millis)));

  /**
   * From errata:
   *
   *  Module: Transmit Logic
   *
   *  In Half-Duplex mode, a hardware transmission abort - caused by excessive collisions, a late collision
   *  or excessive deferrals - may stall the internal transmit logic. The next packet transmit initiated by
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
  if (unlikely((eir_status == 0) ||
      (((eir_status & EIR_TXIF) != 0)))) {

    enc28j60_reset_tx_logic(dev);
    return UINT8_MAX;
  }

  /**
   * Set the read pointer to the end of the transmission buffer.
   * Then read the Transmit Status Vector
   */
  enc28j60_write_register_word(dev, ERDPT,
    (ENC28J60_TXSTART + 1 + tx_size + 2));

  uint8_t tsv;
  enc28j60_buffer_read(dev, &tsv, sizeof(tsv));

  /**
   * Reset the read pointer to the next packet pointer position
   */
  enc28j60_write_register_word(dev, ERDPT,
    ENC28J60_GET_NEXT_PACKET_POINTER(dev));

  if (tsv & TSV_TX_DONE) {
    return 0;
  } else {
    return INT8_MAX;
  }
}

eth_frm_len_t enc28j60_write_tx_buffer(
  enc28j60_dev_t* dev,
  const uint8_t* buffer,
  const eth_frm_len_t buffer_size) {

  const eth_frm_len_t offset = ENC28J60_GET_TX_SIZE(dev);
  if (buffer_size > 0) {
    const eth_frm_len_t tx_size = offset + buffer_size;
    if (likely(tx_size < ENC28J60_MAX_FRAME_SIZE)) {
      // Set the write pointer to the offset
      enc28j60_write_register_word(dev, EWRPT,
        (ENC28J60_TXSTART + 1 + offset));

      // Write data into the buffer
      enc28j60_buffer_write(dev, buffer, buffer_size);

      // Update the current write buffer offset
      ENC28J60_SET_TX_SIZE(dev, tx_size);

      return ENC28J60_MAX_FRAME_SIZE - tx_size;
    } else {
      return -1;
    }
  } else {
    return ENC28J60_MAX_FRAME_SIZE - offset;
  }
}

enc28j60_tx_status_t enc28j60_execute_tx(enc28j60_dev_t* dev) {
  const eth_frm_len_t tx_size = ENC28J60_GET_TX_SIZE(dev);
  if (likely(tx_size > 0)) {
    /**
     * Appropriately program the ETXND Pointer. It should point to the last byte in the data payload.
     */
    enc28j60_write_register_word(dev, ETXND,
      (ENC28J60_TXSTART + 1 + tx_size));

    /**
     * Start transmission
     */
    const uint8_t result = enc28j60_tx_start(dev, tx_size);

    /**
     * Reset transmit memory for next write
     */
    enc28j60_write_register_word(dev, ETXST, ENC28J60_TXSTART);

    /**
     * Set the write pointer
     */
    enc28j60_write_register_word(dev, EWRPT, ENC28J60_TXSTART);

    /**
     * Write the per packet control byte
     */
    enc28j60_write_op(dev, ENC28J60_WRITE_BUF_MEM, 0, 0);

    ENC28J60_SET_TX_SIZE(dev, 0);

    if (result == 0) {
      return ENC28J60_TX_SUCCESS;
    } else if (result == UINT8_MAX) {
      return ENC28J60_TX_TIMEOUT;
    } else {
      return ENC28J60_TX_ERROR;
    }
  } else {
    return ENC28J60_TX_BUFFER_EMPTY;
  }
}

/**
 * There is a lot of "magic" here that is blindly implemented from the datasheet.
 * This function however is a really good test to determine if the ENC28J60
 * is working as expected
 */
static uint8_t enc28j60_built_in_self_test(
  enc28j60_dev_t* dev,
  const uint8_t mode) {

  enc28j60_reset(dev);

  /**
   * From the datasheet.
   *
   * To use the BIST:
   */

  /**
   * 1. Program the EDMAST register pair to 0000h.
   */
  enc28j60_write_register_word(dev, EDMAST, 0);

  /**
   * 2. Program EDMAND and ERXND register pairs to 1FFFh.
   */
  enc28j60_write_register_word(dev, EDMAND, ENC28J60_MEMORY_SIZE);
  enc28j60_write_register_word(dev, ERXND, ENC28J60_MEMORY_SIZE);

  /**
   * 3. Configure the DMA for checksum generation by setting CSUMEN in ECON1.
   */
  enc28j60_set_register_bit(dev, ECON1, ECON1_CSUMEN);

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

    enc28j60_write_register_byte(dev, EBSTCON,
      (EBSTCON_TME | EBSTCON_BISTST | mode));
  } else {
    /**
     * 4. Write the seed/initial shift value byte to the EBSTSD register 
     * (this is not necessary if Address Fill mode is used).
     */
    enc28j60_write_register_byte(dev, EBSTSD, 0b10101010u);

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

      enc28j60_write_register_byte(dev, EBSTCON,
        (EBSTCON_TME | EBSTCON_PSEL | EBSTCON_BISTST | mode));
    } else {
      return 0;
    }
  }

  uint8_t count = 8;
  do {
    hardware_platform_sleep_us(100);
    if (0 == --count) {
     break;
    }
  } while(enc28j60_read_op(dev, ENC28J60_READ_CTRL_REG, EBSTCON) & EBSTCON_BISTST);

  if (likely(count)) {
    enc28j60_clear_register_bit(dev, EBSTCON, EBSTCON_TME);
  } else {
    return 0;
  }

  /** 
   * 7. Start the DMA checksum by setting DMAST in ECON1. The DMA controller will read the
   * memory at the same rate the BIST controller will write to it, so the DMA can be
   * started any time after the BIST is started.
   */
  enc28j60_set_register_bit(dev, ECON1, ECON1_DMAST);

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
    if (0 == --count) {
      break;
    }
  } while(enc28j60_read_op(dev, ENC28J60_READ_CTRL_REG, ECON1) & ECON1_DMAST);

  if (likely(count)) {
    enc28j60_write_register_byte(dev, EBSTCON, 0);
    enc28j60_set_register_bit(dev, ECON1, 0);
  } else {
    return 0;
  }

  /** 
   * 9. Compare the EDMACS registers with the EBSTCS registers
   */
  const uint16_t edmacs = enc28j60_read_register_word(dev, EDMACS);
  const uint16_t ebstcs = enc28j60_read_register_word(dev, EBSTCS);

  if (unlikely(edmacs != ebstcs)) {
    return 0;
  } else if (unlikely((mode == BIST_ADDRESS_FILL) && (0xF807u != edmacs))) {
    return 0;
  } else {
    return 1;
  }
}

enc28j60_init_status_t enc28j60_initialize(
    enc28j60_dev_t* dev,

    #if (CUSTOM_CHIP_SELECT_FUNCTION == FALSE)
      enc28j60_chip_select_f chip_select_function,
    #endif

    #if (CUSTOM_SPI_TXRX_FUNCTION == FALSE)
      enc28j60_spi_txrx_f spi_txrx_function,
    #endif

    const mac_addr_t mac_address,
    const enc28j60_init_options_t options) {

  if (unlikely(dev == 0 ||

    #if (CUSTOM_CHIP_SELECT_FUNCTION == FALSE)
      chip_select_function == 0 ||
    #endif

    #if (CUSTOM_SPI_TXRX_FUNCTION == FALSE)
      spi_txrx_function == 0 ||
    #endif

    mac_address == 0)) {

    return ENC28J60_INVALID_CONFIGURATION;
  }

  #if (CUSTOM_CHIP_SELECT_FUNCTION == FALSE)
    dev->chip_select_function = chip_select_function;
  #endif

  #if (CUSTOM_SPI_TXRX_FUNCTION == FALSE)
    dev->spi_txrx_function = spi_txrx_function;
  #endif

  if (!(options & ENC28J60_SKIP_SELF_TEST)) {
    /**
     * Use hardware validation to assert the hardware is working and the device is connected
     *
     * This method adds no more than 1 second of latency during init and
     * it is a very good method to determine that the device responds as expected
     */

    if (!enc28j60_built_in_self_test(dev, BIST_ADDRESS_FILL)) {
      enc28j60_reset(dev);
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
    const enc28j60_rev_t revision = enc28j60_read_register_byte(dev, EREVID);
    if ((0 >= revision) || (revision >= INT8_MAX)) {
      return ENC28J60_INVALID_REVISION;
    }
  }

  const uint8_t full_duplex = (options & ENC28J60_FULL_DUPLEX) ? 1 : 0;
  if (!enc28j60_configure(dev, mac_address, full_duplex)) {
    enc28j60_reset(dev);
    return ENC28J60_CONFIGURE_ERROR;
  }

  return ENC28J60_INIT_SUCCESS;
}

void enc28j60_close(enc28j60_dev_t* dev) {
  enc28j60_reset(dev);
}

enc28j60_rev_t enc28j60_hardware_revision(enc28j60_dev_t* dev) {
  return enc28j60_read_register_byte(dev, EREVID);
}

void enc28j60_mac_address(enc28j60_dev_t* dev, mac_addr_t mac_address) {
  mac_address[0] = enc28j60_read_register_byte(dev, MAADR5);
  mac_address[1] = enc28j60_read_register_byte(dev, MAADR4);
  mac_address[2] = enc28j60_read_register_byte(dev, MAADR3);
  mac_address[3] = enc28j60_read_register_byte(dev, MAADR2);
  mac_address[4] = enc28j60_read_register_byte(dev, MAADR1);
  mac_address[5] = enc28j60_read_register_byte(dev, MAADR0);
}

enc28j60_link_status_t enc28j60_link_status(enc28j60_dev_t* dev) {
  const uint16_t phstat2 = enc28j60_phy_read(dev, PHSTAT2);

  if (!(phstat2 & PHSTAT2_LSTAT)) {
    return ENC28J60_LINK_DOWN;
  } else if (phstat2 & PHSTAT2_DPXSTAT) {
    return ENC28J60_10M_FULL_DUPLEX;
  } else {
    return ENC28J60_10M_HALF_DUPLEX;
  }
}
