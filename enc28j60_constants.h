#ifndef __ENC28J60_CONSTANTS_H__
#define __ENC28J60_CONSTANTS_H__

#include "external/enc28j60_hw.h"
/** 
 * Download from 
 * https://raw.githubusercontent.com/torvalds/linux/master/drivers/net/ethernet/microchip/enc28j60_hw.h
 *
 * To directory "external"
 */

/**
 * Most of the constants here are a rehash of enc28j60_hw.h
 * Mostly for readability or my own sanity
 */

#define ENC28J60_MEMORY_SIZE      0x1FFFu
#define ENC28J60_MAX_FRAME_SIZE   1518u

#define ENC28J60_TXEND            ENC28J60_MEMORY_SIZE
#define ENC28J60_TXSTART          (ENC28J60_TXEND - ENC28J60_MAX_FRAME_SIZE)

/**
 * From errata:
 * 
 * Module: Memory (Ethernet Buffer)
 * The receive hardware maintains an internal Write Pointer that defines the area in the receive buffer
 * where bytes arriving over the Ethernet are written. This internal Write Pointer should be updated with
 * the value stored in ERXST whenever the Receive Buffer Start Pointer, ERXST, or the Receive Buffer
 * End Pointer, ERXND, is written to by the host microcontroller. Sometimes, when ERXST or ERXND is written to,
 * the exact value, 0000h, is stored in the Internal Receive Write Pointer instead of the ERXST address.
 * 
 * Work around
 * Use the lower segment of the buffer memory for the receive buffer, starting at address 0000h. For
 * example, use the range (0000h to n) for the receive buffer, and ((n + 1) – 8191) for the transmit
 * buffer.
 */
#define ENC28J60_RXSTART          0
#define ENC28J60_RXEND            ENC28J60_TXSTART - 2

/**
 * Word registers. Each has a L and H, L being the lower address and H the upper.
 * H is always L + 1. Thus the write_register_word simply writes address and address + 1
 * 
 * Here the L is dropped to make it consistent with the datasheet naming
 */
#define EDMAST    EDMASTL
#define EDMAND    EDMANDL
#define ERXND     ERXNDL
#define EDMACS    EDMACSL
#define EBSTCS    EBSTCSL
#define MIWR      MIWRL
#define MIRD      MIRDL
#define ERDPT     ERDPTL
#define EWRPT     EWRPTL
#define ERXWRPT   ERXWRPTL
#define ERXST     ERXSTL
#define ERXRDPT   ERXRDPTL
#define ETXST     ETXSTL
#define ETXND     ETXNDL
#define MAIPG     MAIPGL
#define MAMXFL    MAMXFLL

#define MACON2              (0x01|0x40|SPRD_MASK)
#define MACON4_DEFER        0x40

#define ALL_BANKS           0xE0

#define RSV_RX_OK           0b010000000u
#define TSV_TX_DONE         0b010000000u


/**
 * Built-in self test (BIST) bits
 */
#define EBSTCON_PSV2        0x80
#define EBSTCON_PSV1        0x40
#define EBSTCON_PSV0        0x20
#define EBSTCON_PSEL        0x10
#define EBSTCON_TMSEL1      0x08
#define EBSTCON_TMSEL0      0x04
#define EBSTCON_TME         0x02
#define EBSTCON_BISTST      0x01

#define BIST_RANDOM_FILL    0x00
#define BIST_ADDRESS_FILL   EBSTCON_TMSEL0
#define BIST_PATTERN_SHIFT  EBSTCON_TMSEL1

#define ENC28J60_TRANSMIT_TIMEOUT_MS 500

#endif
