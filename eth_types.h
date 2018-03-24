#ifndef __ETH__TYPES__H__
#define __ETH__TYPES__H__

#ifdef __cplusplus
extern "C" {
#endif

  #include <stdint.h>

  typedef uint8_t mac_addr_t[6];
  typedef uint8_t ipv4_t[4];
  typedef uint8_t ipv6_t[16];
  typedef int16_t eth_frm_len_t;
  typedef uint16_t ethertype_t;

  #define ETHERTYPE_IPV4         0x0800
  #define ETHERTYPE_ARP          0x0806
  #define ETHERTYPE_WOL          0x0842
  #define ETHERTYPE_REVERSE_ARP  0x8035
  #define ETHERTYPE_VLAN_SINGLE  0x8100
  #define ETHERTYPE_IPV6         0x86DD

  typedef enum {
    ETH_DEVICE_ERROR,
    ETH_LINK_DOWN,
    ETH_10M_HALF_DUPLEX,
    ETH_10M_FULL_DUPLEX,
    ETH_100M_HALF_DUPLEX,
    ETH_100M_FULL_DUPLEX,
    ETH_1G_FULL_DUPLEX
  } eth_link_status_t;

#ifdef __cplusplus
}
#endif

#endif
