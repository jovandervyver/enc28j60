#ifndef __HARDWARE__PLATFORM__DEFS__H__
#define __HARDWARE__PLATFORM__DEFS__H__

#ifndef BIG_ENDIAN
  #define BIG_ENDIAN      4321
#endif

#ifndef LITTLE_ENDIAN
  #define LITTLE_ENDIAN   1234
#endif

#ifndef BYTE_ORDER
  #warning "BYTE_ORDER was not defined, defaulting to LITTLE_ENDIAN"
  #define BYTE_ORDER      LITTLE_ENDIAN
#endif

#if defined __IAR_SYSTEMS_ICC__ || defined ATOP
  #define packed_struct __packed struct

  #define likely(expr)   expr
  #define unlikely(expr) expr
#elif defined __WATCOMC__
  #define packed_struct   _Packed struct

  #define likely(expr)   expr
  #define unlikely(expr) expr
#else
  #define packed_struct struct __attribute__((packed))

  #ifdef __GNUC__
    #define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)

    #if ((GCC_VERSION >= 40800))
      #define BYTESWAP_GCC

      #define likely(expr)       __builtin_expect(!!(expr), 1)
      #define unlikely(expr)     __builtin_expect(!!(expr), 0)

    #endif
  #endif
#endif

#if BYTE_ORDER == BIG_ENDIAN
  #define HTONS(n) (n)
  #define NTOHS(n) (n)
  #define HTONL(n) (n)
  #define NTOHL(n) (n)
#else
  #define STATIC_HTONS(n) (((((unsigned short)(n) & 0xFF)) << 8) | (((unsigned short)(n) & 0xFF00) >> 8))
  #define STATIC_NTOHS(n) STATIC_HTONS(n)

  #define STATIC_HTONL(n) (((((unsigned long)(n) & 0xFF)) << 24) | \
                           ((((unsigned long)(n) & 0xFF00)) << 8) | \
                           ((((unsigned long)(n) & 0xFF0000)) >> 8) | \
                           ((((unsigned long)(n) & 0xFF000000)) >> 24))

  #define STATIC_NTOHL(n) STATIC_HTONL(n)

  #ifdef BYTESWAP_GCC
    #define HTONS(n) ((uint16_t) __builtin_bswap16(n))
    #define NTOHS(n) HTONS(n)

    #define HTONL(n) ((uint32_t) __builtin_bswap32(n))
    #define NTOHL(n) HTONL(n)
  #else
    #define HTONS(n) STATIC_HTONS(n)
    #define NTOHS(n) STATIC_NTOHS(n)

    #define HTONL(n) STATIC_HTONL(n)

    #define NTOHL(n) STATIC_NTOHL(n)
  #endif
#endif

#define htons(n) HTONS(n)
#define ntohs(n) NTOHS(n)

#define htonl(n) HTONL(n)
#define ntohl(n) NTOHL(n)

#endif
