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
  #ifndef packed_struct
    #define packed_struct __packed struct
  #endif

  #ifndef likely
    #define likely(expr)   expr
  #endif

  #define unlikely(expr) expr
#elif defined __WATCOMC__
  #ifndef packed_struct
    #define packed_struct   _Packed struct
  #endif

  #ifndef likely
    #define likely(expr)   expr
  #endif

  #ifndef unlikely
    #define unlikely(expr) expr
  #endif
#else
  #ifndef packed_struct
    #define packed_struct struct __attribute__((packed))
  #endif

  #ifdef __GNUC__
    #define __GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)

    #if ((__GCC_VERSION >= 40800))
      #define BYTESWAP_GCC

      #ifndef likely
        #define likely(expr)       __builtin_expect(!!(expr), 1)
      #endif

      #ifndef unlikely
        #define unlikely(expr)     __builtin_expect(!!(expr), 0)
      #endif

    #endif
  #endif
#endif

#if BYTE_ORDER == BIG_ENDIAN

  #ifndef HTONS
    #define HTONS(n) (n)
  #endif

  #ifndef NTOHS
    #define NTOHS(n) (n)
  #endif

  #ifndef HTONL
    #define HTONL(n) (n)
  #endif

  #ifndef NTOHL
    #define NTOHL(n) (n)
  #endif

#else
  #define _STATIC_HTONS(n) (((((unsigned short)(n) & 0xFF)) << 8) | (((unsigned short)(n) & 0xFF00) >> 8))
  #define _STATIC_NTOHS(n) _STATIC_HTONS(n)

  #define _STATIC_HTONL(n) (((((unsigned long)(n) & 0xFF)) << 24) | \
                           ((((unsigned long)(n) & 0xFF00)) << 8) | \
                           ((((unsigned long)(n) & 0xFF0000)) >> 8) | \
                           ((((unsigned long)(n) & 0xFF000000)) >> 24))

  #define _STATIC_NTOHL(n) _STATIC_HTONL(n)

  #ifdef BYTESWAP_GCC
    #ifndef HTONS
      #define HTONS(n) ((uint16_t) __builtin_bswap16(n))
    #endif

    #ifndef NTOHS
      #define NTOHS(n) HTONS(n)
    #endif

    #ifndef HTONL
      #define HTONL(n) ((uint32_t) __builtin_bswap32(n))
    #endif

    #ifndef NTOHL
      #define NTOHL(n) HTONL(n)
    #endif
  #else
    #ifndef HTONS
      #define HTONS(n) _STATIC_HTONS(n)
    #endif

    #ifndef NTOHS
      #define NTOHS(n) _STATIC_NTOHS(n)
    #endif

    #ifndef HTONL
      #define HTONL(n) _STATIC_HTONL(n)
    #endif

    #ifndef NTOHL
      #define NTOHL(n) _STATIC_NTOHL(n)
    #endif
  #endif
#endif

#ifndef htons
  #define htons(n) HTONS(n)
#endif

#ifndef ntohs
  #define ntohs(n) NTOHS(n)
#endif

#ifndef htonl
  #define htonl(n) HTONL(n)
#endif

#ifndef ntohl
  #define ntohl(n) NTOHL(n)
#endif

#endif
