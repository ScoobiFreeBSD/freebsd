#ifndef __LINUX_COMPAT_H__
#define __LINUX_COMPAT_H__

#include <sys/endian.h>

#include "linux-compat-80211.h"

#define WARN_ON(x) (x)

#define BIT(x) (1 << (x))
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define BITS_TO_LONGS(x) DIV_ROUND_UP(x, 8 * sizeof(long))
#define __printf(a, b) __attribute__((format(printf, a, b)))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define u8  uint8_t
#define __u8  u8
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t
#define s8  int8_t
#define s16 int16_t
#define s32 int32_t
#define s64 int64_t

#define __le8 u8
#define __le16 u16
#define __le32 u32
#define __le64 u64

#define le32_to_cpu(x) le32toh(x)

#define __force
#define __bitwise__

#define __ARG_PLACEHOLDER_1 0,
#define config_enabled(cfg) _config_enabled(cfg)
#define _config_enabled(value) __config_enabled(__ARG_PLACEHOLDER_##value)
#define __config_enabled(arg1_or_junk) ___config_enabled(arg1_or_junk 1, 0)
#define ___config_enabled(__ignored, val, ...) val

#define IS_BUILTIN(option) config_enabled(option)

#define IS_MODULE(option) config_enabled(option##_MODULE)

#define IS_REACHABLE(option) (config_enabled(option) || \
		 (config_enabled(option##_MODULE) && config_enabled(MODULE)))

#define IS_ENABLED(option) (IS_BUILTIN(option) || IS_MODULE(option))

#define	NBBY	8		/* number of bits in a byte */
#define	NB_BITS_PER_LONG		(sizeof(long) * NBBY)
#define	__bit_word(b)			((b) / NB_BITS_PER_LONG)
#define	__bit_mask(b)			(1UL << (b) % NB_BITS_PER_LONG)
#define	__bit_addr(p, b)		((const volatile u_long *)(p) + __bit_word(b))

#define	clear_bit(b, p) \
    atomic_clear_long(__bit_addr(p, b), __bit_mask(b))
#define	set_bit(b, p) \
    atomic_set_long(__bit_addr(p, b), __bit_mask(b))
#define	test_bit(b, p) \
    ((*__bit_addr(p, b) & __bit_mask(b)) != 0)

typedef vm_paddr_t dma_addr_t;

#define ETHTOOL_FWVERS_LEN 32

#ifndef __cplusplus

#define	false	0
#define	true	1

#define	bool	_Bool
#if __STDC_VERSION__ < 199901L && __GNUC__ < 3 && !defined(__INTEL_COMPILER)
typedef	int	_Bool;
#endif

#endif /* !__cplusplus */

#endif /* !__LINUX_COMPAT_H__ */
