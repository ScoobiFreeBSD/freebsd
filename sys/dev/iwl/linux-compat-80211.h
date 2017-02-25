#ifndef __LINUX_COMPAT_80211_H__
#define __LINUX_COMPAT_80211_H__

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/malloc.h>
#include <net80211/ieee80211.h>
#include <net80211/_ieee80211.h>
#include <net80211/ieee80211_crypto.h>

enum ieee80211_smps_mode {
	IEEE80211_SMPS_AUTOMATIC = 0x0008,	// Not supported in FreeBSD
	IEEE80211_SMPS_OFF = IEEE80211_HTCAP_SMPS_OFF,
	IEEE80211_SMPS_STATIC = IEEE80211_HTCAP_SMPS_ENA,
	IEEE80211_SMPS_DYNAMIC = IEEE80211_HTCAP_SMPS_DYNAMIC,

	/* keep last */
	IEEE80211_SMPS_NUM_MODES
};

typedef uint64_t netdev_features_t;

enum nl80211_iftype {
	NL80211_IFTYPE_UNSPECIFIED,
	NL80211_IFTYPE_ADHOC,
	NL80211_IFTYPE_STATION,
	NL80211_IFTYPE_AP,
	NL80211_IFTYPE_AP_VLAN,
	NL80211_IFTYPE_WDS,
	NL80211_IFTYPE_MONITOR,
	NL80211_IFTYPE_MESH_POINT,
	NL80211_IFTYPE_P2P_CLIENT,
	NL80211_IFTYPE_P2P_GO,
	NL80211_IFTYPE_P2P_DEVICE,
	NL80211_IFTYPE_OCB,

	/* keep last */
	NUM_NL80211_IFTYPES,
	NL80211_IFTYPE_MAX = NUM_NL80211_IFTYPES - 1
};

struct napi_struct {
};

#define ieee80211_cipher_scheme ieee80211_cipher

#endif /* !__LINUX_COMPAT_80211_H__ */
