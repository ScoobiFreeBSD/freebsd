# $FreeBSD$

.PATH:  ${.CURDIR}/../../dev/iwl

KMOD=	if_iwl

# Main driver
SRCS+=	iwl-io.o iwl-io.o iwl-drv.o iwl-debug.o iwl-notif-wait.o
SRCS+=	iwl-eeprom-read.o iwl-eeprom-parse.o iwl-phy-db.o iwl-nvm-parse.o
SRCS+=	iwl-1000.o iwl-2000.o iwl-5000.o iwl-6000.o
SRCS+=	iwl-7000.o iwl-8000.o
SRCS+=	iwl-trans.o
SRCS+=	iwl-devtrace.o

# PCIe bus
SRCS+=	pcie/drv.o pcie/rx.o pcie/tx.o pcie/trans.o

SRCS_dvm=	iwldvm.o
SRCS_dvm+=	main.o rs.o mac80211.o ucode.o tx.o
SRCS_dvm+=	lib.o calib.o tt.o sta.o rx.o
SRCS_dvm+=	power.o
SRCS_dvm+=	scan.o
SRCS_dvm+=	rxon.o devices.o
SRCS_dvm+=	led.o
SRCS_dvm+=	debugfs.o
SRCS+=	${SRCS_dvm:S/^/dvm\//}

SRCS_mvm=	iwlmvm.o
SRCS_mvm+=	fw.o mac80211.o nvm.o ops.o phy-ctxt.o mac-ctxt.o
SRCS_mvm+=	utils.o rx.o tx.o binding.o quota.o sta.o sf.o
SRCS_mvm+=	scan.o time-event.o rs.o
SRCS_mvm+=	power.o coex.o coex_legacy.o
SRCS_mvm+=	tt.o offloading.o tdls.o
SRCS_mvm+=	debugfs.o debugfs-vif.o
SRCS_mvm+=	led.o
SRCS_mvm+=	tof.o
SRCS_mvm+=	d3.o
SRCS+=	${SRCS_mvm:S/^/mvm\//}

CFLAGS+=	-DIWM_DEBUG -D__CHECK_ENDIAN__

.include <bsd.kmod.mk>
