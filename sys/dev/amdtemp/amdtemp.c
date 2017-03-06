/*-
 * Copyright (c) 2008, 2009 Rui Paulo <rpaulo@FreeBSD.org>
 * Copyright (c) 2009 Norikatsu Shigemura <nork@FreeBSD.org>
 * Copyright (c) 2009 - 2012 Jung-uk Kim <jkim@FreeBSD.org>
 * Copyright (c) 2013 - 2017 Rozhuk Ivan <rozhuk.im@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Driver for the AMD CPU on-die thermal sensors.
 * Initially based on the k8temp Linux driver.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>
#include <machine/cputypes.h>
#include <machine/pci_cfgreg.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>


struct amdtemp_softc {
	device_t		dev;
	struct mtx		lock; /* Read/write lock for some registers. */
	uint32_t		cpu_ncores;
	uint32_t		flags;
	uint32_t		tts_flags; /* Thermaltrip Status flags. */
	int32_t			tts_temp_offset[4];
	int32_t			rtc_temp_offset;
	int32_t			tsi_temp_offset[8];
	struct sysctl_oid	*sysctl_cpu[MAXCPU]; /* dev.cpu.X.temperature oids. */
	struct intr_config_hook	sc_ich;
};
#define	AMDTEMP_F_TTS		1	/* Thermaltrip Status. */
#define	AMDTEMP_F_HTC		2	/* Hardware Thermal Control (HTC). */
#define	AMDTEMP_F_RTC		4	/* Reported Temperature Control. */
#define	AMDTEMP_F_TSI		8	/* TSI via CPU registers. */
#define	AMDTEMP_F_SBTSI		16	/* TSI via SMBus. */

#define	AMDTEMP_TTS_F_CS_SWAP	0x01	/* ThermSenseCoreSel is inverted. */
#define	AMDTEMP_TTS_F_CT_10BIT	0x02	/* CurTmp is 10-bit wide. */
#define	AMDTEMP_TTS_F_OFF28	0x04	/* CurTmp starts at -28C. */


#define	AMDTEMP_LOCK(__sc)	mtx_lock(&(__sc)->lock)
#define	AMDTEMP_UNLOCK(__sc)	mtx_unlock(&(__sc)->lock)


/* D18F3xFC CPUID Family/Model/Stepping */
#define	AMD_REG_CPUID		0xfc

/*
 * Thermaltrip Status Register
 * BIOS and Kernel Developer’s Guide for AMD NPT Family 0Fh Processors
 * 32559 Rev. 3.16 November 2009
 */
/* D18F3xE4 Thermtrip Status Register */
#define	AMD_REG_THERMTRIP_STAT	0xe4
union reg_amd_thermtrip_status_desc {
	uint32_t u32;
	struct reg_amd_thermtrip_status_bits {
		uint32_t r0:1;		/* 0 Reserved. */
		uint32_t Thermtp:1;	/* 1 ro The processor has entered the THERMTRIP state. */
		uint32_t ThermSenseCoreSel:1; /* 2 rw  */
		uint32_t ThermtpSense0:1; /* 3 ro  */
		uint32_t ThermtpSense1:1; /* 4 ro  */
		uint32_t ThermtpEn:1;	/* 5 ro The THERMTRIP state is supported by the processor. */
		uint32_t ThermSenseSel:1; /* 6 rw  */
		uint32_t r1:1;		/* 7 Reserved. */
		uint32_t DiodeOffset:6;	/* 13:8 ro Thermal diode offset is used to correct the measurement made by an external temperature sensor. */
		uint32_t CurTmp:10;	/* 23:14 ro This field returns the current value of the internal thermal sensor. */
		uint32_t TjOffset:5;	/* 28:24 ro This field is the offset from CurTmp used to normalize to Tcontrol. */
		uint32_t r2:2;		/* 30:29 Reserved. */
		uint32_t SwThermtp:1;	/* 31 rw  */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_thermtrip_status_bits) == sizeof(uint32_t));


/* DRAM Configuration High Register */
#define	AMD_REG_DRAM_CONF_HIGH	0x94	/* Function 2 */
#define	AMD_REG_DRAM_MODE_DDR3	0x0100

/*
 * The default value of the HTC temperature threshold (Tctl_max) is specified
 * in the AMD Family 14h Processor Power and Thermal Datasheet.
 */
/* D18F3x64 Hardware Thermal Control (HTC) */
#define	AMD_REG_HTC_CTRL	0x64
union reg_amd_htc_desc {
	uint32_t u32;
	struct reg_amd_htc_bits {
		uint32_t HtcEn:1;	/* 0 rw 1=HTC is enabled; the processor is capable of entering the HTC-active state. */
		uint32_t r0:3;		/* 3:1 Reserved. */
		uint32_t HtcAct:1;	/* 4 ro 1=The processor is currently in the HTC-active state. */
		uint32_t HtcActSts:1;	/* 5 ro Read; set-by-hardware; write-1-to-clear. Reset: 0. This bit is set by hardware when the processor enters the HTC-active state. It is cleared by writing a 1 to it. */
		uint32_t PslApicHiEn:1;	/* 6 rw P-state limit higher value change APIC interrupt enable. */
		uint32_t PslApicLoEn:1;	/* 7 rw  P-state limit lower value change APIC interrupt enable. */
		uint32_t r1:8;		/* 15:8 Reserved. */
		uint32_t HtcTmpLmt:7;	/* 22:16 rw HTC temperature limit. */
		uint32_t HtcSlewSel:1;	/* 23 rw HTC slew-controlled temperature select. */
		uint32_t HtcHystLmt:4;	/* 27:24 rw HTC hysteresis. The processor exits the HTC active state when the temperature selected by HtcSlewSel is less than the HTC temperature limit (HtcTmpLmt) minus the HTC hysteresis (HtcHystLmt). */
		uint32_t HtcPstateLimit:3; /* 30:28 rw  HTC P-state limit select. */
		uint32_t HtcLock:1;	/* 31 Read; write-1-only. 1=HtcPstateLimit, HtcHystLmt, HtcTmpLmt, and HtcEn are read-only. */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_htc_bits) == sizeof(uint32_t));


/* D18F3xA4 Reported Temperature Control Register */
#define	AMD_REG_REPTMP_CTRL	0xa4
union reg_amd_rep_tmp_ctrl_desc {
	uint32_t u32;
	struct reg_amd_rep_tmp_ctrl_bits {
		uint32_t PerStepTimeUp:5; /* 4:0 rw per 1/8th step time up. */
		uint32_t TmpMaxDiffUp:2;/* 6:5 rw temperature maximum difference up. */
		uint32_t TmpSlewDnEn:1;	/* 7 rw temperature slew downward enable. */
		uint32_t PerStepTimeDn:5;/* 12:8 rw per 1/8th step time down. */
		uint32_t r0:3;		/* 15:13 Reserved. */
		uint32_t CurTmpTjSel:2;	/* 17:16 rw Current temperature select. */
		uint32_t CurTmpTjSlewSel:1;/* 18 rw  */
		uint32_t CurTmpRangeSel:1;/* 19 rw  */
		uint32_t r1:1;		/* 20 Reserved. */
		uint32_t CurTmp:11;	/* 31:21 ro/rw current temperature. */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_rep_tmp_ctrl_bits) == sizeof(uint32_t));
/* CurTmpTjSel valid family 10h, 15h, 16h processors. */


/* SB-TSI */
#define AMD_REG_SBI_CTRL	0x1e4 /* SBI Control */
union reg_amd_sbi_ctrl_desc {
	uint32_t u32;
	struct reg_amd_sbi_ctrl_bits {
		uint32_t r0:1;		/* 0 Reserved. */
		uint32_t SbRmiDis:1;	/* 1 ro SMBus-based sideband remote management interface disable. */
		uint32_t r1:1;		/* 2 Reserved. */
		uint32_t SbTsiDis:1;	/* 3 ro SMBus-based sideband temperature sensor interface disable. */
		uint32_t SbiAddr:3;	/* 6:4 rw SMBus-based sideband interface address. */
		uint32_t r2:1;		/* 7 Reserved. */
		uint32_t LvtOffset:4;	/* 11:8 rw local vector table offset. */
		uint32_t r3:19;		/* 30:12 Reserved. */
		uint32_t SbiRegWrDn:1;	/* 31 ro SBI register write complete. */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_sbi_ctrl_bits) == sizeof(uint32_t));
/*
 * AMD Family 10h Processor BKDG: SbRmiDis (bit offset: 1) + SbTsiDis (bit offset: 3)
 * AMD Family 11h Processor BKDG: SbTsiDis (bit offset: 1)
 * AMD Family 12h Processor BKDG: SbTsiDis (bit offset: 1)
 * BKDG for AMD Family 14h Models 00h-0Fh Processors: SbTsiDis (bit offset: 1)
 * BKDG for AMD Family 15h Models 00h-0Fh Processors: SbRmiDis, no TSI
 * BKDG for AMD Family 16h Models 00h-0Fh Processors: ??? 48751 Rev 3.00 - May 30, 2013
 */
#define AMD_REG_SBI_ADDR	0x1e8 /* SBI Address */
#define AMD_REG_SBI_ADDR_MASK	0x07
#define AMD_REG_SBI_DATA	0x1ec /* SBI Data */
#define AMD_SBI_WRITE_TIMEOUT	100 /* XXX should be increased? */

/* SB-TSI registers. */
#define SB_TSI_REG_CPU_TEMP_HB		0x01 /* CPU Temperature High Byte Register. */
#define SB_TSI_REG_STATUS		0x02 /* SB-TSI Status Register. */
#define SB_TSI_REG_CFG			0x03 /* SB-TSI Configuration Register. */
#define SB_TSI_REG_UPD_RATE		0x04 /* Update Rate Register. */
#define SB_TSI_REG_HIGH_TEMP_THB	0x07 /* High Temperature Threshold High Byte Register. */
#define SB_TSI_REG_LOW_TEMP_THB		0x08 /* Low Temperature Threshold High Byte Register.*/
#define SB_TSI_REG_CFG2			0x09 /* SB-TSI Configuration Register. */
#define SB_TSI_REG_CPU_TEMP_LB		0x10 /* CPU Temperature Low Byte Register. */
#define SB_TSI_REG_CPU_TEMP_OFF_HB	0x11 /* CPU Temperature Offset High Byte Register. */
#define SB_TSI_REG_CPU_TEMP_OFF_LB	0x12 /* CPU Temperature Offset Low Byte Register. */
#define SB_TSI_REG_HIGH_TEMP_TLB	0x13 /* High Temperature Threshold Low Byte Register. */
#define SB_TSI_REG_LOW_TEMP_TLB		0x14 /* Low Temperature Threshold Low Byte Register. */
#define SB_TSI_REG_TIMEOUT_CFG		0x22 /* Timeout Configuration Register. */
#define SB_TSI_REG_ALERT_THRESHOLD	0x32 /* Alert Threshold Register. */
#define SB_TSI_REG_ALERT_CFG		0xbf /* Alert Configuration Register. */
#define SB_TSI_REG_MANUFACTURE_ID	0xfe /* Manufacture ID Register. */
#define SB_TSI_REG_REVISION		0xff /* SB-TSI Revision Register. */



#define	AMDTEMP_ZERO_C_TO_K	2732


#define ARG2_GET_REG(__arg)	((__arg) & 0xffff)
#define ARG2_GET_A1(__arg)	(((__arg) >> 16) & 0xff)
#define ARG2_GET_A2(__arg)	(((__arg) >> 24) & 0xff)
#define MAKE_ARG2(__reg, __a1, __a2)					\
    (((__reg) & 0xff) | (((__a1) & 0xff) << 16) | (((__a2) & 0xff) << 24))

struct amdtemp_sysctl_reg {
	uint16_t	reg;
	uint8_t		a1;
	uint8_t		a2;
	uint32_t	flags;
	char		*fmt;
	int 		(*oid_handler)(SYSCTL_HANDLER_ARGS);
	char		*name;
	char		*descr;
};

static void	amdtemp_sysctl_reg_add(struct amdtemp_softc *sc,
		    struct sysctl_oid_list *child,
		    struct amdtemp_sysctl_reg *regs);
static void	amdtemp_sysctl_reg_add2(struct amdtemp_softc *sc,
		    struct sysctl_oid_list *child,
		    struct amdtemp_sysctl_reg *regs, uint32_t a2);
static int	amdtemp_sysctl_reg_bits(SYSCTL_HANDLER_ARGS);

static uint32_t	amdtemp_tts_get_temp(struct amdtemp_softc *sc,
		    uint32_t reg, uint8_t core, uint8_t sense);
static int	amdtemp_tts_temp_reg_sysctl(SYSCTL_HANDLER_ARGS);

static int	amdtemp_htc_temp_sysctl(SYSCTL_HANDLER_ARGS);

static int	amdtemp_rtc_temp_sysctl(SYSCTL_HANDLER_ARGS);

static void	amdtemp_sbi_set_addr(struct amdtemp_softc *sc,
		    uint32_t sbi_addr);
static uint32_t	amdtemp_sbi_read(struct amdtemp_softc *sc,
		    uint32_t sbi_addr, uint32_t reg_addr);
static int	amdtemp_sbi_write(struct amdtemp_softc *sc,
		    uint32_t sbi_addr, uint32_t reg_addr, uint8_t data);
static int	amdtemp_tsi_reg_sysctl(SYSCTL_HANDLER_ARGS);
static int	amdtemp_tsi_temp_reg_sysctl(SYSCTL_HANDLER_ARGS);


/* D18F3xE4 Thermtrip Status Register */
static struct amdtemp_sysctl_reg amdtemp_thermtrip_status_reg_bits[] = {
	{
		AMD_REG_THERMTRIP_STAT,
		24,
		5,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"TjOffset",
		__DESCR("This field is the offset from CurTmp used to "
		"normalize to Tcontrol.")
	}, {
		AMD_REG_THERMTRIP_STAT,
		8,
		6,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"DiodeOffset",
		__DESCR("Thermal diode offset is used to correct the "
		"measurement made by an external temperature sensor.")
	}, {
		AMD_REG_THERMTRIP_STAT,
		5,
		1,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"ThermtpEn",
		__DESCR("The THERMTRIP state is supported by the "
		"processor.")
	}, {
		AMD_REG_THERMTRIP_STAT,
		1,
		1,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"Thermtrip",
		__DESCR("The processor has entered the THERMTRIP state.")
	},
	{ 0, 0, 0, 0, NULL, NULL, NULL, NULL }
};

/* D18F3x64 Hardware Thermal Control (HTC) */
static struct amdtemp_sysctl_reg amdtemp_htc_reg_bits[] = {
	{
		AMD_REG_HTC_CTRL,
		16,
		7,
		(CTLFLAG_RD | CTLTYPE_INT),
		"IK",
		amdtemp_htc_temp_sysctl,
		"HtcTmpLmt",
		__DESCR("HTC temperature limit")
	}, {
		AMD_REG_HTC_CTRL,
		24,
		4,
		(CTLFLAG_RW | CTLTYPE_INT),
		"IK",
		amdtemp_htc_temp_sysctl,
		"HtcHystLmt",
		__DESCR("HTC hysteresis. The processor exits the "
		"HTC active state when the temperature selected by "
		"HtcSlewSel is less than the HTC temperature limit "
		"(HtcTmpLmt) minus the HTC hysteresis (HtcHystLmt).")
	}, {
		AMD_REG_HTC_CTRL,
		0,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcEn",
		__DESCR("HTC is enabled; the processor is capable of "
		"entering the HTC-active state.")
	}, {
		AMD_REG_HTC_CTRL,
		31,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcLock",
		__DESCR("HtcPstateLimit, HtcHystLmt, HtcTmpLmt, and "
		"HtcEn are read-only.")
	}, {
		AMD_REG_HTC_CTRL,
		23,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcSlewSel",
		__DESCR("HTC slew-controlled temperature select.")
	}, {
		AMD_REG_HTC_CTRL,
		28,
		3,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcPstateLimit",
		__DESCR("HTC P-state limit select.")
	}, {
		AMD_REG_HTC_CTRL,
		4,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcAct",
		__DESCR("The processor is currently in the HTC-active "
		"state.")
	}, {
		AMD_REG_HTC_CTRL,
		5,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"HtcActSts",
		__DESCR("set-by-hardware; write-1-to-clear. Reset: 0. "
		"This bit is set by hardware when the processor enters "
		"the HTC-active state. It is cleared by writing a 1 to "
		"it.")
	}, {
		AMD_REG_HTC_CTRL,
		6,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"PslApicHiEn",
		__DESCR("P-state limit higher value change APIC "
		"interrupt enable.")
	}, {
		AMD_REG_HTC_CTRL,
		7,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"PslApicLoEn",
		__DESCR("P-state limit lower value change APIC "
		"interrupt enable.")
	},
	{ 0, 0, 0, 0, NULL, NULL, NULL, NULL }
};

/* D18F3xA4 Reported Temperature Control Register */
static struct amdtemp_sysctl_reg amdtemp_reptmp_reg_bits[] = {
	{
		AMD_REG_REPTMP_CTRL,
		21,
		11,
		(CTLFLAG_RD | CTLTYPE_INT),
		"IK",
		amdtemp_rtc_temp_sysctl,
		"CurTmp",
		__DESCR("Provides the current control temperature, "
		"Tctl, after the slew-rate controls have been applied.")
	}, {
		AMD_REG_REPTMP_CTRL,
		16,
		2,
		(CTLFLAG_RW | CTLTYPE_INT),
		"IK",
		amdtemp_rtc_temp_sysctl,
		"CurTmpTjSel",
		__DESCR("Specifies a value used to create Tctl.")
	},
	/*{
		AMD_REG_REPTMP_CTRL,
		18,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"CurTmpTjSlewSel",
		__DESCR("")
	}, {
		AMD_REG_REPTMP_CTRL,
		19,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"CurTmpRangeSel",
		__DESCR("")
	},*/
	{
		AMD_REG_REPTMP_CTRL,
		7,
		1,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"TmpSlewDnEn",
		__DESCR("Temperature slew downward enable.")
	}, {
		AMD_REG_REPTMP_CTRL,
		5,
		2,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"TmpMaxDiffUp",
		__DESCR("Specifies the maximum difference, (Tctlm - "
		"Tctl), when Tctl immediatly updates to Tctlm.")
	}, {
		AMD_REG_REPTMP_CTRL,
		8,
		5,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"PerStepTimeDn",
		__DESCR("Specifies the time that Tctlm must remain "
		"below Tctl before applying a 0.125 downward step.")
	}, {
		AMD_REG_REPTMP_CTRL,
		0,
		5,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_sysctl_reg_bits,
		"PerStepTimeUp",
		__DESCR("Specifies the time that Tctlm must remain "
		"above Tctl before applying a 0.125 upward step.")
	},
	{ 0, 0, 0, 0, NULL, NULL, NULL, NULL }
};

/* SB-TSI registers. */
static struct amdtemp_sysctl_reg amdtemp_tsi_regs[] = {
	{
		SB_TSI_REG_CPU_TEMP_LB,
		SB_TSI_REG_CPU_TEMP_HB,
		0,
		(CTLFLAG_RD | CTLTYPE_INT),
		"IK",
		amdtemp_tsi_temp_reg_sysctl,
		"cpu_temperature",
		__DESCR("CPU Temperature")
	}, {
		SB_TSI_REG_HIGH_TEMP_TLB,
		SB_TSI_REG_HIGH_TEMP_THB,
		0,
		(CTLFLAG_RD | CTLTYPE_INT),
		"IK",
		amdtemp_tsi_temp_reg_sysctl,
		"high_temperature_threshold",
		__DESCR("High Temperature Threshold")
	}, {
		SB_TSI_REG_LOW_TEMP_TLB,
		SB_TSI_REG_LOW_TEMP_THB,
		0,
		(CTLFLAG_RD | CTLTYPE_INT),
		"IK",
		amdtemp_tsi_temp_reg_sysctl,
		"low_temperature_threshold",
		__DESCR("Low Temperature Threshold")
	},

	{
		SB_TSI_REG_CPU_TEMP_OFF_HB,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"cpu_temperature_offset_hi",
		__DESCR("CPU Temperature Offset High Byte")
	}, {
		SB_TSI_REG_CPU_TEMP_OFF_LB,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"cpu_temperature_offset_lo",
		__DESCR("CPU Temperature Offset Low Byte")
	},

	{
		SB_TSI_REG_STATUS,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"status",
		__DESCR("SB-TSI Status")
	}, {
		SB_TSI_REG_CFG,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"cfg3",
		__DESCR("SB-TSI Configuration Register 0x03")
	}, {
		SB_TSI_REG_CFG2,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"cfg9",
		__DESCR("SB-TSI Configuration Register 0x09")
	}, {
		SB_TSI_REG_UPD_RATE,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"upd_rate",
		__DESCR("Update Rate")
	}, {
		SB_TSI_REG_TIMEOUT_CFG,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"timeout_cfg",
		__DESCR("Timeout Configuration")
	}, {
		SB_TSI_REG_ALERT_THRESHOLD,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"alert_threshold",
		__DESCR("Alert Threshold")
	}, {
		SB_TSI_REG_ALERT_CFG,
		0,
		0,
		(CTLFLAG_RW | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"alert_cfg",
		__DESCR("Alert Configuration")
	}, {
		SB_TSI_REG_MANUFACTURE_ID,
		0,
		0,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"manufacture_id",
		__DESCR("Manufacture ID")
	}, {
		SB_TSI_REG_REVISION,
		0,
		0,
		(CTLFLAG_RD | CTLTYPE_UINT),
		"IU",
		amdtemp_tsi_reg_sysctl,
		"revision",
		__DESCR("SB-TSI Revision")
	},

	{ 0, 0, 0, 0, NULL, NULL, NULL, NULL }
};


/*
 * Device methods.
 */
static void	amdtemp_identify(driver_t *driver, device_t parent);
static int	amdtemp_probe(device_t dev);
static int	amdtemp_attach(device_t dev);
static int	amdtemp_detach(device_t dev);
static void	amdtemp_intrhook(void *arg);



static device_method_t amdtemp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	amdtemp_identify),
	DEVMETHOD(device_probe,		amdtemp_probe),
	DEVMETHOD(device_attach,	amdtemp_attach),
	DEVMETHOD(device_detach,	amdtemp_detach),

	DEVMETHOD_END
};

static driver_t amdtemp_driver = {
	"amdtemp",
	amdtemp_methods,
	sizeof(struct amdtemp_softc),
};
static devclass_t amdtemp_devclass;
DRIVER_MODULE(amdtemp, hostb, amdtemp_driver, amdtemp_devclass, NULL, NULL);
MODULE_VERSION(amdtemp, 1);


static int
amdtemp_dev_check(device_t dev)
{
	uint32_t cpuid;

	if (resource_disabled("amdtemp", 0))
		return (ENXIO);
	/*
	 * Device 18h Function 3 Configuration Registers:
	 * vendor = AMD (0x1022)
	 * class = bridge (0x06000000)
	 * function = 3
	 */
	if (pci_get_vendor(dev) != CPU_VENDOR_AMD ||
	    pci_get_class(dev) != PCIC_BRIDGE ||
	    pci_get_function(dev) != 3)
		return (ENXIO);
	/* Does processor have Temperature sensor / THERMTRIP / HTC ? */
	if ((amd_pminfo & (AMDPM_TS | AMDPM_TTP | AMDPM_TM)) == 0)
		return (ENXIO);
	/* Check minimum cpu family. */
	cpuid = pci_read_config(dev, AMD_REG_CPUID, 4);
	if (CPUID_TO_FAMILY(cpuid) < 0x0f)
		return (ENXIO);

	return (0);
}

static void
amdtemp_identify(driver_t *driver, device_t parent)
{
	device_t child;

	/* Make sure we're not being doubly invoked. */
	if (device_find_child(parent, "amdtemp", -1))
		return;
	if (amdtemp_dev_check(parent))
		return;
	child = device_add_child(parent, "amdtemp", -1);
	if (child == NULL) {
		device_printf(parent, "add amdtemp child failed.\n");
	}
}

static int
amdtemp_probe(device_t dev)
{

	if (amdtemp_dev_check(dev))
		return (ENXIO);
	device_set_desc(dev, "AMD CPU On-Die Thermal Sensors");

	return (BUS_PROBE_GENERIC);
}

static int
amdtemp_attach(device_t dev)
{
	struct amdtemp_softc *sc = device_get_softc(dev);
	uint32_t i, cpuid, model;
	union reg_amd_thermtrip_status_desc reg_tts;
	union reg_amd_htc_desc reg_htc;
	union reg_amd_sbi_ctrl_desc reg_sbi;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child, *list;
	struct sysctl_oid *node, *sub_node;
	char str[32];
	int tsi_ok, erratum319 = 0;
	u_int regs[4], bid;

	sc->dev = dev;
	/* Find number of cores per package. */
	sc->cpu_ncores = ((amd_feature2 & AMDID2_CMP) ?
	    ((cpu_procinfo2 & AMDID_CMP_CORES) + 1) : 1);
	if (sc->cpu_ncores > MAXCPU)
		return (ENXIO);
	mtx_init(&sc->lock, device_get_nameunit(dev), "amdtemp", MTX_DEF);

	/* Detect supported therm interfaces. */
	cpuid = pci_read_config(dev, AMD_REG_CPUID, 4);
	model = CPUID_TO_MODEL(cpuid);
	switch (CPUID_TO_FAMILY(cpuid)) {
	case 0x0f:
		/*
		 * Thermaltrip Status Register
		 *
		 * - ThermSenseCoreSel
		 *
		 * Revision F & G:	0 - Core1, 1 - Core0
		 * Other:		0 - Core0, 1 - Core1
		 *
		 * - CurTmp
		 *
		 * Revision G:		bits 23-14
		 * Other:		bits 23-16
		 *
		 * XXX According to the BKDG, CurTmp, ThermSenseSel and
		 * ThermSenseCoreSel bits were introduced in Revision F
		 * but CurTmp seems working fine as early as Revision C.
		 * However, it is not clear whether ThermSenseSel and/or
		 * ThermSenseCoreSel work in undocumented cases as well.
		 * In fact, the Linux driver suggests it may not work but
		 * we just assume it does until we find otherwise.
		 *
		 * XXX According to Linux, CurTmp starts at -28C on
		 * Socket AM2 Revision G processors, which is not
		 * documented anywhere.
		 * XXX check TjOffset and DiodeOffset for -49C / -28C
		 */
		if ((amd_pminfo & AMDPM_TTP) == 0) /* No TTP: THERMTRIP */
			break;
		reg_tts.u32 = pci_read_config(dev, AMD_REG_THERMTRIP_STAT, 4);
		if (reg_tts.bits.ThermtpEn == 0)
			break;
		if ((model == 0x04 && (cpuid & CPUID_STEPPING) == 0) ||
		    (model == 0x05 && (cpuid & CPUID_STEPPING) <= 1))
			break; /* No ThermalTrip. */
		sc->flags |= AMDTEMP_F_TTS;
		if (model >= 0x40) {
			sc->tts_flags |= AMDTEMP_TTS_F_CS_SWAP;
		}
		if (model >= 0x60 && model != 0xc1) {
			do_cpuid(0x80000001, regs);
			bid = ((regs[1] >> 9) & 0x1f);
			switch (model) {
			case 0x68: /* Socket S1g1 */
			case 0x6c:
			case 0x7c:
				break;
			case 0x6b: /* Socket AM2 and ASB1 (2 cores) */
				if (bid != 0x0b && bid != 0x0c) {
					sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				}
				break;
			case 0x6f: /* Socket AM2 and ASB1 (1 core) */
			case 0x7f:
				if (bid != 0x07 && bid != 0x09 &&
				    bid != 0x0c) {
					sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				}
				break;
			default:
				sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				break;
			}
			sc->tts_flags |= AMDTEMP_TTS_F_CT_10BIT;
		}
		break;
	case 0x10:
		sc->flags |= AMDTEMP_F_RTC;
		reg_sbi.u32 = pci_read_config(dev, AMD_REG_SBI_CTRL, 4);
		if (reg_sbi.bits.SbTsiDis == 0) {
			sc->flags |= AMDTEMP_F_TSI;
		}
		/*
		 * Erratum 319 Inaccurate Temperature Measurement
		 * http://support.amd.com/us/Processor_TechDocs/41322.pdf
		 */
		do_cpuid(0x80000001, regs);
		switch (((regs[1] >> 28) & 0x0f)) {
		case 0:	/* Socket F */
			erratum319 = 1;
			break;
		case 1:	/* Socket AM2+ or AM3 */
			if ((pci_cfgregread(pci_get_bus(dev), pci_get_slot(dev), 2,
			    AMD_REG_DRAM_CONF_HIGH, 2) & AMD_REG_DRAM_MODE_DDR3) ||
			    model > 0x04 ||
			    (model == 0x04 && (cpuid & CPUID_STEPPING) >= 3))
				break;
			/* XXX 00100F42h (RB-C2) exists in both formats. */
			erratum319 = 1;
			break;
		}
		break;
	case 0x11:
	case 0x12:
	case 0x14:
	case 0x15:
	case 0x16:
	default:
		sc->flags |= AMDTEMP_F_RTC;
		reg_sbi.u32 = pci_read_config(dev, AMD_REG_SBI_CTRL, 4);
		if (reg_sbi.bits.SbRmiDis == 0 || /* = SbTsiDis */
		    reg_sbi.bits.SbTsiDis == 0) {
			sc->flags |= AMDTEMP_F_TSI;
		}
		/* XXX TODO: read TSI via SMBus. */
		break;
	}
	/* Hardware Thermal Control (HTC). */
	if (amd_pminfo & AMDPM_TM) {
		reg_htc.u32 = pci_read_config(dev, AMD_REG_HTC_CTRL, 4);
		if (reg_htc.bits.HtcEn) {
			sc->flags |= AMDTEMP_F_HTC;
		}
	}

	/* Init sysctl interface. */
	ctx = device_get_sysctl_ctx(dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	if (sc->flags & AMDTEMP_F_TTS) { /* Thermaltrip Status */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "tts",
		    CTLFLAG_RD, NULL, "Thermaltrip Status");
		list = SYSCTL_CHILDREN(node);
		amdtemp_sysctl_reg_add(sc, list, amdtemp_thermtrip_status_reg_bits);
		for (i = 0; i < sc->cpu_ncores && i < 2; i ++) {
			snprintf(str, sizeof(str), "core%i", i);
			sub_node = SYSCTL_ADD_NODE(ctx, list, OID_AUTO,
			    str, CTLFLAG_RD, NULL, "CPU core sensors");
			SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor0",
			    (CTLTYPE_INT | CTLFLAG_RD), sc,
			    MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 0),
			    amdtemp_tts_temp_reg_sysctl, "IK",
			    "Sensor 0 temperature");
			SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor1",
			    (CTLTYPE_INT | CTLFLAG_RD), sc,
			    MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 1),
			    amdtemp_tts_temp_reg_sysctl, "IK",
			    "Sensor 1 temperature");
			SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor0_offset", CTLFLAG_RW,
			    &sc->tts_temp_offset[((i << 1) | 0)], 0,
			    "Temperature sensor offset");
			SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor1_offset", CTLFLAG_RW,
			    &sc->tts_temp_offset[((i << 1) | 1)], 0,
			    "Temperature sensor offset");
		}
	}
	if (sc->flags & AMDTEMP_F_HTC) { /* Hardware Thermal Control (HTC) */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "htc",
		    CTLFLAG_RD, NULL, "Hardware Thermal Control (HTC)");
		amdtemp_sysctl_reg_add(sc, SYSCTL_CHILDREN(node),
		    amdtemp_htc_reg_bits);
	}
	if (sc->flags & AMDTEMP_F_RTC) { /* Reported Temperature Control */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "rtc",
		    CTLFLAG_RD, NULL, "Reported Temperature Control");
		list = SYSCTL_CHILDREN(node);
		amdtemp_sysctl_reg_add(sc, list, amdtemp_reptmp_reg_bits);
		SYSCTL_ADD_INT(ctx, list, OID_AUTO, "sensor_offset",
		    CTLFLAG_RW, &sc->rtc_temp_offset, 0,
		    "Temperature sensor offset");
	}
	if (sc->flags & AMDTEMP_F_TSI) { /* Temperature Sensor Interface */
		tsi_ok = 0;
		for (i = 0; i < 8; i ++) {
			if (amdtemp_sbi_read(sc, i, SB_TSI_REG_REVISION) == 0)
				continue;
			if (tsi_ok == 0) { /* First time add node. */
				node = SYSCTL_ADD_NODE(ctx, child,
				    OID_AUTO, "tsi", CTLFLAG_RD, NULL,
				    "Temperature Sensor Interface");
				list = SYSCTL_CHILDREN(node);
				tsi_ok ++;
			}
			snprintf(str, sizeof(str), "sensor%i", i);
			sub_node = SYSCTL_ADD_NODE(ctx, list, OID_AUTO,
			    str, CTLFLAG_RD, NULL, "TSI sensor");
			amdtemp_sysctl_reg_add2(sc,
			    SYSCTL_CHILDREN(sub_node),
			    amdtemp_tsi_regs, i);
			SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor_offset", CTLFLAG_RW,
			    &sc->tsi_temp_offset[i], 0,
			    "Temperature sensor offset");
		}
		if (tsi_ok == 0) { /* Unset flag if no TSI sensors found. */
			sc->flags &= ~AMDTEMP_F_TSI;
		}
	}

	if (bootverbose) {
		/* CPUID Fn8000_0007_EDX Advanced Power Management Information
		 * 0 TS: Temperature sensor.
		 * 3 TTP: THERMTRIP. Value: Fuse[ThermTripEn].
		 * 4 TM: hardware thermal control (HTC). Value: ~Fuse[HtcDis].
		 */
		if (amd_pminfo & AMDPM_TS) {
			device_printf(dev, "CPU have TS: Temperature "
			    "sensor.\n");
		}
		if (sc->flags & AMDTEMP_F_TTS) {
			device_printf(dev, "Found: Thermaltrip Status "
			    "(TTS).\n");
		}
		if (sc->flags & AMDTEMP_F_RTC) {
			device_printf(dev, "Found: Reported "
			    "Temperature Control (RTC).\n");
		}
		if (sc->flags & AMDTEMP_F_TSI) {
			device_printf(dev, "Found: Temperature Sensor "
			    "Interface via CPU registers (TSI).\n");
		}
		if (amd_pminfo & AMDPM_TM) {
			device_printf(dev, "Found: Hardware Thermal "
			    "Control (HTC).\n");
		}
	}
	if (erratum319) {
		device_printf(dev, "Erratum 319: temperature "
		    "measurement may be inaccurate.\n");
	}
	/*
	 * Try to create dev.cpu sysctl entries and setup intrhook function.
	 * This is needed because the cpu driver may be loaded late on boot,
	 * after us.
	 */
	amdtemp_intrhook(sc);
	if (sc->sysctl_cpu[0] == NULL) {
		sc->sc_ich.ich_func = amdtemp_intrhook;
		sc->sc_ich.ich_arg = sc;
		if (config_intrhook_establish(&sc->sc_ich)) {
			amdtemp_detach(dev);
			device_printf(dev, "config_intrhook_establish "
			    "failed!\n");
			return (ENXIO);
		}
	}

	return (0);
}

int
amdtemp_detach(device_t dev)
{
	struct amdtemp_softc *sc = device_get_softc(dev);
	uint32_t i;

	for (i = 0; i < sc->cpu_ncores; i ++) {
		if (sc->sysctl_cpu[i]) {
			sysctl_remove_oid(sc->sysctl_cpu[i], 1, 0);
		}
	}
	/* NewBus removes the dev.amdtemp.N tree by itself. */
	if (sc->sc_ich.ich_arg) {
		sc->sc_ich.ich_arg = NULL;
		config_intrhook_disestablish(&sc->sc_ich);
	}
	mtx_destroy(&sc->lock);

	return (0);
}

void
amdtemp_intrhook(void *arg)
{
	struct amdtemp_softc *sc = arg;
	device_t dev = sc->dev, acpi, cpu, nexus;
	int (*sysctl_handler)(SYSCTL_HANDLER_ARGS);
	intptr_t sysctl_arg2;
	uint32_t i, unit_base;

	if (sc->sc_ich.ich_arg) {
		sc->sc_ich.ich_arg = NULL;
		config_intrhook_disestablish(&sc->sc_ich);
	}

	/* dev.cpu.N.temperature. */
	nexus = device_find_child(root_bus, "nexus", 0);
	acpi = device_find_child(nexus, "acpi", 0);
	/* XXX: cpu_ncores not constant for different CPUs... */
	unit_base = (device_get_unit(dev) * sc->cpu_ncores);

	for (i = 0; i < sc->cpu_ncores; i ++) {
		if (sc->sysctl_cpu[i])
			continue;
		cpu = device_find_child(acpi, "cpu", (unit_base + i));
		if (cpu == NULL)
			continue;
		sysctl_handler = NULL;
		if ((sc->flags & AMDTEMP_F_TSI) &&
		    amdtemp_sbi_read(sc, i, SB_TSI_REG_REVISION)) {
			/* Temperature Sensor Interface */
			sysctl_handler = amdtemp_tsi_temp_reg_sysctl;
			sysctl_arg2 = MAKE_ARG2(SB_TSI_REG_CPU_TEMP_LB,
			    SB_TSI_REG_CPU_TEMP_HB, i);
		} else if (sc->flags & AMDTEMP_F_RTC) {
			/* Reported Temperature Control */
			sysctl_handler = amdtemp_rtc_temp_sysctl;
			sysctl_arg2 = MAKE_ARG2(AMD_REG_REPTMP_CTRL, 21, 11);
		} else if (sc->flags & AMDTEMP_F_TTS) {
			/* Thermaltrip Status */
			sysctl_handler = amdtemp_tts_temp_reg_sysctl;
			sysctl_arg2 = MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 0xff);
		}
		if (sysctl_handler == NULL)
			continue;
		sc->sysctl_cpu[i] = SYSCTL_ADD_PROC(
		    device_get_sysctl_ctx(cpu),
		    SYSCTL_CHILDREN(device_get_sysctl_tree(cpu)),
		    OID_AUTO, "temperature", (CTLTYPE_INT | CTLFLAG_RD),
		    sc, sysctl_arg2, sysctl_handler, "IK",
		    "Current temperature");
	}
}


/* Sysctl staff. */
static void
amdtemp_sysctl_reg_add(struct amdtemp_softc *sc,
    struct sysctl_oid_list *child, struct amdtemp_sysctl_reg *regs)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	uint32_t i;

	for (i = 0; regs[i].oid_handler; i ++) {
		SYSCTL_ADD_OID(ctx, child, OID_AUTO, regs[i].name,
		    regs[i].flags, sc,
		    MAKE_ARG2(regs[i].reg, regs[i].a1, regs[i].a2),
		    regs[i].oid_handler, regs[i].fmt, regs[i].descr);
	}
}

static void
amdtemp_sysctl_reg_add2(struct amdtemp_softc *sc,
    struct sysctl_oid_list *child,
    struct amdtemp_sysctl_reg *regs, uint32_t a2)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	uint32_t i;

	for (i = 0; regs[i].oid_handler; i ++) {
		SYSCTL_ADD_OID(ctx, child, OID_AUTO, regs[i].name,
		    regs[i].flags, sc,
		    MAKE_ARG2(regs[i].reg, regs[i].a1, a2),
		    regs[i].oid_handler, regs[i].fmt, regs[i].descr);
	}
}

static int
amdtemp_sysctl_reg_bits(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	uint32_t i, reg_data, reg_num, bits_off, bits_len, bits_mask = 0;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	bits_off = ARG2_GET_A1(arg2);
	bits_len = ARG2_GET_A2(arg2);
	reg_data = pci_read_config(sc->dev, reg_num, 4);

	for (i = 0; i < bits_len; i ++) {
		bits_mask |= (((uint32_t)1) << i);
	}

	val = ((reg_data >> bits_off) & bits_mask);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL || val == reg_data)
		return (error);
	reg_data &= ~(bits_mask << bits_off); /* Clear all bits at offset. */
	reg_data |= ((val & bits_mask) << bits_off); /* Set value bits. */
	pci_write_config(sc->dev, reg_num, reg_data, 4);

	return (0);
}


/* Thermaltrip Status Register */
static uint32_t
amdtemp_tts_get_temp(struct amdtemp_softc *sc, uint32_t reg,
    uint8_t core, uint8_t sense)
{
	union reg_amd_thermtrip_status_desc reg_tts;
	uint32_t val;

	reg_tts.u32 = 0;
	if ((sc->tts_flags & AMDTEMP_TTS_F_CS_SWAP) == 0) {
		reg_tts.bits.ThermSenseCoreSel = (core ? 1 : 0);
	} else { /* Swap. */
		reg_tts.bits.ThermSenseCoreSel = (core ? 0 : 1);
	}
	reg_tts.bits.ThermSenseSel = (sense ? 1 : 0);

	AMDTEMP_LOCK(sc);
	pci_write_config(sc->dev, reg, reg_tts.u32, 4);
	reg_tts.u32 = pci_read_config(sc->dev, reg, 4);
	AMDTEMP_UNLOCK(sc);

	val = reg_tts.bits.CurTmp;
	if ((sc->tts_flags & AMDTEMP_TTS_F_CT_10BIT) == 0) {
		val &= ~0x00000003; /* Clear first 2 bits. */
	}
	val = (AMDTEMP_ZERO_C_TO_K + ((val * 5) / 2) -
	    ((sc->tts_flags & AMDTEMP_TTS_F_OFF28) ? 280 : 490));
	val += (sc->tts_temp_offset[((reg_tts.bits.ThermSenseCoreSel << 1) |
	    reg_tts.bits.ThermSenseSel)] * 10);

	return (val);
}
/* If 0xff == ARG2_GET_A2(arg2) then retun max temp for core. */
static int
amdtemp_tts_temp_reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	uint32_t reg_num;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	if (ARG2_GET_A2(arg2) == 0xff) {
		val = imax(amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2), 0),
		    amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2), 1));
	} else {
		val = amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2),
		    ARG2_GET_A2(arg2));
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	return (0);
}


/* D18F3x64 Hardware Thermal Control (HTC) */
static int
amdtemp_htc_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	union reg_amd_htc_desc reg_htc;
	uint32_t reg_num, bits_off;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	bits_off = ARG2_GET_A1(arg2);

	reg_htc.u32 = pci_read_config(sc->dev, reg_num, 4);
	switch (bits_off) {
	case 16: /* HtcTmpLmt */
		val = (((reg_htc.bits.HtcTmpLmt * 10) / 2) + 520);
		break;
	case 24: /* HtcHystLmt */
		val = ((reg_htc.bits.HtcHystLmt * 10) / 2);
		break;
	}
	val += AMDTEMP_ZERO_C_TO_K;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	/* pci_write_config(sc->dev, reg_num, reg_htc.u32, 4); */

	return (0);
}


/* D18F3xA4 Reported Temperature Control Register */
static int
amdtemp_rtc_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	union reg_amd_rep_tmp_ctrl_desc reg_rtc;
	uint32_t reg_num, bits_off;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	bits_off = ARG2_GET_A1(arg2);

	AMDTEMP_LOCK(sc);
	reg_rtc.u32 = pci_read_config(sc->dev, reg_num, 4);
	switch (bits_off) {
	case 16: /* CurTmpTjSel */
		reg_rtc.bits.CurTmpTjSel = 3;
		break;
	case 21: /* CurTmp */
		reg_rtc.bits.CurTmpTjSel = 0;
		break;
	}
	pci_write_config(sc->dev, reg_num, reg_rtc.u32, 4);
	reg_rtc.u32 = pci_read_config(sc->dev, reg_num, 4);
	if (bits_off == 16) { /* CurTmpTjSel: switch back to CurTmp. */
		reg_rtc.bits.CurTmpTjSel = 0;
		pci_write_config(sc->dev, reg_num, reg_rtc.u32, 4);
	}
	AMDTEMP_UNLOCK(sc);

	val = (AMDTEMP_ZERO_C_TO_K + ((reg_rtc.bits.CurTmp * 10) / 8));
	if (bits_off == 16) { /* CurTmpTjSel */
		val -= 490;
	} else {
		val += (sc->rtc_temp_offset * 10);
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	/* pci_write_config(sc->dev, reg_num, reg_rtc.u32, 4); */

	return (0);
}


/* Set SMBus-based sideband interface address: 0-7. */
static void
amdtemp_sbi_set_addr(struct amdtemp_softc *sc, uint32_t sbi_addr)
{
	union reg_amd_sbi_ctrl_desc reg_sbi;

	sbi_addr &= AMD_REG_SBI_ADDR_MASK;
	reg_sbi.u32 = pci_read_config(sc->dev, AMD_REG_SBI_CTRL, 4);
	if (reg_sbi.bits.SbiAddr == sbi_addr) /* Is address allready set? */
		return;
	reg_sbi.bits.SbiAddr = sbi_addr;
	pci_write_config(sc->dev, AMD_REG_SBI_CTRL, reg_sbi.u32, 4);
}

static uint32_t
amdtemp_sbi_read(struct amdtemp_softc *sc, uint32_t sbi_addr,
    uint32_t reg_addr)
{
	uint32_t ret;

	if ((sc->flags & AMDTEMP_F_TSI) == 0)
		return (0);

	AMDTEMP_LOCK(sc);
	amdtemp_sbi_set_addr(sc, sbi_addr);
	pci_write_config(sc->dev, AMD_REG_SBI_ADDR, reg_addr, 4);
	ret = pci_read_config(sc->dev, AMD_REG_SBI_DATA, 4);
	AMDTEMP_UNLOCK(sc);

	return (ret);
}

static int
amdtemp_sbi_write(struct amdtemp_softc *sc, uint32_t sbi_addr,
    uint32_t reg_addr, uint8_t data)
{
	union reg_amd_sbi_ctrl_desc reg_sbi;
	uint32_t data32 = data;

	AMDTEMP_LOCK(sc);
	amdtemp_sbi_set_addr(sc, sbi_addr);
	pci_write_config(sc->dev, AMD_REG_SBI_ADDR, reg_addr, 4);
	pci_write_config(sc->dev, AMD_REG_SBI_DATA, data32, 4);
	/* Wait write. */
	data32 = AMD_SBI_WRITE_TIMEOUT;
	while (data32 --) {
		reg_sbi.u32 = pci_read_config(sc->dev, AMD_REG_SBI_CTRL, 4);
		if (reg_sbi.bits.SbiRegWrDn)
			break;
		DELAY(100);
	}
	AMDTEMP_UNLOCK(sc);

	if (data32 == 0) {
		device_printf(sc->dev, "timeout waiting for SBI "
		    "write.\n");
		return (1);
	}

	return (0);
}

static int
amdtemp_tsi_reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	uint32_t reg_data, reg_addr, sbi_addr;
	unsigned val;
	int error;

	reg_addr = ARG2_GET_REG(arg2);
	sbi_addr = (ARG2_GET_A2(arg2) & AMD_REG_SBI_ADDR_MASK);
	reg_data = amdtemp_sbi_read(sc, sbi_addr, reg_addr);
	val = reg_data;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL || val == reg_data)
		return (error);

	return (amdtemp_sbi_write(sc, sbi_addr, reg_addr, val));
}

static int
amdtemp_tsi_temp_reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct amdtemp_softc *sc = arg1;
	uint32_t reg_data_lo, reg_data_hi, sbi_addr;
	unsigned val;
	int error;

	sbi_addr = (ARG2_GET_A2(arg2) & AMD_REG_SBI_ADDR_MASK);
	reg_data_lo = amdtemp_sbi_read(sc, sbi_addr, ARG2_GET_REG(arg2));
	reg_data_hi = amdtemp_sbi_read(sc, sbi_addr, ARG2_GET_A1(arg2));
	val = (AMDTEMP_ZERO_C_TO_K + (reg_data_hi * 10));
	/* Apply offset only to sensor. */
	if (ARG2_GET_REG(arg2) == SB_TSI_REG_CPU_TEMP_LB) {
		val += (sc->tsi_temp_offset[sbi_addr] * 10);
	}
	if (reg_data_lo & 0x80) {
		val += 5; /* 0,5 C */
	}
	if (reg_data_lo & 0x40) {
		val += 3; /* 0,25 C */
	}
	if (reg_data_lo & 0x20) {
		val += 1; /* 0,125 C */
	}

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	return (0);
}
