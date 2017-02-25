# $FreeBSD$
#
# Common rules for building firmware.  Note this gets auto-included
# by the subdir Makefile's as a consequence of included bsd.kmod.mk.

.ifndef IMGS
IMGS=	${IMG}
.endif
.for IMG in ${IMGS}
CLEANFILES+=	${IMG}.fw
FIRMWS+=	${IMG}.fw:${IMG}:${IMG:C/.*-//}

#
# Note that a license ack is not needed for iwn.
#
#FIRMWARE_LICENSE=

${IMG}.fw: ${.CURDIR}/../../../contrib/dev/iwm/${IMG}.fw.uu
	uudecode -p $? > ${.TARGET}
.endfor
