# $ATUWI: $Id: Makefile,v 1.1 2004/08/24 14:39:35 daan Exp $

S=	${.CURDIR}/../..
.PATH: $S/dev/usb

KMOD=	if_atuwi
SRCS=	if_atuwi.c opt_bdg.h opt_usb.h device_if.h bus_if.h
SRCS+=	miibus_if.h

.include <bsd.kmod.mk>
