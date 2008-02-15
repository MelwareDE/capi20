##
## $Id: Makefile.am,v 1.1 2003/08/27 05:59:19 acs Exp $
##

AUTOMAKE_OPTIONS	= foreign
CLEANFILES              = *~ comperr
MAINTAINERCLEANFILES    = configure aclocal.m4 Makefile.in config.h.in \
			  stamp-h.in comperr

INCLUDES = $(all_includes)
CFLAGS = -Wall -O2
LDFLAGS = $(all_libraries)
LDADD = -lcapi20

sbin_PROGRAMS = rcapid
man_MANS = rcapid.8

rcapid_SOURCES = rcapid.c
