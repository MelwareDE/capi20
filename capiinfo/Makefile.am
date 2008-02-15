##
## $Id: Makefile.am,v 1.1 2003/08/27 05:59:18 acs Exp $
##

AUTOMAKE_OPTIONS	= foreign
CLEANFILES              = *~ comperr
MAINTAINERCLEANFILES    = configure aclocal.m4 Makefile.in config.h.in \
			  stamp-h.in comperr

INCLUDES = -I../capi20 $(all_includes)
CFLAGS = -Wall -O2
LDFLAGS = -L../capi20/.libs -L../capi20 $(all_libraries)
LDADD = -lcapi20

bin_PROGRAMS = capiinfo

capiinfo_SOURCES = capiinfo.c

man_MANS = capiinfo.8
EXTRA_DIST = $(man_MANS)

