PREFIX=/usr
DEBUG?=n
DMALLOC?=n
CROSSCOMPILE?=n
ZEROCONF?=avahi

ifeq ($(CROSSCOMPILE),y)
CC=arm-linux-gcc
NM=arm-linux-nm
AR=arm-linux-ar
CXX=arm-linux-g++
CPP=arm-linux-cpp
LD=arm-linux-ld
endif

ifeq ($(DMALLOC),y)
DEBUGFLAGS=-DDMALLOC -DDMALLOC_FUNC_CHECK
EXTRALIBS=-ldmallocth
DEBUG=y
endif

ifeq ($(DEBUG),y)
DEBUGFLAGS := $(DEBUGFLAGS) -g -O0 -D_DEBUG -DDEBUG
else
DEBUGFLAGS=-O2
endif

ifeq ($(ZEROCONF),avahi)
ZEROCONFFILE=zeroconf_avahi.o
else
ZEROCONFFILE=zeroconf.o
endif

LIBS := -lphidget21 -lpthread -lm -ldl $(EXTRALIBS)
CFLAGS := $(CFLAGS) -Wall -Wno-strict-aliasing $(DEBUGFLAGS) -fpic -D_LINUX -I. -I../phidget21 -I../phidget21/utils

webservice21objs = phidgetinterface.o \
	eventhandlers.o \
	PhidgetWebservice21.o \
	stdafx.o \
	../phidget21/pdict.o \
	../phidget21/pdictserver.o \
	../phidget21/cphidgetconstants.o \
        ../phidget21/cphidgetlist.o \
        ../phidget21/md5.o \
	../phidget21/utils/plist.o \
	../phidget21/utils/ptree.o \
	../phidget21/utils/utils.o \
	$(ZEROCONFFILE)

all: webservice21

$(webservice21objs): %.o: %.c phidget21internal.h
	$(CC) -c $(CFLAGS) $< -o $@

webservice21: $(webservice21objs)
	$(CC) -o phidgetwebservice21 $(webservice21objs) $(LIBS)

phidget21internal.h:
	echo "#ifndef PHIDGET_H" > $@.tmp
	echo "#define PHIDGET_H" >> $@.tmp
	echo "#ifdef __cplusplus" >> $@.tmp
	echo "extern \"C\" {" >> $@.tmp
	echo "#endif" >> $@.tmp
	$(CC) -g -E -P -D_LINUX ../phidget21/phidget21int.h | sed -e "s/extern \"C\" //g" >> $@.tmp
	cat ../phidget21/cphidgetconstantsinternal.h >> $@.tmp
	cat ../phidget21/cphidgetconstants.h >> $@.tmp
	echo "#ifdef __cplusplus" >> $@.tmp
	echo "}" >> $@.tmp
	echo "#endif" >> $@.tmp
	echo "#endif" >> $@.tmp
	mv $@.tmp $@

clean: 
	rm *.o phidgetwebservice21 phidget21internal.h

