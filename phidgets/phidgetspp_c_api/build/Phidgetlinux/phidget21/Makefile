PREFIX=/usr
DEBUG?=n
DMALLOC?=n
JAVA?=y
CROSSCOMPILE?=n
ZEROCONF?=avahi
ZEROCONF_LOOKUP?=n

ifeq ($(CROSSCOMPILE),y)
CC=arm-linux-gcc
NM=arm-linux-nm
AR=arm-linux-ar
CXX=arm-linux-g++
CPP=arm-linux-cpp
LD=arm-linux-ld
endif

ifeq ($(JAVA),y)
JAVAHOME := /usr/java/j2sdk1.4.2_18
JAVAFLAGS=-I$(JAVAHOME)/include -I$(JAVAHOME)/include/linux
else
JAVAFLAGS=
endif

ifeq ($(DMALLOC),y)
DEBUGFLAGS=-DDMALLOC -DDMALLOC_FUNC_CHECK
EXTRALIBS=-ldmalloc
DEBUG=y
endif

ifeq ($(DEBUG),y)
DEBUGFLAGS := $(DEBUGFLAGS) -g -O0 -D_DEBUG -DDEBUG
else
DEBUGFLAGS=-Os
endif

ifeq ($(ZEROCONF),avahi)
ZEROCONFFILE=zeroconf_avahi.c
else
ZEROCONFFILE=zeroconf.c
endif

ifeq ($(ZEROCONF_LOOKUP),y)
ZEROCONFFILE := $(ZEROCONFFILE) zeroconf_lookup.c
CFLAGS := $(CFLAGS) -DZEROCONF_LOOKUP
endif

INSTALL = install
INSTALL_FILE = $(INSTALL) -m 644
INSTALL_LIB  = $(INSTALL) -m 644

LIBS := -lm -lpthread -ldl $(EXTRALIBS)
CFLAGS := $(CFLAGS) -Wall -Wno-strict-aliasing $(DEBUGFLAGS) -fpic -D_LINUX $(JAVAFLAGS) -I. -Iutils

libphidget21cs := cphidget.c \
	cphidgetaccelerometer.c \
	cphidgetadvancedservo.c \
	cphidgetencoder.c \
	cphidgetgps.c \
	cphidgetgyroscope.c \
	cphidgetinterfacekit.c \
	cphidgetled.c \
	cphidgetmotorcontrol.c \
	cphidgetphsensor.c \
	cphidgetrfid.c \
	cphidgetservo.c \
	cphidgetstepper.c \
	cphidgettemperaturesensor.c \
	cphidgettextlcd.c \
	cphidgettextled.c \
	cphidgetweightsensor.c \
	cphidgetmanager.c \
	cphidgetdictionary.c \
	cphidgetgeneric.c \
	cphidgetsbc.c \
	cphidgetspatial.c \
	cphidgetir.c \
	clog.c \
	md5.c \
	csocketevents.c \
	csocketopen.c \
	cphidgetconstants.c \
	cphidgetlist.c \
	cthread.c \
	cusblinux.c \
	phidget21.c \
	pdict.c \
	pdictclient.c \
	utils/utils.c \
	utils/ptree.c \
	utils/plist.c \
	$(ZEROCONFFILE)

libusb := libusb/usb.c \
	libusb/descriptors.c \
	libusb/linux.c \
	libusb/error.c

PHIDGET21_JNI_C_SRC := Java/com_phidgets_Phidget.c \
	Java/com_phidgets_AccelerometerPhidget.c \
	Java/com_phidgets_AdvancedServoPhidget.c \
	Java/com_phidgets_EncoderPhidget.c \
	Java/com_phidgets_InterfaceKitPhidget.c \
	Java/com_phidgets_LEDPhidget.c \
	Java/com_phidgets_MotorControlPhidget.c \
	Java/com_phidgets_PHSensorPhidget.c \
	Java/com_phidgets_RFIDPhidget.c \
	Java/com_phidgets_ServoPhidget.c \
	Java/com_phidgets_StepperPhidget.c \
	Java/com_phidgets_TemperatureSensorPhidget.c \
	Java/com_phidgets_TextLCDPhidget.c \
	Java/com_phidgets_TextLEDPhidget.c \
	Java/com_phidgets_Manager.c \
	Java/com_phidgets_Dictionary.c \
	Java/com_phidgets_DictionaryKeyListener.c \
	Java/com_phidgets_WeightSensorPhidget.c \
	Java/phidget_jni.c

PHIDGET21_JAVA_SRC = \
	com/phidgets/*.java \
	com/phidgets/event/*.java

libphidget21objs := $(libphidget21cs:.c=.o)

libusbobjs := $(libusb:.c=.o)

jnistuffobjs := $(PHIDGET21_JNI_C_SRC:.c=.o)

all: libphidget21.so phidget21.h

libphidget21.a: $(libusbobjs) $(libphidget21objs) 
	$(AR) r libphidget21.a $(libusbobjs) $(libphidget21objs) 

libphidget21.so: $(libusbobjs) $(libphidget21objs)
	$(CC) -g -shared -o libphidget21.so $(libusbobjs) $(libphidget21objs) $(LIBS)

jni: $(libusbobjs) $(libphidget21objs) $(jnistuffobjs) phidget21.h
	$(CC) -g -shared -o libphidget21.so $(libusbobjs) $(libphidget21objs) $(jnistuffobjs) $(LIBS)

clean: 
	-rm *.a *~ *.so *.o libusb/*.o Java/*.o phidget21.h utils/*.o

phidget21.h: 
	echo "#ifndef PHIDGET_H" > $@
	echo "#define PHIDGET_H" >> $@
	echo "#ifdef __cplusplus" >> $@
	echo "extern \"C\" {" >> $@
	echo "#endif" >> $@
	#$(CC) -g -E -P -C -D_LINUX -DEXTERNALPROTO phidget21int.h | sed -e "s/extern \"C\" //g" >> $@
	$(CC) -g -E -P -D_LINUX -DEXTERNALPROTO -DREMOVE_DEPRECATED phidget21int.h | sed -e "s/extern \"C\" //g" >> $@
	cat cphidgetconstants.h >> $@
	echo "#ifdef __cplusplus" >> $@
	echo "}" >> $@
	echo "#endif" >> $@
	echo "#endif" >> $@
	
phidget21matlab.h:
	echo "/*" > $@
	echo " * Phidget21 Header for Matlab" >> $@
	echo " * -All pointers to structs have been changed to long, as Matlab seems" >> $@
	echo " *  to be unhappy with either the undefined stucts or the double pointers." >> $@
	echo " */" >> $@
	$(CC) -g -E -P -D_LINUX -DEXTERNALPROTO -DREMOVE_DEPRECATED phidget21int.h | sed -e "s/extern \"C\" //g" | sed -e "s/struct [a-zA-Z_]* \*/long /" >> $@
	cat cphidgetconstants.h >> $@

phidget21.jar:
	cd Java ; javac $(PHIDGET21_JAVA_SRC)
	cd Java ; jar cf ../$@ com/phidgets/event/*.class com/phidgets/*.class

install: all
	$(INSTALL_LIB) libphidget21.so $(INSTALLPREFIX)/$(PREFIX)/lib
	$(INSTALL_FILE) phidget21.h $(INSTALLPREFIX)/$(PREFIX)/include
	ldconfig

uninstall:
	-rm $(INSTALLPREFIX)/$(PREFIX)/lib/libphidget21.*
	-rm $(INSTALLPREFIX)/$(PREFIX)/include/phidget21.h
