#
# Copyright (c) 2004,  Theodore A. Roth
# Copyright (c) 2005,2006,2007,2009  Anatoly Sokolov
# Copyright (c) 2005,2008  Joerg Wunsch
# Copyright (c) 2012  Shawn Wilson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of the copyright holders nor the names of
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Process this file with autoconf to produce a configure script.
#

#
# This is the autoconf file for building a .hex file which avrdude can 
# upload to an arduino.  It builds the hex file straight from a normal 
# arduino pde file.
#
# In order to build the hex file, you will have to specify the include
# and lib directory of the arduino libraries that match your AVR CPU.  The 
# directories get passed in the CFLAGS and LDFLAGS variables from the
# configure command line.
#


AC_INIT(grbl, 0.7)
AC_PREREQ(2.59)

# Set the default --host parameter to 'avr'.
if test "$host" == "" ; then
   host=${host:-'avr'}
   host_alias=${host_alias:-'avr'}
   ac_tool_prefix=${ac_tool_prefix:-'avr-'}
   cross_compiling=yes
fi

#
# See if the user specified '--build=`../config.guess` --host=avr' when
# running ../configure.
#
AC_MSG_CHECKING([if configuring for cross compile])
if test "x${build_alias}" = "x${host_alias}"; then
   if test "x${build_alias}" = "x" ; then
      build_alias='`./config.guess`'
   fi
   AC_MSG_RESULT(no)
   AC_MSG_WARN()
   AC_MSG_WARN(libarduino must be built using an avr cross-compiler.)
   AC_MSG_WARN(Try configuring with:)
   AC_MSG_WARN("$0 --build=$build_alias --host=avr")
   AC_MSG_WARN()
   AC_MSG_ERROR(aborting configure)
else
   AC_MSG_RESULT([yes])
fi

AC_MSG_CHECKING([if target host is avr])
case "$host" in
   avr* )
      AC_MSG_RESULT([yes])
      ;;
   * )
      AC_MSG_RESULT(no)
      AC_MSG_WARN()
      AC_MSG_WARN(libarduino must be built using an avr cross-compiler.)
      AC_MSG_WARN(Try configuring with:)
      AC_MSG_WARN("$0 --build=$build --host=avr")
      AC_MSG_WARN()
      AC_MSG_ERROR(aborting configure)
      ;;
esac

dnl The default check whether the C compiler can create an executable
dnl is inappropriate for us as it requires an existing library.
AC_NO_EXECUTABLES


# Get the host and build variables.
AC_CANONICAL_HOST
AC_CANONICAL_BUILD

# Unless the user specified some CXXFLAGS or CFLAGS, set the flags 
# to blank.
AC_SUBST(CXXFLAGS, [${CXXFLAGS:-''}])
AC_SUBST(CFLAGS, [${CFLAGS:-''}])

# Minimum version of automake is 1.8.
AM_INIT_AUTOMAKE([1.8])


#
# This is where all the program variables get set.  Most of them get 
# prefixed with 'avr-' during this process.
#
AC_PROG_CC
AC_PROG_CXX
AC_CHECK_TOOL(OBJCOPY, objcopy, objcopy)
AC_CHECK_TOOL(OBJDUMP, objdump, objdump)

# Make sure that we found the right avr cross-compiler.
case "${CXX}" in
   *avr-g++*) ;;
   *) AC_MSG_ERROR(Wrong C compiler found; You need avr-g++ installed!) ;;
esac
case "${OBJCOPY}" in
   *avr-objcopy*) ;;
   *) AC_MSG_ERROR(Wrong objcopy found; You need avr-objcopy installed!) ;;
esac
case "${OBJDUMP}" in
   *avr-objdump*) ;;
   *) AC_MSG_ERROR(Wrong objdump found; You need avr-objcopy installed!) ;;
esac

AC_PROG_INSTALL


########## USER PARAMETERS ##########

AC_ARG_VAR(EXTRALIBS, "A list of extra libraries to include when linking.")
AC_SUBST(EXTRALIBS, [${EXTRALIBS:-''}])


#
# Allow the user to specify the AVR CPU type an speed in megahertz.
# By default, we set the AVR CPU type to be an Arduino Uno.
#
AC_ARG_VAR(MMCU, "Default processor type.")
AC_SUBST(MMCU, [${MMCU:-'atmega328p'}])

AC_ARG_VAR(MHZ, "Default processor speed in MHz.")
AC_SUBST(MHZ, [${MHZ:-'16'}])

#
# These parameters get passed to avrdude on 'make install'
#
AC_ARG_VAR(AVRDUDE_PARTNO, "Default avrdude part number id")
AC_SUBST(AVRDUDE_PARTNO, [${AVRDUDE_PARTNO:-'m328p'}])

AC_ARG_VAR(AVRDUDE_PROGRAMMER, 
           "Default avrdude programmer id")
AC_SUBST(AVRDUDE_PROGRAMMER, [${AVRDUDE_PROGRAMMER:-'arduino'}])

AC_ARG_VAR(AVRDUDE_SERIAL_PORT, "Default avrdude serial port parameter")
AC_SUBST(AVRDUDE_SERIAL_PORT, [${AVRDUDE_SERIAL_PORT:-'/dev/ttyACM0'}])


AC_CONFIG_FILES([Makefile])
AC_OUTPUT
