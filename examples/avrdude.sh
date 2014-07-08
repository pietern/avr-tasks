#!/usr/bin/env bash

set -e

if [ -z "$1" ]
then
  echo "Usage: $0 example.hex"
  exit 1
fi

make $1

avrdude -q -V -D \
  -p atmega328p \
  -c arduino \
  -b 115200 \
  -P /dev/ttyUSB0 \
  -U flash:w:$1:i
