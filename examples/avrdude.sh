#!/usr/bin/env bash

set -e

if [ -z "$1" ]
then
  echo "Usage: $0 DIR"
  exit 1
fi

pushd $1
make
popd

hex=$(echo $1/*.hex)

avrdude -q -V -D \
  -p atmega328p \
  -c arduino \
  -b 115200 \
  -P /dev/ttyUSB0 \
  -U flash:w:${hex}:i
