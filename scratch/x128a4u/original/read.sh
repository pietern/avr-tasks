#!/bin/sh

for memory in eeprom application apptable boot flash usersig; do
    avrdude -c atmelice_pdi -p x128a4u -U ${memory}:r:memory.${memory}:i
done
