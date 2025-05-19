#!/bin/bash
cd /home/ILMModule/ILMModule
arduino-cli compile --fqbn arduino:avr:mega /home/ILMModule/ILMModule/ILMMCodes/integrated_movement/integrated_movement.ino 2>&1 | grep error
echo "Compilation check complete"
