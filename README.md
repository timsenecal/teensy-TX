# teensy-TX

this is an arduino IDE based source code for a Teensy 4.1 based transmitter for r/c vehicles.  

based on work done for a stm32 "blue-pill", which had too much interference on the analog inputs...
i switched to the teensy to try to resolve that... switching to the 4.1 has some advantages, more input pins, 
and a much higher performance cpu, more ram, more flash, as well as a built in microsd card reader.

it isn't currently finished, only about 50% of the features work.

current plans: it is designed to drive a 128x64 pixel monochrome LCD panel, with backlight, have 4 analog inputs, 2 2 position switches,
and 2 3 position switches, as well as trim features for four analog inputs, and has serial input for telemetry coming 
back from the receiver...  it currently only handles PPM output to a single RF module, presumably 2.4 ghz.
It also supports writing and reading model information from the microsd card, it will feature generic mixing of channels, as well as a
few preset mixes.
