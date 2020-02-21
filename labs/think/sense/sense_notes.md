# Sense Notes

## IR Sensors
Analog pins 1 to 5 are used for IR sensors going from furthest Port to furthest
Starboard.

Advertised range is 10 to 80 cm. Sensors are usually long by 2cm using the 
SharpIR library. Documentation gives the conversion as Distance = 29.988 X POW(Volt , -1.173)
This can be rewritten, filtered, and/or scaled depending on the 
accurarcy we wish we to achieve. It should be noted that it is steady.

## Mapping
Because the IR sensors are not arranged around the center of rotation of the 
boat, an offset is applied when placing them on our map. For now the map assumes
we are only rotating in place.


