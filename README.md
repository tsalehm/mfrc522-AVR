# mfrc522-AVR
mfrc522 library port for CodeVisionAVR\
descriptios are in the library

## AVR settings

### PORTS :
\
miso : input, pull up\
mosi : output, 0V\
sck : output, 0V\
sda : set in the library (cs_clr,cs_set)

### SPI :
\
spi mode : mode 0\
clock phase : cycle start\
clock polarity : low\
spi type : master\
data order : MSB first\
clock rate : 500 kHz
