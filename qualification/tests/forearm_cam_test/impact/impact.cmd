setmode -bscan
setcable -p auto
addDevice -p 1 -part xc3s500e
attachflash -position 1 -spi "AT45DB081D"
assignfiletoattachedflash -position 1 -file "default.mcs"
program -p 1 -spionly -e -v -loadfpga
quit
