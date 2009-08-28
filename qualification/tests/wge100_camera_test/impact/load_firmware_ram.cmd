setmode -bscan
setcable -p auto
addDevice -p 1 -part xc3s500e
assignfile -p 1 -file "default.bit"
program -p 1 
quit
