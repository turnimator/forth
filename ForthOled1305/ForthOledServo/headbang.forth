: headbangx 1 150 servo> 2 0 do 0 1 servo> 500 ms 0 300 servo> 500 ms loop ;
: headbangy 0 150 servo> 4 0 do 1 1 servo> 300 ms 1 300 servo> 300 ms loop ;
: headbang 4 0 do headbangy headbangx loop ;
headbang

