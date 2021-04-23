: not  0 = if -1 else 0 then ;
: pi 355 113 * / ;
led output pinmode
: ledon led high digitalwrite ;
: ledoff led low digitalwrite ;
: blinkx dup dup ledon ms ledoff ms drop ;
: xblink 0 do 500 blinkx loop ;
: xblinkx 0 do dup blinkx loop drop ;
400 10 xblinkx
: GI5 BEGIN DUP 2 > WHILE i .
      DUP 5 < WHILE DUP 1+ REPEAT 
      123 ELSE 345 THEN ; 
