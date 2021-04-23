
21 constant pin_trig
22 constant pin_echo
pin_trig output pinmode
pin_echo input pinmode

: echo 4096 0 do
     PIN_ECHO digitalread
     HIGH = if
     	i quit
     then
     loop
     -1
     ;
     

: trig low PIN_TRIG pin 20 ms high PIN_TRIG pin 40 ms low PIN_TRIG pin ;
: echo PIN_ECHO HIGH 1000 pulsein ;
: scan trig 2 ms echo . cr ;

