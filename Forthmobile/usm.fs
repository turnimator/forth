 19 constant PIN_ECHO
18 constant PIN_TRIG

PIN_TRIG output pinmode 
PIN_ECHO input pinmode 

: trig low PIN_TRIG pin 2 ms high PIN_TRIG pin 20 ms low PIN_TRIG pin ;
: echo PIN_ECHO HIGH 1 pulsein ;
: scan trig echo . cr ;

