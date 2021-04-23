19 constant trigpin
18 constant echopin
: init trigpin output pinmode echopin input pinmode ; init
: o. oledcls oledhome olednum oleddisplay ;

: trig low trigpin pin 2 ms high trigpin pin 10 ms low trigpin pin ;
: echo echopin pulsein ;
: scan trig echo o. ;
: tst 100 0 do scan loop ;

