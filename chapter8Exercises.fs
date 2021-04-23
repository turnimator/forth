create pies 0 ,

: eat-pie pies @ 0 = if ." What pie?" else 1 - ! ." Thank you" then;
: bake-pie pies @ 1 + ! ;
