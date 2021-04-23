create pies 0 ,

: eat-pie pies @ 0 > if 1 - pies ! ." Thank you" cr else ." What pie?" cr then ;
: bake-pie pies @ 1 + pies ! ;

create frozen-pies 0 ,

: freeze-pies pies @ frozen-pies ! 0 pies ! ;

: .base base @ dup \ one of print one for restore
	." Setting to DECIMAL for print" cr
	decimal . base !
	;
	
variable board 9 cells allot
: clear
	board 9 cells 32 fill
    ;
     
: zeros
    board 9 cells 48 fill
    ;
        
: .bcell 32 emit board swap cells + @ emit 32 emit ;

: .bline dup dup dup 3 * .bcell ." |" 3 * 1 + .bcell ." |" 3 * 2 + .bcell drop ;

: .board 
	0 .bline cr 
	." ---+---+---" cr
	1 .bline cr
	." ---+---+---" cr
	2 .bline cr
 ;

: bcell cells board + ;

: x! dup bcell 88 swap ! .board drop ;

: 0! dup bcell 48 swap ! .board drop ;


clear
.board



