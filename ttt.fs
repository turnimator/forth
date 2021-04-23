variable board 9 cells allot
: bclear
	board 9 cells 32 fill
    ;
     
: bzeros
    board 9 cells 48 fill
    ;
        
: .bcell ( u -- )
	32 emit board swap cells + @ emit 32 emit ;

: .bline ( u -- )
	dup dup dup 3 * .bcell ." |" 3 * 1 + .bcell ." |" 3 * 2 + .bcell drop ;

: .board ( -- )
	cr
	0 .bline cr 
	." ---+---+---" cr
	1 .bline cr
	." ---+---+---" cr
	2 .bline cr
	cr
 ;

: bcell ( u -- addr board cell # u)
	cells board + 
	;

: x! dup bcell 88 swap ! .board drop ;

: 0! dup bcell 48 swap ! .board drop ;
  	
: TIaR ( u -- b see if there are two in a row of u )
	8 0 do
		dup
		i bcell @ 
		= if			  \ is it the one we're looking for?
			i 1 + bcell @ \ Is the next one the same?
			= if
				i 3 / i 1 + 3 / = if \ ... and on the same line?
					i quit
				then
			then
		then
	loop
	-1
	;

: blookup ( u1 u2 -- i index of u1 starting at index u2)
	9 swap do 
		dup i bcell @ = 
		if 
			drop i quit 
		else 
		then 
	loop
	drop
	-1 
	;

bclear
.board

4 x!
88 0 blookup 1 + 0!
3 x!
7 0!



