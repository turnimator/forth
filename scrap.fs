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


: ccountLine ( c l - u return count 0-3 of how many charaters c are on line l )
	3 * ." Starting at " dup . cr \ l is 0 1 or 2, which * 3 gives us first cell
	." stack=" .s cr \ tos is cell no
	3 0 do 	\ for each cell on line
		." Stack before bcell=" .s
		i  dup . ." =" bcell @ . cr
		." Stack after bcell=" .s
	loop
	cr
	;

	
: cl ccountLine . ;


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

: forall 9 0 do 
		dup execute
	loop 
	;
 

4 x!
88 0 blookup 1 + 0!
3 x!
7 0!


