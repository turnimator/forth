variable buf 256 cells allot
	
: rdlx
	." Enter rdlx" .s cr
	40 0 DO
		dup execute dup emit
		dup 10 = if
			drop
			i
			." QUIT rdlx " .s cr
			QUIT
		then
		buf i cells + !
	LOOP
	0 DO
		buf i cells + @ emit
	LOOP
	." Falling off end of loop rdlx " .s cr
	-1
	;
	
: rdl ['] key rdlx ;


