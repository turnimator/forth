variable buf 256 cells allot

: skipto$ 
	BEGIN
		serial1.read
		36 = IF
			.s cr
			LEAVE
		THEN
	AGAIN
	." Fell through looking for $ " cr .s cr
	;
	
: rdl 
	skipto$
	255 0 DO
		serial1.read
		dup 10 = IF
			LEAVE
		THEN
		buf i + C!
	LOOP
	;
	

9600 serial1.begin


: .buf
	buf C@ 0 DO
		buf i + C@ emit
	LOOP
	;
	
: gpsrd
  BEGIN
	rdl
	.buf
	cr
	." SPACE to end, Enter to continue ... " cr
	key 32 =
  UNTIL 
 ;
 
