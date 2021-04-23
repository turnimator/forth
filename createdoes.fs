
: pindef ( pinmode pin# ) over over create , , pinmode
	does> dup 1 cells + 
	@ output = IF
	 ." digitalwrite" @ swap digitalwrite
	ELSE
	 ." digitalread" @ digitalread
	THEN
	;
	
