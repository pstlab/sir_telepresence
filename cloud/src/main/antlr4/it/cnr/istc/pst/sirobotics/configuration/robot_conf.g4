grammar robot_conf;

configuration: starting? ending?;

starting: 'starting' ':' predicate (',' predicate)* ';';

ending: 'ending' ':' predicate (',' predicate)* ';';

predicate: ID ';';

ID: ('a' ..'z' | 'A' ..'Z' | '_') (
		'a' ..'z'
		| 'A' ..'Z'
		| '0' ..'9'
		| '_'
	)*;

LINE_COMMENT: '//' .*? '\r'? '\n' -> skip;

COMMENT: '/*' .*? '*/' -> skip;

WS: [ \r\t\u000C\n]+ -> skip;