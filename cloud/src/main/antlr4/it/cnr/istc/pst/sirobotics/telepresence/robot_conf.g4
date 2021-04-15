grammar robot_conf;

predicates: predicate*;

predicate: ID '(' (argument (',' argument)*)? ')';

argument: type ID;

type: primitive_type | class_type;

primitive_type: BOOL | INT | REAL | STRING;

class_type: ID ('.' ID)*;

BOOL: 'bool';

INT: 'int';

REAL: 'real';

STRING: 'string';

ID: ('a' ..'z' | 'A' ..'Z' | '_') (
		'a' ..'z'
		| 'A' ..'Z'
		| '0' ..'9'
		| '_'
	)*;