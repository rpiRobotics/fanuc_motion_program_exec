/PROG  TEST_RECORD
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 654;
CREATE		= DATE 22-03-09  TIME 17:28:08;
MODIFIED	= DATE 22-03-09  TIME 17:33:10;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 17;
MEMORY_SIZE	= 966;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  DO[101]=ON ;
   2:  RUN DATARECORDER ;
   3:   ;
   4:J P[1] 100% FINE    ;
   5:J P[2] 100% FINE    ;
   6:   ;
   7:  DO[101]=OFF ;
   8:   ;
   9:   ;
  10:   ;
  11:   ;
  12:   ;
  13:   ;
  14:   ;
  15:   ;
  16:   ;
  17:   ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,	
	J1=     -.000 deg,	J2=     -.000 deg,	J3=    40.000 deg,
	J4=     0.000 deg,	J5=     0.000 deg,	J6=      .000 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,	
	J1=     -.000 deg,	J2=     -.000 deg,	J3=     -.000 deg,
	J4=     0.000 deg,	J5=     0.000 deg,	J6=      .000 deg
};
/END
