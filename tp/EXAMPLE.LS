/PROG  EXAMPLE
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 832;
CREATE		= DATE 22-03-08  TIME 18:16:28;
MODIFIED	= DATE 22-03-08  TIME 18:36:02;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 14;
MEMORY_SIZE	= 1148;
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
   1:  UFRAME_NUM=0 ;
   2:  UTOOL_NUM=1 ;
   3:  PAYLOAD[1:spray] ;
   4:  OVERRIDE=30% ;
   5:   ;
   6:J P[1] 100% CNT100    ;
   7:J P[2] 100% FINE    ;
   8:L P[3] 500mm/sec CNT100    ;
   9:C P[4]    
    :  P[5] 100mm/sec CNT100    ;
  10:L P[6] 100mm/sec FINE    ;
  11:   ;
  12:   ;
  13:   ;
  14:   ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  1349.372  mm,	Y =     0.000  mm,	Z =   964.278  mm,
	W =   180.000 deg,	P =   -86.371 deg,	R =     0.000 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   981.051  mm,	Y =     0.000  mm,	Z =      .094  mm,
	W =   180.000 deg,	P =   -35.568 deg,	R =     0.000 deg
};
P[3]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  1013.182  mm,	Y =   469.943  mm,	Z =   244.454  mm,
	W =  -180.000 deg,	P =   -14.270 deg,	R =    24.883 deg
};
P[4]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   747.137  mm,	Y =   625.501  mm,	Z =   443.159  mm,
	W =   180.000 deg,	P =   -14.789 deg,	R =    39.936 deg
};
P[5]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N D B, 0, 0, 0',
	X =   897.200  mm,	Y =   352.000  mm,	Z =   672.300  mm,
	W =  -180.000 deg,	P =   -14.800 deg,	R =    39.900 deg
};
P[6]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   970.814  mm,	Y =   413.682  mm,	Z =   921.370  mm,
	W =  -180.000 deg,	P =   -40.349 deg,	R =    39.936 deg
};
/END
