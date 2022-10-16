/PROG  TMP
/ATTR
DEFAULT_GROUP	= 1,*,*,*,*;
/MN
   1:  UFRAME_NUM=1 ;
   2:  UTOOL_NUM=2 ;
   3:  R[81:Log Data Switch]=1    ;
   4:  RUN DATARECORDER ;
   5:J P[1] 100% FINE    ;
   6:  R[81:Log Data Switch]=0    ;
/POS
P[1]{
   GP1:
	UF : 1, UT : 2,	
	J1=     0.000 deg,	J2=    20.000 deg,	J3=   -30.000 deg,
	J4=     0.000 deg,	J5=   -20.000 deg,	J6=    10.000 deg
};
/END
