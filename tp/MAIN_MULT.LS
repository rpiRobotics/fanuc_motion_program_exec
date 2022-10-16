/PROG  MAIN_MULT
/ATTR
DEFAULT_GROUP	= *,*,*,*,*;
/MN
   1:  R[80]=0 ;
   2:  LBL[1] ;
   3:  WAIT R[80]>0    ;
   4:  R[81]=1 ;
   5:  R[82]=1 ;
   6:  R[83]=1 ;
   7:  RUN DATARECORDER ;
   8:  RUN TMPA    ;
   9:  CALL TMPB    ;
   10:  WAIT R[82]<=0    ;
   11:  WAIT R[83]<=0    ;
   12:  R[81]=0 ;
   13:  R[80]=0 ;
   14:  JMP LBL[1] ;
/POS
/END
