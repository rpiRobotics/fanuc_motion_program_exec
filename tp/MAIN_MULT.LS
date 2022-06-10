/PROG  MAIN_MULT
/ATTR
DEFAULT_GROUP	= *,*,*,*,*;
/MN
   1:  DO[102]=OFF ;
   2:  LBL[1] ;
   3:  WAIT DO[102]=ON    ;
   4:  DO[101]=ON ;
   5:  DO[103]=ON ;
   6:  DO[104]=ON ;
   7:  RUN DATARECORDER ;
   8:  RUN TMPA    ;
   9:  CALL TMPB    ;
   10:  WAIT DO[103]=OFF    ;
   11:  WAIT DO[104]=OFF    ;
   12:  DO[101]=OFF ;
   13:  DO[102]=OFF ;
   14:  JMP LBL[1] ;
/POS
/END
