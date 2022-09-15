/PROG  MAIN_CONNECT_2
/ATTR
/MN
   1:  DO[102]=OFF ;
   2:  LBL[1] ;
   3:  WAIT DO[102]=ON    ;
   4:  CALL TMP    ;
   5:  WAIT DO[102]=OFF    ;
   6:  JMP LBL[1] ;
/POS
/END
