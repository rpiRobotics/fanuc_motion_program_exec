/PROG  MAIN_CONNECT_1
/ATTR
/MN
   1:  DO[102]=OFF ;
   2:  DO[100]=OFF ;
   3:  LBL[1] ;
   4:  WAIT DO[102]=ON    ;
   5:  DO[100]=ON  ;
   6:  CALL TMP    ;
   7:  DO[100]=OFF ;
   8:  DO[102]=OFF ;
   9:  JMP LBL[1] ;
/POS
/END
