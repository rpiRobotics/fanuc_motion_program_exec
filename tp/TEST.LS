/PROG  TEST
/ATTR
/MN
   1:  UFRAME_NUM=0 ;
   2:  UTOOL_NUM=1 ;
   3:  DO[101]=ON ;
   4:  RUN DATARECORDER ;
   5:L P[1] 50mm/sec CNT100 ;
   6:L P[2] 50mm/sec FINE ;
   7:  DO[101]=OFF ;
/POS
P[1]{
   GP1:
   UF : 0, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 1050.000 mm,  Y = 100.000 mm,  Z = 350.000 mm,
   W = -180.000 deg,  P = 0.000 deg,  R = 0.000 deg
};
P[2]{
   GP1:
   UF : 0, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 1050.000 mm,  Y = 200.000 mm,  Z = 350.000 mm,
   W = -180.000 deg,  P = 0.000 deg,  R = 0.000 deg
};
/END
