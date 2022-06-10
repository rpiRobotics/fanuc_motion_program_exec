/PROG  TMPB
/ATTR
DEFAULT_GROUP	= *,1,*,*,*;
/MN
   1:  UFRAME_NUM=1 ;
   2:  UTOOL_NUM=1 ;
   3:J P[1] 100% FINE ;
   4:L P[2] 50mm/sec CNT100 ;
   5:L P[3] 50mm/sec FINE ;
   6:  DO[104]=OFF ;
/POS
P[1]{
   GP2:
   UF : 1, UT : 1,
   J1 = -49.700 deg,  J2 = 4.300 deg,  J3 = -35.200 deg,
   J4 = -20.900 deg,  J5 = -35.800 deg,  J6 = 52.100 deg
};
P[2]{
   GP2:
   UF : 1, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 1383.100 mm,  Y = -484.000 mm,  Z = 940.600 mm,
   W = 171.500 deg,  P = -26.800 deg,  R = -9.800 deg
};
P[3]{
   GP2:
   UF : 1, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 1166.000 mm,  Y = 0.000 mm,  Z = 1430.000 mm,
   W = 180.000 deg,  P = 0.000 deg,  R = 0.000 deg
};
/END
