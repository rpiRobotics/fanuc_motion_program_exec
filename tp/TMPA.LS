/PROG  TMPA
/ATTR
DEFAULT_GROUP	= 1,*,*,*,*;
/MN
   1:  UFRAME_NUM=1 ;
   2:  UTOOL_NUM=1 ;
   3:J P[1] 100% FINE ;
   4:L P[2] 50mm/sec CNT100 ;
   5:L P[3] 50mm/sec FINE ;
   6:  DO[103]=OFF ;
/POS
P[1]{
   GP1:
   UF : 1, UT : 1,
   J1 = 38.300 deg,  J2 = 23.300 deg,  J3 = -34.000 deg,
   J4 = 45.700 deg,  J5 = -101.900 deg,  J6 = -48.300 deg
};
P[2]{
   GP1:
   UF : 1, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 994.000 mm,  Y = 924.900 mm,  Z = 1739.500 mm,
   W = 163.100 deg,  P = 1.500 deg,  R = -1.000 deg
};
P[3]{
   GP1:
   UF : 1, UT : 1,     CONFIG : 'N U T, 0, 0, 0',
   X = 1620.000 mm,  Y = 0.000 mm,  Z = 1930.000 mm,
   W = 180.000 deg,  P = 0.000 deg,  R = 0.000 deg
};
/END
