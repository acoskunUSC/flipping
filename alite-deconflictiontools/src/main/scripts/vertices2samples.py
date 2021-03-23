#!/usr/bin/env python

vertices = "1532,759 1784,759 2351,759 713,822 776,822 839,822 902,822 1595,822 1721,822 1784,822 1847,822 2036,822 2099,822 2162,822 2351,822 713,885 839,885 902,885 1595,885 1721,885 1784,885 1847,885 902,948 1532,948 2036,948 2351,948 2666,948 3170,948 3359,948 713,1011 1091,1011 1154,1011 1217,1011 1406,1011 1469,1011 1532,1011 2351,1011 2477,1011 2540,1011 2792,1011 2855,1011 713,1074 1091,1074 1154,1074 1217,1074 1280,1074 1343,1074 1406,1074 1469,1074 1532,1074 2351,1074 2477,1074 2540,1074 2792,1074 2855,1074 587,1137 650,1137 713,1137 1028,1137 1091,1137 1154,1137 1217,1137 1406,1137 1469,1137 1532,1137 2351,1137 2666,1137 2792,1137 2855,1137 1154,1200 1217,1200 1406,1200 1469,1200 1532,1200 2855,1200 1028,1263 1091,1263 1154,1263 1217,1263 1343,1263 1469,1263 1532,1263 1595,1263 1721,1263 1847,1263 2855,1263 3044,1263 3170,1263 3233,1263 3296,1263 3359,1263 3422,1263 3485,1263 1721,1326 1784,1326 1847,1326 1910,1326 1973,1326 2036,1326 2099,1326 2162,1326 2225,1326 2288,1326 2351,1326 2414,1326 2477,1326 2540,1326 2603,1326 2666,1326 2729,1326 2792,1326 2855,1326 2918,1326 2981,1326 3044,1326 3107,1326 3170,1326 3233,1326 3296,1326 3359,1326 3422,1326 3485,1326 1721,1389 1784,1389 1847,1389 1910,1389 1973,1389 2036,1389 2099,1389 2162,1389 2351,1389 2414,1389 2477,1389 2540,1389 2603,1389 2666,1389 2729,1389 2792,1389 2855,1389 3359,1389 3422,1389 3485,1389 1721,1452 1784,1452 1847,1452 1910,1452 1973,1452 2036,1452 2099,1452 2162,1452 2351,1452 2540,1452 2603,1452 2666,1452 2729,1452 2792,1452 2855,1452 3359,1452 3422,1452 3485,1452 1721,1515 1910,1515 1973,1515 2036,1515 2099,1515 2162,1515 2225,1515 2351,1515 2477,1515 2540,1515 2729,1515 2792,1515 2855,1515 3422,1515 2164,1489 2156,1474 2158,1458 2168,1445 2181,1436 2195,1431 2210,1433 2237,1503 2206,1511 2191,1510 2178,1503 2441,1389 2454,1388 2467,1393 2476,1403 2480,1417 2480,1464 2474,1480 2450,1515 2410,1522 2400,1511 2398,1496 2405,1420 2411,1405 2424,1395 1375,1297 1375,1280 1384,1265 1400,1259 1432,1256 1445,1258 1456,1265 1463,1277 1468,1296 1025,1181 1038,1177 1082,1173 1096,1175 1108,1184 1113,1197 1116,1218 1114,1234 1104,1246 1089,1251 1050,1253 1033,1249 1022,1236 1216,1240 1214,1227 1217,1214 1226,1204 1238,1199 1255,1196 1257,1189 1265,1178 1276,1172 1288,1171 1342,1178 1356,1183 1365,1195 1367,1209 1364,1239 1358,1254 1345,1264 1329,1265 1307,1260 1260,1274 1247,1275 1234,1270 1225,1259 1263,1109 1267,1094 1279,1084 1294,1082 1316,1084 1328,1088 1338,1097 1342,1109 1346,1141 1344,1157 1332,1169 1316,1173 1288,1172 1274,1168 1264,1157 1261,1142 2543,1078 2486,1095 2499,1088 2365,1059 2472,1070 2484,1082 2487,1097 2383,1145 2377,1139 2357,1101 2354,1087 2357,1074 1019,1003 1032,1001 1045,1005 1062,1015 1073,1025 1077,1040 1077,1087 1073,1101 1062,1111 1048,1115 1034,1112 1023,1102 2364,1049 2362,1035 2366,1022 2376,1012 2405,996 2420,993 2434,996 2474,1019 2484,1029 2488,1042 2487,1055 2479,1066 1246,1038 1244,1025 1247,1012 1256,1002 1269,998 1323,991 1338,993 1350,1003 1362,1020 1367,1033 1365,1048 1357,1059 1344,1065 1287,1075 1274,1074 1263,1069 1255,1058 2547,1022 2549,1006 2558,994 2573,989 2629,984 2643,986 2654,994 2657,1001 2659,999 2672,994 2721,988 2735,989 2746,997 2752,1009 2763,1049 2763,1064 2755,1078 2749,1083 2755,1107 2754,1121 2747,1134 2735,1141 2721,1142 2701,1138 2688,1132 2680,1121 2678,1108 2679,1090 2657,1089 2652,1101 2642,1112 2634,1115 2628,1128 2555,1077 1892,999 1899,993 1912,990 2002,990 2092,989 2126,989 2141,992 2152,1003 2153,1007 2159,998 2173,992 2262,978 2278,976 2293,977 2305,986 2327,1013 2333,1025 2333,1038 2318,1103 2320,1150 2334,1143 2352,1140 2381,1145 2624,1162 2714,1161 2804,1160 2815,1160 2829,1163 2840,1172 2848,1184 2852,1199 2849,1213 2833,1245 2817,1260 2786,1271 2777,1272 2687,1275 2597,1277 2507,1280 2417,1283 2341,1285 2337,1295 2327,1304 2310,1314 2299,1318 2287,1317 2271,1312 2256,1302 2251,1285 2250,1278 2169,1286 2167,1286 2159,1293 2145,1296 2131,1293 2121,1282 2116,1275 2034,1266 1945,1276 1933,1277 1923,1277 1877,1267 1866,1262 1858,1253 1858,1252 1805,1252 1794,1262 1782,1269 1768,1269 1708,1254 1631,1273 1615,1272 1601,1262 1596,1247 1589,1157 1583,1067 1580,1031 1582,1016 1592,1004 1607,1000 1697,995 1771,991 1783,992 1811,1003 1845,990 1865,990 3163,901 1879,735 1866,817 1855,808 1843,791 1838,776 1841,760 1853,749 433,980 437,988 439,997 529,993 619,990 644,989 653,990 701,1003 715,1011 722,1025 721,1041 711,1054 680,1077 662,1082 572,1080 556,1080 556,1093 569,1098 581,1107 587,1121 586,1136 577,1148 562,1160 564,1174 654,1174 744,1174 755,1174 757,1084 757,1078 748,1076 735,1070 727,1059 725,1045 727,1025 731,1011 742,1002 755,999 845,997 935,996 994,995 1009,998 1023,1113 1024,1293 1024,1294 1040,1304 1130,1302 1220,1301 1298,1299 1304,1299 1554,1302 1590,1303 1640,1291 1653,1291 1666,1298 1673,1309 1696,1366 1703,1367 1714,1379 1717,1395 1712,1457 1711,1460 1721,1467 1731,1480 1733,1496 1726,1510 1704,1534 1769,1533 1771,1498 1776,1481 1790,1471 1807,1471 1871,1489 1882,1494 1889,1503 1905,1531 1943,1531 1960,1519 1973,1514 1988,1516 1999,1525 2005,1532 2095,1531 2167,1531 2169,1526 2183,1514 2201,1512 2243,1522 2219,1402 2220,1388 2230,1361 2240,1348 2256,1343 2305,1340 2318,1342 2329,1350 2335,1363 2346,1417 2342,1437 2327,1463 2337,1483 2340,1504 2332,1533 2344,1532 2347,1527 2358,1516 2372,1512 2387,1515 2403,1525 2406,1523 2472,1533 2562,1532 2587,1532 2587,1531 2594,1501 2601,1488 2612,1480 2626,1479 2657,1483 2670,1487 2679,1497 2682,1510 2683,1532 2773,1532 2863,1531 2894,1531 2893,1441 2893,1368 2896,1354 2905,1343 2919,1339 3009,1330 3043,1327 3045,1327 3135,1327 3225,1326 3286,1326 3301,1329 3311,1340 3315,1354 3317,1436 3316,1446 3305,1475 3314,1496 3315,1510 3313,1515 3317,1512 3342,1499 3355,1496 3368,1498 3378,1507 3395,1528 3440,1528 3444,1522 3452,1511 3465,1506 3478,1507 3494,1512 3506,1493 3517,1484 3530,1481 3544,1484 3545,1484 3543,1394 3541,1304 3540,1245 3450,1246 3360,1246 3270,1247 3180,1248 3169,1248 3104,1269 3088,1269 3074,1261 3066,1252 2976,1250 2973,1250 2973,1252 3005,1266 3017,1276 3022,1291 3019,1306 3008,1318 2992,1321 2927,1318 2909,1311 2886,1290 2877,1274 2867,1225 2867,1213 2888,1125 2889,1119 2886,1029 2885,1011 2888,995 2899,984 2914,981 3004,984 3094,987 3184,990 3274,993 3364,995 3454,998 3536,1001 3535,987 3445,983 3409,982 3397,980 3388,973 3377,961 3370,946 3371,940 3363,939 3347,931 3339,916 3335,897 3302,962 3290,974 3273,977 3210,970 3193,962 3185,946 3160,904 3151,917 3137,923 3120,926 3119,942 3115,956 3104,966 3090,969 3000,967 2910,965 2820,964 2730,962 2688,961 2672,956 2662,944 2662,943 2660,948 2649,958 2636,961 2546,962 2456,964 2391,965 2377,962 2366,952 2362,938 2361,920 2364,904 2376,893 2391,890 2407,891 2407,801 2407,761 2396,760 2383,757 2373,748 2369,735 2368,730 2340,730 2344,735 2348,751 2344,767 2331,778 2326,780 2324,816 2349,848 2354,866 2353,920 2350,933 2341,944 2304,969 2286,973 2196,968 2106,964 2069,962 2054,957 2044,944 2042,928 2049,914 2073,888 2082,881 2093,879 2095,878 2099,867 2109,857 2123,854 2168,853 2182,856 2192,865 2209,889 2221,889 2219,861 2211,858 2202,847 2200,832 2205,818 2216,808 2217,808 2214,807 2202,798 2195,785 2196,770 2197,769 2195,769 2185,762 2152,790 2137,796 2120,793 2108,781 2099,764 2096,764 2082,761 2072,751 2068,738 2067,732 2037,733 2041,740 2045,753 2043,767 2034,778 2028,782 2023,814 2021,821 2027,824 2040,842 2048,871 2048,889 2029,938 2018,952 1997,966 1980,970 1890,969 1800,969 1735,968 1722,965 1711,956 1707,943 1709,929 1715,914 1725,900 1742,896 1832,896 1873,896 1875,887 1883,872 1899,864 1917,861 1917,849 1910,839 1903,838 1882,826 1815,735 1795,735 1792,746 1782,756 1764,766 1760,767 1759,775 1747,788 1727,800 1713,804 1699,801 1688,791 1681,780 1678,781 1670,783 1669,841 1679,842 1693,847 1703,859 1705,874 1696,940 1692,953 1682,962 1670,965 1580,972 1579,972 1562,968 1550,955 1543,939 1541,926 1544,913 1553,904 1565,899 1594,895 1595,805 1595,770 1593,769 1578,764 1556,749 1546,738 1545,732 1518,732 1529,763 1530,777 1526,786 1535,791 1542,808 1545,838 1543,852 1543,853 1549,856 1557,872 1555,889 1523,956 1512,968 1497,972 1407,972 1317,973 1227,973 1137,974 1047,974 957,975 947,975 933,972 923,963 918,950 909,883 910,870 917,859 929,852 937,849 937,766 921,764 911,760 892,805 882,817 867,822 790,825 773,821 762,808 758,799 705,801 707,878 701,897 698,901 749,901 755,889 772,881 789,879 807,882 819,896 823,904 832,906 847,913 856,928 855,945 850,958 839,971 823,976 733,977 643,978 553,979 463,980"

for vertice in vertices.split(" "):
    (x,y) = vertice.split(",")
    print(x + " " + y)      




