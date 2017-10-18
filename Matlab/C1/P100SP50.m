figure(1);
hold on;
xlabel('Time (ms)');
ylabel('Theta (Deg)');
title('PID Controller | P100SP50');
plot([0 1500], [50 50], 'r--');
Kp10i0d0 = [12.0, 0; 22.0, 0; 32.0, 1; 42.0, 2; 52.0, 3; 62.0, 6; 72.0, 8; 82.0, 11; 92.0, 13; 102.0, 17; 112.0, 19; 122.0, 23; 132.0, 25; 142.0, 28; 152.0, 30; 162.0, 33; 172.0, 35; 182.0, 37; 192.0, 39; 202.0, 41; 212.0, 42; 222.0, 44; 232.0, 45; 242.0, 46; 252.0, 47; 262.0, 48; 272.0, 48; 282.0, 49; 292.0, 49; 302.0, 49; 312.0, 49; 322.0, 49; 332.0, 49; 342.0, 49; 352.0, 49; 362.0, 49; 372.0, 49; 382.0, 49; 392.0, 49; 402.0, 49; 412.0, 49; 422.0, 49; 432.0, 49; 442.0, 49; 452.0, 49; 462.0, 49; 472.0, 49; 482.0, 49; 492.0, 49; 502.0, 49; 513.0, 49; 523.0, 49; 533.0, 49; 543.0, 49; 553.0, 49; 563.0, 49; 573.0, 49; 583.0, 49; 593.0, 49; 603.0, 49; 613.0, 49; 623.0, 49; 633.0, 49; 643.0, 49; 653.0, 49; 663.0, 49; 673.0, 49; 683.0, 49; 693.0, 49; 703.0, 49; 713.0, 49; 723.0, 49; 733.0, 49; 743.0, 49; 753.0, 49; 763.0, 49; 773.0, 49; 783.0, 49; 793.0, 49; 803.0, 49; 813.0, 49; 823.0, 49; 833.0, 49; 843.0, 49; 853.0, 49; 863.0, 49; 873.0, 49; 883.0, 49; 893.0, 49; 903.0, 49; 913.0, 49; 923.0, 49; 933.0, 49; 943.0, 49; 953.0, 49; 963.0, 49; 973.0, 49; 983.0, 49; 993.0, 49; 1003.0, 49; 1013.0, 49; 1023.0, 49; 1033.0, 49; 1043.0, 49; 1053.0, 49; 1063.0, 49; 1073.0, 49; 1083.0, 49; 1093.0, 49; 1103.0, 49; 1113.0, 49; 1123.0, 49; 1133.0, 49; 1143.0, 49; 1153.0, 49; 1163.0, 49; 1173.0, 49; 1183.0, 49; 1193.0, 49; 1203.0, 49; 1213.0, 49; 1223.0, 49; 1233.0, 49; 1243.0, 49; 1253.0, 49; 1263.0, 49; 1273.0, 49; 1283.0, 49; 1293.0, 49; 1303.0, 49; 1313.0, 49; 1323.0, 49; 1333.0, 49; 1343.0, 49; 1353.0, 49; 1363.0, 49; 1373.0, 49; 1383.0, 49; 1393.0, 49; 1403.0, 49; 1413.0, 49; 1423.0, 49; 1433.0, 49; 1443.0, 49; 1453.0, 49; 1463.0, 49; 1473.0, 49; 1497.0, 49; 1507.0, 49; ];
plot(Kp10i0d0(:,1),Kp10i0d0(:,2))
% y = max(Kp10i0d0(:,2));
% x = Kp10i0d0(find(Kp10i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp10i0d0');
% pause;

Kp20i0d0 = [29.0, 0; 39.0, 1; 58.0, 4; 69.0, 8; 79.0, 10; 89.0, 16; 99.0, 20; 109.0, 26; 119.0, 30; 129.0, 36; 139.0, 39; 149.0, 44; 159.0, 47; 169.0, 52; 179.0, 54; 189.0, 57; 199.0, 58; 209.0, 60; 219.0, 61; 229.0, 62; 239.0, 62; 249.0, 61; 259.0, 61; 269.0, 60; 279.0, 59; 289.0, 58; 299.0, 57; 309.0, 55; 319.0, 54; 329.0, 53; 339.0, 52; 349.0, 51; 359.0, 51; 369.0, 50; 379.0, 50; ];
plot(Kp20i0d0(:,1),Kp20i0d0(:,2))
% y = max(Kp20i0d0(:,2));
% x = Kp20i0d0(find(Kp20i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp20i0d0');
% pause;

Kp30i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 40.0, 4; 50.0, 7; 62.0, 12; 72.0, 18; 82.0, 23; 92.0, 30; 102.0, 35; 112.0, 42; 122.0, 47; 132.0, 53; 142.0, 56; 152.0, 61; 162.0, 63; 172.0, 66; 182.0, 67; 192.0, 67; 202.0, 67; 212.0, 66; 222.0, 65; 232.0, 62; 242.0, 60; 252.0, 57; 262.0, 56; 272.0, 53; 282.0, 51; 292.0, 49; 302.0, 48; 312.0, 47; 322.0, 46; 332.0, 45; 342.0, 45; 352.0, 45; 362.0, 46; 372.0, 46; 382.0, 47; 392.0, 47; 402.0, 48; 412.0, 49; 422.0, 49; 432.0, 50; 442.0, 50; ];
plot(Kp30i0d0(:,1),Kp30i0d0(:,2))
% y = max(Kp30i0d0(:,2));
% x = Kp30i0d0(find(Kp30i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp30i0d0');
% pause;

Kp40i0d0 = [14.0, 0; 24.0, 0; 34.0, 1; 44.0, 4; 54.0, 6; 64.0, 11; 74.0, 15; 84.0, 21; 94.0, 26; 104.0, 34; 114.0, 39; 124.0, 47; 134.0, 52; 144.0, 58; 154.0, 62; 164.0, 66; 174.0, 68; 184.0, 69; 194.0, 69; 204.0, 68; 214.0, 67; 224.0, 64; 234.0, 61; 244.0, 57; 254.0, 54; 264.0, 51; 274.0, 49; 284.0, 46; 294.0, 44; 304.0, 43; 314.0, 42; 324.0, 42; 334.0, 42; 344.0, 43; 354.0, 44; 364.0, 45; 374.0, 46; 384.0, 48; 394.0, 49; 404.0, 50; 414.0, 51; 424.0, 52; 434.0, 53; 444.0, 53; 454.0, 53; 464.0, 53; 474.0, 52; 484.0, 52; 494.0, 52; 504.0, 51; 514.0, 50; 524.0, 50; ];
plot(Kp40i0d0(:,1),Kp40i0d0(:,2))
% y = max(Kp40i0d0(:,2));
% x = Kp40i0d0(find(Kp40i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp40i0d0');
% pause;

Kp50i0d0 = [12.0, 0; 22.0, 0; 32.0, 1; 42.0, 3; 52.0, 6; 62.0, 11; 72.0, 15; 82.0, 21; 92.0, 26; 102.0, 34; 112.0, 39; 122.0, 47; 132.0, 52; 142.0, 59; 152.0, 63; 162.0, 67; 172.0, 69; 182.0, 70; 192.0, 70; 202.0, 69; 212.0, 67; 222.0, 64; 232.0, 61; 242.0, 56; 252.0, 53; 262.0, 48; 272.0, 46; 282.0, 43; 292.0, 41; 302.0, 40; 312.0, 40; 322.0, 41; 332.0, 42; 342.0, 44; 352.0, 46; 362.0, 48; 372.0, 50; 382.0, 51; 392.0, 52; 402.0, 54; 412.0, 54; 422.0, 54; 432.0, 54; 442.0, 53; 452.0, 53; 462.0, 52; 472.0, 51; 482.0, 50; 492.0, 50; ];
plot(Kp50i0d0(:,1),Kp50i0d0(:,2))
% y = max(Kp50i0d0(:,2));
% x = Kp50i0d0(find(Kp50i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp50i0d0');
% pause;

Kp60i0d0 = [10.0, 0; 23.0, 0; 33.0, 2; 43.0, 5; 53.0, 8; 63.0, 13; 73.0, 17; 83.0, 24; 93.0, 29; 103.0, 37; 113.0, 42; 123.0, 50; 133.0, 55; 143.0, 62; 153.0, 65; 163.0, 69; 173.0, 70; 183.0, 71; 193.0, 70; 203.0, 69; 213.0, 67; 223.0, 63; 233.0, 60; 243.0, 54; 253.0, 50; 263.0, 46; 273.0, 43; 283.0, 40; 293.0, 38; 303.0, 38; 313.0, 38; 323.0, 40; 333.0, 42; 343.0, 44; 353.0, 47; 363.0, 50; 373.0, 52; 383.0, 54; 393.0, 56; 403.0, 57; 413.0, 57; 423.0, 56; 433.0, 56; 443.0, 54; 453.0, 53; 463.0, 51; 473.0, 50; 483.0, 48; 493.0, 47; 503.0, 46; 513.0, 46; 523.0, 47; 533.0, 47; 543.0, 48; 553.0, 48; 563.0, 50; 573.0, 50; ];
plot(Kp60i0d0(:,1),Kp60i0d0(:,2))
% y = max(Kp60i0d0(:,2));
% x = Kp60i0d0(find(Kp60i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp60i0d0');
% pause;

Kp70i0d0 = [12.0, 0; 22.0, 0; 32.0, 2; 42.0, 4; 52.0, 7; 62.0, 12; 72.0, 16; 82.0, 22; 92.0, 27; 102.0, 35; 112.0, 40; 122.0, 48; 132.0, 54; 142.0, 61; 152.0, 64; 162.0, 69; 172.0, 71; 182.0, 72; 192.0, 72; 202.0, 70; 212.0, 68; 222.0, 64; 232.0, 61; 242.0, 56; 252.0, 52; 262.0, 46; 272.0, 43; 282.0, 40; 292.0, 38; 302.0, 37; 312.0, 38; 322.0, 39; 332.0, 41; 342.0, 45; 352.0, 47; 362.0, 51; 372.0, 53; 382.0, 56; 392.0, 57; 402.0, 58; 412.0, 57; 422.0, 56; 432.0, 55; 442.0, 53; 452.0, 52; 462.0, 50; 472.0, 48; 482.0, 47; 492.0, 46; 503.0, 46; 513.0, 46; 523.0, 46; 533.0, 47; 543.0, 48; 553.0, 49; 563.0, 50; 573.0, 51; 583.0, 52; 593.0, 53; 603.0, 53; 613.0, 52; 623.0, 52; 633.0, 52; 643.0, 51; 653.0, 50; 663.0, 49; 673.0, 49; 683.0, 49; 693.0, 48; 703.0, 48; 713.0, 49; 723.0, 49; 733.0, 49; 743.0, 50; 753.0, 50; ];
plot(Kp70i0d0(:,1),Kp70i0d0(:,2))
% y = max(Kp70i0d0(:,2));
% x = Kp70i0d0(find(Kp70i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp70i0d0');
% pause;

Kp80i0d0 = [11.0, 0; 21.0, 0; 31.0, 1; 41.0, 3; 51.0, 7; 61.0, 11; 71.0, 16; 81.0, 21; 91.0, 28; 101.0, 33; 111.0, 41; 121.0, 47; 131.0, 55; 141.0, 60; 151.0, 65; 161.0, 69; 171.0, 71; 181.0, 72; 191.0, 73; 201.0, 72; 211.0, 70; 221.0, 68; 231.0, 63; 241.0, 60; 252.0, 54; 262.0, 49; 272.0, 44; 282.0, 41; 292.0, 37; 302.0, 36; 312.0, 35; 322.0, 35; 332.0, 37; 342.0, 39; 352.0, 43; 362.0, 45; 372.0, 50; 382.0, 53; 392.0, 57; 402.0, 59; 412.0, 61; 422.0, 61; 432.0, 59; 442.0, 58; 452.0, 55; 462.0, 53; 472.0, 49; 482.0, 47; 492.0, 44; 502.0, 43; 512.0, 42; 522.0, 43; 532.0, 44; 542.0, 45; 552.0, 47; 562.0, 49; 572.0, 52; 582.0, 53; 592.0, 55; 602.0, 55; 612.0, 55; 622.0, 55; 632.0, 54; 642.0, 53; 652.0, 51; 662.0, 49; 672.0, 48; 682.0, 47; 692.0, 46; 702.0, 46; 712.0, 47; 722.0, 47; 732.0, 48; 742.0, 49; 752.0, 50; 762.0, 51; 772.0, 52; 782.0, 52; 792.0, 52; 802.0, 52; 812.0, 51; 822.0, 51; 832.0, 50; 842.0, 50; ];
plot(Kp80i0d0(:,1),Kp80i0d0(:,2))
% y = max(Kp80i0d0(:,2));
% x = Kp80i0d0(find(Kp80i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp80i0d0');
% pause;

Kp90i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 40.0, 3; 50.0, 6; 60.0, 11; 70.0, 14; 80.0, 21; 90.0, 26; 100.0, 34; 110.0, 39; 120.0, 48; 130.0, 54; 140.0, 61; 150.0, 65; 160.0, 70; 170.0, 72; 180.0, 75; 190.0, 75; 200.0, 74; 210.0, 73; 220.0, 70; 230.0, 68; 240.0, 63; 250.0, 59; 260.0, 52; 270.0, 48; 280.0, 42; 290.0, 39; 300.0, 35; 310.0, 34; 320.0, 33; 330.0, 33; 340.0, 35; 350.0, 37; 360.0, 41; 370.0, 45; 380.0, 50; 390.0, 54; 400.0, 58; 410.0, 60; 420.0, 62; 430.0, 62; 440.0, 62; 450.0, 61; 460.0, 58; 470.0, 55; 480.0, 51; 490.0, 48; 500.0, 44; 510.0, 42; 520.0, 41; 530.0, 40; 540.0, 41; 550.0, 43; 560.0, 46; 570.0, 48; 580.0, 52; 590.0, 54; 600.0, 56; 610.0, 57; 620.0, 58; 630.0, 57; 640.0, 55; 650.0, 54; 660.0, 51; 670.0, 49; 680.0, 47; 690.0, 46; 700.0, 44; 710.0, 44; 720.0, 45; 730.0, 46; 740.0, 48; 750.0, 50; 760.0, 52; 770.0, 53; 780.0, 54; 790.0, 54; 800.0, 54; 810.0, 53; 820.0, 52; 830.0, 51; 840.0, 49; 850.0, 48; 860.0, 47; 870.0, 47; 880.0, 47; 890.0, 47; 900.0, 48; 910.0, 49; 920.0, 50; 930.0, 51; 940.0, 52; 950.0, 52; 960.0, 52; 970.0, 52; 980.0, 52; 990.0, 51; 1000.0, 50; 1010.0, 50; ];
plot(Kp90i0d0(:,1),Kp90i0d0(:,2))
% y = max(Kp90i0d0(:,2));
% x = Kp90i0d0(find(Kp90i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp90i0d0');
% pause;

Kp100i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 40.0, 4; 50.0, 6; 60.0, 11; 70.0, 15; 80.0, 22; 90.0, 27; 100.0, 35; 110.0, 40; 120.0, 49; 130.0, 55; 140.0, 62; 150.0, 66; 160.0, 71; 170.0, 73; 180.0, 75; 190.0, 76; 200.0, 75; 210.0, 74; 220.0, 71; 230.0, 68; 240.0, 63; 250.0, 59; 260.0, 52; 270.0, 48; 280.0, 42; 290.0, 38; 300.0, 35; 310.0, 33; 320.0, 32; 330.0, 33; 340.0, 34; 350.0, 36; 360.0, 40; 370.0, 44; 380.0, 50; 390.0, 54; 400.0, 58; 410.0, 61; 420.0, 63; 430.0, 64; 440.0, 63; 450.0, 62; 460.0, 59; 470.0, 56; 480.0, 51; 490.0, 48; 502.0, 44; 512.0, 41; 522.0, 40; 532.0, 39; 542.0, 39; 552.0, 41; 562.0, 43; 572.0, 47; 582.0, 50; 592.0, 54; 602.0, 56; 612.0, 58; 622.0, 59; 632.0, 59; 642.0, 58; 652.0, 55; 662.0, 52; 672.0, 49; 682.0, 46; 692.0, 44; 702.0, 43; 712.0, 42; 722.0, 43; 732.0, 45; 742.0, 47; 752.0, 50; 762.0, 52; 772.0, 54; 782.0, 56; 792.0, 56; 802.0, 56; 812.0, 54; 822.0, 53; 832.0, 50; 842.0, 49; 852.0, 47; 862.0, 46; 872.0, 45; 882.0, 45; 892.0, 46; 902.0, 47; 912.0, 49; 922.0, 51; 932.0, 52; 942.0, 53; 952.0, 54; 962.0, 54; 972.0, 53; 982.0, 52; 992.0, 51; 1002.0, 50; 1012.0, 49; 1022.0, 48; 1032.0, 47; 1042.0, 47; 1052.0, 48; 1062.0, 48; 1072.0, 49; 1082.0, 50; 1092.0, 51; 1102.0, 52; 1112.0, 52; 1122.0, 52; 1132.0, 52; 1142.0, 51; 1152.0, 51; 1162.0, 50; 1172.0, 49; 1182.0, 49; 1192.0, 48; 1202.0, 48; 1212.0, 49; 1222.0, 50; 1232.0, 50; ];
plot(Kp100i0d0(:,1),Kp100i0d0(:,2))
% y = max(Kp100i0d0(:,2));
% x = Kp100i0d0(find(Kp100i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp100i0d0');
% pause;

Kp110i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 40.0, 3; 50.0, 6; 60.0, 11; 70.0, 14; 80.0, 21; 90.0, 26; 100.0, 34; 110.0, 39; 120.0, 48; 130.0, 54; 140.0, 61; 150.0, 66; 160.0, 70; 170.0, 73; 180.0, 75; 190.0, 75; 200.0, 75; 210.0, 74; 220.0, 70; 230.0, 68; 240.0, 62; 250.0, 58; 260.0, 52; 270.0, 47; 280.0, 41; 290.0, 38; 300.0, 34; 310.0, 33; 320.0, 32; 330.0, 32; 340.0, 34; 350.0, 36; 360.0, 40; 370.0, 43; 380.0, 49; 390.0, 53; 400.0, 58; 410.0, 61; 420.0, 63; 430.0, 64; 440.0, 64; 450.0, 63; 460.0, 60; 470.0, 58; 480.0, 53; 490.0, 49; 502.0, 44; 512.0, 41; 522.0, 39; 532.0, 38; 542.0, 38; 552.0, 40; 562.0, 41; 572.0, 45; 582.0, 48; 592.0, 53; 602.0, 56; 612.0, 59; 629.0, 60; 639.0, 60; 649.0, 59; 659.0, 56; 669.0, 53; 679.0, 49; 689.0, 46; 699.0, 43; 709.0, 42; 719.0, 42; 729.0, 42; 739.0, 44; 749.0, 46; 759.0, 50; 769.0, 52; 779.0, 55; 789.0, 56; 799.0, 57; 809.0, 56; 819.0, 55; 829.0, 53; 839.0, 50; 849.0, 48; 859.0, 46; 869.0, 45; 879.0, 45; 889.0, 45; 899.0, 46; 909.0, 47; 919.0, 50; 929.0, 51; 939.0, 53; 949.0, 54; 959.0, 54; 969.0, 54; 979.0, 53; 989.0, 52; 999.0, 50; 1009.0, 49; 1019.0, 47; 1031.0, 46; 1041.0, 46; 1051.0, 47; 1061.0, 48; 1071.0, 50; 1081.0, 51; 1091.0, 52; 1101.0, 53; 1111.0, 53; 1121.0, 53; 1131.0, 52; 1141.0, 51; 1151.0, 50; 1161.0, 49; 1171.0, 48; 1181.0, 48; 1191.0, 48; 1201.0, 48; 1211.0, 49; 1221.0, 49; 1231.0, 50; 1241.0, 51; 1251.0, 52; 1261.0, 52; 1271.0, 52; 1281.0, 51; 1291.0, 50; 1301.0, 50; ];
plot(Kp110i0d0(:,1),Kp110i0d0(:,2))
% y = max(Kp110i0d0(:,2));
% x = Kp110i0d0(find(Kp110i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp110i0d0');
% pause;

Kp120i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 42.0, 4; 52.0, 8; 62.0, 12; 72.0, 18; 82.0, 22; 92.0, 30; 102.0, 35; 112.0, 44; 122.0, 49; 132.0, 58; 142.0, 62; 152.0, 68; 162.0, 71; 172.0, 74; 182.0, 75; 192.0, 74; 202.0, 74; 212.0, 71; 222.0, 69; 232.0, 64; 242.0, 60; 252.0, 54; 262.0, 50; 272.0, 43; 282.0, 40; 292.0, 36; 302.0, 34; 312.0, 33; 322.0, 33; 332.0, 35; 342.0, 36; 352.0, 40; 362.0, 44; 372.0, 49; 382.0, 53; 392.0, 58; 402.0, 60; 412.0, 62; 422.0, 63; 432.0, 62; 442.0, 61; 452.0, 58; 462.0, 56; 472.0, 51; 482.0, 48; 492.0, 43; 502.0, 41; 512.0, 40; 522.0, 39; 532.0, 40; 542.0, 41; 552.0, 44; 562.0, 47; 572.0, 52; 582.0, 54; 592.0, 57; 602.0, 58; 612.0, 59; 622.0, 58; 632.0, 56; 642.0, 54; 652.0, 50; 662.0, 48; 672.0, 44; 682.0, 43; 692.0, 42; 702.0, 43; 712.0, 44; 722.0, 46; 732.0, 50; 742.0, 52; 752.0, 55; 762.0, 56; 772.0, 57; 782.0, 56; 792.0, 54; 802.0, 52; 812.0, 50; 822.0, 48; 832.0, 46; 842.0, 45; 852.0, 44; 862.0, 45; 872.0, 47; 882.0, 49; 892.0, 51; 902.0, 53; 912.0, 54; 922.0, 55; 932.0, 54; 942.0, 54; 952.0, 52; 962.0, 50; 972.0, 48; 982.0, 47; 992.0, 46; 1002.0, 46; 1012.0, 47; 1022.0, 48; 1032.0, 50; 1042.0, 51; 1052.0, 52; 1062.0, 53; 1072.0, 53; 1082.0, 53; 1092.0, 52; 1102.0, 51; 1112.0, 50; 1122.0, 49; 1132.0, 48; 1142.0, 48; 1152.0, 48; 1162.0, 48; 1172.0, 49; 1182.0, 49; 1192.0, 50; 1202.0, 51; 1212.0, 52; 1222.0, 52; 1232.0, 52; 1242.0, 51; 1252.0, 51; 1262.0, 50; 1272.0, 49; 1282.0, 49; 1292.0, 49; 1302.0, 49; 1312.0, 49; 1322.0, 49; 1332.0, 50; 1342.0, 50; ];
plot(Kp120i0d0(:,1),Kp120i0d0(:,2))
% y = max(Kp120i0d0(:,2));
% x = Kp120i0d0(find(Kp120i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp120i0d0');
% pause;

Kp130i0d0 = [11.0, 0; 21.0, 0; 31.0, 1; 41.0, 3; 51.0, 7; 61.0, 10; 71.0, 16; 81.0, 20; 91.0, 28; 101.0, 32; 111.0, 41; 121.0, 46; 131.0, 55; 141.0, 60; 151.0, 66; 161.0, 70; 171.0, 73; 188.0, 75; 198.0, 75; 208.0, 74; 218.0, 72; 228.0, 68; 238.0, 65; 248.0, 60; 258.0, 55; 268.0, 48; 278.0, 44; 288.0, 39; 298.0, 36; 308.0, 33; 318.0, 33; 328.0, 33; 338.0, 34; 348.0, 36; 358.0, 39; 368.0, 44; 378.0, 48; 388.0, 54; 398.0, 57; 408.0, 61; 418.0, 62; 428.0, 64; 438.0, 63; 448.0, 62; 458.0, 60; 468.0, 56; 478.0, 52; 488.0, 47; 498.0, 44; 508.0, 41; 518.0, 40; 528.0, 39; 538.0, 39; 548.0, 42; 558.0, 44; 568.0, 48; 578.0, 51; 588.0, 55; 598.0, 57; 608.0, 59; 618.0, 60; 628.0, 58; 638.0, 57; 648.0, 54; 658.0, 51; 668.0, 47; 678.0, 45; 688.0, 42; 698.0, 42; 708.0, 42; 718.0, 43; 728.0, 46; 738.0, 48; 748.0, 52; 758.0, 54; 768.0, 57; 778.0, 57; 788.0, 57; 798.0, 57; 808.0, 54; 818.0, 52; 828.0, 48; 838.0, 46; 848.0, 44; 858.0, 44; 868.0, 44; 878.0, 44; 888.0, 46; 898.0, 49; 908.0, 52; 918.0, 54; 928.0, 56; 938.0, 56; 948.0, 56; 958.0, 55; 968.0, 52; 978.0, 50; 988.0, 48; 998.0, 46; 1008.0, 45; 1018.0, 44; 1028.0, 45; 1038.0, 46; 1048.0, 49; 1058.0, 51; 1068.0, 53; 1078.0, 55; 1088.0, 56; 1098.0, 55; 1108.0, 54; 1118.0, 52; 1128.0, 50; 1138.0, 48; 1148.0, 46; 1158.0, 45; 1168.0, 45; 1178.0, 45; 1188.0, 47; 1198.0, 48; 1208.0, 51; 1218.0, 53; 1228.0, 54; 1238.0, 55; 1248.0, 55; 1258.0, 54; 1268.0, 52; 1278.0, 50; 1288.0, 48; 1298.0, 47; 1308.0, 46; 1318.0, 46; 1328.0, 46; 1338.0, 47; 1348.0, 49; 1358.0, 50; 1368.0, 52; 1378.0, 54; 1388.0, 54; 1398.0, 54; 1408.0, 53; 1418.0, 52; 1428.0, 50; 1438.0, 48; 1448.0, 47; 1458.0, 46; 1468.0, 46; 1478.0, 46; 1488.0, 48; 1498.0, 49; 1508.0, 51; ];
plot(Kp130i0d0(:,1),Kp130i0d0(:,2))
% y = max(Kp130i0d0(:,2));
% x = Kp130i0d0(find(Kp130i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp130i0d0');
% pause;

Kp140i0d0 = [10.0, 0; 20.0, 0; 30.0, 1; 40.0, 4; 50.0, 6; 60.0, 11; 70.0, 14; 80.0, 21; 90.0, 25; 100.0, 33; 110.0, 38; 120.0, 47; 130.0, 52; 140.0, 60; 150.0, 65; 160.0, 69; 170.0, 72; 180.0, 74; 190.0, 75; 200.0, 74; 210.0, 73; 220.0, 70; 230.0, 67; 240.0, 62; 250.0, 58; 260.0, 51; 270.0, 47; 280.0, 41; 290.0, 38; 300.0, 34; 310.0, 33; 320.0, 32; 330.0, 33; 340.0, 35; 350.0, 37; 360.0, 41; 370.0, 44; 380.0, 50; 390.0, 54; 400.0, 59; 410.0, 62; 420.0, 64; 430.0, 65; 440.0, 65; 450.0, 63; 460.0, 61; 470.0, 58; 480.0, 53; 490.0, 49; 501.0, 44; 511.0, 39; 521.0, 38; 531.0, 36; 541.0, 36; 551.0, 37; 561.0, 39; 571.0, 42; 581.0, 45; 591.0, 51; 601.0, 54; 611.0, 58; 621.0, 60; 631.0, 61; 641.0, 61; 651.0, 60; 661.0, 58; 677.0, 53; 687.0, 48; 697.0, 45; 707.0, 42; 717.0, 40; 727.0, 39; 737.0, 40; 747.0, 42; 757.0, 44; 767.0, 48; 777.0, 51; 787.0, 55; 797.0, 57; 807.0, 59; 817.0, 59; 827.0, 58; 837.0, 57; 847.0, 53; 857.0, 51; 867.0, 47; 877.0, 45; 887.0, 42; 897.0, 42; 907.0, 43; 917.0, 44; 927.0, 47; 937.0, 49; 947.0, 53; 957.0, 55; 967.0, 57; 977.0, 57; 987.0, 56; 997.0, 55; 1007.0, 52; 1017.0, 50; 1027.0, 47; 1037.0, 45; 1047.0, 44; 1057.0, 44; 1067.0, 45; 1077.0, 46; 1087.0, 49; 1097.0, 51; 1107.0, 54; 1117.0, 55; 1127.0, 56; 1137.0, 55; 1147.0, 54; 1157.0, 52; 1167.0, 49; 1177.0, 48; 1187.0, 46; 1197.0, 45; 1207.0, 45; 1217.0, 46; 1227.0, 48; 1237.0, 49; 1247.0, 52; 1257.0, 53; 1267.0, 55; 1277.0, 55; 1287.0, 54; 1297.0, 52; 1307.0, 50; 1317.0, 48; 1327.0, 47; 1337.0, 46; 1347.0, 46; 1357.0, 46; 1367.0, 48; 1377.0, 50; 1387.0, 52; 1397.0, 53; 1407.0, 54; 1417.0, 54; 1427.0, 53; 1437.0, 52; 1447.0, 50; 1457.0, 49; 1467.0, 47; 1477.0, 46; 1487.0, 46; 1497.0, 47; 1507.0, 48; ];
plot(Kp140i0d0(:,1),Kp140i0d0(:,2))
% y = max(Kp140i0d0(:,2));
% x = Kp140i0d0(find(Kp140i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp140i0d0');
% pause;

Kp150i0d0 = [10.0, -1; 20.0, 0; 30.0, 1; 40.0, 3; 50.0, 7; 60.0, 10; 70.0, 16; 80.0, 20; 90.0, 27; 100.0, 32; 110.0, 41; 120.0, 46; 130.0, 55; 140.0, 60; 150.0, 67; 160.0, 70; 170.0, 74; 180.0, 75; 190.0, 76; 200.0, 76; 210.0, 74; 220.0, 72; 230.0, 68; 240.0, 65; 250.0, 59; 260.0, 55; 270.0, 48; 280.0, 43; 290.0, 38; 300.0, 35; 310.0, 32; 320.0, 31; 330.0, 31; 340.0, 32; 350.0, 34; 360.0, 37; 370.0, 41; 380.0, 45; 390.0, 51; 400.0, 55; 410.0, 60; 420.0, 62; 430.0, 65; 440.0, 65; 450.0, 65; 460.0, 64; 470.0, 60; 480.0, 58; 490.0, 53; 501.0, 49; 511.0, 44; 521.0, 41; 531.0, 38; 541.0, 37; 551.0, 36; 561.0, 37; 571.0, 39; 581.0, 41; 591.0, 45; 601.0, 49; 611.0, 54; 621.0, 57; 631.0, 60; 641.0, 62; 651.0, 62; 661.0, 62; 671.0, 60; 681.0, 58; 691.0, 53; 701.0, 50; 711.0, 45; 721.0, 42; 731.0, 40; 741.0, 39; 751.0, 39; 761.0, 39; 771.0, 42; 781.0, 44; 791.0, 49; 801.0, 52; 811.0, 56; 821.0, 59; 831.0, 61; 841.0, 61; 851.0, 60; 861.0, 59; 871.0, 56; 881.0, 53; 891.0, 48; 901.0, 45; 911.0, 42; 921.0, 40; 931.0, 40; 941.0, 40; 951.0, 42; 961.0, 44; 971.0, 48; 981.0, 51; 991.0, 55; 1001.0, 58; 1011.0, 60; 1021.0, 60; 1031.0, 59; 1041.0, 58; 1051.0, 55; 1061.0, 52; 1071.0, 48; 1081.0, 45; 1091.0, 42; 1101.0, 41; 1111.0, 40; 1121.0, 41; 1131.0, 42; 1141.0, 44; 1151.0, 48; 1161.0, 51; 1171.0, 55; 1181.0, 58; 1191.0, 60; 1201.0, 60; 1211.0, 59; 1221.0, 58; 1231.0, 55; 1241.0, 52; 1251.0, 48; 1261.0, 45; 1271.0, 42; 1281.0, 40; 1291.0, 40; 1301.0, 40; 1311.0, 42; 1321.0, 44; 1331.0, 48; 1341.0, 52; 1351.0, 56; 1361.0, 58; 1371.0, 60; 1381.0, 60; 1391.0, 59; 1401.0, 58; 1411.0, 54; 1421.0, 52; 1431.0, 48; 1441.0, 45; 1451.0, 42; 1461.0, 41; 1471.0, 40; 1481.0, 41; 1491.0, 43; 1501.0, 45; ];
plot(Kp150i0d0(:,1),Kp150i0d0(:,2))
% y = max(Kp150i0d0(:,2));
% x = Kp150i0d0(find(Kp150i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp150i0d0');
% pause;

Kp160i0d0 = [12.0, 0; 22.0, 1; 32.0, 2; 42.0, 4; 52.0, 8; 62.0, 11; 72.0, 17; 82.0, 21; 92.0, 28; 102.0, 33; 112.0, 41; 122.0, 47; 132.0, 56; 142.0, 61; 152.0, 67; 162.0, 71; 172.0, 74; 182.0, 76; 192.0, 77; 202.0, 76; 212.0, 74; 222.0, 72; 232.0, 68; 242.0, 65; 252.0, 59; 262.0, 55; 272.0, 48; 282.0, 43; 292.0, 38; 302.0, 35; 312.0, 32; 322.0, 31; 332.0, 31; 342.0, 31; 352.0, 33; 362.0, 35; 372.0, 39; 382.0, 43; 392.0, 49; 402.0, 53; 412.0, 59; 422.0, 62; 432.0, 64; 442.0, 65; 452.0, 66; 462.0, 65; 472.0, 62; 482.0, 60; 492.0, 55; 502.0, 52; 512.0, 46; 522.0, 43; 532.0, 39; 542.0, 38; 552.0, 37; 562.0, 37; 572.0, 38; 582.0, 40; 592.0, 44; 602.0, 47; 612.0, 52; 622.0, 56; 632.0, 59; 642.0, 61; 652.0, 62; 662.0, 62; 672.0, 60; 682.0, 58; 692.0, 54; 702.0, 51; 712.0, 46; 722.0, 44; 732.0, 41; 742.0, 40; 752.0, 40; 762.0, 40; 772.0, 43; 782.0, 45; 792.0, 49; 802.0, 52; 812.0, 56; 822.0, 58; 832.0, 60; 843.0, 60; 853.0, 59; 863.0, 57; 873.0, 54; 883.0, 51; 893.0, 47; 903.0, 44; 913.0, 42; 923.0, 41; 933.0, 40; 943.0, 41; 953.0, 44; 963.0, 46; 973.0, 50; 983.0, 53; 993.0, 57; 1003.0, 58; 1013.0, 59; 1023.0, 59; 1033.0, 58; 1043.0, 56; 1053.0, 52; 1063.0, 49; 1073.0, 45; 1083.0, 43; 1093.0, 40; 1103.0, 40; 1113.0, 41; 1123.0, 42; 1133.0, 45; 1143.0, 47; 1153.0, 52; 1163.0, 55; 1173.0, 58; 1183.0, 60; 1193.0, 60; 1203.0, 60; 1213.0, 58; 1223.0, 56; 1233.0, 52; 1243.0, 48; 1253.0, 44; 1263.0, 42; 1273.0, 40; 1283.0, 40; 1293.0, 40; 1303.0, 41; 1313.0, 44; 1323.0, 46; 1333.0, 51; 1343.0, 54; 1361.0, 60; 1373.0, 62; 1383.0, 63; 1393.0, 62; 1403.0, 61; 1413.0, 58; 1423.0, 55; 1433.0, 50; 1443.0, 47; 1453.0, 42; 1463.0, 40; 1473.0, 38; 1483.0, 38; 1493.0, 39; 1503.0, 40; ];
plot(Kp160i0d0(:,1),Kp160i0d0(:,2))
% y = max(Kp160i0d0(:,2));
% x = Kp160i0d0(find(Kp160i0d0(:,2) == max(y), 1, 'first'));
% text(x, y, 'Kp160i0d0');
% pause;

caseList = {'Kp10i0d0', 'Kp20i0d0', 'Kp30i0d0', 'Kp40i0d0', 'Kp50i0d0', 'Kp60i0d0', 'Kp70i0d0', 'Kp80i0d0', 'Kp90i0d0', 'Kp100i0d0', 'Kp110i0d0', 'Kp120i0d0', 'Kp130i0d0', 'Kp140i0d0', 'Kp150i0d0', 'Kp160i0d0'};
legend('50 Deg', caseList{1,:});

t = {Kp10i0d0, Kp20i0d0, Kp30i0d0, Kp40i0d0, Kp50i0d0, Kp60i0d0, Kp70i0d0, Kp80i0d0, Kp90i0d0, Kp100i0d0, Kp110i0d0, Kp120i0d0, Kp130i0d0, Kp140i0d0, Kp150i0d0, Kp160i0d0};

figure(2);
hold on;
xlabel('Time (ms)');
ylabel('Theta (Deg)');
title('PID Controller | P100SP50 | Good Options');
plot([0 1500], [50 50], 'r--');
upper = 50;
idx = 0;
while idx < 6
    for e = t
        if max(e{1}(:,2)) <= upper && max(e{1}(:,2)) >= 50
            idx = idx + 1;
        end
    end
    upper = upper + 1;
end
upper = upper - 1;
legendTwo = {'50 Deg'};
idx = 0;
for e = t
    idx = idx + 1;
    if max(e{1}(:,2)) <= upper && max(e{1}(:,2)) >= 50
        plot(e{1}(:,1),e{1}(:,2));
        legendTwo{end+1} = caseList{idx};
    end
end
newLegend = legend();
newLegend = newLegend.String;
idx = size(legendTwo);
newLegend(end-idx(2)+1:end) = legendTwo(1:end);
legend(newLegend);