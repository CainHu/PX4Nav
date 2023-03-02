//
// Created by Cain on 2023/3/1.
//

#ifndef ECL_GEO_MAGNETIC_TABLES_HPP
#define ECL_GEO_MAGNETIC_TABLES_HPP

#include <stdint.h>

static constexpr float SAMPLING_RES = 10;
static constexpr float SAMPLING_MIN_LAT = -90;
static constexpr float SAMPLING_MAX_LAT = 90;
static constexpr float SAMPLING_MIN_LON = -180;
static constexpr float SAMPLING_MAX_LON = 180;

static constexpr int LAT_DIM = 19;
static constexpr int LON_DIM = 37;


// *INDENT-OFF*
// Magnetic declination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.1534,
static constexpr const int16_t declination_table[19][37] {
        //    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
        /* LAT: -90 */ {  25997, 24251, 22506, 20761, 19015, 17270, 15525, 13779, 12034, 10289,  8543,  6798,  5053,  3308,  1562,  -183, -1928, -3674, -5419, -7164, -8909,-10655,-12400,-14145,-15891,-17636,-19381,-21127,-22872,-24617,-26363,-28108,-29854, 31233, 29487, 27742, 25997, },
        /* LAT: -80 */ {  22564, 20431, 18490, 16715, 15073, 13535, 12070, 10659,  9283,  7933,  6601,  5282,  3973,  2668,  1361,    42, -1299, -2670, -4079, -5530, -7024, -8559,-10136,-11754,-13418,-15135,-16920,-18791,-20773,-22890,-25164,-27598,-30166, 30025, 27405, 24897, 22564, },
        /* LAT: -70 */ {  14973, 13576, 12451, 11491, 10623,  9793,  8953,  8066,  7114,  6095,  5025,  3932,  2846,  1790,   767,  -243, -1279, -2383, -3582, -4880, -6258, -7687, -9133,-10574,-12000,-13415,-14843,-16327,-17948,-19857,-22371,-26158, 30755, 24163, 19625, 16845, 14973, },
        /* LAT: -60 */ {   8403,  8161,  7884,  7613,  7362,  7111,  6805,  6374,  5761,  4943,  3943,  2832,  1710,   680,  -204,  -964, -1693, -2513, -3513, -4706, -6033, -7400, -8720, -9931,-10996,-11896,-12613,-13108,-13280,-12831,-10735, -3540,  4856,  7622,  8402,  8534,  8403, },
        /* LAT: -50 */ {   5471,  5510,  5456,  5369,  5298,  5263,  5230,  5105,  4764,  4104,  3096,  1825,   483,  -701, -1577, -2147, -2546, -2984, -3664, -4670, -5908, -7182, -8323, -9227, -9828,-10072, -9882, -9129, -7624, -5265, -2363,   384,  2493,  3918,  4792,  5266,  5471, },
        /* LAT: -40 */ {   3944,  4038,  4048,  4006,  3948,  3916,  3923,  3913,  3742,  3211,  2192,   753,  -812, -2131, -3001, -3459, -3647, -3709, -3874, -4432, -5400, -6473, -7360, -7903, -8016, -7645, -6761, -5379, -3666, -1962,  -500,   721,  1764,  2628,  3283,  3713,  3944, },
        /* LAT: -30 */ {   2975,  3062,  3094,  3083,  3027,  2950,  2892,  2859,  2734,  2259,  1230,  -286, -1894, -3153, -3900, -4251, -4336, -4132, -3700, -3468, -3818, -4562, -5262, -5618, -5499, -4918, -3962, -2755, -1543,  -596,    83,   684,  1307,  1908,  2416,  2776,  2975, },
        /* LAT: -20 */ {   2333,  2379,  2399,  2405,  2365,  2273,  2167,  2092,  1947,  1455,   405, -1080, -2560, -3628, -4168, -4291, -4094, -3543, -2684, -1880, -1593, -1940, -2589, -3062, -3104, -2749, -2116, -1298,  -519,   -28,   236,   539,   978,  1454,  1875,  2180,  2333, },
        /* LAT: -10 */ {   1939,  1934,  1915,  1915,  1890,  1808,  1700,  1611,  1427,   880,  -177, -1552, -2828, -3664, -3940, -3720, -3150, -2372, -1525,  -767,  -297,  -322,  -789, -1296, -1513, -1426, -1113,  -614,  -114,   129,   179,   340,   709,  1144,  1535,  1821,  1939, },
        /* LAT:   0 */ {   1726,  1693,  1640,  1634,  1626,  1560,  1458,  1347,  1094,   475,  -562, -1788, -2846, -3440, -3448, -2955, -2190, -1408,  -754,  -215,   200,   316,    42,  -375,  -632,  -690,  -591,  -329,   -37,    54,    -5,    84,   424,   863,  1280,  1598,  1726, },
        /* LAT:  10 */ {   1591,  1600,  1560,  1579,  1607,  1560,  1442,  1258,   884,   167,  -852, -1929, -2760, -3107, -2911, -2310, -1535,  -822,  -304,    79,   405,   557,   399,    77,  -159,  -268,  -290,  -208,  -103,  -143,  -281,  -254,    46,   498,   977,  1382,  1591, },
        /* LAT:  20 */ {   1409,  1559,  1621,  1715,  1803,  1785,  1634,  1334,   791,   -65, -1107, -2062, -2664, -2780, -2458, -1860, -1149,  -498,   -35,   276,   536,   684,   595,   352,   152,    34,   -56,  -119,  -198,  -387,  -623,  -681,  -449,    -3,   536,  1053,  1409, },
        /* LAT:  30 */ {   1112,  1479,  1739,  1964,  2126,  2138,  1953,  1533,   807,  -222, -1342, -2223, -2644, -2588, -2193, -1615,  -960,  -341,   123,   431,   664,   812,   794,   646,   496,   371,   216,     7,  -274,  -648, -1016, -1177, -1019,  -594,   -19,   590,  1112, },
        /* LAT:  40 */ {    758,  1345,  1839,  2232,  2481,  2526,  2312,  1781,   870,  -358, -1600, -2473, -2800, -2652, -2208, -1617,  -963,  -333,   179,   546,   818,  1019,  1115,  1105,  1029,   881,   614,   206,  -330,  -936, -1458, -1704, -1589, -1174,  -575,    98,   758, },
        /* LAT:  50 */ {    472,  1220,  1900,  2457,  2824,  2930,  2702,  2048,   907,  -593, -2022, -2943, -3242, -3053, -2564, -1918, -1208,  -509,   109,   617,  1037,  1395,  1677,  1849,  1869,  1684,  1242,   537,  -354, -1257, -1938, -2227, -2105, -1664, -1028,  -294,   472, },
        /* LAT:  60 */ {    282,  1136,  1942,  2636,  3140,  3352,  3137,  2324,   805, -1162, -2886, -3872, -4126, -3867, -3294, -2547, -1720,  -877,   -68,   686,  1380,  2012,  2557,  2956,  3118,  2935,  2302,  1190,  -222, -1543, -2416, -2729, -2564, -2069, -1374,  -570,   282, },
        /* LAT:  70 */ {     59,   996,  1895,  2694,  3307,  3600,  3344,  2202,   -53, -2783, -4742, -5567, -5573, -5086, -4313, -3376, -2349, -1280,  -203,   859,  1886,  2854,  3724,  4434,  4885,  4922,  4320,  2857,   670, -1439, -2742, -3189, -3030, -2496, -1744,  -871,    59, },
        /* LAT:  80 */ {   -610,   314,  1168,  1861,  2245,  2056,   836, -1821, -5074, -7200, -7938, -7797, -7159, -6235, -5139, -3935, -2666, -1357,   -30,  1300,  2616,  3899,  5128,  6267,  7264,  8025,  8372,  7936,  6038,  2313, -1268, -2997, -3374, -3053, -2381, -1532,  -610, },
        /* LAT:  90 */ { -29988,-28242,-26497,-24751,-23006,-21260,-19515,-17770,-16025,-14279,-12534,-10789, -9044, -7298, -5553, -3808, -2063,  -318,  1427,  3173,  4918,  6663,  8408, 10154, 11899, 13644, 15390, 17135, 18880, 20626, 22371, 24117, 25862, 27608, 29353, 31099,-29988, },
};

// Magnetic inclination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.1534,
static constexpr const int16_t inclination_table[19][37] {
        //    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
        /* LAT: -90 */ { -12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574,-12574, },
        /* LAT: -80 */ { -13659,-13526,-13365,-13185,-12991,-12789,-12585,-12384,-12193,-12017,-11860,-11726,-11615,-11529,-11466,-11424,-11404,-11407,-11434,-11488,-11572,-11686,-11831,-12004,-12202,-12419,-12648,-12881,-13109,-13322,-13508,-13659,-13763,-13815,-13812,-13758,-13659, },
        /* LAT: -70 */ { -14108,-13790,-13471,-13148,-12815,-12471,-12116,-11758,-11414,-11105,-10852,-10668,-10555,-10502,-10489,-10493,-10500,-10510,-10534,-10590,-10698,-10872,-11117,-11431,-11803,-12221,-12668,-13132,-13598,-14051,-14469,-14815,-15005,-14952,-14722,-14424,-14108, },
        /* LAT: -60 */ { -13520,-13167,-12829,-12496,-12153,-11781,-11365,-10910,-10442,-10011, -9680, -9502, -9498, -9634, -9838,-10032,-10159,-10205,-10194,-10180,-10227,-10384,-10671,-11078,-11575,-12130,-12716,-13312,-13901,-14462,-14959,-15249,-15075,-14691,-14286,-13893,-13520, },
        /* LAT: -50 */ { -12496,-12155,-11825,-11503,-11179,-10833,-10433, -9962, -9431, -8909, -8517, -8388, -8586, -9045, -9610,-10123,-10486,-10652,-10626,-10477,-10335,-10338,-10555,-10969,-11509,-12100,-12687,-13226,-13673,-13972,-14080,-14008,-13804,-13521,-13193,-12846,-12496, },
        /* LAT: -40 */ { -11240,-10892,-10546,-10201, -9862, -9523, -9162, -8736, -8214, -7648, -7219, -7173, -7628, -8449, -9371,-10202,-10864,-11303,-11449,-11286,-10946,-10672,-10660,-10938,-11398,-11902,-12349,-12673,-12833,-12839,-12749,-12611,-12431,-12199,-11913,-11586,-11240, },
        /* LAT: -30 */ {  -9602, -9225, -8847, -8457, -8065, -7689, -7331, -6939, -6426, -5814, -5359, -5435, -6197, -7396, -8647, -9745,-10662,-11376,-11781,-11778,-11411,-10904,-10566,-10563,-10812,-11133,-11391,-11506,-11445,-11269,-11094,-10959,-10812,-10604,-10323, -9978, -9602, },
        /* LAT: -20 */ {  -7371, -6933, -6518, -6091, -5647, -5216, -4824, -4409, -3842, -3156, -2703, -2956, -4057, -5666, -7295, -8668, -9738,-10518,-10962,-11003,-10644,-10033, -9473, -9226, -9273, -9435, -9577, -9596, -9425, -9152, -8957, -8869, -8763, -8555, -8238, -7827, -7371, },
        /* LAT: -10 */ {  -4414, -3882, -3432, -2997, -2539, -2089, -1673, -1216,  -587,   118,   485,    67, -1253, -3166, -5143, -6757, -7860, -8499, -8769, -8710, -8305, -7624, -6960, -6612, -6571, -6664, -6782, -6807, -6620, -6324, -6167, -6174, -6132, -5914, -5530, -5004, -4414, },
        /* LAT:   0 */ {   -905,  -287,   174,   575,   993,  1410,  1802,  2248,  2836,  3417,  3623,  3135,  1847,   -58, -2103, -3759, -4775, -5210, -5272, -5104, -4667, -3955, -3250, -2877, -2816, -2886, -3012, -3085, -2956, -2720, -2664, -2804, -2863, -2675, -2251, -1623,  -905, },
        /* LAT:  10 */ {   2562,  3184,  3615,  3954,  4308,  4676,  5029,  5419,  5874,  6247,  6287,  5804,  4727,  3166,  1475,    95,  -716,  -968,  -880,  -646,  -237,   398,  1031,  1369,  1432,  1386,  1285,  1194,  1237,  1341,  1267,  1004,   815,   889,  1241,  1842,  2562, },
        /* LAT:  20 */ {   5417,  5942,  6319,  6613,  6926,  7269,  7614,  7964,  8300,  8502,  8418,  7959,  7127,  6034,  4908,  3996,  3464,  3341,  3488,  3730,  4059,  4523,  4983,  5238,  5294,  5275,  5226,  5169,  5158,  5141,  4969,  4639,  4344,  4256,  4428,  4849,  5417, },
        /* LAT:  30 */ {   7569,  7940,  8256,  8538,  8847,  9195,  9553,  9896, 10175, 10291, 10148,  9729,  9102,  8391,  7730,  7217,  6924,  6879,  7020,  7230,  7475,  7774,  8062,  8235,  8292,  8304,  8305,  8296,  8274,  8193,  7973,  7622,  7268,  7049,  7033,  7226,  7569, },
        /* LAT:  40 */ {   9266,  9486,  9742, 10027, 10354, 10715, 11084, 11424, 11678, 11761, 11613, 11255, 10782, 10308,  9912,  9627,  9477,  9471,  9580,  9741,  9915, 10098, 10270, 10395, 10472, 10532, 10586, 10619, 10604, 10495, 10254,  9905,  9539,  9254,  9111,  9123,  9266, },
        /* LAT:  50 */ {  10801, 10923, 11124, 11394, 11718, 12072, 12428, 12745, 12968, 13028, 12890, 12592, 12228, 11882, 11606, 11419, 11324, 11320, 11385, 11487, 11600, 11716, 11832, 11947, 12065, 12187, 12300, 12372, 12364, 12241, 11996, 11669, 11329, 11045, 10855, 10775, 10801, },
        /* LAT:  60 */ {  12319, 12392, 12543, 12762, 13033, 13334, 13637, 13904, 14080, 14108, 13972, 13723, 13433, 13162, 12940, 12783, 12692, 12660, 12675, 12722, 12789, 12874, 12979, 13111, 13270, 13445, 13610, 13719, 13726, 13607, 13383, 13105, 12826, 12589, 12419, 12327, 12319, },
        /* LAT:  70 */ {  13758, 13802, 13898, 14040, 14220, 14425, 14634, 14815, 14920, 14899, 14761, 14559, 14341, 14137, 13963, 13828, 13733, 13678, 13659, 13671, 13713, 13784, 13887, 14023, 14189, 14376, 14560, 14701, 14748, 14677, 14516, 14318, 14124, 13959, 13838, 13771, 13758, },
        /* LAT:  80 */ {  14999, 15012, 15050, 15110, 15187, 15273, 15352, 15397, 15378, 15298, 15185, 15060, 14937, 14823, 14723, 14641, 14578, 14538, 14519, 14522, 14549, 14598, 14669, 14761, 14873, 14999, 15134, 15266, 15373, 15418, 15381, 15295, 15200, 15116, 15052, 15012, 14999, },
        /* LAT:  90 */ {  15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, 15394, },
};

// Magnetic strength data in milli-Gauss * 10
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.1534,
static constexpr const int16_t strength_table[19][37] {
        //    LONGITUDE:  -180, -170, -160, -150, -140, -130, -120, -110, -100,  -90,  -80,  -70,  -60,  -50,  -40,  -30,  -20,  -10,    0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,
        /* LAT: -90 */ {  5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, 5454, },
        /* LAT: -80 */ {  6060, 5997, 5918, 5826, 5724, 5612, 5494, 5372, 5250, 5130, 5016, 4911, 4817, 4737, 4674, 4629, 4604, 4602, 4623, 4669, 4739, 4833, 4947, 5079, 5222, 5372, 5522, 5665, 5796, 5910, 6002, 6072, 6116, 6136, 6132, 6106, 6060, },
        /* LAT: -70 */ {  6305, 6172, 6022, 5858, 5679, 5487, 5282, 5067, 4848, 4633, 4430, 4248, 4090, 3959, 3854, 3777, 3730, 3717, 3746, 3821, 3949, 4128, 4356, 4624, 4918, 5226, 5529, 5813, 6063, 6267, 6418, 6514, 6556, 6549, 6500, 6416, 6305, },
        /* LAT: -60 */ {  6190, 5998, 5797, 5589, 5370, 5136, 4880, 4601, 4309, 4019, 3752, 3524, 3344, 3209, 3110, 3037, 2986, 2966, 2992, 3081, 3249, 3500, 3826, 4211, 4633, 5067, 5488, 5872, 6197, 6446, 6610, 6689, 6691, 6629, 6516, 6365, 6190, },
        /* LAT: -50 */ {  5846, 5617, 5385, 5155, 4923, 4678, 4408, 4105, 3776, 3444, 3142, 2901, 2739, 2648, 2599, 2565, 2532, 2506, 2511, 2582, 2754, 3043, 3438, 3908, 4415, 4921, 5396, 5815, 6154, 6396, 6535, 6576, 6534, 6424, 6263, 6066, 5846, },
        /* LAT: -40 */ {  5395, 5149, 4905, 4666, 4432, 4195, 3941, 3657, 3343, 3017, 2718, 2494, 2376, 2350, 2369, 2391, 2396, 2385, 2373, 2401, 2530, 2803, 3218, 3731, 4277, 4802, 5270, 5659, 5951, 6139, 6228, 6231, 6162, 6032, 5851, 5633, 5395, },
        /* LAT: -30 */ {  4879, 4639, 4401, 4167, 3941, 3722, 3503, 3272, 3017, 2741, 2483, 2300, 2229, 2253, 2320, 2392, 2458, 2509, 2532, 2544, 2610, 2806, 3165, 3652, 4183, 4681, 5100, 5418, 5622, 5722, 5749, 5722, 5643, 5511, 5332, 5115, 4879, },
        /* LAT: -20 */ {  4322, 4109, 3901, 3697, 3501, 3319, 3150, 2987, 2810, 2612, 2421, 2287, 2244, 2286, 2375, 2486, 2614, 2743, 2833, 2870, 2895, 2988, 3225, 3607, 4056, 4484, 4833, 5070, 5176, 5184, 5154, 5107, 5025, 4897, 4731, 4534, 4322, },
        /* LAT: -10 */ {  3790, 3630, 3478, 3332, 3196, 3076, 2973, 2881, 2785, 2671, 2549, 2449, 2402, 2425, 2510, 2638, 2794, 2954, 3079, 3142, 3157, 3183, 3304, 3554, 3879, 4201, 4466, 4630, 4667, 4615, 4547, 4483, 4394, 4270, 4121, 3957, 3790, },
        /* LAT:   0 */ {  3412, 3320, 3236, 3164, 3109, 3071, 3045, 3027, 3003, 2956, 2877, 2782, 2701, 2668, 2708, 2810, 2942, 3078, 3194, 3270, 3302, 3323, 3397, 3552, 3760, 3974, 4154, 4261, 4268, 4201, 4113, 4020, 3908, 3777, 3643, 3519, 3412, },
        /* LAT:  10 */ {  3283, 3252, 3232, 3229, 3254, 3301, 3357, 3412, 3448, 3438, 3369, 3255, 3127, 3031, 3003, 3043, 3123, 3222, 3322, 3408, 3471, 3533, 3621, 3738, 3873, 4011, 4130, 4201, 4204, 4143, 4034, 3890, 3729, 3570, 3435, 3338, 3283, },
        /* LAT:  20 */ {  3400, 3403, 3430, 3484, 3577, 3698, 3828, 3946, 4027, 4040, 3967, 3825, 3657, 3516, 3439, 3425, 3459, 3531, 3628, 3725, 3815, 3913, 4024, 4135, 4243, 4354, 4456, 4522, 4533, 4475, 4339, 4137, 3910, 3701, 3538, 3438, 3400, },
        /* LAT:  30 */ {  3723, 3730, 3786, 3886, 4030, 4201, 4377, 4534, 4643, 4670, 4597, 4441, 4251, 4085, 3979, 3931, 3934, 3984, 4070, 4168, 4265, 4370, 4486, 4604, 4723, 4849, 4969, 5056, 5083, 5026, 4871, 4632, 4358, 4104, 3905, 3777, 3723, },
        /* LAT:  40 */ {  4222, 4222, 4288, 4412, 4580, 4768, 4951, 5108, 5213, 5239, 5171, 5022, 4834, 4658, 4529, 4453, 4426, 4447, 4507, 4585, 4671, 4768, 4883, 5016, 5167, 5328, 5479, 5589, 5629, 5576, 5421, 5184, 4910, 4651, 4440, 4295, 4222, },
        /* LAT:  50 */ {  4832, 4826, 4883, 4994, 5141, 5300, 5450, 5571, 5645, 5654, 5590, 5462, 5299, 5134, 4996, 4898, 4842, 4830, 4854, 4904, 4972, 5063, 5182, 5331, 5507, 5691, 5857, 5976, 6021, 5977, 5847, 5651, 5426, 5210, 5030, 4901, 4832, },
        /* LAT:  60 */ {  5392, 5381, 5410, 5474, 5560, 5655, 5742, 5807, 5840, 5831, 5776, 5682, 5561, 5432, 5312, 5215, 5148, 5114, 5111, 5139, 5195, 5281, 5398, 5545, 5712, 5880, 6028, 6133, 6177, 6156, 6073, 5947, 5800, 5657, 5535, 5444, 5392, },
        /* LAT:  70 */ {  5726, 5707, 5705, 5718, 5740, 5766, 5790, 5804, 5803, 5784, 5744, 5686, 5615, 5539, 5465, 5400, 5352, 5324, 5319, 5338, 5383, 5452, 5544, 5652, 5770, 5884, 5984, 6057, 6096, 6100, 6071, 6018, 5950, 5880, 5815, 5762, 5726, },
        /* LAT:  80 */ {  5789, 5772, 5758, 5746, 5736, 5727, 5717, 5705, 5690, 5671, 5649, 5624, 5596, 5569, 5544, 5523, 5509, 5504, 5509, 5524, 5549, 5585, 5627, 5675, 5725, 5773, 5816, 5850, 5875, 5888, 5890, 5883, 5869, 5851, 5830, 5809, 5789, },
        /* LAT:  90 */ {  5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, 5681, },
};


#endif //ECL_GEO_MAGNETIC_TABLES_HPP