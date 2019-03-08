/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t __attribute__((optimize("O0"))) *tag36h11_create()
{
   apriltag_family_t *tf = (apriltag_family_t *)calloc(1, sizeof(apriltag_family_t));
#ifdef WINRT
   tf->name = _strdup("tag36h11");
#else
   tf->name = strdup("tag36h11");
#endif
   tf->h = 11;
   tf->ncodes = 587;
   tf->codes = (uint64_t *)calloc(587, sizeof(uint64_t));
   tf->codes[0] = 0x0000000d7e00984bUL;
   tf->codes[1] = 0x0000000dda664ca7UL;
   tf->codes[2] = 0x0000000dc4a1c821UL;
   tf->codes[3] = 0x0000000e17b470e9UL;
   tf->codes[4] = 0x0000000ef91d01b1UL;
   tf->codes[5] = 0x0000000f429cdd73UL;
   tf->codes[6] = 0x000000005da29225UL;
   tf->codes[7] = 0x00000001106cba43UL;
   tf->codes[8] = 0x0000000223bed79dUL;
   tf->codes[9] = 0x000000021f51213cUL;
   tf->codes[10] = 0x000000033eb19ca6UL;
   tf->codes[11] = 0x00000003f76eb0f8UL;
   tf->codes[12] = 0x0000000469a97414UL;
   tf->codes[13] = 0x000000045dcfe0b0UL;
   tf->codes[14] = 0x00000004a6465f72UL;
   tf->codes[15] = 0x000000051801db96UL;
   tf->codes[16] = 0x00000005eb946b4eUL;
   tf->codes[17] = 0x000000068a7cc2ecUL;
   tf->codes[18] = 0x00000006f0ba2652UL;
   tf->codes[19] = 0x000000078765559dUL;
   tf->codes[20] = 0x000000087b83d129UL;
   tf->codes[21] = 0x000000086cc4a5c5UL;
   tf->codes[22] = 0x00000008b64df90fUL;
   tf->codes[23] = 0x00000009c577b611UL;
   tf->codes[24] = 0x0000000a3810f2f5UL;
   tf->codes[25] = 0x0000000af4d75b83UL;
   tf->codes[26] = 0x0000000b59a03fefUL;
   tf->codes[27] = 0x0000000bb1096f85UL;
   tf->codes[28] = 0x0000000d1b92fc76UL;
   tf->codes[29] = 0x0000000d0dd509d2UL;
   tf->codes[30] = 0x0000000e2cfda160UL;
   tf->codes[31] = 0x00000002ff497c63UL;
   tf->codes[32] = 0x000000047240671bUL;
   tf->codes[33] = 0x00000005047a2e55UL;
   tf->codes[34] = 0x0000000635ca87c7UL;
   tf->codes[35] = 0x0000000691254166UL;
   tf->codes[36] = 0x000000068f43d94aUL;
   tf->codes[37] = 0x00000006ef24bdb6UL;
   tf->codes[38] = 0x00000008cdd8f886UL;
   tf->codes[39] = 0x00000009de96b718UL;
   tf->codes[40] = 0x0000000aff6e5a8aUL;
   tf->codes[41] = 0x0000000bae46f029UL;
   tf->codes[42] = 0x0000000d225b6d59UL;
   tf->codes[43] = 0x0000000df8ba8c01UL;
   tf->codes[44] = 0x0000000e3744a22fUL;
   tf->codes[45] = 0x0000000fbb59375dUL;
   tf->codes[46] = 0x000000018a916828UL;
   tf->codes[47] = 0x000000022f29c1baUL;
   tf->codes[48] = 0x0000000286887d58UL;
   tf->codes[49] = 0x000000041392322eUL;
   tf->codes[50] = 0x000000075d18ecd1UL;
   tf->codes[51] = 0x000000087c302743UL;
   tf->codes[52] = 0x00000008c6317ba9UL;
   tf->codes[53] = 0x00000009e40f36d7UL;
   tf->codes[54] = 0x0000000c0e5a806aUL;
   tf->codes[55] = 0x0000000cc78cb87cUL;
   tf->codes[56] = 0x000000012d2f2d01UL;
   tf->codes[57] = 0x0000000379f36a21UL;
   tf->codes[58] = 0x00000006973f59acUL;
   tf->codes[59] = 0x00000007789ea9f4UL;
   tf->codes[60] = 0x00000008f1c73e84UL;
   tf->codes[61] = 0x00000008dd287a20UL;
   tf->codes[62] = 0x000000094a4eee4cUL;
   tf->codes[63] = 0x0000000a455379b5UL;
   tf->codes[64] = 0x0000000a9e92987dUL;
   tf->codes[65] = 0x0000000bd25cb40bUL;
   tf->codes[66] = 0x0000000be98d3582UL;
   tf->codes[67] = 0x0000000d3d5972b2UL;
   tf->codes[68] = 0x000000014c53d7c7UL;
   tf->codes[69] = 0x00000004f1796936UL;
   tf->codes[70] = 0x00000004e71fed1aUL;
   tf->codes[71] = 0x000000066d46fae0UL;
   tf->codes[72] = 0x0000000a55abb933UL;
   tf->codes[73] = 0x0000000ebee1accaUL;
   tf->codes[74] = 0x00000001ad4ba6a4UL;
   tf->codes[75] = 0x0000000305b17571UL;
   tf->codes[76] = 0x0000000553611351UL;
   tf->codes[77] = 0x000000059ca62775UL;
   tf->codes[78] = 0x00000007819cb6a1UL;
   tf->codes[79] = 0x0000000edb7bc9ebUL;
   tf->codes[80] = 0x00000005b2694212UL;
   tf->codes[81] = 0x000000072e12d185UL;
   tf->codes[82] = 0x0000000ed6152e2cUL;
   tf->codes[83] = 0x00000005bcdadbf3UL;
   tf->codes[84] = 0x000000078e0aa0c6UL;
   tf->codes[85] = 0x0000000c60a0b909UL;
   tf->codes[86] = 0x0000000ef9a34b0dUL;
   tf->codes[87] = 0x0000000398a6621aUL;
   tf->codes[88] = 0x0000000a8a27c944UL;
   tf->codes[89] = 0x00000004b564304eUL;
   tf->codes[90] = 0x000000052902b4e2UL;
   tf->codes[91] = 0x0000000857280b56UL;
   tf->codes[92] = 0x0000000a91b2c84bUL;
   tf->codes[93] = 0x0000000e91df939bUL;
   tf->codes[94] = 0x00000001fa405f28UL;
   tf->codes[95] = 0x000000023793ab86UL;
   tf->codes[96] = 0x000000068c17729fUL;
   tf->codes[97] = 0x00000009fbf3b840UL;
   tf->codes[98] = 0x000000036922413cUL;
   tf->codes[99] = 0x00000004eb5f946eUL;
   tf->codes[100] = 0x0000000533fe2404UL;
   tf->codes[101] = 0x000000063de7d35eUL;
   tf->codes[102] = 0x0000000925eddc72UL;
   tf->codes[103] = 0x000000099b8b3896UL;
   tf->codes[104] = 0x0000000aace4c708UL;
   tf->codes[105] = 0x0000000c22994af0UL;
   tf->codes[106] = 0x00000008f1eae41bUL;
   tf->codes[107] = 0x0000000d95fb486cUL;
   tf->codes[108] = 0x000000013fb77857UL;
   tf->codes[109] = 0x00000004fe0983a3UL;
   tf->codes[110] = 0x0000000d559bf8a9UL;
   tf->codes[111] = 0x0000000e1855d78dUL;
   tf->codes[112] = 0x0000000fec8daaadUL;
   tf->codes[113] = 0x000000071ecb6d95UL;
   tf->codes[114] = 0x0000000dc9e50e4cUL;
   tf->codes[115] = 0x0000000ca3a4c259UL;
   tf->codes[116] = 0x0000000740d12bbfUL;
   tf->codes[117] = 0x0000000aeedd18e0UL;
   tf->codes[118] = 0x0000000b509b9c8eUL;
   tf->codes[119] = 0x00000005232fea1cUL;
   tf->codes[120] = 0x000000019282d18bUL;
   tf->codes[121] = 0x000000076c22d67bUL;
   tf->codes[122] = 0x0000000936beb34bUL;
   tf->codes[123] = 0x000000008a5ea8ddUL;
   tf->codes[124] = 0x0000000679eadc28UL;
   tf->codes[125] = 0x0000000a08e119c5UL;
   tf->codes[126] = 0x000000020a6e3e24UL;
   tf->codes[127] = 0x00000007eab9c239UL;
   tf->codes[128] = 0x000000096632c32eUL;
   tf->codes[129] = 0x0000000470d06e44UL;
   tf->codes[130] = 0x00000008a70212fbUL;
   tf->codes[131] = 0x00000000a7e4251bUL;
   tf->codes[132] = 0x00000009ec762cc0UL;
   tf->codes[133] = 0x0000000d8a3a1f48UL;
   tf->codes[134] = 0x0000000db680f346UL;
   tf->codes[135] = 0x00000004a1e93a9dUL;
   tf->codes[136] = 0x0000000638ddc04fUL;
   tf->codes[137] = 0x00000004c2fcc993UL;
   tf->codes[138] = 0x000000001ef28c95UL;
   tf->codes[139] = 0x0000000bf0d9792dUL;
   tf->codes[140] = 0x00000006d27557c3UL;
   tf->codes[141] = 0x0000000623f977f4UL;
   tf->codes[142] = 0x000000035b43be57UL;
   tf->codes[143] = 0x0000000bb0c428d5UL;
   tf->codes[144] = 0x0000000a6f01474dUL;
   tf->codes[145] = 0x00000005a70c9749UL;
   tf->codes[146] = 0x000000020ddabc3bUL;
   tf->codes[147] = 0x00000002eabd78cfUL;
   tf->codes[148] = 0x000000090aa18f88UL;
   tf->codes[149] = 0x0000000a9ea89350UL;
   tf->codes[150] = 0x00000003cdb39b22UL;
   tf->codes[151] = 0x0000000839a08f34UL;
   tf->codes[152] = 0x0000000169bb814eUL;
   tf->codes[153] = 0x00000001a575ab08UL;
   tf->codes[154] = 0x0000000a04d3d5a2UL;
   tf->codes[155] = 0x0000000bf7902f2bUL;
   tf->codes[156] = 0x0000000095a5e65cUL;
   tf->codes[157] = 0x000000092e8fce94UL;
   tf->codes[158] = 0x000000067ef48d12UL;
   tf->codes[159] = 0x00000006400dbcacUL;
   tf->codes[160] = 0x0000000b12d8fb9fUL;
   tf->codes[161] = 0x00000000347f45d3UL;
   tf->codes[162] = 0x0000000b35826f56UL;
   tf->codes[163] = 0x0000000c546ac6e4UL;
   tf->codes[164] = 0x000000081cc35b66UL;
   tf->codes[165] = 0x000000041d14bd57UL;
   tf->codes[166] = 0x00000000c052b168UL;
   tf->codes[167] = 0x00000007d6ce5018UL;
   tf->codes[168] = 0x0000000ab4ed5edeUL;
   tf->codes[169] = 0x00000005af817119UL;
   tf->codes[170] = 0x0000000d1454b182UL;
   tf->codes[171] = 0x00000002badb090bUL;
   tf->codes[172] = 0x000000003fcb4c0cUL;
   tf->codes[173] = 0x00000002f1c28fd8UL;
   tf->codes[174] = 0x000000093608c6f7UL;
   tf->codes[175] = 0x00000004c93ba2b5UL;
   tf->codes[176] = 0x000000007d950a5dUL;
   tf->codes[177] = 0x0000000e54b3d3fcUL;
   tf->codes[178] = 0x000000015560cf9dUL;
   tf->codes[179] = 0x0000000189e4958aUL;
   tf->codes[180] = 0x000000062140e9d2UL;
   tf->codes[181] = 0x0000000723bc1cdbUL;
   tf->codes[182] = 0x00000002063f26faUL;
   tf->codes[183] = 0x0000000fa08ab19fUL;
   tf->codes[184] = 0x00000007955641dbUL;
   tf->codes[185] = 0x0000000646b01daaUL;
   tf->codes[186] = 0x000000071cd427ccUL;
   tf->codes[187] = 0x000000009a42f7d4UL;
   tf->codes[188] = 0x0000000717edc643UL;
   tf->codes[189] = 0x000000015eb94367UL;
   tf->codes[190] = 0x00000008392e6bb2UL;
   tf->codes[191] = 0x0000000832408542UL;
   tf->codes[192] = 0x00000002b9b874beUL;
   tf->codes[193] = 0x0000000b21f4730dUL;
   tf->codes[194] = 0x0000000b5d8f24c9UL;
   tf->codes[195] = 0x00000007dbaf6931UL;
   tf->codes[196] = 0x00000001b4e33629UL;
   tf->codes[197] = 0x000000013452e710UL;
   tf->codes[198] = 0x0000000e974af612UL;
   tf->codes[199] = 0x00000001df61d29aUL;
   tf->codes[200] = 0x000000099f2532adUL;
   tf->codes[201] = 0x0000000e50ec71b4UL;
   tf->codes[202] = 0x00000005df0a36e8UL;
   tf->codes[203] = 0x00000004934e4ceaUL;
   tf->codes[204] = 0x0000000e34a0b4bdUL;
   tf->codes[205] = 0x0000000b7b26b588UL;
   tf->codes[206] = 0x00000000f255118dUL;
   tf->codes[207] = 0x0000000d0c8fa31eUL;
   tf->codes[208] = 0x000000006a50c94fUL;
   tf->codes[209] = 0x0000000f28aa9f06UL;
   tf->codes[210] = 0x0000000131d194d8UL;
   tf->codes[211] = 0x0000000622e3da79UL;
   tf->codes[212] = 0x0000000ac7478303UL;
   tf->codes[213] = 0x0000000c8f2521d7UL;
   tf->codes[214] = 0x00000006c9c881f5UL;
   tf->codes[215] = 0x000000049e38b60aUL;
   tf->codes[216] = 0x0000000513d8df65UL;
   tf->codes[217] = 0x0000000d7c2b0785UL;
   tf->codes[218] = 0x00000009f6f9d75aUL;
   tf->codes[219] = 0x00000009f6966020UL;
   tf->codes[220] = 0x00000001e1a54e33UL;
   tf->codes[221] = 0x0000000c04d63419UL;
   tf->codes[222] = 0x0000000946e04cd7UL;
   tf->codes[223] = 0x00000001bdac5902UL;
   tf->codes[224] = 0x000000056469b830UL;
   tf->codes[225] = 0x0000000ffad59569UL;
   tf->codes[226] = 0x000000086970e7d8UL;
   tf->codes[227] = 0x00000008a4b41e12UL;
   tf->codes[228] = 0x0000000ad4688e3bUL;
   tf->codes[229] = 0x000000085f8f5df4UL;
   tf->codes[230] = 0x0000000d833a0893UL;
   tf->codes[231] = 0x00000002a36fdd7cUL;
   tf->codes[232] = 0x0000000d6a857cf2UL;
   tf->codes[233] = 0x00000008829bc35cUL;
   tf->codes[234] = 0x00000005e50d79bcUL;
   tf->codes[235] = 0x0000000fbb8035e4UL;
   tf->codes[236] = 0x0000000c1a95bebfUL;
   tf->codes[237] = 0x0000000036b0baf8UL;
   tf->codes[238] = 0x0000000e0da964eaUL;
   tf->codes[239] = 0x0000000b6483689bUL;
   tf->codes[240] = 0x00000007c8e2f4c1UL;
   tf->codes[241] = 0x00000005b856a23bUL;
   tf->codes[242] = 0x00000002fc183995UL;
   tf->codes[243] = 0x0000000e914b6d70UL;
   tf->codes[244] = 0x0000000b31041969UL;
   tf->codes[245] = 0x00000001bb478493UL;
   tf->codes[246] = 0x0000000063e2b456UL;
   tf->codes[247] = 0x0000000f2a082b9cUL;
   tf->codes[248] = 0x00000008e5e646eaUL;
   tf->codes[249] = 0x000000008172f8f6UL;
   tf->codes[250] = 0x00000000dacd923eUL;
   tf->codes[251] = 0x0000000e5dcf0e2eUL;
   tf->codes[252] = 0x0000000bf9446baeUL;
   tf->codes[253] = 0x00000004822d50d1UL;
   tf->codes[254] = 0x000000026e710bf5UL;
   tf->codes[255] = 0x0000000b90ba2a24UL;
   tf->codes[256] = 0x0000000f3b25aa73UL;
   tf->codes[257] = 0x0000000809ad589bUL;
   tf->codes[258] = 0x000000094cc1e254UL;
   tf->codes[259] = 0x00000005334a3adbUL;
   tf->codes[260] = 0x0000000592886b2fUL;
   tf->codes[261] = 0x0000000bf64704aaUL;
   tf->codes[262] = 0x0000000566dbf24cUL;
   tf->codes[263] = 0x000000072203e692UL;
   tf->codes[264] = 0x000000064e61e809UL;
   tf->codes[265] = 0x0000000d7259aad6UL;
   tf->codes[266] = 0x00000007b924aedcUL;
   tf->codes[267] = 0x00000002df2184e8UL;
   tf->codes[268] = 0x0000000353d1eca7UL;
   tf->codes[269] = 0x0000000fce30d7ceUL;
   tf->codes[270] = 0x0000000f7b0f436eUL;
   tf->codes[271] = 0x000000057e8d8f68UL;
   tf->codes[272] = 0x00000008c79e60dbUL;
   tf->codes[273] = 0x00000009c8362b2bUL;
   tf->codes[274] = 0x000000063a5804f2UL;
   tf->codes[275] = 0x00000009298353dcUL;
   tf->codes[276] = 0x00000006f98a71c8UL;
   tf->codes[277] = 0x0000000a5731f693UL;
   tf->codes[278] = 0x000000021ca5c870UL;
   tf->codes[279] = 0x00000001c2107fd3UL;
   tf->codes[280] = 0x00000006181f6c39UL;
   tf->codes[281] = 0x000000019e574304UL;
   tf->codes[282] = 0x0000000329937606UL;
   tf->codes[283] = 0x0000000043d5c70dUL;
   tf->codes[284] = 0x00000009b18ff162UL;
   tf->codes[285] = 0x00000008e2ccfebfUL;
   tf->codes[286] = 0x000000072b7b9b54UL;
   tf->codes[287] = 0x00000009b71f4f3cUL;
   tf->codes[288] = 0x0000000935d7393eUL;
   tf->codes[289] = 0x000000065938881aUL;
   tf->codes[290] = 0x00000006a5bd6f2dUL;
   tf->codes[291] = 0x0000000a19783306UL;
   tf->codes[292] = 0x0000000e6472f4d7UL;
   tf->codes[293] = 0x000000081163df5aUL;
   tf->codes[294] = 0x0000000a838e1cbdUL;
   tf->codes[295] = 0x0000000982748477UL;
   tf->codes[296] = 0x0000000050c54febUL;
   tf->codes[297] = 0x00000000d82fbb58UL;
   tf->codes[298] = 0x00000002c4c72799UL;
   tf->codes[299] = 0x000000097d259ad6UL;
   tf->codes[300] = 0x000000022d9a43edUL;
   tf->codes[301] = 0x0000000fdb162a9fUL;
   tf->codes[302] = 0x00000000cb4a727dUL;
   tf->codes[303] = 0x00000004fae2e371UL;
   tf->codes[304] = 0x0000000535b5be8bUL;
   tf->codes[305] = 0x000000048795908aUL;
   tf->codes[306] = 0x0000000ce7c18962UL;
   tf->codes[307] = 0x00000004ea154d80UL;
   tf->codes[308] = 0x000000050c064889UL;
   tf->codes[309] = 0x00000008d97fc75dUL;
   tf->codes[310] = 0x0000000c8bd9ec61UL;
   tf->codes[311] = 0x000000083ee8e8bbUL;
   tf->codes[312] = 0x0000000c8431419aUL;
   tf->codes[313] = 0x00000001aa78079dUL;
   tf->codes[314] = 0x00000008111aa4a5UL;
   tf->codes[315] = 0x0000000dfa3a69feUL;
   tf->codes[316] = 0x000000051630d83fUL;
   tf->codes[317] = 0x00000002d930fb3fUL;
   tf->codes[318] = 0x00000002133116e5UL;
   tf->codes[319] = 0x0000000ae5395522UL;
   tf->codes[320] = 0x0000000bc07a4e8aUL;
   tf->codes[321] = 0x000000057bf08ba0UL;
   tf->codes[322] = 0x00000006cb18036aUL;
   tf->codes[323] = 0x0000000f0e2e4b75UL;
   tf->codes[324] = 0x00000003eb692b6fUL;
   tf->codes[325] = 0x0000000d8178a3faUL;
   tf->codes[326] = 0x0000000238cce6a6UL;
   tf->codes[327] = 0x0000000e97d5cdd7UL;
   tf->codes[328] = 0x0000000fe10d8d5eUL;
   tf->codes[329] = 0x0000000b39584a1dUL;
   tf->codes[330] = 0x0000000ca03536fdUL;
   tf->codes[331] = 0x0000000aa61f3998UL;
   tf->codes[332] = 0x000000072ff23ec2UL;
   tf->codes[333] = 0x000000015aa7d770UL;
   tf->codes[334] = 0x000000057a3a1282UL;
   tf->codes[335] = 0x0000000d1f3902dcUL;
   tf->codes[336] = 0x00000006554c9388UL;
   tf->codes[337] = 0x0000000fd01283c7UL;
   tf->codes[338] = 0x0000000e8baa42c5UL;
   tf->codes[339] = 0x000000072cee6adfUL;
   tf->codes[340] = 0x0000000f6614b3faUL;
   tf->codes[341] = 0x000000095c3778a2UL;
   tf->codes[342] = 0x00000007da4cea7aUL;
   tf->codes[343] = 0x0000000d18a5912cUL;
   tf->codes[344] = 0x0000000d116426e5UL;
   tf->codes[345] = 0x000000027c17bc1cUL;
   tf->codes[346] = 0x0000000b95b53bc1UL;
   tf->codes[347] = 0x0000000c8f937a05UL;
   tf->codes[348] = 0x0000000ed220c9bdUL;
   tf->codes[349] = 0x00000000c97d72abUL;
   tf->codes[350] = 0x00000008fb1217aeUL;
   tf->codes[351] = 0x000000025ca8a5a1UL;
   tf->codes[352] = 0x0000000b261b871bUL;
   tf->codes[353] = 0x00000001bef0a056UL;
   tf->codes[354] = 0x0000000806a51179UL;
   tf->codes[355] = 0x0000000eed249145UL;
   tf->codes[356] = 0x00000003f82aecebUL;
   tf->codes[357] = 0x0000000cc56e9acfUL;
   tf->codes[358] = 0x00000002e78d01ebUL;
   tf->codes[359] = 0x0000000102cee17fUL;
   tf->codes[360] = 0x000000037caad3d5UL;
   tf->codes[361] = 0x000000016ac5b1eeUL;
   tf->codes[362] = 0x00000002af164eceUL;
   tf->codes[363] = 0x0000000d4cd81dc9UL;
   tf->codes[364] = 0x000000012263a7e7UL;
   tf->codes[365] = 0x000000057ac7d117UL;
   tf->codes[366] = 0x00000009391d9740UL;
   tf->codes[367] = 0x00000007aedaa77fUL;
   tf->codes[368] = 0x00000009675a3c72UL;
   tf->codes[369] = 0x0000000277f25191UL;
   tf->codes[370] = 0x0000000ebb6e64b9UL;
   tf->codes[371] = 0x00000007ad3ef747UL;
   tf->codes[372] = 0x000000012759b181UL;
   tf->codes[373] = 0x0000000948257d4dUL;
   tf->codes[374] = 0x0000000b63a850f6UL;
   tf->codes[375] = 0x00000003a52a8f75UL;
   tf->codes[376] = 0x00000004a019532cUL;
   tf->codes[377] = 0x0000000a021a7529UL;
   tf->codes[378] = 0x0000000cc661876dUL;
   tf->codes[379] = 0x00000004085afd05UL;
   tf->codes[380] = 0x0000000e7048e089UL;
   tf->codes[381] = 0x00000003f979cdc6UL;
   tf->codes[382] = 0x0000000d9da9071bUL;
   tf->codes[383] = 0x0000000ed2fc5b68UL;
   tf->codes[384] = 0x000000079d64c3a1UL;
   tf->codes[385] = 0x0000000fd44e2361UL;
   tf->codes[386] = 0x00000008eea46a74UL;
   tf->codes[387] = 0x000000042233b9c2UL;
   tf->codes[388] = 0x0000000ae4d1765dUL;
   tf->codes[389] = 0x00000007303a094cUL;
   tf->codes[390] = 0x00000002d7033abeUL;
   tf->codes[391] = 0x00000003dcc2b0b4UL;
   tf->codes[392] = 0x00000000f0967d09UL;
   tf->codes[393] = 0x000000006f0cd7deUL;
   tf->codes[394] = 0x000000009807aca0UL;
   tf->codes[395] = 0x00000003a295cad3UL;
   tf->codes[396] = 0x00000002b106b202UL;
   tf->codes[397] = 0x00000003f38a828eUL;
   tf->codes[398] = 0x000000078af46596UL;
   tf->codes[399] = 0x0000000bda2dc713UL;
   tf->codes[400] = 0x00000009a8c8c9d9UL;
   tf->codes[401] = 0x00000006a0f2ddceUL;
   tf->codes[402] = 0x0000000a76af6fe2UL;
   tf->codes[403] = 0x0000000086f66fa4UL;
   tf->codes[404] = 0x0000000d52d63f8dUL;
   tf->codes[405] = 0x000000089f7a6e73UL;
   tf->codes[406] = 0x0000000cc6b23362UL;
   tf->codes[407] = 0x0000000b4ebf3c39UL;
   tf->codes[408] = 0x0000000564f300faUL;
   tf->codes[409] = 0x0000000e8de3a706UL;
   tf->codes[410] = 0x000000079a033b61UL;
   tf->codes[411] = 0x0000000765e160c5UL;
   tf->codes[412] = 0x0000000a266a4f85UL;
   tf->codes[413] = 0x0000000a68c38c24UL;
   tf->codes[414] = 0x0000000dca0711fbUL;
   tf->codes[415] = 0x000000085fba85baUL;
   tf->codes[416] = 0x000000037a207b46UL;
   tf->codes[417] = 0x0000000158fcc4d0UL;
   tf->codes[418] = 0x00000000569d79b3UL;
   tf->codes[419] = 0x00000007b1a25555UL;
   tf->codes[420] = 0x0000000a8ae22468UL;
   tf->codes[421] = 0x00000007c592bdfdUL;
   tf->codes[422] = 0x00000000c59a5f66UL;
   tf->codes[423] = 0x0000000b1115daa3UL;
   tf->codes[424] = 0x0000000f17c87177UL;
   tf->codes[425] = 0x00000006769d766bUL;
   tf->codes[426] = 0x00000002b637356dUL;
   tf->codes[427] = 0x000000013d8685acUL;
   tf->codes[428] = 0x0000000f24cb6ec0UL;
   tf->codes[429] = 0x00000000bd0b56d1UL;
   tf->codes[430] = 0x000000042ff0e26dUL;
   tf->codes[431] = 0x0000000b41609267UL;
   tf->codes[432] = 0x000000096f9518afUL;
   tf->codes[433] = 0x0000000c56f96636UL;
   tf->codes[434] = 0x00000004a8e10349UL;
   tf->codes[435] = 0x0000000863512171UL;
   tf->codes[436] = 0x0000000ea455d86cUL;
   tf->codes[437] = 0x0000000bd0e25279UL;
   tf->codes[438] = 0x0000000e65e3f761UL;
   tf->codes[439] = 0x000000036c84a922UL;
   tf->codes[440] = 0x000000085fd1b38fUL;
   tf->codes[441] = 0x0000000657c91539UL;
   tf->codes[442] = 0x000000015033fe04UL;
   tf->codes[443] = 0x000000009051c921UL;
   tf->codes[444] = 0x0000000ab27d80d8UL;
   tf->codes[445] = 0x0000000f92f7d0a1UL;
   tf->codes[446] = 0x00000008eb6bb737UL;
   tf->codes[447] = 0x000000010b5b0f63UL;
   tf->codes[448] = 0x00000006c9c7ad63UL;
   tf->codes[449] = 0x0000000f66fe70aeUL;
   tf->codes[450] = 0x0000000ca579bd92UL;
   tf->codes[451] = 0x0000000956198e4dUL;
   tf->codes[452] = 0x000000029e4405e5UL;
   tf->codes[453] = 0x0000000e44eb885cUL;
   tf->codes[454] = 0x000000041612456cUL;
   tf->codes[455] = 0x0000000ea45e0abfUL;
   tf->codes[456] = 0x0000000d326529bdUL;
   tf->codes[457] = 0x00000007b2c33cefUL;
   tf->codes[458] = 0x000000080bc9b558UL;
   tf->codes[459] = 0x00000007169b9740UL;
   tf->codes[460] = 0x0000000c37f99209UL;
   tf->codes[461] = 0x000000031ff6dab9UL;
   tf->codes[462] = 0x0000000c795190edUL;
   tf->codes[463] = 0x0000000a7636e95fUL;
   tf->codes[464] = 0x00000009df075841UL;
   tf->codes[465] = 0x000000055a083932UL;
   tf->codes[466] = 0x0000000a7cbdf630UL;
   tf->codes[467] = 0x0000000409ea4ef0UL;
   tf->codes[468] = 0x000000092a1991b6UL;
   tf->codes[469] = 0x00000004b078dee9UL;
   tf->codes[470] = 0x0000000ae18ce9e4UL;
   tf->codes[471] = 0x00000005a6e1ef35UL;
   tf->codes[472] = 0x00000001a403bd59UL;
   tf->codes[473] = 0x000000031ea70a83UL;
   tf->codes[474] = 0x00000002bc3c4f3aUL;
   tf->codes[475] = 0x00000005c921b3cbUL;
   tf->codes[476] = 0x0000000042da05c5UL;
   tf->codes[477] = 0x00000001f667d16bUL;
   tf->codes[478] = 0x0000000416a368cfUL;
   tf->codes[479] = 0x0000000fbc0a7a3bUL;
   tf->codes[480] = 0x00000009419f0c7cUL;
   tf->codes[481] = 0x000000081be2fa03UL;
   tf->codes[482] = 0x000000034e2c172fUL;
   tf->codes[483] = 0x000000028648d8aeUL;
   tf->codes[484] = 0x0000000c7acbb885UL;
   tf->codes[485] = 0x000000045f31eb6aUL;
   tf->codes[486] = 0x0000000d1cfc0a7bUL;
   tf->codes[487] = 0x000000042c4d260dUL;
   tf->codes[488] = 0x0000000cf6584097UL;
   tf->codes[489] = 0x000000094b132b14UL;
   tf->codes[490] = 0x00000003c5c5df75UL;
   tf->codes[491] = 0x00000008ae596fefUL;
   tf->codes[492] = 0x0000000aea8054ebUL;
   tf->codes[493] = 0x00000000ae9cc573UL;
   tf->codes[494] = 0x0000000496fb731bUL;
   tf->codes[495] = 0x0000000ebf105662UL;
   tf->codes[496] = 0x0000000af9c83a37UL;
   tf->codes[497] = 0x0000000c0d64cd6bUL;
   tf->codes[498] = 0x00000007b608159aUL;
   tf->codes[499] = 0x0000000e74431642UL;
   tf->codes[500] = 0x0000000d6fb9d900UL;
   tf->codes[501] = 0x0000000291e99de0UL;
   tf->codes[502] = 0x000000010500ba9aUL;
   tf->codes[503] = 0x00000005cd05d037UL;
   tf->codes[504] = 0x0000000a87254fb2UL;
   tf->codes[505] = 0x00000009d7824a37UL;
   tf->codes[506] = 0x00000008b2c7b47cUL;
   tf->codes[507] = 0x000000030c788145UL;
   tf->codes[508] = 0x00000002f4e5a8beUL;
   tf->codes[509] = 0x0000000badb884daUL;
   tf->codes[510] = 0x0000000026e0d5c9UL;
   tf->codes[511] = 0x00000006fdbaa32eUL;
   tf->codes[512] = 0x000000034758eb31UL;
   tf->codes[513] = 0x0000000565cd1b4fUL;
   tf->codes[514] = 0x00000002bfd90fb0UL;
   tf->codes[515] = 0x0000000093052a6bUL;
   tf->codes[516] = 0x0000000d3c13c4b9UL;
   tf->codes[517] = 0x00000002daea43bfUL;
   tf->codes[518] = 0x0000000a279762bcUL;
   tf->codes[519] = 0x0000000f1bd9f22cUL;
   tf->codes[520] = 0x00000004b7fec94fUL;
   tf->codes[521] = 0x0000000545761d5aUL;
   tf->codes[522] = 0x00000007327df411UL;
   tf->codes[523] = 0x00000001b52a442eUL;
   tf->codes[524] = 0x000000049b0ce108UL;
   tf->codes[525] = 0x000000024c764bc8UL;
   tf->codes[526] = 0x0000000374563045UL;
   tf->codes[527] = 0x0000000a3e8f91c6UL;
   tf->codes[528] = 0x00000000e6bd2241UL;
   tf->codes[529] = 0x0000000e0e52ee3cUL;
   tf->codes[530] = 0x000000007e8e3caaUL;
   tf->codes[531] = 0x000000096c2b7372UL;
   tf->codes[532] = 0x000000033acbdfdaUL;
   tf->codes[533] = 0x0000000b15d91e54UL;
   tf->codes[534] = 0x0000000464759ac1UL;
   tf->codes[535] = 0x00000006886a1998UL;
   tf->codes[536] = 0x000000057f5d3958UL;
   tf->codes[537] = 0x00000005a1f5c1f5UL;
   tf->codes[538] = 0x00000000b58158adUL;
   tf->codes[539] = 0x0000000e712053fbUL;
   tf->codes[540] = 0x00000005352ddb25UL;
   tf->codes[541] = 0x0000000414b98ea0UL;
   tf->codes[542] = 0x000000074f89f546UL;
   tf->codes[543] = 0x000000038a56b3c3UL;
   tf->codes[544] = 0x000000038db0dc17UL;
   tf->codes[545] = 0x0000000aa016a755UL;
   tf->codes[546] = 0x0000000dc72366f5UL;
   tf->codes[547] = 0x00000000cee93d75UL;
   tf->codes[548] = 0x0000000b2fe7a56bUL;
   tf->codes[549] = 0x0000000a847ed390UL;
   tf->codes[550] = 0x00000008713ef88cUL;
   tf->codes[551] = 0x0000000a217cc861UL;
   tf->codes[552] = 0x00000008bca25d7bUL;
   tf->codes[553] = 0x0000000455526818UL;
   tf->codes[554] = 0x0000000ea3a7a180UL;
   tf->codes[555] = 0x0000000a9536e5e0UL;
   tf->codes[556] = 0x00000009b64a1975UL;
   tf->codes[557] = 0x00000005bfc756bcUL;
   tf->codes[558] = 0x0000000046aa169bUL;
   tf->codes[559] = 0x000000053a17f76fUL;
   tf->codes[560] = 0x00000004d6815274UL;
   tf->codes[561] = 0x0000000cca9cf3f6UL;
   tf->codes[562] = 0x00000004013fcb8bUL;
   tf->codes[563] = 0x00000003d26cdfa5UL;
   tf->codes[564] = 0x00000005786231f7UL;
   tf->codes[565] = 0x00000007d4ab09abUL;
   tf->codes[566] = 0x0000000960b5ffbcUL;
   tf->codes[567] = 0x00000008914df0d4UL;
   tf->codes[568] = 0x00000002fc6f2213UL;
   tf->codes[569] = 0x0000000ac235637eUL;
   tf->codes[570] = 0x0000000151b28ed3UL;
   tf->codes[571] = 0x000000046f79b6dbUL;
   tf->codes[572] = 0x00000001382e0c9fUL;
   tf->codes[573] = 0x000000053abf983aUL;
   tf->codes[574] = 0x0000000383c47adeUL;
   tf->codes[575] = 0x00000003fcf88978UL;
   tf->codes[576] = 0x0000000eb9079df7UL;
   tf->codes[577] = 0x000000009af0714dUL;
   tf->codes[578] = 0x0000000da19d1bb7UL;
   tf->codes[579] = 0x00000009a02749f8UL;
   tf->codes[580] = 0x00000001c62dab9bUL;
   tf->codes[581] = 0x00000001a137e44bUL;
   tf->codes[582] = 0x00000002867718c7UL;
   tf->codes[583] = 0x000000035815525bUL;
   tf->codes[584] = 0x00000007cd35c550UL;
   tf->codes[585] = 0x00000002164f73a0UL;
   tf->codes[586] = 0x0000000e8b772fe0UL;
   tf->nbits = 36;
   tf->bit_x = (uint32_t *)calloc(36, sizeof(uint32_t));
   tf->bit_y = (uint32_t *)calloc(36, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 4;
   tf->bit_y[3] = 1;
   tf->bit_x[4] = 5;
   tf->bit_y[4] = 1;
   tf->bit_x[5] = 2;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 3;
   tf->bit_y[6] = 2;
   tf->bit_x[7] = 4;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 3;
   tf->bit_y[8] = 3;
   tf->bit_x[9] = 6;
   tf->bit_y[9] = 1;
   tf->bit_x[10] = 6;
   tf->bit_y[10] = 2;
   tf->bit_x[11] = 6;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 6;
   tf->bit_y[12] = 4;
   tf->bit_x[13] = 6;
   tf->bit_y[13] = 5;
   tf->bit_x[14] = 5;
   tf->bit_y[14] = 2;
   tf->bit_x[15] = 5;
   tf->bit_y[15] = 3;
   tf->bit_x[16] = 5;
   tf->bit_y[16] = 4;
   tf->bit_x[17] = 4;
   tf->bit_y[17] = 3;
   tf->bit_x[18] = 6;
   tf->bit_y[18] = 6;
   tf->bit_x[19] = 5;
   tf->bit_y[19] = 6;
   tf->bit_x[20] = 4;
   tf->bit_y[20] = 6;
   tf->bit_x[21] = 3;
   tf->bit_y[21] = 6;
   tf->bit_x[22] = 2;
   tf->bit_y[22] = 6;
   tf->bit_x[23] = 5;
   tf->bit_y[23] = 5;
   tf->bit_x[24] = 4;
   tf->bit_y[24] = 5;
   tf->bit_x[25] = 3;
   tf->bit_y[25] = 5;
   tf->bit_x[26] = 4;
   tf->bit_y[26] = 4;
   tf->bit_x[27] = 1;
   tf->bit_y[27] = 6;
   tf->bit_x[28] = 1;
   tf->bit_y[28] = 5;
   tf->bit_x[29] = 1;
   tf->bit_y[29] = 4;
   tf->bit_x[30] = 1;
   tf->bit_y[30] = 3;
   tf->bit_x[31] = 1;
   tf->bit_y[31] = 2;
   tf->bit_x[32] = 2;
   tf->bit_y[32] = 5;
   tf->bit_x[33] = 2;
   tf->bit_y[33] = 4;
   tf->bit_x[34] = 2;
   tf->bit_y[34] = 3;
   tf->bit_x[35] = 3;
   tf->bit_y[35] = 4;
   tf->width_at_border = 8;
   tf->total_width = 10;
   tf->reversed_border = false;
   return tf;
}

void tag36h11_destroy(apriltag_family_t *tf)
{
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
