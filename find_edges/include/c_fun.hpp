//
// Created by lx on 19-11-1.
//

#ifndef C_FUN_H
#define C_FUN_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef  unsigned char		uint8_t;
typedef unsigned int		uint32_t;



uint32_t PSP_find_edges_in_bimg(uint8_t *p_img,
                                uint32_t *p_img_w,
                                uint32_t *p_img_h,
                                uint8_t is_4);

void test_fun();
#endif //FIND_EDGES_C_FUN_H
