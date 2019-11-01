
#include "c_fun.hpp"

uint32_t PSP_find_edges_in_bimg(uint8_t *p_img,
                                uint32_t *p_img_w,
                                uint32_t *p_img_h,
                                uint8_t is_4)
{
    register uint8_t *p_img_tmp;
    register uint32_t img_size = *p_img_w * *p_img_h;
    p_img_tmp = (uint8_t    *) malloc(sizeof(uint8_t) * img_size);
    memcpy(p_img_tmp, p_img, sizeof(uint8_t) * img_size);

    uint32_t i,j;
    for( i=1; i < p_img_h[0]-1; ++i)
    {
        for ( j = 1; j < p_img_w[0]-1; ++j)
        {
            if (*(p_img_tmp + j + i * p_img_w[0]) == 1)
            {
                if ( *(p_img_tmp + (j-1) + i * p_img_w[0]) & //left
                     *(p_img_tmp + (j+1) + i * p_img_w[0]) )  //bottom  //when left == right, go on
                {
                    *(p_img + j + i * p_img_w[0]) = 0;
                }
            }
        }
    }
    free(p_img_tmp);
    return 1;
}
void test_fun()
{

}
