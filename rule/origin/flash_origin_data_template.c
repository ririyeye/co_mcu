#include "flash_origin.h"

/* 自动生成文件 - 请勿手动修改 */
const struct origin _origin[] = {
{
    .name   = "boot",             /* 分区1 */
    .length = 32 * 1024,           /* 32K */
    .offset = 0,                   /* auto */
},
{
    .name   = "app",              /* 分区2 */
    .length = 96 * 1024,           /* - */
    .offset = 32 * 1024,           /* - */
}
};

const struct flash _flash = {
    .addr         = 0x08000000,        /* 0x08000000 */
    .length       = 128 * 1024,        /* 128K */
    .p_origin     = _origin,
    .origin_count = sizeof(_origin) / sizeof(_origin[0]),
};
