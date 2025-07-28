
#include "flash_origin.h"
#include <string.h>

extern const struct flash _flash;

const struct flash* get_flash_info(void)
{
    return &_flash;
}

const struct origin* get_origin_info(const struct flash* flash, const char* name)
{
    for (size_t i = 0; i < flash->origin_count; i++) {
        if (!strcmp(flash->p_origin[i].name, name)) {
            return &flash->p_origin[i];
        }
    }
    return NULL;
}
