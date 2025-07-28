
#include <stdint.h>

struct origin {
    const char* name;
    uint32_t    length;
    uint32_t    offset;
};

struct flash {
    uint32_t             addr;
    uint32_t             length;
    const struct origin* p_origin;
    uint32_t             origin_count;
};

const struct flash*  get_flash_info(void);
const struct origin* get_origin_info(const struct flash* flash, const char* name);
