#pragma once

#include "usrlist.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define USING_WQ_NAME 0

struct worknode;
typedef void (*work_func_t)(struct worknode* work);
struct worknode {
    struct list_head ws_node;
    work_func_t      func;
};

struct workqueue;
typedef void (*wq_trig)(struct workqueue* work);

struct workqueue {
    struct list_head ws_head;
    wq_trig          trig;
#if USING_WQ_NAME
    char names[16];
#endif
};

int  workqueue_once(struct workqueue* wq);
void workqueue_post(struct workqueue* wq, struct worknode* pnode);
void workqueue_add_new_nolock(struct workqueue* wq, struct worknode* pnode);
void workqueue_trig_once(struct workqueue* wq);

#ifdef __cplusplus
}
#endif /* __cplusplus */
