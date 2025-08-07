#include "workqueue.h"
#include "lock.h"

void workqueue_post(struct workqueue* wq, struct worknode* pnode)
{
    if (!wq || !pnode) {
        return;
    }
    uint32_t lk = lock_acquire();
    list_del(&pnode->ws_node);
    list_add_tail(&pnode->ws_node, &wq->ws_head);
    lock_release(lk);
    if (wq->trig) {
        wq->trig(wq);
    }
}

void workqueue_add_new_nolock(struct workqueue* wq, struct worknode* pnode)
{
    if (!wq || !pnode) {
        return;
    }
    list_del(&pnode->ws_node);
    list_add_tail(&pnode->ws_node, &wq->ws_head);
}

void workqueue_trig_once(struct workqueue* wq)
{
    if (!wq) {
        return;
    }
    if (wq->trig) {
        wq->trig(wq);
    }
}

static struct worknode* get_work_node(struct workqueue* wq)
{
    struct worknode* pnod = NULL;
    if (list_empty(&wq->ws_head)) {
        pnod = NULL;
    } else {
        pnod = list_first_entry(&wq->ws_head, struct worknode, ws_node);
        list_del(&pnod->ws_node);
    }
    return pnod;
}

int workqueue_once(struct workqueue* wq)
{
    uint32_t         lk   = lock_acquire();
    struct worknode* pnod = get_work_node(wq);
    lock_release(lk);

    if (pnod) {
        if (pnod->func) {
            pnod->func(pnod);
        }
        return 1;
    }
    return 0;
}
