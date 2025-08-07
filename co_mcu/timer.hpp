#pragma once
#include "co_base/task.hpp"
#include "lock.h"
#include "worker.hpp"
#include "workqueue.h"
#include <chrono>

namespace co_mcu {

struct Delayed_worknode : worknode {
    std::chrono::steady_clock::time_point expire;
    Delayed_worknode*                     child   = nullptr;
    Delayed_worknode*                     sibling = nullptr;
};

struct Timer_check_queue : worknode {
private:
    workqueue&        _executor;
    Delayed_worknode* heap_root;

    /**
     * @brief 合并两个最小堆（pairing heap）的根节点，返回新的根节点
     *
     * 算法说明：
     * 1. 如果任一堆为空，则直接返回另一个堆根。
     * 2. 比较 a->expire 和 b->expire，较小（更早到期）的节点作为新根。
     * 3. 将另一节点作为子堆插入到新根的 child 链表首位：
     *    - 当 a 是新根时，b->sibling 指向 a->child，a->child 更新为 b。
     *    - 当 b 是新根时，a->sibling 指向 b->child，b->child 更新为 a。
     * 4. 返回合并后的新根，保持 pairing heap 的结构。
     */
    static Delayed_worknode* heap_meld(Delayed_worknode* a, Delayed_worknode* b) noexcept
    {
        if (!a)
            return b;
        if (!b)
            return a;
        if (a->expire <= b->expire) {
            b->sibling = a->child;
            a->child   = b;
            return a;
        } else {
            a->sibling = b->child;
            b->child   = a;
            return b;
        }
    }

    /**
     * @brief 对 pairing heap 的子节点链表执行两遍合并（two-pass merge），返回新的子堆根
     *
     * 算法说明：
     * 1. 如果节点为空或没有 sibling，则该链表只有一个子堆，直接返回该节点。
     * 2. 将链表分为三部分：a（第一个节点）、b（第二个节点）、rest（从第三个节点开始的链表）。
     * 3. 将 a->sibling 和 b->sibling 清空，使它们成为独立的单节点堆。
     * 4. 先用 heap_meld 合并 a 和 b，得到中间堆 root1；
     * 5. 递归对 rest 调用 two_pass_merge，得到中间堆 root2；
     * 6. 最后用 heap_meld 合并 root1 和 root2，并返回最终堆根。
     */
    static Delayed_worknode* two_pass_merge(Delayed_worknode* node) noexcept
    {
        if (!node || !node->sibling)
            return node;
        Delayed_worknode* a    = node;
        Delayed_worknode* b    = a->sibling;
        Delayed_worknode* rest = b->sibling;
        a->sibling             = nullptr;
        b->sibling             = nullptr;
        return heap_meld(heap_meld(a, b), two_pass_merge(rest));
    }

    // 删除最小元素（根）并返回新的堆
    static Delayed_worknode* delete_min(Delayed_worknode* root) noexcept { return two_pass_merge(root->child); }

    void tim_chk_cb()
    {
        auto     now      = std::chrono::steady_clock::now();
        int      trig_flg = 0;
        uint32_t lk       = lock_acquire();
        while (heap_root && heap_root->expire <= now) {
            Delayed_worknode* dpos = heap_root;
            heap_root              = delete_min(heap_root);
            workqueue_add_new_nolock(&_executor, dpos);
            trig_flg = 1;
        }
        lock_release(lk);
        if (trig_flg) {
            workqueue_trig_once(&_executor);
        }
    }

public:
    explicit Timer_check_queue(workqueue& executor) : _executor(executor)
    {
        INIT_LIST_HEAD(&ws_node);
        heap_root = nullptr;
        func      = [](struct worknode* work) {
            Timer_check_queue* tcq = static_cast<Timer_check_queue*>(work);
            tcq->tim_chk_cb();
        };
    }

    void post_delayed_work(Delayed_worknode* dwork, uint32_t ms)
    {
        auto now       = std::chrono::steady_clock::now();
        dwork->expire  = now + std::chrono::milliseconds(ms);
        dwork->child   = nullptr;
        dwork->sibling = nullptr;

        uint32_t lk = lock_acquire();
        heap_root   = heap_meld(heap_root, dwork);
        lock_release(lk);
    }

    void tick_update()
    {
        if (!heap_root) {
            return;
        }
        workqueue_post(&_executor, this);
    }
};

struct DelayAwaiter : Delayed_worknode {

    explicit DelayAwaiter(Timer_check_queue& delay_queue, uint32_t ms) : _delay_queue(delay_queue) { wait_ms = ms; }

    std::coroutine_handle<> mCoroutine;
    uint32_t                wait_ms;
    Timer_check_queue&      _delay_queue;

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<Work_Promise<void>> coroutine) noexcept
    {
        mCoroutine = coroutine;
        INIT_LIST_HEAD(&ws_node);
        func = timer_callback;
        _delay_queue.post_delayed_work(this, wait_ms);
    }

    void await_resume() const noexcept { }

    static void timer_callback(struct worknode* pws)
    {
        DelayAwaiter* self = static_cast<DelayAwaiter*>(pws);

        if (self->mCoroutine) {
            self->mCoroutine.resume();
        }
    }
};

inline auto delay_ms(struct Timer_check_queue& dly_wkq, uint32_t ms)
{
    return DelayAwaiter(dly_wkq, ms);
}

}
