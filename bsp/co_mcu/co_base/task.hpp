#pragma once

#include <coroutine>
#include <utility>
#include "uninitialized.hpp"
#include "previous_awaiter.hpp"

namespace co_mcu {

#define USE_EXCEPTION 0
#if USE_EXCEPTION
#include <exception>
#endif

template <class T>
struct Promise {

#if 0
    void* operator new(std::size_t size)
    {
        auto ret = pvPortMalloc(size);
        return ret;
    }

    void operator delete(void* ptr) {
        vPortFree(ptr);
    }
#endif
    auto initial_suspend() noexcept {
        return std::suspend_always();
    }

    auto final_suspend() noexcept {
        return PreviousAwaiter(mPrevious);
    }

    void unhandled_exception() noexcept {
#if USE_EXCEPTION
        mException = std::current_exception();
#endif
    }

    void return_value(T &&ret) {
        mResult.putValue(std::move(ret));
    }

    void return_value(T const &ret) {
        mResult.putValue(ret);
    }

    T result() {
        #if USE_EXCEPTION
        if (mException) [[unlikely]] {
            std::rethrow_exception(mException);
        }
        #endif
        return mResult.moveValue();
    }

    auto get_return_object() {
        return std::coroutine_handle<Promise>::from_promise(*this);
    }

    std::coroutine_handle<> mPrevious;
#if USE_EXCEPTION
    std::exception_ptr mException{};
#endif
    Uninitialized<T> mResult; // destructed??

    Promise &operator=(Promise &&) = delete;
};

template <>
struct Promise<void> {
#if 0
    void* operator new(std::size_t size)
    {
        auto ret = pvPortMalloc(size);
        return ret;
    }

    void operator delete(void* ptr) {
        vPortFree(ptr);
    }
#endif
    auto initial_suspend() noexcept {
        return std::suspend_always();
    }

    auto final_suspend() noexcept {
        return PreviousAwaiter(mPrevious);
    }

    void unhandled_exception() noexcept
    {
#if USE_EXCEPTION
        mException = std::current_exception();
#endif
    }

    void return_void() noexcept {}

    void result()
    {
#if USE_EXCEPTION
        if (mException) [[unlikely]] {
            std::rethrow_exception(mException);
        }
#endif
    }

    auto get_return_object() { return std::coroutine_handle<Promise>::from_promise(*this); }

    std::coroutine_handle<> mPrevious;
#if USE_EXCEPTION
    std::exception_ptr mException {};
#endif
    Promise& operator=(Promise&&) = delete;
};

template <class T = void, class P = Promise<T>>
struct [[nodiscard]] Task {
    using promise_type = P;

    Task(std::coroutine_handle<promise_type> coroutine = nullptr) noexcept
        : mCoroutine(coroutine) {}

    Task(Task &&that) noexcept : mCoroutine(that.mCoroutine) {
        that.mCoroutine = nullptr;
    }

    Task &operator=(Task &&that) noexcept {
        std::swap(mCoroutine, that.mCoroutine);
    }

    ~Task() {
        if (mCoroutine)
            mCoroutine.destroy();
    }

    void detach() noexcept {
        mCoroutine = nullptr;
    }

    struct Awaiter {
        bool await_ready() const noexcept {
            return false;
        }

        std::coroutine_handle<promise_type>
        await_suspend(std::coroutine_handle<> coroutine) const noexcept {
            promise_type &promise = mCoroutine.promise();
            promise.mPrevious = coroutine;
            return mCoroutine;
        }

        T await_resume() const {
            return mCoroutine.promise().result();
        }

        std::coroutine_handle<promise_type> mCoroutine;
    };

    auto operator co_await() const noexcept {
        return Awaiter(mCoroutine);
    }

    operator std::coroutine_handle<promise_type>() const noexcept {
        return mCoroutine;
    }

// private:
    std::coroutine_handle<promise_type> mCoroutine;
};

template <class Loop, class T, class P>
T run_task(Loop &loop, Task<T, P> const &t) {
    auto a = t.operator co_await();
    a.await_suspend(std::noop_coroutine()).resume();
    while (loop.run())
        ;
    return a.await_resume();
}

template <class T, class P>
void spawn_task(Task<T, P> const &t) {
    auto a = t.operator co_await();
    a.await_suspend(std::noop_coroutine()).resume();
}

}
