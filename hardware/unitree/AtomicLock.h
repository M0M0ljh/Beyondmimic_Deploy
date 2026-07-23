#ifndef DEPLOY_REAL_HARDWARE_UNITREE_ATOMIC_LOCK_H
#define DEPLOY_REAL_HARDWARE_UNITREE_ATOMIC_LOCK_H

#include <atomic>

namespace deploy_real
{

class AtomicFlagLock
{
public:
    void Lock()
    {
        while (flag_.test_and_set(std::memory_order_acquire))
        {
        }
    }

    void Unlock()
    {
        flag_.clear(std::memory_order_release);
    }

private:
    std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

template <typename Lock>
class ScopedLock
{
public:
    explicit ScopedLock(Lock &lock) : lock_(lock)
    {
        lock_.Lock();
    }

    ~ScopedLock()
    {
        lock_.Unlock();
    }

    ScopedLock(const ScopedLock &) = delete;
    ScopedLock &operator=(const ScopedLock &) = delete;

private:
    Lock &lock_;
};

} // namespace deploy_real

#endif
