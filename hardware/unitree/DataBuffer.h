#ifndef DEPLOY_REAL_HARDWARE_UNITREE_DATA_BUFFER_H
#define DEPLOY_REAL_HARDWARE_UNITREE_DATA_BUFFER_H

#include "hardware/unitree/AtomicLock.h"

#include <memory>

namespace deploy_real
{

template <typename T>
class DataBuffer
{
public:
    void SetDataPtr(const std::shared_ptr<T> &data)
    {
        ScopedLock<AtomicFlagLock> lock(lock_);
        data_ = data;
    }

    void SetData(const T &data)
    {
        ScopedLock<AtomicFlagLock> lock(lock_);
        data_ = std::make_shared<T>(data);
    }

    std::shared_ptr<T> GetDataPtr()
    {
        ScopedLock<AtomicFlagLock> lock(lock_);
        return data_;
    }

private:
    std::shared_ptr<T> data_;
    AtomicFlagLock lock_;
};

} // namespace deploy_real

#endif
