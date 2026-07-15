#pragma once

namespace robot {

template <typename Mutex>
class ScopedLock {
 public:
  explicit ScopedLock(Mutex& mutex) : mutex_(mutex) { mutex_.lock(); }
  ~ScopedLock() { mutex_.unlock(); }

  ScopedLock(const ScopedLock&) = delete;
  ScopedLock& operator=(const ScopedLock&) = delete;

 private:
  Mutex& mutex_;
};

// The lock is explicit so the V5 adapter can provide a PROS-compatible
// BasicLockable and PC tests can provide std::mutex without coupling the core
// contract to either platform.
template <typename T, typename Mutex>
class SnapshotBox {
 public:
  void publish(const T& next) {
    ScopedLock<Mutex> lock(mutex_);
    value_ = next;
    initialized_ = true;
  }

  T read() const {
    ScopedLock<Mutex> lock(mutex_);
    return value_;
  }

  bool initialized() const {
    ScopedLock<Mutex> lock(mutex_);
    return initialized_;
  }

 private:
  mutable Mutex mutex_;
  T value_{};
  bool initialized_{false};
};

}  // namespace robot
