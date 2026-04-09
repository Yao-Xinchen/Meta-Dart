#pragma once

#include <atomic>
#include <array>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Lock-free triple buffer
//
// One producer, one consumer.  The producer always writes fresh data; the
// consumer always reads the latest complete write.  Neither side ever blocks.
//
// State encoding (packed into one atomic<uint8_t>):
//   bits [1:0]  index of the "middle" (ready-to-swap) slot
//   bit  [2]    dirty flag — set when producer has written a new value
//
// Three slots: producer owns "back", consumer owns "front", middle is shared.
// ─────────────────────────────────────────────────────────────────────────────
template <typename T>
class TripleBuffer {
public:
    TripleBuffer() : back_(0), front_(1), state_(2 /* middle slot */ | 0x00) {
        // slots_[0] = back, slots_[1] = front, slots_[2] = middle (initial)
    }

    // Called by the producer thread.
    // Writes value into the back slot, then atomically swaps back ↔ middle
    // and marks the buffer dirty.
    void write(const T& value) {
        slots_[back_] = value;

        // Swap back and middle; mark dirty (bit 2)
        uint8_t old_state = state_.load(std::memory_order_relaxed);
        uint8_t new_state;
        do {
            uint8_t mid = old_state & 0x03u;
            new_state   = back_ | 0x04u;  // new middle = old back, dirty = 1
            back_       = mid;             // producer takes over old middle
        } while (!state_.compare_exchange_weak(
                     old_state, new_state,
                     std::memory_order_release,
                     std::memory_order_relaxed));
    }

    // Called by the consumer thread.
    // Returns true and updates `out` if new data is available since last read.
    // Returns false immediately if nothing new.
    bool read(T& out) {
        uint8_t old_state = state_.load(std::memory_order_relaxed);
        if (!(old_state & 0x04u)) return false;  // not dirty

        uint8_t new_state;
        do {
            uint8_t mid = old_state & 0x03u;
            new_state   = front_;  // new middle = old front, dirty cleared
            front_      = mid;     // consumer takes the fresh slot
        } while (!state_.compare_exchange_weak(
                     old_state, new_state,
                     std::memory_order_acquire,
                     std::memory_order_relaxed));

        out = slots_[front_];
        return true;
    }

    // Blocking read variant — spins until new data arrives.
    // Prefer the non-blocking version in real-time loops.
    T read_blocking() {
        T out;
        while (!read(out)) {}
        return out;
    }

private:
    std::array<T, 3>       slots_;
    uint8_t                back_;    // index owned by producer
    uint8_t                front_;   // index owned by consumer
    std::atomic<uint8_t>   state_;   // bits[1:0]=middle index, bit[2]=dirty
};
