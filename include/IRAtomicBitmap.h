// Copyright (c) 2026 INSIGHT RAIL LLC. All rights reserved.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

/**
 * @brief A tightly packed bitmap with atomic bits and synchronized resizing.
 *
 * @details Bits are stored in unpadded, lock-free 64-bit atomic blocks.
 * Each bit access holds a shared storage lock, allowing concurrent readers
 * and writers, including accesses to bits in the same block. resize() takes
 * an exclusive storage lock: it waits for active accesses to finish before
 * changing or freeing storage, and blocks bit accesses while holding that
 * lock. Concurrent resize() calls are serialized by the same lock. Copy
 * construction also takes an exclusive lock on the source, producing a
 * consistent snapshot with independent storage and its own mutex. Copy
 * assignment exclusively locks both bitmaps with deadlock avoidance before
 * replacing the destination's size and bits. Moving uses the same exclusive
 * locking scheme to transfer storage and empty the source. Mutexes themselves
 * are never copied or moved.
 *
 * @note Synchronization covers individual operations, not entire worker
 * threads or sequences of calls. size() followed by check(i), check(i)
 * followed by set(i), and assignment between two proxies are separate
 * operations; another operation, including resize(), may intervene.
 * Bounds are checked under the storage lock, including in release builds.
 *
 * @note Bit operations use relaxed atomics. Do not use bits as publication
 * flags for unrelated data without additional synchronization.
 *
 * @note Storage locking adds per-access overhead; the public operations are
 * not lock-free. std::shared_mutex does not guarantee writer fairness or a
 * bounded wait for exclusive operations under continuous contention.
 *
 * @warning An object must be fully constructed before other threads access it,
 * and destruction must not overlap accesses to that object. Copy and move
 * construction may run concurrently with operations on the source. The bitmap
 * must outlive all uses of its Reference proxies.
 */
class AtomicBitmap final {
    using word_type = std::uint64_t;
    using atomic_word = std::atomic<word_type>;
    static constexpr std::size_t bits_per_word = 64;

    // Fail at compile time instead of silently using a locking or padded
    // implementation on a platform that cannot meet these requirements.
    static_assert(atomic_word::is_always_lock_free,
                  "AtomicBitmap requires lock-free 64-bit atomics");
    static_assert(sizeof(atomic_word) == sizeof(word_type),
                  "AtomicBitmap requires tightly packed 64-bit atomics");

    static constexpr std::size_t word_count(std::size_t bits) noexcept {
        // Unlike (bits + 63) / 64, this cannot overflow.
        return bits / bits_per_word + (bits % bits_per_word != 0);
    }

    static constexpr word_type bit_mask(std::size_t i) noexcept {
        return word_type{1} << (i % bits_per_word);
    }

    static void write_bit(atomic_word& word, word_type mask,
                          bool value) noexcept {
        if (value) {
            word.fetch_or(mask, std::memory_order_relaxed);
        } else {
            word.fetch_and(~mask, std::memory_order_relaxed);
        }
    }

    static std::unique_ptr<atomic_word[]> allocate(std::size_t count) {
        if (count == 0) {
            return nullptr;
        }
        // C++20 default construction value-initializes each atomic's
        // underlying integer to zero.
        return std::make_unique<atomic_word[]>(count);
    }

    // The caller must already hold shared or exclusive storage access.
    void validate_index(std::size_t i) const {
        if (i >= bit_count_) {
            throw std::out_of_range("AtomicBitmap: bit index out of range");
        }
    }

public:
    /**
     * @brief A writable proxy referring to one bitmap index.
     * @details Stores an owner and index, never a pointer into the allocation.
     * Each read or assignment locks the owner and validates the index anew.
     * Retaining a proxy does not hold a lock or prevent resizing.
     * @note Proxies survive reallocation. If shrinking removes their index,
     * accesses throw std::out_of_range until the index exists again. After
     * regrowth, the proxy refers to the newly initialized bit at that index.
     * @note Proxies remain bound to their original bitmap object when that
     * bitmap is moved. They do not follow the transferred storage. Accessing a
     * proxy on an empty moved-from bitmap throws std::out_of_range. Destination
     * proxies refer to the new bits at their indices after move assignment.
     * @warning The owner must outlive all reads and assignments via the proxy.
     */
    class Reference final {
        friend class AtomicBitmap;

        AtomicBitmap* const owner_;
        const std::size_t index_;

        Reference(AtomicBitmap& owner, std::size_t index) noexcept
            : owner_(&owner), index_(index) {}

    public:
        /**
         * @brief Copy a proxy's owner and index without reading its bit.
         * @param[in] other Proxy identifying the same bit as the new proxy.
         * @note Does not lock storage or validate the index.
         */
        Reference(const Reference& other) noexcept = default;

        /** @brief Destroy the proxy without changing its bit or owner. */
        ~Reference() = default;

        /**
         * @brief Read the bit through the owner's synchronized check().
         * @return True if the bit is set; otherwise false.
         * @throws std::out_of_range If the index is outside the current size.
         * @throws std::system_error If acquiring the storage lock fails.
         * @note May wait for an ongoing resize to finish.
         */
        [[nodiscard]] operator bool() const {
            return owner_->check(index_);
        }

        /**
         * @brief Set or clear the bit through the owner's synchronized set().
         * @param[in] value True to set the bit; false to clear it.
         * @return This proxy, enabling chained assignments.
         * @throws std::out_of_range If the index is outside the current size.
         * @throws std::system_error If acquiring the storage lock fails.
         * @note May wait for an ongoing resize to finish.
         */
        Reference& operator=(bool value) {
            owner_->set(index_, value);
            return *this;
        }

        /**
         * @brief Read another proxy's bit and assign its value to this bit.
         * @param[in] other Proxy supplying the value to copy.
         * @return This proxy; its owner and index remain unchanged.
         * @throws std::out_of_range If either index is invalid when accessed.
         * @throws std::system_error If acquiring a storage lock fails.
         * @note The source read and destination write are separately protected,
         * not one atomic transaction. No two storage locks are held together,
         * including when both proxies belong to the same bitmap.
         */
        Reference& operator=(const Reference& other) {
            return *this = static_cast<bool>(other);
        }
    };

    /**
     * @brief Construct an empty bitmap containing zero addressable bits.
     * @throws std::system_error If storage-lock initialization fails.
     * @note Allocates no backing bit storage. Use resize() to add bits later.
     */
    AtomicBitmap() : AtomicBitmap(0) {}

    /**
     * @brief Construct a bitmap with all bits initially false.
     * @param[in] bit_count Number of addressable bits; zero is permitted.
     * @throws std::bad_alloc If backing storage cannot be allocated.
     * @throws std::system_error If storage-lock initialization fails.
     * @note Uses ceil(bit_count / 64) atomic blocks plus fixed object metadata
     * and allocator overhead.
     */
    explicit AtomicBitmap(std::size_t bit_count)
        : bit_count_(bit_count), words_(allocate(word_count(bit_count))) {}

    /**
     * @brief Destroy the bitmap and release its backing storage.
     * @pre All concurrent operations have finished and no new operations,
     * including accesses through Reference proxies, will start.
     * @note The caller manages object lifetime; internal resizing
     * synchronization does not make concurrent destruction safe.
     */
    ~AtomicBitmap() = default;

    /**
     * @brief Construct an independent copy of a consistent source snapshot.
     * @param[in] other Bitmap whose size and bit values are copied.
     * @pre The source remains alive for the duration of this call.
     * @throws std::bad_alloc If storage for the copy cannot be allocated.
     * @throws std::system_error If initializing the new mutex or locking the
     * source mutex fails.
     *
     * @details Takes an exclusive lock on the source before reading its size
     * or bits. Source bit accesses, resizing, and other copy constructors
     * reading that source wait while this lock is held. This gives one
     * consistent snapshot rather than a mixture of independently read blocks.
     *
     * The copy owns separate atomic blocks and a newly initialized mutex.
     * Later changes to either bitmap do not affect the other. Source bit
     * proxies continue to refer to the source, not the copy.
     *
     * @note Allocation and copying occur while the source is locked. Work is
     * proportional to the number of 64-bit blocks. The source is unchanged
     * by this constructor, including if construction throws.
     */
    AtomicBitmap(const AtomicBitmap& other) : AtomicBitmap() {
        const std::unique_lock lock(other.storage_mutex_);
        const std::size_t count = word_count(other.bit_count_);
        auto copy = allocate(count);

        for (std::size_t i = 0; i < count; ++i) {
            copy[i].store(other.words_[i].load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
        }

        words_ = std::move(copy);
        bit_count_ = other.bit_count_;
    }

    /**
     * @brief Replace this bitmap with an independent copy of another bitmap.
     * @param[in] other Bitmap whose size and bit values are copied.
     * @return This bitmap, enabling chained assignments.
     * @pre Both bitmaps remain alive for the duration of this call.
     * @throws std::bad_alloc If replacement storage cannot be allocated.
     * @throws std::system_error If acquiring either storage lock fails.
     *
     * @details Self-assignment is a no-op. Otherwise, std::scoped_lock acquires
     * exclusive ownership of both storage mutexes using deadlock avoidance.
     * Opposite-direction assignments, such as a = b and b = a, can therefore
     * run concurrently without deadlocking on these mutexes.
     *
     * While both locks are held, source and destination bit accesses, resizing,
     * and copying wait. Replacement blocks are allocated and populated before
     * committing the destination's new size and storage. The source is not
     * changed by this operation. Each bitmap retains its own mutex, and later
     * changes to either bitmap do not affect the other.
     *
     * @note If allocation or locking fails, this assignment makes no change
     * to the destination. Existing destination proxies keep their owner and
     * index and refer to the replacement bits; accesses to indices removed by
     * assignment throw std::out_of_range until those indices exist again.
     *
     * @note Allocation and copying hold both locks. Work is proportional to
     * the source's number of 64-bit blocks, plus releasing old storage.
     */
    AtomicBitmap& operator=(const AtomicBitmap& other) {
        if (this == &other) {
            return *this;
        }

        const std::scoped_lock lock(storage_mutex_, other.storage_mutex_);
        const std::size_t count = word_count(other.bit_count_);
        auto copy = allocate(count);

        for (std::size_t i = 0; i < count; ++i) {
            copy[i].store(other.words_[i].load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
        }

        words_ = std::move(copy);
        bit_count_ = other.bit_count_;
        return *this;
    }

    /**
     * @brief Construct a bitmap by taking ownership of another bitmap's bits.
     * @param[in,out] other Bitmap whose storage is transferred to this bitmap.
     * @pre The source remains alive for the duration of this call.
     * @throws std::system_error If initializing the new mutex or locking the
     * source mutex fails. In that case, this constructor does not alter source
     * size or storage.
     *
     * @details Exclusively locks the source, waits for active accesses to
     * finish, transfers its storage and size, and sets its size to zero.
     * No bit blocks are allocated or copied. This object gets its own mutex;
     * the source keeps its existing mutex and can be resized or assigned again.
     *
     * @note The source is empty when the transfer commits under its lock.
     * Other threads may modify it again after the lock is released. Existing
     * source proxies stay bound to the source; they do not refer to this object.
     *
     * @note The storage transfer takes constant time apart from lock waits and
     * mutex initialization. This constructor is deliberately not noexcept
     * because mutex initialization and lock acquisition may throw.
     */
    AtomicBitmap(AtomicBitmap&& other) : AtomicBitmap() {
        const std::unique_lock lock(other.storage_mutex_);
        words_ = std::move(other.words_);
        bit_count_ = std::exchange(other.bit_count_, 0);
    }

    /**
     * @brief Replace this bitmap by taking ownership of another bitmap's bits.
     * @param[in,out] other Bitmap whose storage is transferred to this bitmap.
     * @return This bitmap, enabling chained assignments.
     * @pre Both bitmaps remain alive for the duration of this call.
     * @throws std::system_error If acquiring either storage lock fails. In
     * that case, this assignment makes no change to either bitmap.
     *
     * @details Self-move assignment is a no-op. Otherwise, std::scoped_lock
     * exclusively locks both storage mutexes using deadlock avoidance. While
     * both locks are held, the destination releases its old storage and takes
     * the source's storage and size; the source becomes empty. Each bitmap
     * retains its own mutex. No replacement bit blocks are allocated or copied.
     *
     * @note The source can be resized or assigned again. Its empty state is
     * established under the lock; later concurrent operations may change it.
     * Source proxies remain bound to the source, not the destination. Existing
     * destination proxies refer to the replacement bits at their indices, or
     * throw std::out_of_range if their index is outside the new size.
     *
     * @note Pointer and size transfer takes constant time, excluding lock
     * waits and releasing the old destination allocation. This operator is
     * deliberately not noexcept because acquiring the locks may throw.
     */
    AtomicBitmap& operator=(AtomicBitmap&& other) {
        if (this == &other) {
            return *this;
        }

        const std::scoped_lock lock(storage_mutex_, other.storage_mutex_);
        words_ = std::move(other.words_);
        bit_count_ = std::exchange(other.bit_count_, 0);
        return *this;
    }

    /**
     * @brief Read the current number of bits under a shared storage lock.
     * @return The size observed while this call holds the storage lock.
     * @throws std::system_error If acquiring the storage lock fails.
     * @note Resizing, assignment, or moving from this bitmap may change the
     * size immediately afterward; this result does not reserve indices for
     * subsequent operations.
     */
    [[nodiscard]] std::size_t size() const {
        const std::shared_lock lock(storage_mutex_);
        return bit_count_;
    }

    /**
     * @brief Atomically read a bit while holding a shared storage lock.
     * @param[in] i Zero-based bit index.
     * @return True if bit i is set; otherwise false.
     * @throws std::out_of_range If i is outside the size observed under lock.
     * @throws std::system_error If acquiring the storage lock fails.
     * @note May run alongside bit reads and writes, but never overlaps storage
     * changes made by resize(). Uses a relaxed atomic load.
     */
    [[nodiscard]] bool check(std::size_t i) const {
        const std::shared_lock lock(storage_mutex_);
        validate_index(i);
        return (words_[i / bits_per_word].load(std::memory_order_relaxed)
                & bit_mask(i)) != 0;
    }

    /**
     * @brief Atomically set or clear a bit under a shared storage lock.
     * @param[in] i Zero-based bit index.
     * @param[in] value True to set the bit; false to clear it. Defaults to true.
     * @throws std::out_of_range If i is outside the size observed under lock.
     * @throws std::system_error If acquiring the storage lock fails.
     * @note Other bits in the block are preserved using relaxed fetch_or or
     * fetch_and. Concurrent writers share the storage lock and are protected
     * from each other by the atomic block operation.
     */
    void set(std::size_t i, bool value = true) {
        const std::shared_lock lock(storage_mutex_);
        validate_index(i);
        write_bit(words_[i / bits_per_word], bit_mask(i), value);
    }

    /**
     * @brief Create a proxy for reading or assigning a bit.
     * @param[in] i Zero-based bit index to access through the proxy.
     * @return A Reference identifying this bitmap and index i.
     * @note This call does not access storage or acquire a lock. Bounds checks
     * and synchronization occur when the proxy is read or assigned. Merely
     * forming bitmap[i] does not validate i.
     * @see Reference
     */
    [[nodiscard]] Reference operator[](std::size_t i) noexcept {
        return Reference(*this, i);
    }

    /**
     * @brief Read a bit from a const bitmap using synchronized check().
     * @param[in] i Zero-based bit index.
     * @return True if bit i is set; otherwise false.
     * @throws std::out_of_range If i is outside the size observed under lock.
     * @throws std::system_error If acquiring the storage lock fails.
     * @note Returns a bool value, not a proxy or a reference to backing storage.
     */
    [[nodiscard]] bool operator[](std::size_t i) const {
        return check(i);
    }

    /**
     * @brief Change the number of bits while exclusively locking storage.
     * @param[in] new_size New number of addressable bits; zero is permitted.
     * @throws std::bad_alloc If replacement storage cannot be allocated.
     * @throws std::system_error If acquiring the storage lock fails.
     *
     * @details Waits for active bit accesses to finish before touching storage.
     * Once the exclusive lock is acquired, bit accesses and other resize()
     * calls wait until it is released. No external resize lock or thread join
     * is required. This waits for individual operations, not entire worker
     * loops; scheduling fairness is implementation-dependent.
     *
     * Bits below min(old_size, new_size) are preserved. New bits are false.
     * Removed bits are discarded and never restored by later growth. Proxies
     * retain their owner and index, and validate that index on each use.
     *
     * @note If allocation or lock acquisition fails, this call makes no change.
     * Changing the number of 64-bit blocks reallocates and copies retained
     * blocks; changing size within the same block count requires no allocation.
     */
    void resize(std::size_t new_size) {
        const std::unique_lock lock(storage_mutex_);
        if (new_size == bit_count_) {
            return;
        }

        const std::size_t old_count = word_count(bit_count_);
        const std::size_t new_count = word_count(new_size);

        if (new_count != old_count) {
            auto replacement = allocate(new_count);
            const std::size_t overlap =
                old_count < new_count ? old_count : new_count;

            for (std::size_t i = 0; i < overlap; ++i) {
                replacement[i].store(
                    words_[i].load(std::memory_order_relaxed),
                    std::memory_order_relaxed);
            }

            words_ = std::move(replacement);
        }

        // Keep unused bits in the final word zero. In particular, shrinking
        // within one word must clear removed bits before any later growth.
        const std::size_t remainder = new_size % bits_per_word;
        if (remainder != 0) {
            const word_type keep = (word_type{1} << remainder) - 1;
            auto& last = words_[new_count - 1];
            last.store(last.load(std::memory_order_relaxed) & keep,
                       std::memory_order_relaxed);
        }

        bit_count_ = new_size;
    }

private:
    mutable std::shared_mutex storage_mutex_;
    std::size_t bit_count_;
    std::unique_ptr<atomic_word[]> words_;
};

