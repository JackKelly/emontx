/*
 * CircularBuffer.h
 *
 *  Created on: 18 Dec 2012
 *      Author: jack
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

/**
 * A CircularBuffer implementation for Arduino where
 * we don't use new[] and delete[] (because these can cause
 * memory fragmentation); instead we allocate on the stack.
 */
template<class T>
class CircularBuffer {

public:

    CircularBuffer(): start(0), count(0) {}

    /**
     * Return pointer to next element for writing.
     * Does not clear old data.
     */
    T& write()
    {
        uint8_t end = (start + count) % SIZE;

        if (end == start) { // Full so overwrite
            start = (start+1) % SIZE;
        } else {
            count++;
        }

        return items[end];
    }

    /**
     * Return a reference to the oldest element.
     * Does not delete that element.
     * Repeated calls to read() will return the same pointer.
     * Call erase() to erase the oldest element.
     */
    const T& read(const uint8_t i=0) const { return items[start]; }

    /**
     * Erase the oldest element.
     */
    void erase()
    {
        start = (start+1) % SIZE;
        count--;
    }

    uint8_t get_count() { return count; }

    bool is_full() const { return count == SIZE; }

private:
    static const uint8_t SIZE = 10; /* Total number of elements we can store */
    T items[SIZE];
    uint8_t start, /* Index of first valid element */
            count; /* Number of valid elements currently stored */


};

#endif /* CIRCULARBUFFER_H_ */
