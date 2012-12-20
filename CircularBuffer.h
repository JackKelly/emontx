/*
 * CircularBuffer.h
 *
 *  Created on: 18 Dec 2012
 *      Author: jack
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <stdint.h>

/**
 * A CircularBuffer implementation for Arduino where
 * we don't use new[] and delete[] (because these can cause
 * memory fragmentation); instead we allocate on the stack.
 *
 * As written, can cope with a max of 255 items
 *
 * Adapted from http://en.wikipedia.org/wiki/Circular_buffer#Use_a_Fill_Count
 */
template<class T, uint8_t size = 10>
class CircularBuffer {
public:
    CircularBuffer(): start(0), count(0), id_of_first_item(0) {}

    /**
     * Return reference to next element for writing.
     * Does not clear old data.
     *
     * Returning a reference is rather odd and leads to
     * some slightly odd-to-read code in the calling function.
     * But it should be fast (especially for large objects)
     * because it means we don't have to copy every member variable.
     */
    T& write()
    {
        uint8_t end = (start + count) % size;

        if (count == size) { // Full so overwrite
            start = (start+1) % size;
            id_of_first_item++;
        } else {
            count++;
        }

        return items[end];
    }

    /**
     * Return a reference to the oldest element.
     * Does not delete that element.
     * Repeated calls to read() will return the reference.
     * Call erase() to erase the oldest element.
     */
    const T& read(const uint8_t i=0) const { return items[start]; }

    /**
     * Deallocate the oldest element.
     */
    void erase()
    {
        start = (start+1) % size;
        id_of_first_item++;
        count--;
    }

    uint8_t get_count() { return count; }

    uint8_t get_id_of_first_item() { return id_of_first_item; }

    bool is_full() const { return count == size; }

    bool is_empty() const { return count == 0; }

private:
    T items[size];
    uint8_t start, /* Index of first valid element */
            count; /* Number of valid elements currently stored */
    uint8_t id_of_first_item;

};

#endif /* CIRCULARBUFFER_H_ */
