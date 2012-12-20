/*
 * CircularBuffer_test.cpp
 *
 *  Created on: 20 Dec 2012
 *      Author: jack
 */

#include <iostream>

#include "../CircularBuffer.h"
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CcTxArrayTest
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(testNullData)
{
    CircularBuffer<uint8_t, 5> cb;

    BOOST_CHECK(cb.is_empty());

    for (uint8_t i=1; i<6; i++) {
        cb.write() = i;
        BOOST_CHECK_EQUAL(cb.get_count(), i);
    }

    BOOST_CHECK(cb.is_full());

    BOOST_CHECK_EQUAL(cb.read(), 1);
    BOOST_CHECK_EQUAL(cb.read(), 1);

    cb.erase();
    BOOST_CHECK_EQUAL(cb.read(), 2);
    BOOST_CHECK_EQUAL(cb.read(), 2);
    BOOST_CHECK_EQUAL(cb.get_count(), 4);

    cb.erase();
    BOOST_CHECK_EQUAL(cb.read(), 3);
    BOOST_CHECK_EQUAL(cb.get_count(), 3);

    cb.erase();
    BOOST_CHECK_EQUAL(cb.read(), 4);
    BOOST_CHECK_EQUAL(cb.get_count(), 2);

    cb.erase();
    BOOST_CHECK_EQUAL(cb.read(), 5);
    BOOST_CHECK_EQUAL(cb.get_count(), 1);

    cb.erase();
    BOOST_CHECK(cb.is_empty());

    for (uint8_t i=1; i<6; i++) {
        cb.write() = i;
        BOOST_CHECK_EQUAL(cb.get_count(), i);
    }

    BOOST_CHECK(cb.is_full());

    // overwrite
    cb.write() = 6;
    BOOST_CHECK(cb.is_full());
    cb.write() = 7;
    BOOST_CHECK(cb.is_full());

    for (uint8_t i=3; i<8; i++) {
        BOOST_CHECK_EQUAL(cb.read(), i);
        BOOST_CHECK_EQUAL(cb.get_count(), 5-(i-3));
        cb.erase();
    }

    BOOST_CHECK(cb.is_empty());
}
