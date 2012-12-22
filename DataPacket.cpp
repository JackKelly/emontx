/*
 * DataPacket.cpp
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#include "Datum.h"
#include <Packet.h>
#include <utils.h>
#include "DataPacket.h"
#include <stdint.h>

typedef uint8_t byte;

DataPacket::DataPacket()
{
    reset();
    packet[PACKET_TYPE_I] = 0x0E; // For "Open Energy" Monitor
    // TODO: add header and tail.
}

void DataPacket::reset() { i_next_datum = 0; }

void DataPacket::add_datum(const Datum& datum)
{
    if (i_next_datum == 0) {
        utils::uint_to_bytes(millis(), packet+TIME_I); // record time
    }

    if (!is_full()) {
        datum.pack_for_tx(packet+FIRST_DATUM_I+(i_next_datum*DATUM_LENGTH));
        i_next_datum++;
    }
}

bool DataPacket::is_full() { return i_next_datum == NUM_DATUMS; }

void DataPacket::add_checksum()
{
    packet[CHK_I] = modular_sum(packet+PACKET_TYPE_I, 54);
}
