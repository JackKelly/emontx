/*
 * DataPacket.h
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#ifndef DATAPACKET_H_
#define DATAPACKET_H_

#include <Packet.h>

class Datum;

/* Byte indicies of specific parts of the DataPacket */
enum {PACKET_TYPE_I=8, ID_I=9, TIME_I=10,
      FIRST_DATUM_I=14, CHK_I=63, DATUM_LENGTH=8};

/**
 * Represents a collection six of Datums
 * This is the quantity of data sent per data packet.
 *
 * Number of bytes = 6B preamble + 2B sync +
 *    1B packet type + 1B ID + 4B first_datum_time +
 *    6 Datums x 8 bytes per Datum + 1B chk + 2B tail
 */
class DataPacket : public TxPacket<65>
{
public:
    DataPacket();

    void reset();

    void add_datum(const Datum& datum);

    bool is_full();

    void add_checksum();

private:
    static const uint8_t NUM_DATUMS = 6;
    uint8_t i_next_datum;

};

#endif /* DATAPACKET_H_ */
