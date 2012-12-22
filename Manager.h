/*
 * Manager.h
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#ifndef MANAGER_H_
#define MANAGER_H_

#include <Waveform.h>

class Manager {
public:
    Manager();

    void init();

    void run();
private:
    Waveform v, i;
    const float both_calibrations;
};

#endif /* MANAGER_H_ */
