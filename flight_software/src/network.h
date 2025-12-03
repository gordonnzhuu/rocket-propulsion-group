#pragma once

#include "minos_util.h"

#define UDP_TX_PACKET_MAX_SIZE 256

/**
 * @note "motenet" is the name of the system that the old moteware firmware used.
 * This is implemented here and used for now, because I don't feel like reinventing the network scheme right now.
 * @todo We should definitley change this later, at least the packet encoding.
*/
bool init_motenet();
bool rx_motenet(SensorMap& sensors, ActuatorMap& actuators);
bool tx_motenet(SensorMap& sensors, ActuatorMap& actuators);
bool add_sensors(uint8_t pin_num, uint8_t enable_rf, uint8_t interface_type_number, SensorMap& sensors);
