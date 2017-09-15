/*****************************************************************************
* Model: sensor.qm
* File:  ./sensor.h
*
* This code has been generated by QM tool (see state-machine.com/qm).
* DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*****************************************************************************/
/*${.::sensor.h} ...........................................................*/

#if ((QP_VERSION < 591) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpn version 5.9.1 or higher required
#endif

/*${AOs::Sensor} ...........................................................*/
typedef struct Sensor {
/* protected: */
    QActive super;

/* private: */
    uint8_t sensor_data[SENSOR_COUNT][MESSAGE_SIZE];
} Sensor;

/* protected: */
QState Sensor_initial(Sensor * const me);
QState Sensor_Active(Sensor * const me);
QState Sensor_TempSensing(Sensor * const me);
QState Sensor_Emitting(Sensor * const me);
QState Sensor_Sleeping(Sensor * const me);

