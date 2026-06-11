/*
 * flight.h
 *
 *  Created on: 2026. máj. 9.
 *      Author: balin
 */

#ifndef INC_FLIGHT_H_
#define INC_FLIGHT_H_

#include "PID_CONTROL.h"

typedef enum {
	FLIGHT_IDLE		=	0,
    FLIGHT_TAKEOFF,
	FLIGHT_HOVER,
	FLIGHT_ASCENT,
	FLIGHT_DESCENT,
	FLIGHT_LANDING,
	FLIGHT_MOVE,
	FLIGHT_CIRCLE,
	FLIGHT_RENEGATE,
	FLIGHT_END,
} flight_states_t;

typedef struct{
	flight_states_t flight_state;
	uint32_t maneuver_time;
}flight_t;

extern Control_t pid_traget;
extern flight_t flight;

void flight_statemachine();

#endif /* INC_FLIGHT_H_ */
