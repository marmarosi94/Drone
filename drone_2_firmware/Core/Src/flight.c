/*
 * flight.c
 *
 *  Created on: 2026. máj. 9.
 *      Author: balin
 */
#include "main.h"

Control_t pid_traget = {0};


void flight_statemachine()
{
	static flight_t flight = {
			.flight_state = FLIGHT_IDLE,
			.maneuver_time = 600,
	};
	static uint32_t flight_now = 0;
	switch(flight.flight_state)
	{
		case FLIGHT_IDLE:
			//state_changed = 1;
			debug_print("Flight state: FLIGHT_IDLE\r\n");
			pid_control.Throttle = 1180;
			if(flight.maneuver_time <= flight_now){
				flight_now = 0;
				flight.flight_state = FLIGHT_TAKEOFF;
			}
			break;
		case FLIGHT_TAKEOFF:
			pid_traget.Height = pid_traget.Height + 1;
			pid_control.Throttle = 1180 + pid_control.Height;
			sprintf(str, "Flight state: FLIGHT_TAKEOFF\r\nTarget height: %i\r\n",pid_traget.Height);
			debug_print(str);
			if(650 <= flight_now){
				flight_now = 0;
				flight.flight_state = FLIGHT_HOVER;
			}
			break;
		case FLIGHT_HOVER:
			debug_print("Flight state: FLIGHT_HOVERING\r\n");
			pid_traget.Height = 650;
			pid_control.Throttle = 1180 + pid_control.Height;
			if(1200 <= flight_now){
				flight_now = 0;
				flight.flight_state = FLIGHT_LANDING;
			}
			break;
		case FLIGHT_ASCENT:
			debug_print("Flight state: FLIGHT_ASCENT\r\n");


			break;
		case FLIGHT_DESCENT:
			debug_print("Flight state: FLIGHT_DESCENT\r\n");

			break;
		case FLIGHT_LANDING:
			sprintf(str, "Flight state: FLIGHT_LANDING\r\nTarget height: %i\r\n",pid_traget.Height);
			debug_print(str);
			if(0 < pid_traget.Height)
			{
				pid_traget.Height = (float)pid_traget.Height - 0.2f;
			}
			pid_control.Throttle = 1180 + pid_control.Height;
			if(800 <= flight_now)
			{
				flight_now = 0;
				flight.flight_state = FLIGHT_END;
			}
			break;
		case FLIGHT_MOVE:
			debug_print("Flight state: FLIGHT_MOVE\r\n");

			break;
		case FLIGHT_CIRCLE:
			debug_print("Flight state: FLIGHT_CIRCLE\r\n");

			break;
		case FLIGHT_RENEGATE:
			debug_print("Flight state: FLIGHT_RENEGATE\r\n");
			break;
		case FLIGHT_END:
			debug_print("Flight state: END\r\n");
			pid_control.Throttle = 1000;
			break;

	}
	flight_now++;
}

