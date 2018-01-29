#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_analog : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_analog(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);


	// my_source
	uint16_t my_source_lib_range_get_value() { 
	//	if(state.ratiometric)
			return source->voltage_average_ratiometric() * 1000U;}
	//	else
	//		return source->voltage_average() * 1000U; };
protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    // update raw voltage
    void update_voltage(void);

    AP_HAL::AnalogSource *source;
};
