#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_MaxsonarSerialLV : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MaxsonarSerialLV(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

	// my_source
	uint16_t my_source_lib_range_get_value() { return state.voltage_mv; };
protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    char linebuf[10];
    uint8_t linebuf_len = 0;
};