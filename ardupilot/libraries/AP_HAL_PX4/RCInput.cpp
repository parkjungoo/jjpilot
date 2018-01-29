#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include <GCS_MAVLink/GCS.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init()
{
	_perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	if (_rc_sub == -1) {
		AP_HAL::panic("Unable to subscribe to input_rc");
	}
	clear_overrides();
	pthread_mutex_init(&rcin_mutex, nullptr);

	m_bFlag = false;		// my_source
}

bool PX4RCInput::new_input()
{
	pthread_mutex_lock(&rcin_mutex);
	bool valid = _rcin.timestamp_last_signal != _last_read;
	if (_rcin.rc_failsafe) {
		// don't consider input valid if we are in RC failsafe.
		valid = false;
	}
	if (_override_valid) {
		// if we have RC overrides active, then always consider it valid
		valid = true;
	}
	_last_read = _rcin.timestamp_last_signal;
	_override_valid = false;
	pthread_mutex_unlock(&rcin_mutex);
	if (_rcin.input_source != last_input_source) {
		gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s\n", input_source_name(_rcin.input_source));
		last_input_source = _rcin.input_source;
	}
	return valid;
}

uint8_t PX4RCInput::num_channels()
{
	pthread_mutex_lock(&rcin_mutex);
	uint8_t n = _rcin.channel_count;
	pthread_mutex_unlock(&rcin_mutex);
	return n;
}

uint16_t PX4RCInput::read(uint8_t ch)
{
	if (ch >= RC_INPUT_MAX_CHANNELS) {
		return 0;
	}
	pthread_mutex_lock(&rcin_mutex);
	if (_override[ch]) {
		uint16_t v = _override[ch];
		pthread_mutex_unlock(&rcin_mutex);
		return v;
	}
	if (ch >= _rcin.channel_count) {
		pthread_mutex_unlock(&rcin_mutex);
		return 0;
	}
	uint16_t v = _rcin.values[ch];
	pthread_mutex_unlock(&rcin_mutex);
	return v;
}

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len)
{
	if (len > RC_INPUT_MAX_CHANNELS) {
		len = RC_INPUT_MAX_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len)
{
	bool res = false;
	for (uint8_t i = 0; i < len; i++) {
		res |= set_override(i, overrides[i]);
	}
	return res;
}

bool PX4RCInput::set_override(uint8_t channel, int16_t override) {
	if (override < 0) {
		return false; /* -1: no change. */
	}
	if (channel >= RC_INPUT_MAX_CHANNELS) {
		return false;
	}
	_override[channel] = override;
	if (override != 0) {
		_override_valid = true;
		return true;
	}
	return false;
}

void PX4RCInput::clear_overrides()
{
	for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		set_override(i, 0);
	}
}

const char *PX4RCInput::input_source_name(uint8_t id) const
{
	switch(id) {
		case input_rc_s::RC_INPUT_SOURCE_UNKNOWN:         return "UNKNOWN";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM:      return "PX4FMU_PPM";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM:       return "PX4IO_PPM";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM:  return "PX4IO_SPEKTRUM";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS:      return "PX4IO_SBUS";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24:      return "PX4IO_ST24";
		case input_rc_s::RC_INPUT_SOURCE_MAVLINK:         return "MAVLINK";
		case input_rc_s::RC_INPUT_SOURCE_QURT:            return "QURT";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SPEKTRUM: return "PX4FMU_SPEKTRUM";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS:     return "PX4FMU_SBUS";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24:     return "PX4FMU_ST24";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD:     return "PX4FMU_SUMD";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM:      return "PX4FMU_DSM";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_SUMD:      return "PX4IO_SUMD";
		case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL:     return "PX4FMU_SRXL";
		case input_rc_s::RC_INPUT_SOURCE_PX4IO_SRXL:      return "PX4IO_SRXL";
		default:                                          return "ERROR";
	}
}


void PX4RCInput::_timer_tick(void)
{
	perf_begin(_perf_rcin);
	bool rc_updated = false;

	if(m_bFlag == false){
		if (orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
			pthread_mutex_lock(&rcin_mutex);
			orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);

			if (_rcin.rssi != 0 || _rssi != -1) {
				// always zero means not supported
				_rssi = _rcin.rssi;
			}
			pthread_mutex_unlock(&rcin_mutex);
		}
	}
	else{
		if (my_source_lib_rc_orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
			pthread_mutex_lock(&rcin_mutex);
			_rcin.timestamp_publication++;
			_rcin.timestamp_last_signal++;
		
			if(m_bFlag)	my_source_lib_rc_set_pwm();

			if (_rcin.rssi != 0 || _rssi != -1) {
				// always zero means not supported
				_rssi = _rcin.rssi;
			}
			pthread_mutex_unlock(&rcin_mutex);
		}
	}
	// note, we rely on the vehicle code checking new_input()
	// and a timeout for the last valid input to handle failsafe
	perf_end(_perf_rcin);
}

bool PX4RCInput::rc_bind(int dsmMode)
{
	int fd = open("/dev/px4io", 0);
	if (fd == -1) {
		fd = open("/dev/px4fmu", 0);
	}
	if (fd == -1) {
		hal.console->printf("RCInput: failed to open /dev/px4io or /dev/px4fmu\n");
		return false;
	}

	uint32_t mode = (dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES);
	int ret = ioctl(fd, DSM_BIND_START, mode);
	close(fd);
	if (ret != 0) {
		hal.console->printf("RCInput: Unable to start DSM bind\n");
		return false;
	}
	return true;
}


// START my_source 
int PX4RCInput::my_source_lib_rc_orb_check(int handle, bool * updated)
{
	if(m_bFlag){
		*updated = true;
		return 0;
	}
	return 1;
}

void PX4RCInput::my_source_lib_rc_update_pwm(uint16_t values, int ch)
{
	pthread_mutex_lock(&rcin_mutex);
	m_uI16Values[ch] = values;
	pthread_mutex_unlock(&rcin_mutex);
}

void PX4RCInput::my_source_lib_rc_display(void)
{
	pthread_mutex_lock(&rcin_mutex);
	hal.uartC->printf("#");
	hal.uartC->printf("%" PRIu64",", _rcin.timestamp_publication);
	hal.uartC->printf("%" PRIu64",", _rcin.timestamp_last_signal);
	hal.uartC->printf("%d,%d,%d,%d,%d,%d,%d,%d", _rcin.channel_count
			,_rcin.rssi, _rcin.rc_failsafe, _rcin.rc_lost,
			_rcin.rc_lost_frame_count, _rcin.rc_total_frame_count,
			_rcin.rc_ppm_frame_length,_rcin.input_source);
	hal.uartC->printf("(%d,%d,%d,%d,laser:%d)",
			_rcin.values[0], _rcin.values[1], _rcin.values[2], _rcin.values[3],
			_rcin.values[8]);
	hal.uartC->printf("#");
	pthread_mutex_unlock(&rcin_mutex);
}

void PX4RCInput::my_source_lib_rc_init(void)
{
	m_bFlag = true;

	pthread_mutex_lock(&rcin_mutex);
	_rcin.timestamp_publication = _rcin.timestamp_last_signal = 1;
	_rcin.channel_count = 10;
	_rcin.rssi = 40;
	_rcin.rc_ppm_frame_length = 20003;
	_rcin.input_source = 2;

	for(int i=0; i<10; i++)
	{
		if(i==0 || i==1 || i==3)	_rcin.values[i] = m_uI16Values[i] = 1500;
		else						_rcin.values[i] = m_uI16Values[i] = 1000;
	}
	pthread_mutex_unlock(&rcin_mutex);
}

void PX4RCInput::my_source_lib_rc_set_pwm(void)
{
	// must not use the mutex.
	for(int i=0; i<10; i++)	_rcin.values[i] = m_uI16Values[i];
}

// END my_source
#endif
