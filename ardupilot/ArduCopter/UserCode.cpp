#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
	// put your initialisation code here
	// this will be called once at start-up
	m_rcInfo.mode = MY_SOURCE_MODE_DISABLE_ARM;
	m_rcInfo.laserFlag = false;
	m_rcInfo.laserCount = 0;

	m_adcLaser.hit = false;
	m_adcLaser.hitCount = 0;

	m_rcInfo.kp = 6.0;
	m_rcInfo.set_AcZ = -9.72;

	m_rcInfo.autoLandingFlag = 0;

	my_source_init_base_alt();

	m_rcInfo.altHold_kpAcZ = 7.0;
	m_rcInfo.altHold_kpAlt = 10.0;
	m_rcInfo.altHold_alpha = 0.0;

	m_rcInfo.state = MY_SOURCE_MODE_STABILIZE;
}
#endif

void __convert_to_pwm(int16_t coordinate, uint16_t * pwm, uint16_t ch)
{
// version2 : coordinate range -500 ~ 500
	int16_t value = 0;
	uint16_t goal = coordinate + 1500;
	uint16_t sensitivity = 3;
	
	if( (*pwm) > goal )			value = -1 - ( ( ((*pwm)-goal)/100) * sensitivity);
	else if( (*pwm) < goal )	value = 1 + ( ( (goal-(*pwm))/100) * sensitivity);
	else						value = 0;

	(*pwm) += value;
}

#ifdef USERHOOK_200HZLOOP
void Copter::userhook_200Hz()
{
#ifdef MY_SOURCE_RADIO
#else	// for bluetooth
	my_source_read_cds();				// laser_sensor
#endif
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
	// put your 100Hz code here
#ifdef MY_SOURCE_RADIO

#else	// for bluetooth
	if( !m_Queue.QIsEmpty() ){
		char str[MY_SOURCE_STR_LEN] = {0,};
		m_Queue.Pop(str);

//		hal.uartC->printf("#%s#",str);

		my_source_sort_command(str);

		if(m_rcInfo.coordinates[MY_SOURCE_CH_CRD_LASER] == 1){		// LASER
			m_rcInfo.laserFlag = true;
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_LASER] = 0;
		}
	}

	if(m_rcInfo.rcFlag){	
		if( (my_source_is_this_arm() == false) && (my_source_is_this_disarm() == false) ){

			my_source_update_accel();

			if(m_rcInfo.state == MY_SOURCE_MODE_AUTO_LANDING){
				my_source_auto_landing();
				m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] = -500;
			}else if(m_rcInfo.state == MY_SOURCE_MODE_ALT_HOLD){
				my_source_alt_hold();
			}
			else{
				m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] = 
					m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] + 1500;
			}

			m_rcInfo.pwm[MY_SOURCE_CH_PWM_YAW] = 
				m_rcInfo.coordinates[MY_SOURCE_CH_CRD_YAW] + 1500;
			m_rcInfo.pwm[MY_SOURCE_CH_PWM_ROLL] = 
				m_rcInfo.coordinates[MY_SOURCE_CH_CRD_ROLL] + 1500;
			m_rcInfo.pwm[MY_SOURCE_CH_PWM_PITCH] = 
				m_rcInfo.coordinates[MY_SOURCE_CH_CRD_PITCH] + 1500;

			/*
			__convert_to_pwm(m_rcInfo.coordinates[MY_SOURCE_CH_CRD_ROLL],
					&m_rcInfo.pwm[MY_SOURCE_CH_PWM_ROLL], MY_SOURCE_CH_PWM_ROLL);     
			__convert_to_pwm(m_rcInfo.coordinates[MY_SOURCE_CH_CRD_PITCH],
					&m_rcInfo.pwm[MY_SOURCE_CH_PWM_PITCH], MY_SOURCE_CH_PWM_PITCH);     
			
			__convert_to_pwm(m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE], 
					&m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE], MY_SOURCE_CH_PWM_THROTTLE);
			__convert_to_pwm(m_rcInfo.coordinates[MY_SOURCE_CH_CRD_YAW], 
					&m_rcInfo.pwm[MY_SOURCE_CH_PWM_YAW], MY_SOURCE_CH_PWM_YAW);     
			*/
		}

		for(int i=0; i<4; i++){
			if(m_rcInfo.pwm[i] >= 2000) m_rcInfo.pwm[i] = 2000;
			if(m_rcInfo.pwm[i] <= 1000) m_rcInfo.pwm[i] = 1000;			
		}
	}
#endif
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
	// put your 50Hz code here
#ifdef MY_SOURCE_RADIO

#else
	while(hal.uartC->available() > 0){
		int16_t c = hal.uartC->read();

		m_rcInfo.failSafeCount = 0;
		m_rcInfo.rcFlag = true;

		if(m_strLen >= MY_SOURCE_STR_LEN)		// error
		{
			for(int i=0; i<MY_SOURCE_STR_LEN; i++)
				m_str[i] = 0;

			m_strLen = 0;
			m_bStrFlag = false;
		}

		if( c == '@' || c == '#')
		{
			if(m_bStrFlag)		// end str
			{
				m_str[m_strLen] = 0;
				m_Queue.Push(m_str);

				m_strLen = 0;
				m_bStrFlag = false;
			}
			else				// start str
			{
				m_str[m_strLen] = 0;
				m_strLen = 0;

				m_str[m_strLen] = c;
				m_bStrFlag = true;
				m_strLen++;
			}
		}
		else
		{
			if(m_bStrFlag){
				m_str[m_strLen] = c;
				m_strLen++;
			}
		}
	}
#endif
}
#endif

#ifdef USERHOOK_20HZLOOP
void Copter::userhook_20Hz()
{
	if(m_rcInfo.rcFlag)		my_source_check_hit();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
	static uint8_t baseAccelCount = 0;
	static uint8_t endAccelFlag = 0;
//	static uint8_t baseAltCount = 0;
//	static uint8_t endAltFlag = 0;

	// put your 10Hz code here
#ifdef MY_SOURCE_RADIO
#else
	if(m_rcInfo.rcFlag){
		if(m_rcInfo.laserFlag){
			if(m_rcInfo.laserCount >= 1){
				m_rcInfo.laserFlag = false;
			}
			m_rcInfo.pwm[MY_SOURCE_CH_PWM_LASER] = 
				(m_rcInfo.pwm[MY_SOURCE_CH_PWM_LASER] == 
				 MY_SOURCE_PWM_OFF_LASER ? MY_SOURCE_PWM_ON_LASER : MY_SOURCE_PWM_OFF_LASER);
			m_rcInfo.laserCount++;
		}
		else
		{
			m_rcInfo.laserCount = 0;
			m_rcInfo.pwm[MY_SOURCE_CH_PWM_LASER] = MY_SOURCE_PWM_OFF_LASER;
		}
#ifdef MY_SOURCE_MFC
		my_source_display_ins();
#endif
	

	// getting base_accel_z as soon as power on
	if( baseAccelCount < 10){
		my_source_set_base_accel();
		baseAccelCount++;
	}else{
		if(endAccelFlag == 0){
			endAccelFlag++;
			m_rcInfo.Base_Accel_Z /= 10.0;
		}
	}

	// getting base_altitude as soon as arming
	if(m_rcInfo.mode == MY_SOURCE_MODE_ARMING){
		if( m_rcInfo.baseAltCount < 10 ){
			my_source_set_base_alt();
			m_rcInfo.baseAltCount++;
		}else{
			if(m_rcInfo.endAltFlag == 0 ){
				m_rcInfo.endAltFlag++;
				m_rcInfo.Base_Alt /= 10.0;
			}
		}
	}

	}
#endif
}

#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
	// put your 3.3Hz code here
#ifdef MY_SOURCE_RADIO
#else
#endif
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
#ifdef MY_SOURCE_RADIO
#else
	static uint32_t prev_send_t = 0;
	static uint32_t now_send_t = 0;

	if( (m_rcInfo.rcFlag == true) && (m_rcInfo.mode == MY_SOURCE_MODE_DISABLE_ARM) ){
		my_source_rc_init();
	}

//	if( (m_rcInfo.mode == MY_SOURCE_MODE_ARMING) && (m_rcInfo.rcFlag == true))
//		hal.rcin->my_source_lib_rc_display();

	/*
	if( (m_rcInfo.mode == MY_SOURCE_MODE_DISABLE_ARM) ){
		if(init_cnt > 5)	my_source_rc_init();
		else				init_cnt++;
	}
	*/

	if(m_rcInfo.rcFlag){
		if( (m_rcInfo.mode == MY_SOURCE_MODE_ARMING) && (m_rcInfo.failSafeCount++ > 5) ){
			my_source_fail_safe();
			m_rcInfo.mode = MY_SOURCE_MODE_FAILSAFE;
		}

		
		if(m_adcLaser.hitCount > 0){
			now_send_t = AP_HAL::millis();
			if(now_send_t - prev_send_t > 2000){
				hal.uartC->printf("#3#");
				prev_send_t = now_send_t;
			}
			m_adcLaser.hitCount = 0;
		}
				// laser_sensor
	}
#endif
}
#endif
