#include "Copter.h"
#include <string.h>
#include <stdlib.h>

// This is my_source for userhook...

#ifdef MY_SOURCE
void __get_pwm_from_command(char * strCommand, int16_t * coordinates)
{
	char *ptrCommand = strtok(strCommand, ",");      
	for(int i=0; i<5; i++){
		coordinates[i] = atoi(ptrCommand);

		if(i==MY_SOURCE_CH_CRD_PITCH)
			coordinates[i] = coordinates[i] * (-1);

		//		if(	coordinates[i] > MY_SOURCE_CRD_MAX || 
		//			coordinates[i] < MY_SOURCE_CRD_MIN)	coordinates[i] = 0;
		ptrCommand = strtok(NULL, ",");
	}

	coordinates[MY_SOURCE_CH_CRD_THROTTLE] += MY_SOURCE_OFFSET_THROTTLE;
	coordinates[MY_SOURCE_CH_CRD_YAW] += MY_SOURCE_OFFSET_YAW;
	coordinates[MY_SOURCE_CH_CRD_ROLL] += MY_SOURCE_OFFSET_ROLL;
	coordinates[MY_SOURCE_CH_CRD_PITCH] += MY_SOURCE_OFFSET_PITCH;
}

void Copter::my_source_sort_command(char * strCommand)
{
	char whatTheHell = strCommand[0];

	switch(whatTheHell){
		case MY_SOURCE_COMMAND_PWM:
			__get_pwm_from_command(&strCommand[1], &m_rcInfo.coordinates[0]);
			break;
		case MY_SOURCE_COMMAND_MODE:
			my_source_mode_select(&strCommand[1]);
			break;
		default:
#ifdef MY_SOURCE_MFC
			hal.uartC->printf("*[ERROR] sort_command error*"); // for mfc
#endif
			break;
	}
}

// This function will be called when bluetooth connect.
void Copter::my_source_rc_init()
{
	for(int i=0; i<MY_SOURCE_PWM_LEN; i++){
		if(i == 0 || i == 1 || i == 3)  m_rcInfo.pwm[i] = 1500;
		else                            m_rcInfo.pwm[i] = 1000;    
	}
	for(int i=0; i<5; i++)	m_rcInfo.coordinates[i] = MY_SOURCE_CRD_MAX+MY_SOURCE_CRD_MIN;
	m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] = MY_SOURCE_CRD_MIN;	// throttle

	m_rcInfo.mode = MY_SOURCE_MODE_ENABLE_ARM;
	hal.rcin->my_source_lib_rc_init();
}

void Copter::my_source_rc_cal_pwm(void)
{
	static bool flag = false;

	if(flag)
	{
		for(int i=0; i<MY_SOURCE_PWM_LEN; i++)	m_rcInfo.pwm[i] = 1000;
		for(int i=0; i<5; i++)					m_rcInfo.coordinates[i] = -500;
		flag = !flag;
	}
	else
	{
		for(int i=0; i<MY_SOURCE_PWM_LEN; i++)	m_rcInfo.pwm[i] = 2000;
		for(int i=0; i<5; i++)					m_rcInfo.coordinates[i] = 500;
		flag = !flag;
	}
}

void Copter::my_source_mode_select(char * mode)
{
	int16_t what = atoi(mode);

	switch(what){
		case MY_SOURCE_MODE_ENABLE_ARM:	
			//		my_source_rc_init();	
			//		hal.util->my_source_lib_util_on_safety();
			break;
	//	case MY_SOURCE_GET_MODE_MSG(MY_SOURCE_MODE_CAL_PWM):	
			//		if(m_rcInfo.mode != MY_SOURCE_MODE_ARMING)	my_source_rc_cal_pwm();
			//		break;
		case MY_SOURCE_MODE_AUTO_LANDING:
			if(m_rcInfo.state == MY_SOURCE_MODE_AUTO_LANDING){
				m_rcInfo.state = MY_SOURCE_MODE_STABILIZE;
				m_rcInfo.autoLandingFlag = 0;
			}
			else	m_rcInfo.state = MY_SOURCE_MODE_AUTO_LANDING;
			break;
		case MY_SOURCE_MODE_ALT_HOLD:
			if(m_rcInfo.state == MY_SOURCE_MODE_ALT_HOLD){
				m_rcInfo.state = MY_SOURCE_MODE_STABILIZE;
			}
			else	m_rcInfo.state = MY_SOURCE_MODE_ALT_HOLD;
			break;
		case MY_SOURCE_MODE_UP_KP:			m_rcInfo.kp += 0.05;			break;
		case MY_SOURCE_MODE_DOWN_KP:		m_rcInfo.kp -= 0.05;			break;
		case MY_SOURCE_MODE_UP_ACZ:			m_rcInfo.set_AcZ += 0.05;		break;
		case MY_SOURCE_MODE_DOWN_ACZ:		m_rcInfo.set_AcZ -= 0.05;		break;
		case MY_SOURCE_MODE_UP_ACZ_ALTHOLD:		m_rcInfo.altHold_kpAcZ += 0.1;	break;
		case MY_SOURCE_MODE_DOWN_ACZ_ALTHOLD:	m_rcInfo.altHold_kpAcZ -= 0.1;	break;
		case MY_SOURCE_MODE_UP_ALT_ALTHOLD:		m_rcInfo.altHold_kpAlt += 0.1;	break;
		case MY_SOURCE_MODE_DOWN_ALT_ALTHOLD:	m_rcInfo.altHold_kpAlt -= 0.1;	break;
		case MY_SOURCE_MODE_UP_ALPHA_ALTHOLD:	m_rcInfo.altHold_alpha += 0.1;	break;
		case MY_SOURCE_MODE_DOWN_ALPHA_ALTHOLD:	m_rcInfo.altHold_alpha -= 0.1;	break;
	
		default:
#ifdef MY_SOURCE_MFC
			hal.uartC->printf("*[ERROR] mode_select error %d*", what);	// for mfc
#endif
			break;
	}
}

bool Copter::my_source_is_this_arm(void)
{
	if( m_rcInfo.coordinates[MY_SOURCE_CH_CRD_ROLL] == (0+MY_SOURCE_OFFSET_ROLL) && 
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_PITCH] == (0+MY_SOURCE_OFFSET_PITCH) &&
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] <= (MY_SOURCE_CRD_MIN + 100 + MY_SOURCE_OFFSET_THROTTLE) && 
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_YAW] >= (MY_SOURCE_CRD_MAX - 200 + MY_SOURCE_OFFSET_YAW) ){

		m_rcInfo.pwm[MY_SOURCE_CH_PWM_ROLL] = 1500;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_PITCH] = 1500;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] = 1000;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_YAW] = 2000;
		return true;
	}
	return false;
}

bool Copter::my_source_is_this_disarm(void)
{
	if( m_rcInfo.coordinates[MY_SOURCE_CH_CRD_ROLL] == (0+MY_SOURCE_OFFSET_ROLL) && 
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_PITCH] == (0+MY_SOURCE_OFFSET_PITCH) &&
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] <= (MY_SOURCE_CRD_MIN + 100 + MY_SOURCE_OFFSET_THROTTLE) && 
			m_rcInfo.coordinates[MY_SOURCE_CH_CRD_YAW] <= (MY_SOURCE_CRD_MIN + 200 + MY_SOURCE_OFFSET_YAW) ){

		m_rcInfo.pwm[MY_SOURCE_CH_PWM_ROLL] = 1500;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_PITCH] = 1500;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] = 1000;
		m_rcInfo.pwm[MY_SOURCE_CH_PWM_YAW] = 1000;
		return true;
	}
	return false;
}

void Copter::my_source_fail_safe(void)
{
	for(int i=0; i<4; i++)	m_rcInfo.coordinates[i] = MY_SOURCE_CRD_MAX+MY_SOURCE_CRD_MIN;
	m_rcInfo.coordinates[1] = MY_SOURCE_CRD_MIN;	// throttle
}

void Copter::my_source_calc_sigma(void)
{
	if (m_adcLaser.cnt < 10) return;
	m_adcLaser.sigma = sqrt((m_adcLaser.brt_sum_sqr / 10.0) - 
			(m_adcLaser.brt_sum / 10.0)*(m_adcLaser.brt_sum / 10.0));
	m_adcLaser.cnt = 0;
	m_adcLaser.brt_sum = 0;
	m_adcLaser.brt_sum_sqr = 0;

	//	hal.uartC->printf("#%f#",m_adcLaser.sigma);
}
void Copter::my_source_read_cds(void)
{
	m_adcLaser.sig = rangefinder.my_source_lib_range_get_laser();
	m_adcLaser.brt_sum += m_adcLaser.sig;
	m_adcLaser.brt_sum_sqr += m_adcLaser.sig*m_adcLaser.sig;
	m_adcLaser.cnt++;
	my_source_calc_sigma();
}
void Copter::my_source_check_hit(void)
{
	my_source_check_hit1();
	my_source_check_hit2();
}
void Copter::my_source_check_hit1(void)
{
	if (m_adcLaser.hit)				return;
	if (m_adcLaser.sigma < 15.0)	return;
	m_adcLaser.hit = true;
	m_adcLaser.t1 = AP_HAL::millis();
}
void Copter::my_source_check_hit2(void)
{
	if ((!m_adcLaser.hit) || (AP_HAL::millis() - m_adcLaser.t1 < 100)) return;
	if ( m_adcLaser.sigma < 15.0)	return;
	m_adcLaser.hit = false;
	m_adcLaser.hitCount++;
}

void Copter::my_source_update_accel(void)
{
	m_rcInfo.accel = ins.get_accel(0);
}

void Copter::my_source_auto_landing(void)
{
	//	static Vector3f accel;    
	double err = 0.0;
	int16_t pterm = 0;
	//	accel = ins.get_accel(0);

	//	m_rcInfo.accel.x = accel.x;
	//	m_rcInfo.accel.y = accel.y;
	//	m_rcInfo.accel.z = accel.z;
#ifdef MY_SOURCE_MFC
	if(m_rcInfo.autoLandingFlag == 0){
		hal.uartC->printf("#START Auto-Landing...#");
		m_rcInfo.autoLandingFlag++;
	}
#endif
	err = m_rcInfo.accel.z - m_rcInfo.set_AcZ;
	pterm = (int16_t)(m_rcInfo.kp * err);

	m_rcInfo.my_pterm = pterm;

	m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] += pterm;
}

void Copter::my_source_init_base_alt(void)
{
	m_rcInfo.baseAltCount = 0;
	m_rcInfo.endAltFlag = 0;
	m_rcInfo.Base_Alt = 0;
}

void Copter::my_source_set_base_alt(void)
{
	m_rcInfo.Base_Alt += (double)barometer.get_altitude();
}

void Copter::my_source_set_base_accel(void)
{
	Vector3f accel = ins.get_accel(0);
	m_rcInfo.Base_Accel_Z += (double)accel.z;
}

void Copter::my_source_display_ins(void)
{
	//	hal.uartC->printf("#X:%6.2f Y:%6.2f Z:%6.2f ALT:%f (kp:%6.2f AcZ:%6.2f THR:%d, PTERM:%d)#", 
	hal.uartC->printf("#[%d] Z:%6.2f ALT:%6.2f BASE_ALT:%6.2f BASE_Z:%6.2f(KP[AcZ/Alt]:%6.2f/%6.2f alpha:%6.2f THR:%d)#", 
			m_rcInfo.state, (double)m_rcInfo.accel.z, (double)barometer.get_altitude(), 
			(double)m_rcInfo.Base_Alt, (double)m_rcInfo.Base_Accel_Z,
			(double)m_rcInfo.altHold_kpAcZ, (double)m_rcInfo.altHold_kpAlt,
			(double)m_rcInfo.altHold_alpha,
			m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE]);
}

void Copter::my_source_alt_hold(void)
{
	double target_AcZ = m_rcInfo.Base_Accel_Z;
	double target_alt = m_rcInfo.Base_Alt + 1.5;
//	double kp_AcZ = 7.0;//5.0;
//	double kp_Alt = 10.0;//10.0;
//	double alpha = 0.0;
	double err_AcZ = 0.0;
	double ptermAcZ = 0.0;
	double err_Alt = 0.0;
	double ptermAlt = 0.0;


	err_AcZ = m_rcInfo.accel.z - target_AcZ;
	ptermAcZ = m_rcInfo.altHold_kpAcZ * err_AcZ;
	if (ptermAcZ > 30) ptermAcZ = 30;
	if (ptermAcZ < -30) ptermAcZ = -30;

	err_Alt = target_alt - (double)barometer.get_altitude();
	ptermAlt = m_rcInfo.altHold_kpAlt * err_Alt;
	if (ptermAlt > 30) ptermAlt = 30;
	if (ptermAlt < -30) ptermAlt = -30;

	m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] += (m_rcInfo.altHold_alpha * ptermAcZ + (1 - m_rcInfo.altHold_alpha)*ptermAlt);
	m_rcInfo.coordinates[MY_SOURCE_CH_CRD_THROTTLE] = m_rcInfo.pwm[MY_SOURCE_CH_PWM_THROTTLE] - 1500;
}

#endif
