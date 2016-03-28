#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/common/common.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/ardupilotmega.h"
// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);

    }
	for (int i = 0; i < maxStreams; i++) {
		mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
			apm_mav_system, apm_mav_component,
			MAVStreams[i], MAVRates[i], 1);

		/*Bitmask to indicate which dimensions should be ignored by the vehicle(a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored).
		Mapping: bit 1 : x, bit 2 : y, bit 3 : z, bit 4 : vx, bit 5 : vy, bit 6 : vz, 
				 bit 7 : ax, bit 8 : ay, bit 9 : az, bit 10 : is force setpoint, bit 11 : yaw, bit 12 : yaw rate.*/
		mavlink_msg_set_position_target_local_ned_send(MAVLINK_COMM_0, 963497464, apm_mav_system, apm_mav_component,
			MAV_FRAME_LOCAL_NED, 4088/*0b0000111111111000*/, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	}

	
}
void calScaleLongDown()
{
	scaleLongDown = cosf(osd_lat * DEG_TO_RAD);
	if (scaleLongDown > 1.0f)
		scaleLongDown = 1.0f;
	if (scaleLongDown < 0.01f)
		scaleLongDown = 0.01f;

}
void refreshDistance()
{
	float latd = (target_lat+latoffset - osd_lat) * LATLON_TO_CM;
	float lond = (target_lon +lonoffset- osd_lon) * LATLON_TO_CM * scaleLongDown;
	distance =sqrtf(latd*latd + lond*lond);
	//alt_above_origin
}
void DoMission(int idx)
{
	ack = 100;
	for(int i = 0; i < 10;i++)
	{

		rebuildmissions();
		sendmissionrequest(0, 1);

		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			0,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_TAKEOFF,   //command
			1,							   //current
			1,							   //autocontinue
			1,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			0,                            //x
			0,                        //y

			10
			);
		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			1,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_TAKEOFF,   //command
			1,							   //current
			1,							   //autocontinue
			1,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			0,                            //x
			0,                        //y

			10
			);

		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			2,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_WAYPOINT,   //command
			0,							   //current
			1,							   //autocontinue
			3,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			target_lat + latoffset,                            //x
			target_lon + lonoffset,                        //y

			10
			);

		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			3,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_RETURN_TO_LAUNCH,   //command
			0,							   //current
			1,							   //autocontinue
			5,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			0,                            //x
			0,                        //y

			0
			);
		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			4,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_LAND,   //command
			0,							   //current
			1,							   //autocontinue
			0,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			0,                            //x
			0,                        //y

			0
			);
	}
	return;
	switch (idx)
	{
	case 0:
		rebuildmissions();
		break;
	case 1:
		
		mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,
			0,                             //seq
			(uint8_t)MAV_FRAME_GLOBAL,
			(uint8_t)MAV_CMD_NAV_TAKEOFF,   //command
			1,							   //current
			1,							   //autocontinue
			0,                             //param1
			0,                             //p2
			0,                             //p3
			0,                             //p4
			0,                             //x
			0,                             //y
			10.0f
			);
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	default:
		break;
	}
}

void SetCurrent(int idx)
{
	mavlink_msg_mission_current_send(MAVLINK_COMM_0, idx);
}
void sendmissionrequest(int16_t start, int16_t end)
{
	//mavlink_msg_mission_write_partial_list_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component, start, end);
	mavlink_msg_mission_count_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,5);
}
void rebuildmissions()
{
	mavlink_msg_mission_clear_all_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component);

	//mavlink_msg_mission_item_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component,)
}


void read_mavlink(){
	mavlink_message_t msg;
	mavlink_status_t status;

	//grabing data 
	while (Serial.available() > 0) {
		uint8_t c = Serial.read();

		/* allow CLI to be started by hitting enter 3 times, if no
		heartbeat packets have been received */
		if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
			if (c == '\n' || c == '\r') {
				crlf_count++;
			}
			else {
				crlf_count = 0;
			}
			if (crlf_count == 3) {
				uploadFont();
			}
		}

		//trying to grab msg  
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			mavlink_active = 1;
			//handle msg
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
											 mavbeat = 1;
											 apm_mav_system = msg.sysid;
											 apm_mav_component = msg.compid;
											 apm_mav_type = mavlink_msg_heartbeat_get_type(&msg);
											 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
											 osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
											 //Mode (arducoper armed/disarmed)
											 base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
											 if (getBit(base_mode, 7)) motor_armed = 1;
											 else motor_armed = 0;

											 osd_nav_mode = 0;
											 lastMAVBeat = millis();
											 if (waitingMAVBeats == 1){
												 enable_mav_request = 1;
											 }
			}
				break;
			case MAVLINK_MSG_ID_SYS_STATUS:
			{

											  osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
											  osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)         
											  osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
											  //osd_mode = apm_mav_component;//Debug
											  //osd_nav_mode = apm_mav_system;//Debug
			}
				break;

			case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
											   uint8_t	fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);
											   if (fixtype != 100)
											   {
												   osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
												   osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
												   osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
												   osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);

												   if (scaleLongDownCaled == 0)
												   {
													   calScaleLongDown();
													   scaleLongDownCaled = 1;
												   }
											   }
											   else
											   {
												   if (gpslocked == 0)
												   {
													   target_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
													   target_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
													   target_alt = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000.0f;
													   target_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
												   }
											   }
											   refreshDistance();

			}
				break;
			case MAVLINK_MSG_ID_VFR_HUD:
			{
										   osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
										   osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
										   osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
										   osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
										   //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
										   //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
										   osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
										   osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
			}
				break;
			case MAVLINK_MSG_ID_ATTITUDE:
			{
											osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
											osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
											osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
			}
				break;
			case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			{
														 nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
														 nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
														 nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
														 wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
														 wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
														 alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
														 aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
														 xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
			}
				break;
			case MAVLINK_MSG_ID_MISSION_CURRENT:
			{
												   wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
			}
				break;
			case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			{
												   chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
												   chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
												   chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
												   chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
												   osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
												   osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
												   osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
												   osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);

												   onRCInput();
												   osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
			}
				break;
			case MAVLINK_MSG_ID_WIND:
			{
										osd_winddirection = mavlink_msg_wind_get_direction(&msg); // 0..360 deg, 0=north
										osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
										osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
			}
				break;

			case MAVLINK_MSG_ID_MISSION_ACK:
			{
									ack=		   mavlink_msg_mission_ack_get_type(&msg);
			}
				break;
			default:
				//Do nothing
				break;
			}
		}
		delayMicroseconds(138);
		//next one
	}
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
	parse_error += status.parse_error;

}

