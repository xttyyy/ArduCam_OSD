/******* STARTUP PANEL *******/

void startPanels(){
    osd.clear();
    panLogo(); // Display our logo  
    do_converts(); // load the unit conversion preferences
}

//------------------ Panel: Startup ArduCam OSD LOGO -------------------------------

void panLogo(){
    osd.setPanel(7, 4);
    osd.openPanel();
    osd.printf_P(PSTR("Configurable Osd v2.2"));
    osd.closePanel();
}

/******* PANELS - POSITION *******/

void writePanels(){


	if (millis() < (lastMAVBeat + 2200))
	{

		osd.setPanel(3, 10);
		osd.openPanel();
		osd.printf_P(PSTR("Requesting DataStreams..."));
		osd.closePanel();
		/*if (c_configuring == 123)
			panSetup2();
		else
		{
			
			//if (ISd( Warn_BIT)) panWarn(panWarn_XY[0], panWarn_XY[1]); // this must be here so warnings are always checked
			//Testing bits from 8 bit register A 
			if (ISa( Cen_BIT)) panCenter(panCenter_XY[0], panCenter_XY[1]);   //4x2
			if (ISa( Pit_BIT)) panPitch(panPitch_XY[0], panPitch_XY[1]); //5x1
			if (ISa( Rol_BIT)) panRoll(panRoll_XY[0], panRoll_XY[1]); //5x1
			if (ISa( BatA_BIT)) panBatt_A(panBatt_A_XY[0], panBatt_A_XY[1]); //7x1
			//  if(ISa(panel,BatB_BIT)) panBatt_B(panBatt_B_XY[0], panBatt_B_XY[1]); //7x1
			if (ISa( GPSats_BIT)) panGPSats(panGPSats_XY[0], panGPSats_XY[1]); //5x1
			if (ISa( GPL_BIT)) panGPL(panGPL_XY[0], panGPL_XY[1]); //2x1
			if (ISa( GPS_BIT)) panGPS(panGPS_XY[0], panGPS_XY[1]); //12x3
			if (ISa( Bp_BIT)) panBatteryPercent(panBatteryPercent_XY[0], panBatteryPercent_XY[1]); //

			//Testing bits from 8 bit register B
			if (ISb( Rose_BIT)) panRose(panRose_XY[0], panRose_XY[1]);        //13x3
			if (ISb( Head_BIT)) panHeading(panHeading_XY[0], panHeading_XY[1]); //13x3
			if (ISb( MavB_BIT)) panMavBeat(panMavBeat_XY[0], panMavBeat_XY[1]); //13x3

			if (osd_got_home == 1){
				if (ISb( HDis_BIT)) panHomeDis(panHomeDis_XY[0], panHomeDis_XY[1]); //13x3
				if (ISb( HDir_BIT)) panHomeDir(panHomeDir_XY[0], panHomeDir_XY[1]); //13x3
			}

			if (ISb( Time_BIT)) panTime(panTime_XY[0], panTime_XY[1]);
			if (ISb( WDir_BIT)) panWPDir(panWPDir_XY[0], panWPDir_XY[1]); //??x??
			if (ISb( WDis_BIT)) panWPDis(panWPDis_XY[0], panWPDis_XY[1]); //??x??

			//Testing bits from 8 bit register C 
			//if(osd_got_home == 1){
			if (ISc( Alt_BIT)) panAlt(panAlt_XY[0], panAlt_XY[1]); //
			if (ISc( Halt_BIT)) panHomeAlt(panHomeAlt_XY[0], panHomeAlt_XY[1]); //
			if (ISc( Vel_BIT)) panVel(panVel_XY[0], panVel_XY[1]); //
			if (ISc( As_BIT)) panAirSpeed(panAirSpeed_XY[0], panAirSpeed_XY[1]); //

			//}
			if (ISc( Thr_BIT)) panThr(panThr_XY[0], panThr_XY[1]); //
			if (ISc( FMod_BIT)) panFlightMode(panFMod_XY[0], panFMod_XY[1]);  //
			if (ISc( Hor_BIT)) panHorizon(panHorizon_XY[0], panHorizon_XY[1]); //14x5
			if (ISc( CurA_BIT)) panCur_A(panCur_A_XY[0], panCur_A_XY[1]);

			//Testing bits from 8 bit register D 
			//if(ISd(Off_BIT)) panOff(panOff_XY[0], panOff_XY[1]);
			if (ISd( WindS_BIT)) panWindSpeed(panWindSpeed_XY[0], panWindSpeed_XY[1]);
			if (ISd( Climb_BIT)) panClimb(panClimb_XY[0], panClimb_XY[1]);
			if (ISd( Tune_BIT)) panTune(panTune_XY[0], panTune_XY[1]);
			if (ISd( RSSI_BIT)) panRSSI(panRSSI_XY[0], panRSSI_XY[1]); //??x??
			//if(ISd(panel,Eff_BIT)) panEff(panEff_XY[0], panEff_XY[1]);
			//if (ISd( CALLSIGN_BIT)) panCALLSIGN(panCALLSIGN_XY[0], panCALLSIGN_XY[1]);
			
		}*/
		//panConfigure();
	}
	else { // if no mavlink update for 2 secs

		// this could be replaced with a No Mavlink warning so the last seen values still show

		osd.clear();
		waitingMAVBeats = 1;
		// Display our logo and wait... 
		panWaitMAVBeats(5, 10); //Waiting for MAVBeats...
	}

	// OSD debug for development (Shown on top-middle panels) 
#ifdef membug
	osd.setPanel(13,4);
	osd.openPanel();
	osd.printf("%i",freeMem()); 
	osd.closePanel();
#endif

}

/******* PANELS - DEFINITION *******/

/* **************************************************************** */
// Panel  : efficiency
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panEff(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if (osd_throttle > 2){
        if (osd_groundspeed != 0) eff = (float(osd_curr_A * 10) / (osd_groundspeed * converts))* 0.5 + eff * 0.5;
//        eff = eff * 0.2 + eff * 0.8;
          if (eff > 0 && eff <= 9999) osd.printf("%c%4.0f%c", 0x17, (double)eff, 0x82);
          
    }else{
        if (osd_climb < -0.05) {
            
          if (millis() - descendt > 0){
            descendt = millis() + 5000;
            descend = palt - (osd_alt - osd_home_alt);
            palt = (osd_alt - osd_home_alt);
            
            if  (descend != 0 && (osd_alt - osd_home_alt) != 0) glide = ((((osd_home_alt - osd_alt) / (descend / 5)) * osd_groundspeed) * converth)* 0.2 + glide * 0.8;
            }
            osd.printf("%c%4.0f%c", 0x18, (double)glide, high);
        } else if (osd_pitch <= 0){
            osd.printf_P(PSTR("\x18\x20\x20\x90\x91\x20"));
        }
    }

    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panRSSI(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //rssi = (int16_t)osd_rssi;
    //if (rssi > rssical) rssi = rssical;
    //else if (rssi < rssipersent) rssi = rssipersent;

    //if(!rssiraw_on) rssi = (int16_t)((float)(rssi - rssipersent)/(float)(rssical-rssipersent)*100.0f);
    //if (rssi < -99) rssi = -99;
	osd.printf("%c%3i%c", 0xE1, osd_rssi, 0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCALLSIGN
// Needs  : X, Y locations
// Output : Call sign identification
// Size   : 1 x 6Hea  (rows x chars)
// Staus  : done

void panCALLSIGN(int first_col, int first_line){
   /* osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%s", char_call); 
    osd.closePanel();*/
}

/* **************************************************************** */
// Panel  : panSetup
// Needs  : Nothing, uses whole screen
// Output : The settings menu
// Size   : 3 x ?? (rows x chars)
// Staus  : done

//void panSetup(){
//
//    if (millis() > text_timer){
//        text_timer = millis() + 500;
//
//        osd.clear();
//        osd.setPanel(5, 7);
//        osd.openPanel();
//
//        if (chan1_raw_middle == 0 && chan2_raw_middle == 0){
//            chan1_raw_middle = chan1_raw;
//            chan2_raw_middle = chan2_raw;
//        }
//
//        if ((chan2_raw - 100) > chan2_raw_middle ) setup_menu++;  //= setup_menu + 1;
//        else if ((chan2_raw + 100) < chan2_raw_middle ) setup_menu--;  //= setup_menu - 1;
//        if (setup_menu < 0) setup_menu = 0;
//        else if (setup_menu > 2) setup_menu = 2;
//
//        switch (setup_menu){
//        case 0:
//            {
//                osd.printf_P(PSTR("    Overspeed    "));
//                osd.printf("%3.0i%c", overspeed, spe);
//                overspeed = change_val(overspeed, overspeed_ADDR);
//                break;
//            }
//        case 1:
//            {
//                osd.printf_P(PSTR("   Stall Speed   "));
//                osd.printf("%3.0i%c", stall , spe);
//                //overwritedisplay();
//                stall = change_val(stall, stall_ADDR);
//                break;
//            }
//        case 2:
//            {
//                osd.printf_P(PSTR("Battery warning "));
//                osd.printf("%3.1f%c", float(battv)/10.0 , 0x76, 0x20);
//                battv = change_val(battv, battv_ADDR);
//                break;
//            }
//            //      case 4:
//            //        osd.printf_P(PSTR("Battery warning "));
//            //        osd.printf("%3.0i%c", battp , 0x25);
//            //        if ((chan1_raw - 100) > chan1_raw_middle ){
//            //        battp = battp - 1;}
//            //        if ((chan1_raw + 100) < chan1_raw_middle ){
//            //        battp = battp + 1;} 
//            //        EEPROM.write(208, battp);
//            //        break;
//        }
//    }
//    osd.closePanel();
//}

int mode = 0;


void panSetup2()
{
	if (millis() > text_timer)
	{
		for (int i = 0; i < 10; i++)
		{
			if (parrent == -1)  		
			{
				osd.setPanel(5, 1 + i);
				osd.openPanel();
				char rowhead = '\x00';
				if (i == rowDeeps)
					rowhead = '\xba';
				osd.printf("%c%s", rowhead,menus[rowDeeps][0]);
				osd.closePanel();
			}
			else
			{
				osd.setPanel(5, 1 + i);
				osd.openPanel();
				char rowhead = '\x00';
				if (i == rowDeeps)
					rowhead = '\xba';
				osd.printf("%c%s", rowhead, menus[parrent][rowDeeps]);
				osd.closePanel();

				osd.setPanel(15, 1 + i);
				osd.openPanel();
				rowhead = '\x00';
				int value = values[parrent][rowDeeps];

				if (i == rowDeeps && menuDeeps >= 2)
				{
					value = newValue;
					rowhead = '\xbb';
				}

				
				
				osd.printf("%c%5i", rowhead, value);
				osd.closePanel();
			}
		}

		switch (mode)
		{
		case 0:
			for (int i = 0; i < 3; i++)
			{
				
					osd.setPanel(3+i, 3+i);
					osd.openPanel();
					osd.printf("%i",i);
					osd.closePanel();
				
			}
			//osd.closePanel();
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		default :
			break;
		}
		osd.setPanel(8, 8);
		osd.openPanel();
		osd.printf("mode: %i", mode);
		osd.closePanel();
	}
	/*if (millis() > text_timer)
	{
		text_timer = millis() + 500;

		osd.clear();
		osd.setPanel(1, 1);
		osd.openPanel();

		if (chan1_raw_middle == 0 && chan2_raw_middle == 0){
			chan1_raw_middle = chan1_raw;
			chan2_raw_middle = chan2_raw;
		}

		if ((chan2_raw - 100) > chan2_raw_middle) setup_menu++;  //= setup_menu + 1;
		else if ((chan2_raw + 100) < chan2_raw_middle) setup_menu--;  //= setup_menu - 1;
		if (setup_menu < 0) setup_menu = 0;
		else if (setup_menu > 2) setup_menu = 2;

		switch (setup_menu){
		case 0:
		{
				  osd.printf_P(PSTR("    Overspeed    "));
				  osd.printf("%3.0i%c", overspeed, spe);
				  overspeed = change_val(overspeed, overspeed_ADDR);
				  break;
		}
		case 1:
		{
				  osd.printf_P(PSTR("   Stall Speed   "));
				  osd.printf("%3.0i%c", stall, spe);
				  //overwritedisplay();
				  stall = change_val(stall, stall_ADDR);
				  break;
		}
		case 2:
		{
				  osd.printf_P(PSTR("Battery warning "));
				  osd.printf("%3.1f%c", float(battv) / 10.0, 0x76, 0x20);
				  battv = change_val(battv, battv_ADDR);
				  break;
		}
			//      case 4:
			//        osd.printf_P(PSTR("Battery warning "));
			//        osd.printf("%3.0i%c", battp , 0x25);
			//        if ((chan1_raw - 100) > chan1_raw_middle ){
			//        battp = battp - 1;}
			//        if ((chan1_raw + 100) < chan1_raw_middle ){
			//        battp = battp + 1;} 
			//        EEPROM.write(208, battp);
			//        break;
		}
	}
	osd.closePanel();*/
}
void panConfigure()
{
	unsigned long ms = millis();
	if (ms > text_timer)
	{
		//osd.clear();

		text_timer = ms + 100;
		osd.setPanel(1,1);
		osd.openPanel();
		osd.printf("c_getmiddle: %i", c_getchan1middle);
		osd.closePanel();


		osd.setPanel(1, 2);
		osd.openPanel();
		osd.printf("chan1_raw: %i", chan1_raw);
		osd.closePanel();

		osd.setPanel(1, 3);
		osd.openPanel();
		osd.printf("c_chan1_middle: %i", c_chan1_middle);
		osd.closePanel();

		osd.setPanel(1, 4);
		osd.openPanel();
		osd.printf("c_chan1_rev: %i",  c_chan1_rev);
		osd.closePanel();

		osd.setPanel(1,5);
		osd.openPanel();
		osd.printf("c_last_chan1_move_time: %i", c_last_chan1_move_time);
		osd.closePanel();

		osd.setPanel(1, 6);
		osd.openPanel();
		osd.printf("c_configuring: %i", c_configuring);
		osd.closePanel();

		osd.setPanel(1, 7);
		osd.openPanel();
		osd.printf("menuDeeps: %i", menuDeeps);
		osd.closePanel();
	}
			//osd.closePanel();

	
}
//int change_val(int value, int address)
//{
//    uint8_t value_old = value;
//    if (chan1_raw > chan1_raw_middle + 100) value--;
//    if (chan1_raw  < chan1_raw_middle - 100) value++;
//
//    if(value != value_old && setup_menu ) EEPROM.write(address, value);
//    return value;
//}

/* **************************************************************** */
// Panel  : pan wind speed
// Needs  : X, Y locations
// Output : Wind direction symbol (arrow) and velocity
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panWindSpeed(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();

    osd_wind_arrow_rotate_int = round((osd_winddirection - osd_heading)/360.0 * 16.0) + 1; //Convert to int 1-16 
    if(osd_wind_arrow_rotate_int < 0 ) osd_wind_arrow_rotate_int += 16; //normalize
    showArrow((uint8_t)osd_wind_arrow_rotate_int,1); //print data to OSD

    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panOff
// Needs  : X, Y locations
// Output : OSD off
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

//void panOff(){
//    if (ch_toggle == 4){
//        if (((apm_mav_type == 1) && ((osd_mode != 11) && (osd_mode != 1))) || ((apm_mav_type == 2) && ((osd_mode != 6) && (osd_mode != 7)))){
//            if (osd_off_switch != osd_mode){ 
//                osd_off_switch = osd_mode;
//                osd_switch_time = millis();
//
//                if (osd_off_switch == osd_switch_last){
//                    switch(panel){
//                    case 0:
//                        {
//                            panel = 1;                                                        
//                            if (millis() <= 60000){
//                                osd_set = 1;
//                            }else{
//                                osd_set = 0;
//                            }                            
//                            break;
//                        }
//                    case 1:
//                        {
//                            panel = npanels;
//                            osd_set = 0;                            
//                            break;
//                        }
//                    case npanels:
//                        {
//                            panel = 0;
//                            break;
//                        }
//                    }
//                    osd.clear();
//                }
//            }
//            if ((millis() - osd_switch_time) > 2000){
//                osd_switch_last = osd_mode;
//            }
//        }
//    }
//    else {
//        if(ch_toggle == 5) ch_raw = osd_chan5_raw;
//        else if(ch_toggle == 6) ch_raw = osd_chan6_raw;
//        else if(ch_toggle == 7) ch_raw = osd_chan7_raw;
//        else if(ch_toggle == 8) ch_raw = osd_chan8_raw;
//
//        if (switch_mode == 0){
//            if (ch_raw > 1800) {
//                if (millis() <= 60000){
//                    osd_set = 1;
//                }
//                else if (osd_set != 1 && warning != 1){
//                    osd.clear();
//                }
//                panel = npanels; //off panel
//            }
//            else if (ch_raw < 1200 && panel != 0) { //first panel
//                osd_set = 0;
//                osd.clear();
//                panel = 0;
//            }    
//
//            else if (ch_raw >= 1200 && ch_raw <= 1800 && setup_menu != 6 && panel != 1 && warning != 1) { //second panel
//                osd_set = 0;
//                osd.clear();
//                panel = 1;
//            }        
//        } else {
//
//            if (ch_raw > 1200)
//                if (millis() <= 60000 && osd_set != 1){
//                    if (osd_switch_time + 1000 < millis()){
//                        osd_set = 1;
//                        osd_switch_time = millis();
//                    }
//                } else {
//                    if (osd_switch_time + 1000 < millis()){
//                        osd_set = 0;
//                        osd.clear();
//                        if (panel == npanels) {
//                            panel = 0;
//                        } else {
//                            panel++;
//                        }
//                        if (panel > 1) panel = npanels;
//                        osd_switch_time = millis();
//                    }
//                }
//        }    
//    }
//}
//* **************************************************************** */
// Panel  : panTune
// Needs  : X, Y locations
// Output : Current symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done
    
  void panTune(int first_col, int first_line){
  osd.setPanel(first_col, first_line);
  osd.openPanel();

  osd.printf("%c%c%2.0f%c|%c%c%2.0f%c|%c%c%4.0i%c|%c%c%4.0i%c|%c%c%3.0f%c|%c%c%3.0f%c|%c%c%4.0f%c", 0x4E, 0x52, (nav_roll), 0xB0, 0x4E, 0x50, (nav_pitch), 0xB0, 0x4E, 0x48, (nav_bearing), 0xB0, 0x54, 0x42, (wp_target_bearing), 0xB0, 0x41, 0x45, (alt_error * converth), high, 0x58, 0x45, (xtrack_error), 0x6D, 0x41, 0x45, ((aspd_error / 100.0) * converts), spe);

  osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panCur_A(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE4, (float(osd_curr_A) * .01), 0x8F);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panAlt(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd.printf("%c%5.0f%c",0x85, (double)(osd_alt - osd_home_alt), 0x8D);
    osd.printf("%c%5.0f%c",0xE6, (double)(osd_alt * converth), high);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panClimb(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0f%c",0x16, (double)(osd_climb), 0x88);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Alt symbol and home altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panHomeAlt(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd.printf("%c%5.0f%c",0x85, (double)(osd_alt - osd_home_alt), 0x8D);
    osd.printf("%c%5.0f%c",0xE7, (double)((osd_alt - osd_home_alt) * converth), high);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panVel(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0f%c",0xE9,(double)(osd_groundspeed * converts),spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAirSpeed
// Needs  : X, Y locations
// Output : Airspeed value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panAirSpeed(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0f%c", 0xE8, (double)(osd_airspeed * converts), spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Airspeed value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panWarn(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();

    if (millis() > text_timer){ // if the text has been shown for a while
        if (warning_type != 0) {
            last_warning = warning_type; // save the warning type for cycling
            warning_type = 0; // blank the text
            warning = 1;
            warning_timer = millis();            
        } else {
            if ((millis() - 10000) > warning_timer ) warning = 0;

            int x = last_warning; // start the warning checks where we left it last time
            while (warning_type == 0) { // cycle through the warning checks
                x++;
                if (x > 5) x = 1; // change the 6 if you add more warning types
                switch(x) {
                case 1:
                    if ((osd_fix_type) < 2) warning_type = 1; // No GPS Fix
                    break;
                case 2:
                    //if (osd_airspeed * converts < stall && osd_airspeed > 1.12) warning_type = 2;
                    break;
                case 3:
                    //if ((osd_airspeed * converts) > (float)overspeed) warning_type = 3;
                    break;
                case 4:
                    if (osd_vbat_A < float(battv)/10.0 || osd_battery_remaining_A < batt_warn_level) warning_type = 4;
                    break;
                case 5:
                    //if (rssi < rssi_warn_level && rssi != -99 && !rssiraw_on) warning_type = 5;
                    break;    
                }
                if (x == last_warning) break; // if we've done a full cycle then there mustn't be any warnings
            }
        }

        text_timer = millis() + 1000; // blink every 1 secs
        //if (warning == 1){ 
        //    if (panel == 1) osd.clear();
        //    panel = 0; // turn OSD on if there is a warning                  
        //}
        char* warning_string;
        if (motor_armed == 0){
            warning_string = "\x20\x20\x44\x49\x53\x41\x52\x4d\x45\x44\x20\x20";      
        }else{
            switch(warning_type){ 
            case 0:
                //osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
                //warning_string = "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20";
                break;   
            case 1:  
                //osd.printf_P(PSTR("\x20\x4E\x6F\x20\x47\x50\x53\x20\x66\x69\x78\x21"));
                warning_string = "\x20\x4E\x6F\x20\x47\x50\x53\x20\x66\x69\x78\x21";
                break;
            case 2:
                //osd.printf_P(PSTR("\x20\x20\x20\x53\x74\x61\x6c\x6c\x21\x20\x20\x20"));
                //warning_string = "\x20\x20\x20\x53\x74\x61\x6c\x6c\x21\x20\x20\x20";
                break;
            case 3:
                //osd.printf_P(PSTR("\x20\x4f\x76\x65\x72\x53\x70\x65\x65\x64\x21\x20"));
                //warning_string = "\x20\x4f\x76\x65\x72\x53\x70\x65\x65\x64\x21\x20";
                break;
            case 4:
                //osd.printf_P(PSTR("\x42\x61\x74\x74\x65\x72\x79\x20\x4c\x6f\x77\x21"));
                warning_string = "\x42\x61\x74\x74\x65\x72\x79\x20\x4c\x6f\x77\x21";
                break;
            case 5:
                //osd.printf_P(PSTR("\x42\x61\x74\x74\x65\x72\x79\x20\x4c\x6f\x77\x21"));
                //warning_string = "\x20\x20\x4c\x6f\x77\x20\x52\x73\x73\x69\x20\x20";
                break;
                //        case 6:
                //osd.printf_P(PSTR("\x42\x61\x74\x74\x65\x72\x79\x20\x4c\x6f\x77\x21"));
                //            warning_string = "\x20\x20\x44\x49\x53\x41\x52\x4d\x45\x44\x20\x20";
                //            break;
            }
        }
        osd.printf("%s",warning_string);
    }
    osd.closePanel();
}

  
/* **************************************************************** */
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panThr(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0i%c",0x87,osd_throttle,0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery state from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panBatteryPercent(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0i%c", 0xB9, osd_battery_remaining_A, 0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panTime
// Needs  : X, Y locations
// Output : Time from start with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panTime(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    start_Time = millis()/1000;
    osd.printf("%c%2i%c%02i", 0xB3,((int)start_Time/60)%60,0x3A,(int)start_Time%60);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Home Symbol with distance to home in meters
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panHomeDis(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c", 0x1F, (double)((osd_home_distance) * converth), high);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCenter
// Needs  : X, Y locations
// Output : 2 row croshair symbol created by 2 x 4 chars
// Size   : 2 x 4  (rows x chars)
// Staus  : done

void panCenter(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("\x05\x03\x04\x05|\x15\x13\x14\x15"));
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : 12 x 4 Horizon line surrounded by 2 cols (left/right rules)
// Size   : 14 x 4  (rows x chars)
// Staus  : done

void panHorizon(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xd8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xd9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9"));
    osd.closePanel();
    showHorizon((first_col + 1), first_line);
}

/* **************************************************************** */
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panPitch(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c",osd_pitch,0xb0,0xb1);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panRoll(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c",osd_roll,0xb0,0xb2);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
// Size   : 1 x 8  (rows x chars)
// Staus  : done

void panBatt_A(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    /*************** This commented code is for the next ArduPlane Version
    if(osd_battery_remaining_A > 100){
        osd.printf(" %c%5.2f%c", 0xE2, (double)osd_vbat_A, 0x8E);
    else osd.printf("%c%5.2f%c%c", 0xE2, (double)osd_vbat_A, 0x8E, osd_battery_pic_A);
    */
    osd.printf("%c%5.2f%c", 0xE2, (double)osd_vbat_A, 0x8E);
    osd.closePanel();
}

//------------------ Panel: Waiting for MAVLink HeartBeats -------------------------------

void panWaitMAVBeats(int first_col, int first_line){
    panLogo();
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("Waiting for|MAVLink heartbeats..."));
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPL
// Needs  : X, Y locations
// Output : 1 static symbol with changing FIX symbol
// Size   : 1 x 2  (rows x chars)
// Staus  : done

void panGPL(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    char* gps_str;
    if(osd_fix_type == 0 || osd_fix_type == 1) gps_str = "\x10\x20"; 
        //osd.printf_P(PSTR("\x10\x20"));
    else if(osd_fix_type == 2 || osd_fix_type == 3) gps_str = "\x11\x20";
        //osd.printf_P(PSTR("\x11\x20"));
    osd.printf("%s",gps_str);

    /*  if(osd_fix_type <= 1) {
    osd.printf_P(PSTR("\x10"));
    } else {
    osd.printf_P(PSTR("\x11"));
    }  */
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
// Size   : 1 x 5  (rows x chars)
// Staus  : done

void panGPSats(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%2i", 0x0f,osd_satellites_visible);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols as on first char
// Size   : 2 x 12  (rows x chars)
// Staus  : done

void panGPS(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%11.6f|%c%11.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
// Size   : 1 x 5  (rows x chars)
// Staus  : not ready

void panHeading(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4.0f%c", (double)osd_heading, 0xb0);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
// Size   : 2 x 13  (rows x chars)
// Staus  : done

void panRose(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd_heading  = osd_yaw;
    //if(osd_yaw < 0) osd_heading = 360 + osd_yaw;
    osd.printf("%s|%c%s%c", "\x20\xc0\xc0\xc0\xc0\xc0\xc7\xc0\xc0\xc0\xc0\xc0\x20", 0xd0, buf_show, 0xd1);
    osd.closePanel();
}


/* **************************************************************** */
// Panel  : panBoot
// Needs  : X, Y locations
// Output : Booting up text and empty bar after that
// Size   : 1 x 21  (rows x chars)
// Staus  : done

void panBoot(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("Booting up:\xed\xf2\xf2\xf2\xf2\xf2\xf2\xf2\xf3")); 
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panMavBeat
// Needs  : X, Y locations
// Output : 2 symbols, one static and one that blinks on every 50th received 
//          mavlink packet.
// Size   : 1 x 2  (rows x chars)
// Staus  : done

void panMavBeat(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if(mavbeat == 1){
        osd.printf_P(PSTR("\xEA\xEC"));
        mavbeat = 0;
    }
    else{
        osd.printf_P(PSTR("\xEA\xEB"));
    }
    osd.closePanel();
}


/* **************************************************************** */
// Panel  : panWPDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to next waypoint
// Size   : 1 x 2  (rows x chars)
// Staus  : not ready

void panWPDir(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
   
    wp_target_bearing_rotate_int = round(((float)wp_target_bearing - osd_heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
    if(wp_target_bearing_rotate_int < 0 ) wp_target_bearing_rotate_int += 16; //normalize

    showArrow((uint8_t)wp_target_bearing_rotate_int,0);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panWPDis
// Needs  : X, Y locations
// Output : W then distance in Km - Distance to next waypoint
// Size   : 1 x 2  (rows x chars)
// Staus  : not ready TODO - CHANGE the Waypoint symbol - Now only a W!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void panWPDis(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
      osd.printf("%c%2i%c%4.0f%c",0x57,wp_number,0x0,(double)((float)(wp_dist) * converth),high);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
// Size   : 1 x 2  (rows x chars)
// Status : not tested

void panHomeDir(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    showArrow((uint8_t)osd_home_direction,0);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panFlightMode 
// Needs  : X, Y locations
// Output : 2 symbols, one static name symbol and another that changes by flight modes
// Size   : 1 x 2  (rows x chars)
// Status : done

void panFlightMode(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //char c1 = 0xE0 ;//"; char c2; char c3; char c4; char c5; 
    char* mode_str="";
    if (apm_mav_type == 2){ //ArduCopter MultiRotor or ArduCopter Heli
        if (osd_mode == 0)       mode_str = "stab"; //Stabilize: hold level position
        else if (osd_mode == 1)  mode_str = "acro"; //Acrobatic: rate control
        else if (osd_mode == 2)  mode_str = "alth"; //Altitude Hold: auto control
        else if (osd_mode == 3)  mode_str = "auto"; //Auto: auto control
        else if (osd_mode == 4)  mode_str = "guid"; //Guided: auto control
        else if (osd_mode == 5)  mode_str = "loit"; //Loiter: hold a single location
        else if (osd_mode == 6)  mode_str = "retl"; //Return to Launch: auto control
        else if (osd_mode == 7)  mode_str = "circ"; //Circle: auto control
        else if (osd_mode == 8)  mode_str = "posi"; //Position: auto control
        else if (osd_mode == 9)  mode_str = "land"; //Land:: auto control
        else if (osd_mode == 10) mode_str = "oflo"; //OF_Loiter: hold a single location using optical flow sensor
        else if (osd_mode == 11) mode_str = "drif"; //Drift mode: 
        else if (osd_mode == 13) mode_str = "sprt"; //Sport: earth frame rate control
        else if (osd_mode == 14) mode_str = "flip"; //Flip: flip the vehicle on the roll axis
        else if (osd_mode == 15) mode_str = "atun"; //Auto Tune: autotune the vehicle's roll and pitch gains
        else if (osd_mode == 16) mode_str = "hybr"; //Hybrid: position hold with manual override
    } else if(apm_mav_type == 1){ //ArduPlane
        if (osd_mode == 0)       mode_str = "manu"; //Manual
        else if (osd_mode == 1)  mode_str = "circ"; //Circle
        else if (osd_mode == 2)  mode_str = "stab"; //Stabilize
        else if (osd_mode == 3)  mode_str = "trng"; //Training
        else if (osd_mode == 4)  mode_str = "acro"; //Acro
        else if (osd_mode == 5)  mode_str = "fbwa"; //Fly_By_Wire_A
        else if (osd_mode == 6)  mode_str = "fbwb"; //Fly_By_Wire_B
        else if (osd_mode == 7)  mode_str = "crui"; //Cruise
        else if (osd_mode == 8)  mode_str = "atun"; //Auto Tune
        else if (osd_mode == 10) mode_str = "auto"; //Auto
        else if (osd_mode == 11) mode_str = "retl"; //Return to Launch
        else if (osd_mode == 12) mode_str = "loit"; //Loiter
        else if (osd_mode == 15) mode_str = "guid"; //Guided
        else if (osd_mode == 16) mode_str = "init"; //Initializing
    }
    osd.printf("%c%s", 0xE0, mode_str);
    osd.closePanel();
}


// ---------------- EXTRA FUNCTIONS ----------------------
// Show those fancy 2 char arrows
void showArrow(uint8_t rotate_arrow,uint8_t method) {  
    char arrow_set1 = 0x0;
    char arrow_set2 = 0x0;   
    switch(rotate_arrow) {
    case 0: 
        arrow_set1 = 0x90;
        arrow_set2 = 0x91;
        break;
    case 1: 
        arrow_set1 = 0x90;
        arrow_set2 = 0x91;
        break;
    case 2: 
        arrow_set1 = 0x92;
        arrow_set2 = 0x93;
        break;
    case 3: 
        arrow_set1 = 0x94;
        arrow_set2 = 0x95;
        break;
    case 4: 
        arrow_set1 = 0x96;
        arrow_set2 = 0x97;
        break;
    case 5: 
        arrow_set1 = 0x98;
        arrow_set2 = 0x99;
        break;
    case 6: 
        arrow_set1 = 0x9A;
        arrow_set2 = 0x9B;
        break;
    case 7: 
        arrow_set1 = 0x9C;
        arrow_set2 = 0x9D;
        break;
    case 8: 
        arrow_set1 = 0x9E;
        arrow_set2 = 0x9F;
        break;
    case 9: 
        arrow_set1 = 0xA0;
        arrow_set2 = 0xA1;
        break;
    case 10: 
        arrow_set1 = 0xA2;
        arrow_set2 = 0xA3;
        break;
    case 11: 
        arrow_set1 = 0xA4;
        arrow_set2 = 0xA5;
        break;
    case 12: 
        arrow_set1 = 0xA6;
        arrow_set2 = 0xA7;
        break;
    case 13: 
        arrow_set1 = 0xA8;
        arrow_set2 = 0xA9;
        break;
    case 14: 
        arrow_set1 = 0xAA;
        arrow_set2 = 0xAB;
        break;
    case 15: 
        arrow_set1 = 0xAC;
        arrow_set2 = 0xAd;
        break;
    case 16: 
        arrow_set1 = 0xAE;
        arrow_set2 = 0xAF;
        break;
    } 
    if(method == 1) osd.printf("%c%3.0f%c|%c%c",0xFC,(double)(osd_windspeed * converts),spe, arrow_set1, arrow_set2);
    else osd.printf("%c%c", arrow_set1, arrow_set2);
}

// Calculate and shows Artificial Horizon
void showHorizon(int start_col, int start_row) { 

    int x, nose, row, minval, hit, subval = 0;
    const int cols = 12;
    const int rows = 5;
    int col_hit[cols];
    float  pitch, roll;

    (abs(osd_pitch) == 90)?pitch = 89.99 * (90/osd_pitch) * -0.017453293:pitch = osd_pitch * -0.017453293;
    (abs(osd_roll) == 90)?roll = 89.99 * (90/osd_roll) * 0.017453293:roll = osd_roll * 0.017453293;

    nose = round(tan(pitch) * (rows*9));
    for(int col=1;col <= cols;col++){
        x = (col * 12) - (cols * 6) - 6;//center X point at middle of each col
        col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;//calculating hit point on Y plus offset to eliminate negative values
        //col_hit[(col-1)] = nose + (rows * 9);
    }

    for(int col=0;col < cols; col++){
        hit = col_hit[col];
        if(hit > 0 && hit < (rows * 18)){
            row = rows - ((hit-1)/18);
            minval = rows*18 - row*18 + 1;
            subval = hit - minval;
            subval = round((subval*9)/18);
            if(subval == 0) subval = 1;
            printHit(start_col + col, start_row + row - 1, subval);
        }
    }
}

void printHit(byte col, byte row, byte subval){
    osd.openSingle(col, row);
    char subval_char;
        switch (subval){
        case 1:
            //osd.printf_P(PSTR("\x06"));
            subval_char = 0x06;
            break;
        case 2:
            //osd.printf_P(PSTR("\x07"));
            subval_char = 0x07; 
            break;
        case 3:
            //osd.printf_P(PSTR("\x08"));
            subval_char = 0x08;
            break;
        case 4:
            //osd.printf_P(PSTR("\x09"));
            subval_char = 0x09;
            break;
        case 5:
            //osd.printf_P(PSTR("\x0a"));
            subval_char = 0x0a; 
            break;
        case 6:
            //osd.printf_P(PSTR("\x0b"));
            subval_char = 0x0b;
            break;
        case 7:
            //osd.printf_P(PSTR("\x0c"));
            subval_char = 0x0c;
            break;
        case 8:
            //osd.printf_P(PSTR("\x0d"));
            subval_char = 0x0d;
            break;
        case 9:
            //osd.printf_P(PSTR("\x0e"));
            subval_char = 0x0e;
            break;
        }
        osd.printf("%c", subval_char);

}

void do_converts()
{
    if (EEPROM.read(measure_ADDR) == 0) {
        converts = 3.6;
        converth = 1.0;
        spe = 0x81;
        high = 0x8D;
    } else {
        converts = 2.23;
        converth = 3.28;
        spe = 0xfb;
        high = 0x66;
    }
}

