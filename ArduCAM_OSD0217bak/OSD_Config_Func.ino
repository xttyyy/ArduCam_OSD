/* ******************************************************************/
/* *********************** GENERAL FUNCTIONS ********************** */

//Extract functions (get bits from the positioning bytes
#define ISa(whichBit) getBit(panA_REG, whichBit)
#define ISb(whichBit) getBit(panB_REG, whichBit)
#define ISc(whichBit) getBit(panC_REG, whichBit)
#define ISd(whichBit) getBit(panD_REG, whichBit)


boolean getBit(byte Reg, byte whichBit) {
    boolean State;
    State = Reg & (1 << whichBit);
    return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat) {
    if (stat) {
        Reg = Reg | (1 << whichBit);
    } 
    else {
        Reg = Reg & ~(1 << whichBit);
    }
    return Reg;
}

// EEPROM reader
// Functions for reading from the EEPROM

byte readEEPROM(int address) {
    return EEPROM.read(address);
}

void readSettings() {
    //overspeed = EEPROM.read(overspeed_ADDR);
    //stall = EEPROM.read(stall_ADDR);
    battv = EEPROM.read(battv_ADDR);
    //switch_mode = EEPROM.read(switch_mode_ADDR);

    //ch_toggle = EEPROM.read(ch_toggle_ADDR);

    //rssical = EEPROM.read(OSD_RSSI_HIGH_ADDR);
    //rssipersent = EEPROM.read(OSD_RSSI_LOW_ADDR);
    //rssiraw_on = EEPROM.read(OSD_RSSI_RAW_ADDR);

    batt_warn_level = EEPROM.read(OSD_BATT_WARN_ADDR);
    //rssi_warn_level = EEPROM.read(OSD_RSSI_WARN_ADDR);
    int i;
    //for(i=0;i < OSD_CALL_SIGN_TOTAL;i++) 
    //{
    //    char_call[i] = EEPROM.read(OSD_CALL_SIGN_ADDR + i);
    //    if(char_call[i] == 0) break;
    //}
    //char_call[i+1] ='\0'; //null terminate the string 

}

void readPanelSettings() {

    //****** First set of 8 Panels ******
    //uint16_t offset = OffsetBIT0 * 0;

    setBit(panA_REG, Cen_BIT, readEEPROM(panCenter_en_ADDR ));
    panCenter_XY[0] = readEEPROM(panCenter_x_ADDR );
    panCenter_XY[1] = checkPAL(readEEPROM(panCenter_y_ADDR ));

    setBit(panA_REG, Bp_BIT, readEEPROM(panBatteryPercent_en_ADDR ));
    panBatteryPercent_XY[0] = readEEPROM(panBatteryPercent_x_ADDR );
    panBatteryPercent_XY[1] = checkPAL(readEEPROM(panBatteryPercent_y_ADDR ));

    setBit(panA_REG, Pit_BIT, readEEPROM(panPitch_en_ADDR ));
    panPitch_XY[0] = readEEPROM(panPitch_x_ADDR );
    panPitch_XY[1] = checkPAL(readEEPROM(panPitch_y_ADDR ));

    setBit(panA_REG, Rol_BIT, readEEPROM(panRoll_en_ADDR ));
    panRoll_XY[0] = readEEPROM(panRoll_x_ADDR );
    panRoll_XY[1] = checkPAL(readEEPROM(panRoll_y_ADDR ));

    setBit(panA_REG, BatA_BIT, readEEPROM(panBatt_A_en_ADDR ));
    panBatt_A_XY[0] = readEEPROM(panBatt_A_x_ADDR );
    panBatt_A_XY[1] = checkPAL(readEEPROM(panBatt_A_y_ADDR ));

    //setBit(panA_REG, BatB_BIT, readEEPROM(panBatt_B_en_ADDR));
    //panBatt_B_XY[0] = readEEPROM(panBatt_B_x_ADDR);
    //panBatt_B_XY[1] = checkPAL(readEEPROM(panBatt_B_y_ADDR));

    setBit(panA_REG, GPSats_BIT, readEEPROM(panGPSats_en_ADDR ));
    panGPSats_XY[0] = readEEPROM(panGPSats_x_ADDR );
    panGPSats_XY[1] = checkPAL(readEEPROM(panGPSats_y_ADDR ));

    setBit(panA_REG, GPL_BIT, readEEPROM(panGPL_en_ADDR ));
    panGPL_XY[0] = readEEPROM(panGPL_x_ADDR );
    panGPL_XY[1] = checkPAL(readEEPROM(panGPL_y_ADDR ));

    setBit(panA_REG, GPS_BIT, readEEPROM(panGPS_en_ADDR ));
    panGPS_XY[0] = readEEPROM(panGPS_x_ADDR );
    panGPS_XY[1] = checkPAL(readEEPROM(panGPS_y_ADDR ));

    //****** Second set of 8 0s ******

    setBit(panB_REG, Rose_BIT, readEEPROM(panRose_en_ADDR ));
    panRose_XY[0] = readEEPROM(panRose_x_ADDR );
    panRose_XY[1] = checkPAL(readEEPROM(panRose_y_ADDR ));

    setBit(panB_REG, Head_BIT, readEEPROM(panHeading_en_ADDR ));
    panHeading_XY[0] = readEEPROM(panHeading_x_ADDR );
    panHeading_XY[1] = checkPAL(readEEPROM(panHeading_y_ADDR ));

    setBit(panB_REG, MavB_BIT, readEEPROM(panMavBeat_en_ADDR ));
    panMavBeat_XY[0] = readEEPROM(panMavBeat_x_ADDR );
    panMavBeat_XY[1] = checkPAL(readEEPROM(panMavBeat_y_ADDR ));

    setBit(panB_REG, HDis_BIT, readEEPROM(panHomeDis_en_ADDR ));
    panHomeDis_XY[0] = readEEPROM(panHomeDis_x_ADDR );
    panHomeDis_XY[1] = checkPAL(readEEPROM(panHomeDis_y_ADDR ));

    setBit(panB_REG, HDir_BIT, readEEPROM(panHomeDir_en_ADDR ));
    panHomeDir_XY[0] = readEEPROM(panHomeDir_x_ADDR );
    panHomeDir_XY[1] = checkPAL(readEEPROM(panHomeDir_y_ADDR ));

    setBit(panB_REG, WDir_BIT, readEEPROM(panWPDir_en_ADDR ));
    panWPDir_XY[0] = readEEPROM(panWPDir_x_ADDR );
    panWPDir_XY[1] = checkPAL(readEEPROM(panWPDir_y_ADDR ));

    setBit(panB_REG, WDis_BIT, readEEPROM(panWPDis_en_ADDR ));
    panWPDis_XY[0] = readEEPROM(panWPDis_x_ADDR );
    panWPDis_XY[1] = checkPAL(readEEPROM(panWPDis_y_ADDR ));

    setBit(panB_REG, Time_BIT, readEEPROM(panTime_en_ADDR ));
    panTime_XY[0] = readEEPROM(panTime_x_ADDR );
    panTime_XY[1] = checkPAL(readEEPROM(panTime_y_ADDR ));

    //setBit(panB_REG, RSSI_BIT, readEEPROM(panRSSI_en_ADDR));
    //panRSSI_XY[0] = readEEPROM(panRSSI_x_ADDR);
    //panRSSI_XY[1] = checkPAL(readEEPROM(panRSSI_y_ADDR));

    //****** Third set of 8 0s ******

    setBit(panC_REG, CurA_BIT, readEEPROM(panCur_A_en_ADDR ));
    panCur_A_XY[0] = readEEPROM(panCur_A_x_ADDR );
    panCur_A_XY[1] = checkPAL(readEEPROM(panCur_A_y_ADDR ));

    //setBit(panC_REG, CurB_BIT, readEEPROM(panCur_B_en_ADDR));
    //panCur_B_XY[0] = readEEPROM(panCur_B_x_ADDR);
    //panCur_B_XY[1] = checkPAL(readEEPROM(panCur_B_y_ADDR));

    setBit(panC_REG, Alt_BIT, readEEPROM(panAlt_en_ADDR ));
    panAlt_XY[0] = readEEPROM(panAlt_x_ADDR );
    panAlt_XY[1] = checkPAL(readEEPROM(panAlt_y_ADDR ));

    setBit(panC_REG, Halt_BIT, readEEPROM(panHomeAlt_en_ADDR ));
    panHomeAlt_XY[0] = readEEPROM(panHomeAlt_x_ADDR );
    panHomeAlt_XY[1] = checkPAL(readEEPROM(panHomeAlt_y_ADDR ));

    setBit(panC_REG, As_BIT, readEEPROM(panAirSpeed_en_ADDR ));
    panAirSpeed_XY[0] = readEEPROM(panAirSpeed_x_ADDR );
    panAirSpeed_XY[1] = checkPAL(readEEPROM(panAirSpeed_y_ADDR ));

    setBit(panC_REG, Vel_BIT, readEEPROM(panVel_en_ADDR ));
    panVel_XY[0] = readEEPROM(panVel_x_ADDR );
    panVel_XY[1] = checkPAL(readEEPROM(panVel_y_ADDR ));

    setBit(panC_REG, Thr_BIT, readEEPROM(panThr_en_ADDR ));
    panThr_XY[0] = readEEPROM(panThr_x_ADDR );
    panThr_XY[1] = checkPAL(readEEPROM(panThr_y_ADDR ));

    setBit(panC_REG, FMod_BIT, readEEPROM(panFMod_en_ADDR ));
    panFMod_XY[0] = readEEPROM(panFMod_x_ADDR );
    panFMod_XY[1] = checkPAL(readEEPROM(panFMod_y_ADDR ));

    setBit(panC_REG, Hor_BIT, readEEPROM(panHorizon_en_ADDR ));
    panHorizon_XY[0] = readEEPROM(panHorizon_x_ADDR );
    panHorizon_XY[1] = checkPAL(readEEPROM(panHorizon_y_ADDR ));

    setBit(panD_REG, Warn_BIT, readEEPROM(panWarn_en_ADDR ));
    panWarn_XY[0] = readEEPROM(panWarn_x_ADDR );
    panWarn_XY[1] = checkPAL(readEEPROM(panWarn_y_ADDR ));

    //setBit(panD_REG[0], Off_BIT, readEEPROM(panOff_en_ADDR ));
    //panOff_XY[0] = readEEPROM(panOff_x_ADDR );
    //panOff_XY[1] = checkPAL(readEEPROM(panOff_y_ADDR ));

    setBit(panD_REG, WindS_BIT, readEEPROM(panWindSpeed_en_ADDR ));
    panWindSpeed_XY[0] = readEEPROM(panWindSpeed_x_ADDR );
    panWindSpeed_XY[1] = checkPAL(readEEPROM(panWindSpeed_y_ADDR ));

    setBit(panD_REG, Climb_BIT, readEEPROM(panClimb_en_ADDR ));
    panClimb_XY[0] = readEEPROM(panClimb_x_ADDR );
    panClimb_XY[1] = checkPAL(readEEPROM(panClimb_y_ADDR ));

    setBit(panD_REG, Tune_BIT, readEEPROM(panTune_en_ADDR ));
    panTune_XY[0] = readEEPROM(panTune_x_ADDR );
    panTune_XY[1] = checkPAL(readEEPROM(panTune_y_ADDR ));

    setBit(panD_REG, Eff_BIT, readEEPROM(panEff_en_ADDR ));
    panEff_XY[0] = readEEPROM(panEff_x_ADDR );
    panEff_XY[1] = checkPAL(readEEPROM(panEff_y_ADDR ));

    setBit(panD_REG, CALLSIGN_BIT, readEEPROM(panCALLSIGN_en_ADDR ));
    panCALLSIGN_XY[0] = readEEPROM(panCALLSIGN_x_ADDR );
    panCALLSIGN_XY[1] = checkPAL(readEEPROM(panCALLSIGN_y_ADDR ));
}

int checkPAL(int line){
    if(line >= osd.getCenter() && osd.getMode() == 0){
        line -= 3;//Cutting lines offset after center if NTSC
    }
    return line;
}

