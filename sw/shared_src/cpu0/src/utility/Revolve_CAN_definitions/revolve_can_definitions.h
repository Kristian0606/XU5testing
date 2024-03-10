#ifndef REVOLVE_CAN_DEFINITIONS_H_
#define REVOLVE_CAN_DEFINITIONS_H_

// ID
// Module ID
// Bytemapping

// Overview of the can id distribution
// 0x000         Oblig DV IDs
// 0x100        Inverter setpoints for AMK and Simian
// 0x200        Inverter data output from AMK and Simian
// 0x300        VCU outputs
// 0x400        SBS , DASH , IMD(shutdown) , FAN control
// 0x500        Whole field reserved
// 0x600        Parameter communication VCU
// 0x700

//------------------------------------------------------------------------------
/*    -------    ARBITRATION ID DEFINITIONS ---------------------------------------*/
//------------------------------------------------------------------------------
//- ID[3..0]
#define CANR_MODULE_ID0_ID      (0x00)
#define CANR_MODULE_ID1_ID      (0x01)
#define CANR_MODULE_ID2_ID      (0x02)
#define CANR_MODULE_ID3_ID      (0x03)
#define CANR_MODULE_ID4_ID      (0x04)
#define CANR_MODULE_ID5_ID      (0x05)
#define CANR_MODULE_ID6_ID      (0x06)
#define CANR_MODULE_ID7_ID      (0x07)
#define CANR_MODULE_ID8_ID      (0x08)
#define CANR_MODULE_ID9_ID      (0x09)
#define CANR_MODULE_ID10_ID     (0x0A)
#define CANR_MODULE_ID11_ID     (0x0B)
#define CANR_MODULE_ID12_ID     (0x0C)
#define CANR_MODULE_ID13_ID     (0x0D)
#define CANR_MODULE_ID14_ID     (0x0E)
#define CANR_MODULE_ID15_ID     (0x0F)


//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x000  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x000_ID           (0x000)
                                            // 0x000-0x00F        NMT Node Control DV OBLIG        whole field reserved for DV
                                            // 0x010-0x01F
                                            // 0x020-0x02F
                                            // 0x030-0x03F
                                            // 0x040-0x04F
#define CAN_GRP_VCU_DEBUG_ID    (0x050)     // 0x050-0x05F        IDs for debug messages on the VCU
                                            // 0x060-0x06F
                                            // 0x070-0x07F
                                            // 0x080-0x08F        Sync         (DV OBLIG)          whole field reserved for DV
                                            // 0x090-0x09F        Emergency    (DV OBLIG)          whole field reserved for DV
                                            // 0x0A0-0x0AF
                                            // 0x0B0-0x0BF
                                            // 0x0C0-0x0CF
                                            // 0x0D0-0x0DF
                                            // 0x0E0-0x0EF
#define CANR_GRP_MASK           (0x0F0)     // 0x0F0-0x0FF


//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x100  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x100_ID                   (0x100)
                                                    // 0x100-0X10f        Time Stamp (DV OBLIG)    WHOLE FIELD RESERVED TO DV
                                                    // 0x110-0x11F
                                                    // 0x120-0x12F
                                                    // 0x130-0x13F
                                                    // 0x140-0x14F
                                                    // 0x150-0x15F
                                                    // 0x160-0x16F
#define CANR_GRP_INVERTER_R16_ID        (0x70)      // 0x170-0x17F    Inverter Simian
#define CANR_GRP_INVERTER_AMK_ID        (0x80)      // 0x180-0x18F    Inverter AMK
                                                    // 0x190-0x19F    PDO Transmit (DV OBLIG)        WHOLE FIELD RESERVED TO DV
                                                    // 0x190-0x19F
                                                    // 0x1A0-0x1AF
                                                    // 0x1B0-0x1BF
                                                    // 0x1C0-0x1CF
                                                    // 0x1D0-0x1DF
                                                    // 0x1E0-0x1EF
                                                    // 0x1F0-0x1FF

//-------------------------------------------------------------------------------
//                          R16 INVERTER CAN IDS
//-------------------------------------------------------------------------------
#define R16_PARAMETERS                  (CANR_0x100_ID  | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID0_ID)
#define R16_RESERVED_0                  (CANR_0x100_ID  | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID1_ID)
#define R16_RESERVED_1                  (CANR_0x100_ID  | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID2_ID)
#define R16_RESERVED_2                  (CANR_0x100_ID  | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID3_ID)

//-------------------------------------------------------------------------------
//                          AMK INVERTER SETPOINTS
//-------------------------------------------------------------------------------
// The inverter address is calculated with 282, 284, or 183  + node address
// 183 + node address is an address field used by the VCU to send set points to the Inverters
// But I dont see solution with this setup to allocate that room only to the VCU
//(0x184-0x189)
#define VCU_SET_POINT_AMK_FL_ID         (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID4_ID)
#define VCU_SET_POINT_AMK_FR_ID         (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID5_ID)
#define VCU_SET_POINT_AMK_RL_ID         (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID8_ID)
#define VCU_SET_POINT_AMK_RR_ID         (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID9_ID)

//-------------------------------------------------------------------------------
//                          AUTONOMOUS ACTUATOR CONTROL IDS
//-------------------------------------------------------------------------------
// Actuator setpoints from PU. 
// NOTE: Only one of RPM and TORQUE setpoint messages will be sent dependent on what mission is launched.
#define DV_PU_TORQUE_SETPOINT_ID                (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID10_ID)
#define DV_PU_RPM_SETPOINT_ID                   (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID11_ID)

#define DV_PU_STEERING_SETPOINT_ID              (CANR_0x100_ID  | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID12_ID)

//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x200  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x200_ID                   (0x200)
                                                    // 0x200-0x20F
                                                    // 0x210-0x21F     PDO Receive (0x211 DV OBLIG) wHOLE FIELD RESERVED TO DV
                                                    // 0x220-0x22F
                                                    // 0x230-0x23F
                                                    // 0x240-0x24F
                                                    // 0x250-0x25F
                                                    // 0x260-0x26F
// #define CANR_GRP_INVERTER_R16_ID        (0x70)   // 0x270-0x27F    Inverter data  from Simian    0x270-0x273        defined in previous section
// #define CANR_GRP_INVERTER_AMK_ID        (0x80)   // 0x280-0x28F    Inverter data  from AMK       0x284-0x28B        defined in previous section
                                                    // 0x290-0x29F
                                                    // 0x2A0-0x2AF
                                                    // 0x2B0-0x2BF
#define CANR_GRP_ALIVE_ID               (0xC0)      // 0x2C0-0x2CF	  Alive messages
                                                    // 0x2D0-0x2DF
											        // 0x2E0-0x2EF
                                                    // 0x2F0-0x2FF



//-------------------------------------------------------------------------------
//                          R16 INVERTER CAN IDS
//-------------------------------------------------------------------------------
                                            //(0x270-0x27F)
#define R16_DATA_MECHANICAL_ID(n)           ((n << 2) | (CANR_0x200_ID | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID0_ID))
#define R16_DATA_TEMPERATURE_ID(n)          ((n << 2) | (CANR_0x200_ID | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID1_ID))
#define R16_DATA_ELECTRICAL_ID(n)           ((n << 2) | (CANR_0x200_ID | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID2_ID))
#define R16_STATUS_ID(n)                    ((n << 2) | (CANR_0x200_ID | CANR_GRP_INVERTER_R16_ID | CANR_MODULE_ID3_ID))

#define R16_TELEMETRY_TORQUE_REQUEST_ID     (CANR_0x200_ID     | CANR_GRP_TELEMETRY_ID    | CANR_MODULE_ID2_ID)
// Data format
#define R16_MECHANICAL_RPM_16pos            (0)
#define R16_FOC_IQ_16pos                    (0)
#define R16_FOC_ID_16pos                    (1)
#define R16_DC_VOLTAGE_16pos                (0)
#define R16_DC_CURRENT_16pos                (1)
#define R16_STATUS_CURRENT_STATE_32pos      (0)
#define R16_STATUS_ACCUMULATED_STATE_32pos  (1)

// R16 Parameters
#define PARAM_R16_K_p_Q                     (0)
#define PARAM_R16_K_I_Q                     (1)
#define PARAM_R16_K_a_Q                     (2)
#define PARAM_R16_K_p_D                     (3)
#define PARAM_R16_K_I_D                     (4)
#define PARAM_R16_K_a_d                     (5)
#define PARAM_R16_K_p_FW_up                 (6)
#define PARAM_R16_K_p_FW_dn                 (7)

//-------------------------------------------------------------------------------
//                          AMK INVERTER CAN IDS
//-------------------------------------------------------------------------------
// AMK_DATA_ACTUAL_VALUE_1 STRUCTURE:
// Offset 2                                  //(0x280-0x28F)
#define AMK_DATA_ACTUAL_VALUE_1_FL_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID3_ID)
#define AMK_DATA_ACTUAL_VALUE_2_FL_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID5_ID)
// Offset 3
#define AMK_DATA_ACTUAL_VALUE_1_FR_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID4_ID)
#define AMK_DATA_ACTUAL_VALUE_2_FR_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID6_ID)

// Offset 6
#define AMK_DATA_ACTUAL_VALUE_1_RL_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID7_ID)
#define AMK_DATA_ACTUAL_VALUE_2_RL_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID9_ID)

// Offset 7
#define AMK_DATA_ACTUAL_VALUE_1_RR_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID8_ID)
#define AMK_DATA_ACTUAL_VALUE_2_RR_ID        (CANR_0x200_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID10_ID)

//------------------------------------------------------------------------------
/*    -------    ALIVE     ---------------------------------------------------------*/
//------------------------------------------------------------------------------

// ---------------------------------------------------------------------//
// Functionality description:                                           //
// Each module sends out a message with the following properties:       //
// ID = CANR_0x200_ID | CANR_GRP_DASH_ID    | CANR_MODULE_ID7_ID        //
// DATA[0] = MODULE ID                                                  //
// Frequency = 1 Hz                                                     //
// ---------------------------------------------------------------------//

//-ALIVE ID
#define CANR_ALIVE_MSG_ID       (CANR_0x200_ID    |CANR_GRP_ALIVE_ID | CANR_MODULE_ID0_ID)
#define CANR_ALIVE_MSG_DLC      (1)


// Module Alive IDs

#define ALIVE_ADC_FL            (0x01)
#define ALIVE_ADC_FR            (0x02)
#define ALIVE_ADC_REAR          (0x03)
#define ALIVE_SBS_REAR2         (0x04)
#define ALIVE_VCU               (0x07)
#define ALIVE_TELEMETRY         (0x08)
#define ALIVE_DASH              (0x09)
#define ALIVE_FAN               (0x0A)
#define ALIVE_AMS               (0x0B)
#define ALIVE_GLVBMS            (0x0C)
#define ALIVE_IMD               (0x0D)
#define ALIVE_RCAN              (0x0E)
#define ALIVE_INVERTER_0        (0x10)
#define ALIVE_INVERTER_1        (0x11)
#define ALIVE_INVERTER_2        (0x12)
#define ALIVE_INVERTER_3        (0x13)
#define ALIVE_ACU               (0x14)
#define ALIVE_DCU				(0x15)
#define ALIVE_PU				(0x16)

//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x300  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x300_ID           (0x300)
                                         // 0x300-0x30F
                                         // 0x310-0x31F
                                         // 0x320-0x32F
                                         // 0x330-0x33F
                                         // 0x340-0x34F
#define CANR_GRP_VCU_PRI_ID     (0x50)   // 0x350-0x35F    VCU Priority ID
#define CANR_GRP_INS_ID         (0X60)   // 0x360-0x36F    VCU INS data
                                         // 0x370-0x37F
                                         // 0x380-0x38F
                                         // 0x390-0x39F
                                         // 0x3A0-0x3AF    CANR telemetry
                                         // 0x3B0-0x3BF
#define CANR_GRP_LC_ID          (0XC0)   // 0x3C0-0x3DF    VCU LC MONITOR
#define CANR_GRP_TV_ID          (0XD0)   // 0x3D0-0x3DF    VCU TV MONITOR
#define CANR_GRP_ETS_ID         (0XE0)   // 0x3E0-0x3EF    VCU estimator data
                                         // 0x3F0-0x3FF

//-------------------------------------------------------------------------------
//                          VCU Priority ID
//-------------------------------------------------------------------------------
                                                // 0x350
#define VCU_PLAY_RTDS_ID                        (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID0_ID)
#define VCU_DRIVE_ENABLE_ID                     (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID1_ID)
#define VCU_DRIVE_DISABLE_ID                    (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID2_ID)
#define ACU_RESET_AMK_INVERTER_ERROR_ID         (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID3_ID)
#define VCU_DAMPER_POS_SMOOTHED_ID              (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID4_ID)
#define VCU_DAMPER_RATE_SMOOTHED_ID             (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID5_ID)
#define VCU_TPS_BRAKE_DEBUG_MESSAGE_ID          (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID6_ID)
#define VCU_SAFETY_QUEUE_FULL_ID                (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID7_ID)
#define VCU_GPS_TIME_ID                         (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID8_ID)
#define VCU_ALL_PARAMETERS_TO_DATALOGGER_ID     (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID9_ID)
#define VCU_THROTTLE_MIRROR_LEFT_ID             (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID10_ID)    // Not in use (emil 30.03.18)
#define VCU_THROTTLE_MIRROR_RIGHT_ID            (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID11_ID)    // Not In use (emil 30.03.18)
#define VCU_STATUS_MESSAGE_ID                   (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID12_ID)
#define VCU_SOFT_BSPD_STATUS_ID                 (CANR_0x300_ID | CANR_GRP_VCU_PRI_ID | CANR_MODULE_ID13_ID)
//-------------------------------------------------------------------------------
//                          CANR TELEMETRY ID
//-------------------------------------------------------------------------------
// 0x360-0x36F
//-------------------------------------------------------------------------------
//                          VCU INS ID
//-------------------------------------------------------------------------------
// INS data                                     // 0x360-0x36F
#define VCU_INS_STATUSES_ID                     (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID0_ID)
#define VCU_GPS_FIX_AND_NRSATS_ID               (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID1_ID)
#define VCU_INS_VX_VY_VZ_ID                     (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID2_ID)
#define VCU_INS_AX_AY_AZ_ID                     (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID3_ID)
#define VCU_INS_ROLL_PITCH_YAW_RATES_ID         (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID4_ID)
#define VCU_INS_ROLL_PITCH_YAW_ID               (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID5_ID)
#define VCU_GPS_LONGITUDE_LATITUDE_ID           (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID6_ID)
#define VCU_GPS_ALTITUDE_ID                     (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID7_ID)        // IS THIS IN USE? NOT IN UNPACKER
#define VCU_INS_YAW_RATE_ACC_ID                 (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID8_ID)
#define VCU_REQUEST_SAFE_STATE_ID               (CANR_0x300_ID | CANR_GRP_INS_ID | CANR_MODULE_ID9_ID)
//-------------------------------------------------------------------------------
//                          LAUNCH CONTROL MONITOR
//-------------------------------------------------------------------------------
// LAUNCH CONTROL MONITORING                     // 0x3C0-0x3CF
#define VCU_LC_MONITOR_TORQUE_SETPOINTS          (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID0_ID)
#define VCU_LC_MONITOR_SLIP_RATIO_ERROR          (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID1_ID)
#define VCU_LC_MONITOR_SR_ERROR_DERIVATIVE       (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID2_ID)
#define VCU_LC_MONITOR_SR_REF_LAUNCH             (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID3_ID)
#define VCU_LC_MONITOR_SR_LAUNCH                 (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID4_ID)
#define VCU_LC_MONITOR_P_TERM                    (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID5_ID)
#define VCU_LC_MONITOR_I_TERM                    (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID6_ID)
#define VCU_LC_MONITOR_D_TERM                    (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID7_ID)
#define VCU_LC_MONITOR_PID_VALUES                (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID8_ID)
#define VCU_LC_MONITOR_MODES                     (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID9_ID)
#define VCU_LC_MONITOR_POWER_LIMIT_TORQUES       (CANR_0x300_ID |CANR_GRP_LC_ID |CANR_MODULE_ID10_ID)
//-------------------------------------------------------------------------------
//                          TORQUE VECTORING MONITOR
//-------------------------------------------------------------------------------
// TV MONITORING                                 // 0x3D0-0x3DF
#define VCU_TV_MONITORING_MAX_MOTOR_TORQUE       (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID0_ID)
#define VCU_TV_MONITORING_TORQUE_INFO            (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID1_ID)
#define VCU_TV_MONITORING_ERROR_MESSAGE          (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID2_ID)
#define VCU_TV_MONITORING_YAW_MOMENTS            (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID3_ID)
#define VCU_TV_MONITORING_ACTIVE_CONSTRAINTS     (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID4_ID)
#define VCU_TV_MONITORING_YAW_RATES              (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID5_ID)
#define VCU_TV_MONITORING_PD_TERMS               (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID6_ID)
#define VCU_TV_MONITORING_PID_TERMS              (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID7_ID)
#define VCU_TV_MONITORING_MMT_TIRE_FORCE         (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID8_ID)
#define VCU_TV_MONITORING_MMT_TIRE_FORCE_SLIP    (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID9_ID)
#define VCU_TV_MONITORING_SLIP_SCALING_FACTORS   (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID10_ID)
#define VCU_TV_MONITORING_POWERLIM               (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID11_ID)
#define VCU_TV_MONITORING_TORQUE_REDUCTION       (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID12_ID)
#define VCU_TV_MONITORING_EFF_SETPOINTS          (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID13_ID)
#define VCU_TV_MONITORING_TORQUE_ALLOC           (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID14_ID)
#define VCU_TV_MONITORING_TORQUE_TRACTION        (CANR_0x300_ID | CANR_GRP_TV_ID | CANR_MODULE_ID15_ID)
//-------------------------------------------------------------------------------
//                          VCU ESTIMATED STATES
//-------------------------------------------------------------------------------
// Estimated States                               /// 0x3E0-0x3EF
#define VCU_SENSOR_STATUS                        (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID0_ID)
#define VCU_FZ_ID                                (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID1_ID)
#define VCU_FZ_DAMPER_EST_ID                     (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID2_ID)
#define VCU_FZ_ACC_EST_ID                        (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID3_ID)
#define VCU_FZ_EKF_EST_ID                        (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID4_ID)
#define VCU_R_EFF_ID                             (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID5_ID)
#define VCU_FX_ESTIMATED                         (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID6_ID)
#define VCU_SLIP_RATIO_ID                        (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID7_ID)
#define VCU_SLIP_ANGLES_ID                       (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID8_ID)
#define VCU_RPM_DERIVATIVE                       (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID9_ID)
#define VCU_STEERING_WHEEL_DATA_ID               (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID10_ID)
#define VCU_ALFA_R_ID                            (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID11_ID)

#define VCU_TV_MONITORING_TORQUE_POWER           (CANR_0x300_ID | CANR_GRP_ETS_ID | CANR_MODULE_ID15_ID)

//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x400  START*****************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x400_ID               (0x400)
#define CANR_GRP_SBS_F1_ID          (0x00)    // 0x400-0x40F    SBS F1
#define CANR_GRP_SBS_F2_ID          (0x10)    // 0x410-0x41F    SBS F2
#define CANR_GRP_SBS_R1_ID          (0x20)    // 0x420-0x42F    SBS R1
#define CANR_GRP_SBS_R2_ID          (0X30)    // 0x430-0x43F    SBS R2
#define CANR_GRP_SBS_RESERVED       (0x40)    // 0x440-0x446    SBS reserved for future use
                                              // 0x450-0x45F
#define CANR_GRP_DASH_RESERVED_ID   (0x60)    // 0x460-0x46F    DASH CAN IDs reserved for future use
#define CANR_GRP_DASH_ID            (0x70)    // 0x470-0x47F    DASH CAN IDs
#define CANR_GRP_IMD_ID             (0x90)    // 0c490-0x49F    IMD shutdown and state
#define CANR_GRP_TELEMETRY_ID       (0xA0)    // 0x4A0-0x4AF    TELEMETRY CONNECTION ID
#define CANR_GRP_DCU_ID			    (0xB0)    // 0x4B0-0x4BF    DCU CAN IDs
                                              // 0x4C0-0x4CF
                                              // 0x4D0-0x4DF
#define CANR_GRP_FAN_CTRL_ID        (0xE0)    // 0x4E0-0x4EF    FAN control
                                              // 0x4F0-0x4CF

//-------------------------------------------------------------------------------
//                          SBS CAN IDS
//-------------------------------------------------------------------------------
// SBS Front Left                                        //(0X400)
#define SBS_FL_ERROR_FLAGS_ID                           (CANR_0x400_ID | CANR_GRP_SBS_F1_ID | CANR_MODULE_ID0_ID)
#define SBS_FL_APPS_1_2_BRAKE_PRESSURE_1_2_ID           (CANR_0x400_ID | CANR_GRP_SBS_F1_ID | CANR_MODULE_ID1_ID)
#define SBS_FL_KERS_SWPS_ID                             (CANR_0x400_ID | CANR_GRP_SBS_F1_ID | CANR_MODULE_ID2_ID)
#define SBS_FL_RIDEHEIGHT_ID                            (CANR_0x400_ID | CANR_GRP_SBS_F1_ID | CANR_MODULE_ID3_ID)
#define SBS_FL_CALIBRATION_ACK_ID                       (CANR_0x400_ID | CANR_GRP_SBS_F1_ID | CANR_MODULE_ID5_ID)

// SBS Front Right                                       //(0X410)
#define SBS_FR_ERROR_FLAGS_ID                           (CANR_0x400_ID | CANR_GRP_SBS_F2_ID | CANR_MODULE_ID0_ID)
#define SBS_FR_DAMPER_FRONT_ID                          (CANR_0x400_ID | CANR_GRP_SBS_F2_ID | CANR_MODULE_ID1_ID)
#define SBS_FR_ACC_UPRIGHT_FRONT_GEAR_TEMP_FRONT_ID     (CANR_0x400_ID | CANR_GRP_SBS_F2_ID | CANR_MODULE_ID2_ID)
#define SBS_FR_CALIBRATION_ACK_FR_ID                    (CANR_0x400_ID | CANR_GRP_SBS_F2_ID | CANR_MODULE_ID5_ID)

// SBS Rear 1                                            //(0X420)
#define SBS_R1_ERROR_FLAGS_ID                           (CANR_0x400_ID | CANR_GRP_SBS_R1_ID | CANR_MODULE_ID0_ID)
#define SBS_R1_DAMPER_REAR_ID                           (CANR_0x400_ID | CANR_GRP_SBS_R1_ID | CANR_MODULE_ID1_ID)
#define SBS_R1_ACC_UPRIGHT_REAR_GEAR_TEMP_REAR_ID       (CANR_0x400_ID | CANR_GRP_SBS_R1_ID | CANR_MODULE_ID2_ID)
#define SBS_R1_CALIBRATION_ACK_R1_ID                    (CANR_0x400_ID | CANR_GRP_SBS_R1_ID | CANR_MODULE_ID5_ID)

// SBS DV definitions                                   //(0X42E)
#define DV_SBS_R2_EBS_SERVICE_BREAK_ID                  (CANR_0x400_ID | CANR_GRP_SBS_R1_ID | CANR_MODULE_ID14_ID) // 0x42E

// SBS Rear 2 (aka. MID)
// 0x430 reserved for the FSG Datalogger/Energymeter
#define SBS_R2_COOLING_TEMP_ID                          (CANR_0x400_ID | CANR_GRP_SBS_R2_ID | CANR_MODULE_ID1_ID)
#define SBS_R2_INVERTER_TEMP_ID                         (CANR_0x400_ID | CANR_GRP_SBS_R2_ID | CANR_MODULE_ID2_ID)
#define SBS_R2_ERROR_FLAGS_ID                           (CANR_0x400_ID | CANR_GRP_SBS_R2_ID | CANR_MODULE_ID3_ID)

// Calibration defines
#define APPS_READY_DEF                  (0)
#define APPS_MAX_DEF                    (1)
#define APPS_MIN_DEF                    (2)
#define KERS_READY_DEF                  (3)
#define KERS_MAX_DEF                    (4)
#define KERS_MIN_DEF                    (5)
#define SWPS_READY_DEF                  (6)
#define SWPS_LEFT_DEF                   (7)
#define SWPS_RIGHT_DEF                  (8)
#define DAMPER_FRONT_READY_DEF          (9)
#define DAMPER_FRONT_MIN_DEF            (10)
#define DAMPER_REAR_READY_DEF           (11)
#define DAMPER_REAR_MIN_DEF             (12)

#define APPS_CALIB_INIT_DEF             (0)
#define APPS_CALIB_MAX_DEF              (1)
#define APPS_CALIB_MIN_DEF              (2)
#define KERS_CALIB_INIT_DEF             (3)
#define KERS_CALIB_MAX_DEF              (4)
#define KERS_CALIB_MIN_DEF              (5)
#define SWPS_CALIB_INIT_DEF             (6)
#define SWPS_CALIB_MAX_DEF              (7)
#define SWPS_CALIB_MIN_DEF              (8)
#define DAMPER_FRONT_CALIB_INIT_DEF     (9)
#define DAMPER_FRONT_CALIB_MIN_DEF      (10)
#define DAMPER_REAR_CALIB_INIT_DEF      (11)
#define DAMPER_REAR_CALIB_MIN_DEF       (12)

//-------------------------------------------------------------------------------
//                          DASH CAN IDS
//-------------------------------------------------------------------------------
// RESERVED for future dash can ids                  (0x460-0x46F)

// Dash calibration IDs                             (0x46d-0x47F)
#define DASH_RTDS_ACK_ID							(CANR_0x400_ID | CANR_GRP_DASH_RESERVED_ID | CANR_MODULE_ID12_ID)
#define DASH_FRONT_L_CALIBRATION_ID                 (CANR_0x400_ID | CANR_GRP_DASH_RESERVED_ID | CANR_MODULE_ID13_ID)
#define DASH_FRONT_R_CALIBRATION_ID                 (CANR_0x400_ID | CANR_GRP_DASH_RESERVED_ID | CANR_MODULE_ID14_ID)
#define DASH_REAR_CALIBRATION_ID                    (CANR_0x400_ID | CANR_GRP_DASH_RESERVED_ID | CANR_MODULE_ID15_ID)

// Dash can IDs                                     (0x470-0x47F)
#define DASH_REQUEST_DRIVE_ENABLE_ID                (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0_ID)
#define DASH_REQUEST_DRIVE_DISABLE_ID               (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID1_ID)
#define DASH_RTDS_FINISHED_ID                       (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID2_ID)
#define DASH_REQUEST_ACTIVATE_LAUNCH_CONTROL_ID     (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID3_ID)
#define DASH_REQUEST_DEACTIVATE_LAUNCH_CONTROL_ID   (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID4_ID)
#define DASH_KERS_REQUEST_ID                        (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID5_ID)
#define DASH_REQUEST_TS_ID                          (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID6_ID)
#define DASH_FAN_CONTROL_SETTINGS_ID                (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID7_ID)
#define DASH_KERS_CALIBRATION_INIT_ID               (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID8_ID)
#define DASH_STEERING_CALIBRATION_INIT_ID           (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID9_ID)
#define DASH_TORQUE_ALLOCATION_REQUEST_ID           (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID10_ID)
#define DASH_POWER_LIMIT_CONTROL_REQUEST_ID         (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID11_ID)
#define DASH_SENSOR_CALIBRATION_INIT                (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID12_ID)    // 0 = no init, 1 = tps init, 2 = steering init, 3 = KERS
#define DASH_TPS_CALIBRATION_MIN_PRESSED            (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID13_ID)
#define DASH_TPS_CALIBRATION_MAX_PRESSED            (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID14_ID)
#define DASH_DEVICE_STATUS_ID                       (CANR_0x400_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID15_ID)

#define DV_EPOS_PDO4_RX_ID                          (CANR_0x400_ID | CANR_GRP_INVERTER_AMK_ID | CANR_MODULE_ID1_ID)
// sending a 16 bit statusfield.
// ALIVE DEVICE  STATUS BITFIELD
#define DASH_VCU_ALIVE              (0)
#define DASH_ADC_BSPD_ALIVE         (1)
#define DASH_TELEMETRY_ALIVE        (2)
#define DASH_ADC_TEMPFRONT_ALIVE    (3)
#define DASH_ADC_TEMPREAR_ALIVE     (4)
#define DASH_ADC_DAMPERREAR_ALIVE   (5)
#define DASH_ADC_DAMPERFRONT_ALIVE  (6)
#define DASH_FAN_ALIVE              (7)
#define DASH_AMS_ALIVE              (8)
#define DASH_GLVBMS_ALIVE           (9)
#define DASH_IMD_ALIVE              (10)
#define DASH_ADC_P_ALIVE            (11)
#define DASH_INV0_ALIVE             (12)
#define DASH_INV1_ALIVE             (13)
#define DASH_INV2_ALIVE             (14)
#define DASH_INV3_ALIVE             (15)

//-------------------------------------------------------------------------------
//                          IMD CAN IDS
//-------------------------------------------------------------------------------
                                            //(0x490-0x49F)
#define IMD_SHUTDOWN_OPEN_ID                (CANR_0x400_ID | CANR_GRP_IMD_ID | CANR_MODULE_ID0_ID) // 1 = SHUTDOWN OPEN, 0 = SHUTDOWN CLOSE
#define IMD_STATE_ID                        (CANR_0x400_ID | CANR_GRP_IMD_ID | CANR_MODULE_ID1_ID) // byte 1 = status code
#define IMD_DATA_ID                         (CANR_0x400_ID | CANR_GRP_IMD_ID | CANR_MODULE_ID2_ID) // IMD resistance in Ohms
//-------------------------------------------------------------------------------
//                            TELEMETRY IDS
//-------------------------------------------------------------------------------
                                            //(0x4A0-0x4AF)
#define TELEMETRY_CONNECTION_ID             (CANR_0x400_ID | CANR_GRP_TELEMETRY_ID | CANR_MODULE_ID0_ID)
//-------------------------------------------------------------------------------------
//								DCU IDS
//-------------------------------------------------------------------------------------
											//(0x4B0-0x4BF)
#define DCU_STATE_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID0_ID)
#define DCU_ERROR_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID1_ID)
#define DCU_IMU_FL_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID2_ID)
#define DCU_IMU_FR_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID3_ID)
#define DCU_IMU_RL_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID4_ID)
#define DCU_IMU_RR_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID5_ID)
#define DCU_IMU_CG_ID						(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID6_ID)
#define DCU_DAMPER_CONTROL_INFO_ID			(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID7_ID)
#define DCU_CURRENT_MEASUREMENT_ID			(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID8_ID)
#define DCU_CURRENT_CONTROL_INFO_ID			(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID9_ID)

#define LINAK_REFERENCE_ID					(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID14_ID)
#define LINAK_REFERENCE_ACK_ID				(CANR_0x400_ID | CANR_GRP_DCU_ID | CANR_MODULE_ID14_ID)

//-------------------------------------------------------------------------------------
//                          FAN CONTROL IDS
//-------------------------------------------------------------------------------------
                                            //(0x4E0-0x4EF)

// Byte: 0 = battery fan duty-cycle, 1 = radiator fan duty-cycle, 2 = pump (1=ON,0=OFF), 3 = mode, 4 = brake light mode, 5 = 12V-state (0=ERROR)
#define FAN_CTRL_STATUS_ID                  (CANR_0x400_ID | CANR_GRP_FAN_CTRL_ID | CANR_MODULE_ID0_ID)

#define FAN_CTRL_NUM_OF_BRAKE_LIGHT_FAULTS  (CANR_0x400_ID | CANR_GRP_FAN_CTRL_ID | CANR_MODULE_ID1_ID)
//-------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x500  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
// ALL can IDs on the 500 are reserved for the Driverless car. OBSOBS!! some of the IDS are reserved by rule.
                                                // 0x500-0x50F    DV alternative OBLIG IDs, must have both
#define CAN_GRP_DV_ACTUATOR_CONTROL     (0x10)  // 0x510-0x51F    available for DV
#define CAN_GRP_ACU_ID                  (0x20)  // 0x520-0x52F    available for DV
                                                // 0x530-0x53F    DV alternative OBLIG IDs, must have both    DV OBLIG
#define CAN_GRP_EPOS_ID                 (0x40)  // 0x540-0x54F    available for DV
#define CAN_GRP_CCC_ID                  (0x50)  // 0x550-0x55F    available for DV
#define CAN_GRP_HIGHRATE_INS            (0x60)  // 0x560-0x56F    available for DV
                                                // 0x570-0x57F    available for DV
                                                // 0x580-0x58F    available for DV
                                                // 0x590-0x59F    SDO Transmit
                                                // 0x5A0-0x5AF    available for DV

                                                // 0x5C0-0x5CF    available for DV
                                                // 0x5D0-0x5DF    available for DV
                                                // 0x5E0-0x5EF    available for DV
                                                // 0x5F0-0x5FF    available for DV
#define CAN_GRP_PU 0xB0                         //0x5B0-0x5BF available for DV
                                                //0x5C0-0x5CF available for DV
                                                //0x5D0-0x5DF available for DV
                                                //0x5E0-0x5EF available for DV
                                                //0x5F0-0x5FF available for DV

//-------------------------------------------------------------------------------
//                          DV CAN IDS
//-------------------------------------------------------------------------------

#define CANR_FCN_DV_ID              (0x500)

//-------------------------------------------------------------------------------
//                          DV DATA LOGGER IDS
//-------------------------------------------------------------------------------

#define CAN_GRP_DV_DATA_LOGGER      (0x00)

// Byte: 0 = Speed_actual, 1 = Speed_target, 2 = Steering_angle_actual, 3 = Steering_angle_target, 4 = Brake_hydr_actual, 5 = Brake_hydr_target, 6 = Motor_moment_actual, 7 = Motor_moment_target
#define DV_DRIVING_DYNAMICS1_ID     (CANR_FCN_DV_ID    | CAN_GRP_DV_DATA_LOGGER    | CANR_MODULE_ID0_ID)

// Byte: 0-1 = Acceleration longitudinal, 2-3 = Acceleration lateral, 4-5 = Yaw rate
#define DV_DRIVING_DYNAMICS2_ID     (CANR_FCN_DV_ID    | CAN_GRP_DV_DATA_LOGGER    | CANR_MODULE_ID1_ID)

// Bit: 0-2 = ASSI_state, 3-4 = EBS_state, 5-7 = AMI_state, 8 = Steering_state, 9-10 = SB_state, 11-14 = Lap_counter, 15-22 = Cones_count_actual, 23-39 = Cones_count_all  
#define DV_SYSTEM_STATUS_ID         (CANR_FCN_DV_ID    | CAN_GRP_DV_DATA_LOGGER    | CANR_MODULE_ID2_ID)

//-------------------------------------------------------------------------------
//                          DV MISCELLANEOUS IDS
//-------------------------------------------------------------------------------

// 1 == ACCELERATION, 2 == SKIDPAD, 3 == TRACKDRIVE, 4 == BRAKETEST, 5 == INSPECTION, 6 == AUTOCROSS, 7 == MANUAL DRIVING
#define DV_DASH_ACU_MISSION_SELECT_ID			   (CANR_FCN_DV_ID | CAN_GRP_DV_DATA_LOGGER | CANR_MODULE_ID3_ID)
#define DV_ACU_SYSTEM_STATUS_ID					   (CANR_FCN_DV_ID | CAN_GRP_DV_DATA_LOGGER | CANR_MODULE_ID4_ID)
#define DV_VCU_ACU_DRIVE_ENABLE_ID				   (CANR_FCN_DV_ID | CAN_GRP_DV_DATA_LOGGER | CANR_MODULE_ID5_ID)
#define DV_DASH_ACU_EBS_SOUND_FINISHED_ID		   (CANR_FCN_DV_ID | CAN_GRP_DV_DATA_LOGGER | CANR_MODULE_ID5_ID)
#define DV_ACU_INVERTER_STATE_ID                   (CANR_FCN_DV_ID | CAN_GRP_DV_DATA_LOGGER | CANR_MODULE_ID6_ID) //0x506

// Reserved CAN-IDs for ASF messages
#define ASF_DBC_ACU_SDC_LOGIC_MONITORING_ID        (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID1_ID)
#define ASF_DBC_SBS_FL_APPS_BRAKE_PRESSURE_ID      (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID2_ID)
#define ASF_DBC_SBS_R2_EBS_SERVICE_BRAKE_ID        (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID3_ID)
//Reserved										   (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID4_ID)
//Reserved										   (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID5_ID)

// PU messages
#define DV_PU_VCU_THROTTLE_KERS_SETPOINT_ID        (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID6_ID) // float[2] {throttle_setpoint, KERS_setpoint}
#define DV_PU_ACU_SERVICE_BRAKE_SETPOINT_ID        (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID7_ID)
#define DV_PU_ALIVE_ID        	   		           (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID9_ID)
#define DV_PU_TRIGGER_KERS_ID        	   		   (CANR_FCN_DV_ID | CAN_GRP_DV_ACTUATOR_CONTROL | CANR_MODULE_ID10_ID)

//-------------------------------------------------------------------------------
//                          DV ACU IDS
//-------------------------------------------------------------------------------
#define DV_ACU_EPOS_SDO_ID                      (0x601) // Unfortunately the SDO ID cant be changed away from 600 + Node ID
#define DV_ACU_PU_STEERING_STATUS_ID            (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID0_ID) // CAN-ID: 0x520
#define DV_ACU_EPOS_PDO1_CONTROLWORD_ID         (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID1_ID)
#define DV_ACU_EPOS_PDO2_OPERATING_MODE_ID      (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID2_ID)
#define DV_ACU_EPOS_PDO3_POSITION_SETPOINT_ID   (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID3_ID)
#define DV_ACU_DASH_PLAY_EBS_SOUND_ID           (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID4_ID)
#define DV_ACU_DASH_EBS_FAILURE_LED_ID          (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID5_ID) 
#define DV_ACU_SENSOR_STATUS_ID                 (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID6_ID) //TODO: Removed from the ACU code in 2019. Might be interesting for R20 to use in Analyze.
#define DV_ACU_SDC_LOGIC_MONITORING_ID          (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID7_ID) //TODO: Defined this in the ASF for EBS monitoring.
#define DV_ACU_TEST_SDC_LOGIC_ID				(CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID8_ID) 
#define DV_ACU_TEST_CAN_ID						(CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID9_ID)
#define DV_ACU_ERROR_RAD_SETPOINT_ID			(CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID10_ID) // Sends erronous rad_setpoint back if received from PU 
#define DV_ACU_BRAKE_PRESSURES_ID               (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID11_ID)
#define DV_ACU_PU_MEASURED_STEERING_SETPOINT_ID (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID12_ID)
#define DV_ACU_SYSTEM_STATUS_2_ID               (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID13_ID)
#define DV_ACU_R2D_STATE_ID                     (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID14_ID) //0x52E , 0 == RTD false, 1==RTD (and drive enable)
#define DV_ACU_MONITORED_SYSTEM_STATUSES_ID     (CANR_FCN_DV_ID | CAN_GRP_ACU_ID | CANR_MODULE_ID15_ID)


//-------------------------------------------------------------------------------
//							DV EPOS4 (NODE-ID = 0x001) IDS
//-------------------------------------------------------------------------------

#define DV_EPOS_ACU_SDO_ID                      (0x581) // 0x580 + NODE-ID (CANopen)
#define DV_EPOS_ACU_NMT_ID                      (0x701) // 0x700 + NODE-ID (CANopen)
#define DV_EPOS_PDO1_TX_ID                      (CANR_FCN_DV_ID | CAN_GRP_EPOS_ID | CANR_MODULE_ID2_ID)
#define DV_EPOS_PDO2_TX_ID                      (CANR_FCN_DV_ID | CAN_GRP_EPOS_ID | CANR_MODULE_ID3_ID)
#define DV_EPOS_PDO3_TX_ID                      (CANR_FCN_DV_ID | CAN_GRP_EPOS_ID | CANR_MODULE_ID4_ID)

//-------------------------------------------------------------------------------
//                          DV CCC IDS
//-------------------------------------------------------------------------------

#define DV_CCC_COOLING_STATE_ID					(CANR_FCN_DV_ID | CAN_GRP_CCC_ID | CANR_MODULE_ID0_ID) // 0x550
#define DV_CCC_BRAKE_LIGHT_STATE_ID				(CANR_FCN_DV_ID | CAN_GRP_CCC_ID | CANR_MODULE_ID1_ID)

//-------------------------------------------------------------------------------
//                          DV HIGHRATE_INS IDS
//-------------------------------------------------------------------------------
#define HIGHRATE_INS_ROLL                    (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID0_ID)
#define HIGHRATE_INS_YAW                     (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID1_ID)
#define HIGHRATE_INS_PITCH                   (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID2_ID)
#define HIGHRATE_INS_X                       (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID3_ID)
#define HIGHRATE_INS_Y                       (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID4_ID)
#define HIGHRATE_INS_Z                       (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID5_ID)
#define HIGHRATE_INS_LATITUDE                (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID6_ID)
#define HIGHRATE_INS_LONGITUDE               (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID7_ID)
#define HIGHRATE_INS_ALTITUDE                (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID8_ID)
#define HIGHRATE_INS_STATUS                  (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID9_ID)
#define HIGHRATE_INS_MAG_XY                  (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID10_ID)
#define HIGHRATE_INS_MAG_Z_PRES              (CANR_FCN_DV_ID | CAN_GRP_HIGHRATE_INS | CANR_MODULE_ID11_ID)

//-------------------------------------------------------------------------------
//                          DV PU to ACU status message IDS
//-------------------------------------------------------------------------------

#define DV_PU_ACU_MISSION_FINISHED_ID            (CANR_FCN_DV_ID | CAN_GRP_PU | CANR_MODULE_ID0_ID)
#define DV_PU_ACU_TRIGGER_EBS_ID                 (CANR_FCN_DV_ID | CAN_GRP_PU | CANR_MODULE_ID1_ID)
#define DV_PU_CCC_CORE_TEMPERATURES_ID           (CANR_FCN_DV_ID | CAN_GRP_PU | CANR_MODULE_ID2_ID)

//------------------------------------------------------------------------------
//                          DV Diagnostics
//------------------------------------------------------------------------------

//sending a 64 bit statusfield.
//each status uses 1 bit
//DV DIAGNOSTICS STATUS BITFIELD
#define DV_DIAG_VLP                         (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID0_ID)
#define DV_DIAG_OS1                         (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID1_ID)
#define DV_DIAG_BASLER                      (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID2_ID)
#define DV_DIAG_VN_300                      (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID3_ID)

#define DV_DIAG_VLP_DETECTOR                (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID4_ID)
#define DV_DIAG_OS1_DETECTOR                (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID5_ID)
#define DV_DIAG_CAMERA_DETECTOR             (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID6_ID)

#define DV_DIAG_NONLINEAR_OBSERVER          (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID7_ID)
#define DV_DIAG_SLAM                        (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID8_ID)

#define DV_DIAG_PARTICLE_FILTER             (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID9_ID)
#define DV_DIAG_TRACK_FUSER                 (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID10_ID)
#define DV_DIAG_SPEED_PROFILE               (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID11_ID)

#define DV_DIAG_CONTROL                     (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID12_ID) 
#define DV_DIAG_PIPELINE                    (CANR_FCN_DV_ID | CANR_GRP_ALIVE_ID | CANR_MODULE_ID13_ID) 

#define DV_STATE_ESTIMATION_POSE            (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID0_ID)
#define DV_STATE_ESTIMATION_VEHICLE_STATE   (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID1_ID)
#define DV_SLAM_POSE                        (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID2_ID)
#define DV_SLAM_CONES                       (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID3_ID)
#define DV_SLAM_FRONTEND                    (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID4_ID)
#define DV_PARTICLE_FILTER_WPTS             (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID5_ID)
#define DV_TRACK_FUSER_TRACK                (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID6_ID)
#define DV_SPEED_PROFILE_ERROR              (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID7_ID)
#define DV_SPEED_PROFILE_DESIRED            (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID8_ID)
#define DV_CONTROL_STATE                    (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID9_ID)
#define DV_CONTROL_DEBUG                    (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID10_ID)
#define DV_OUSTER_DATA                      (CANR_FCN_DV_ID |       (0xD0)      | CANR_MODULE_ID11_ID)


//---------------------------------------------------------------------------------------------------------------------------------------------
//************************************** 0x600  START******************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------------
#define CANR_0x600_ID                   (0x600)
                                                // 0x600-0x60F
                                                // 0x610-0x61F    SDO Receive at(0x611)            Whole field reserved for DV
#define CANR_GRP_AMS_ID                 (0x20)  // 0x620-0x62F    BMS (temp state error)            from (0x440-0x446)
#define CANR_GRP_AMS_TEMPERATURE_ID     (0x30)  // 0x630-0x63F    BMS temperature ID                from (0x540-0x54F)
                                                // 0x640-0x64F
#define CANR_GRP_AMS_VOLTAGE_ID         (0x60)  // 0x650-0x65F    BMS voltage                        from (0x500-0x50F)
                                                // 0x660-0x66F
                                                // 0x670-0x67F    GLVBMS temperature readout
                                                // 0x680-0x68F    GLVBMS voltage readout
#define CANR_GRP_GLVBMS_ID              (0x90)  // 0x690-0x69F    BMS voltage                        from (0x500-0x50F)
												// 0x6A0-0x6AF    
                                                // 0x6B0-0x6BF
                                                // 0x6C0-0x6CF
                                                // 0x6D0-0x6DF
#define CANR_GRP_GLVBMS_TEMPERATURE_ID  (0xE0)  // 0x6E0-0x6EF
#define CANR_GRP_GLVBMS_VOLTAGE_ID      (0xF0)  // 0x6F0-0x6FF



//-------------------------------------------------------------------------------
//                          AMS CAN IDS
//-------------------------------------------------------------------------------
// defined bit [3..0] for use for all messages.
// These new _BASE IDs were created to avoid a weird issue where the macro was not working as intended
#define AMS_CELL_VOLTAGE_BASE           (CANR_0x600_ID | CANR_GRP_AMS_VOLTAGE_ID     | CANR_MODULE_ID0_ID)            // uint16_t[4]    100µV
#define AMS_CELL_TEMPERATURE_BASE       (CANR_0x600_ID | CANR_GRP_AMS_TEMPERATURE_ID | CANR_MODULE_ID0_ID)        // uint16_t[4]    0.1 deg C
#define AMS_CELL_VOLTAGE(n)             (CANR_0x600_ID | CANR_GRP_AMS_VOLTAGE_ID     | (n))                            // uint16_t[4]    100µV
#define AMS_CELL_TEMPERATURE(n)         (CANR_0x600_ID | CANR_GRP_AMS_TEMPERATURE_ID | (n))                        // uint16_t[4]    0.1 deg C
                                        //(0x640-0x64F)
#define AMS_VOLTAGE_METADATA            (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID0_ID)                    // uint16_t        100 µV
#define AMS_TEMPERATURE_METADATA        (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID1_ID)                    // uint16_t        0.1 deg C
#define AMS_TS_DATA                     (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID2_ID)                    // int16_t        100 mV, 100 mA, 10 W, 0.01%
#define AMS_ERROR_FLAGS                 (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID3_ID)                    // uint16_t        bitfield (see below)
#define AMS_STATE                       (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID4_ID)                    // uint8_t        0: Idle, 1: Precharge, 2: TS active, 3: Error, 4: Charging
#define AMS_TS_POWER                    (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID5_ID)                    // int16_t        Power: 10 W, Sum of cells: 100 mV
#define AMS_COLUMB_COUNT                (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID6_ID)
#define AMS_ISOMAX                      (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID7_ID)                    // IsoSPI max fail
#define AMS_TRANSIENT_VOLTAGE           (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID8_ID)
#define AMS_SOFT_OVERCURRENT            (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID9_ID)
#define AMS_LIMP_HOME_TEMP              (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID10_ID)
#define AMS_LIMP_HOME_VOLTAGE           (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID11_ID)
#define AMS_ACTIVATE_SOFT_OVECURRENT    (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID12_ID)                    // 0 = DEACTIVATE, 1 = ACTIVATE


#define AMS_STATE_IDLE                      (0)
#define AMS_STATE_PRECHARGE                 (1)
#define AMS_STATE_TS_ACTIVE                 (2)
#define AMS_STATE_ERROR                     (3)
#define AMS_STATE_CHARGER_READY             (4)
#define AMS_STATE_CHARGER_ACTIVE            (5)

#define AMS_ERROR_FLAG_OVERVOLTAGE          (1 << 0)
#define AMS_ERROR_FLAG_UNDERVOLTAGE         (1 << 1)
#define AMS_ERROR_FLAG_OVERTEMPERATURE      (1 << 2)
#define AMS_ERROR_FLAG_OVERCURRENT          (1 << 3)
#define AMS_ERROR_FLAG_OVERPOWER            (1 << 4)
#define AMS_ERROR_FLAG_ISOLATIONFAULT       (1 << 5)
#define AMS_ERROR_FLAG_ISOSPI_LOSSOFSIGNAL  (1 << 6)
#define AMS_ERROR_FLAG_CM_LOSSOFSIGNAL      (1 << 7)
#define AMS_ERROR_FLAG_HVM_LOSSOFSIGNAL     (1 << 8)
#define AMS_ERROR_FLAG_OPEN_WIRE            (1 << 9)

#define AMS_SOC_CTRL_SET_TO_MAX             (CANR_0x600_ID | CANR_GRP_AMS_ID | CANR_MODULE_ID8_ID) // Set SOC to 100%


// Charger IDs

#define CHARGER_CONTROL_ID				(0x2FF)
#define CAN_ID_CHARGER_CMD				(0x428)
#define CAN_ID_CHARGER_ACK				(0x429)

#define CHARGER_ADDRESS_MASK			(0xFF0)
#define CHARGER1_ADDRESS				(0x300)
#define CHARGER2_ADDRESS				(0x310)
#define CHARGER_OFFSET_MASK				(0x00F)
#define CHARGER_STATUS1_OFFSET			(0x005)
#define CHARGER_STATUS2_OFFSET			(0x006)
#define CHARGER_ERRORS_OFFSET			(0x007)
#define CHARGER1_SERIAL_NUMBER_OFFSET	(0x008)
#define CHARGER2_SERIAL_NUMBER_OFFSET	(0x018)

#define CHARGER1_STATUS1_ID         (CHARGER1_ADDRESS | CHARGER_STATUS1_OFFSET)
#define CHARGER2_STATUS1_ID         (CHARGER2_ADDRESS | CHARGER_STATUS1_OFFSET)
#define CHARGER1_STATUS2_ID         (CHARGER1_ADDRESS | CHARGER_STATUS2_OFFSET)
#define CHARGER2_STATUS2_ID         (CHARGER2_ADDRESS | CHARGER_STATUS2_OFFSET)
#define CHARGER1_ERRORS_ID          (CHARGER1_ADDRESS | CHARGER_ERRORS_OFFSET)
#define CHARGER2_ERRORS_ID          (CHARGER2_ADDRESS | CHARGER_ERRORS_OFFSET)
#define CHARGER1_SERIAL_NUMBER_ID	(CHARGER1_ADDRESS | CHARGER1_SERIAL_NUMBER_OFFSET)
#define CHARGER2_SERIAL_NUMBER_ID	(CHARGER2_ADDRESS | CHARGER2_SERIAL_NUMBER_OFFSET)

#define TURN_ON_CHARGERS_CMD        (0x02)
#define TURN_OFF_CHARGERS_CMD       (0x03)
#define START_CHARGING_CMD          (0x04)
#define STOP_CHARGING_CMD           (0x05)
#define START_BALANCING_CMD         (0x06)
#define STOP_BALANCING_CMD          (0x07)

//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
//                          GLVBMS CAN IDS
//-------------------------------------------------------------------------------
// VOLTAGE AND TEMP
#define GLVBMS_VOLTAGE(n)                   (CANR_0x600_ID | CANR_GRP_GLVBMS_VOLTAGE_ID        | (n) )    // Voltage readout
#define GLVBMS_CELL_TEMP(n)                 (CANR_0x600_ID | CANR_GRP_GLVBMS_TEMPERATURE_ID | (n) )    // cell temp readout

#define GLVBMS_VOLTAGE_BASE_ID              (CANR_0x600_ID | CANR_GRP_GLVBMS_VOLTAGE_ID        | CANR_MODULE_ID0_ID )    // Voltage readout
#define GLVBMS_CELL_TEMP_BASE_ID            (CANR_0x600_ID | CANR_GRP_GLVBMS_TEMPERATURE_ID | CANR_MODULE_ID0_ID )    // cell temp readout

// GENERAL MESSAGES INPUT PART 1
#define GLVBMS_KILL_ID                      (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID1_ID)  // kill the battery
#define GLVBMS_RESET_FLAGS_ID               (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID2_ID)  // Reset all flags


// COLUMB AND SOC
#define GLVBMS_COLOMB_COUNT                 (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID3_ID)  // the Colomb count
#define GLVBMS_LV_DATA                      (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID4_ID)  // int16_t, 100 mV, 100 mA, 10 W, 0.01% (percentage is charge left)

// ERROR FLAGS
#define GLVBMS_ERROR_FLAGS                  (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID5_ID)  // Error flags out, look below.
#define GLVBMS_VOLTAGE_METADATA             (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID6_ID)  // Error flags out, look below.
#define GLVBMS_TEMPERATURE_METADATA         (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID7_ID)  // Error flags out, look below.

// GENERAL MESSAGES INPUT PART 2
#define GLVBMS_TOGGLE_3S_ID                 (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID8_ID)  // Turn off GLV, wait 3 s, turn on GLV
#define GLVBMS_ERROR_TIMER_ONGOING          (CANR_0x600_ID | CANR_GRP_GLVBMS_ID | CANR_MODULE_ID9_ID)  // Sent when error timer is started or ongoing to give early warning over telemetry/dash

#define GLVBMS_ERROR_FLAG_OVERVOLTAGE       (1 << 0)
#define GLVBMS_ERROR_FLAG_UNDERVOLTAGE      (1 << 1)
#define GLVBMS_ERROR_FLAG_OVERTEMPERATURE   (1 << 2)
#define GLVBMS_ERROR_FLAG_SPI_LOSSOFSIGNAL  (1 << 3)

//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
/*    ------- Module ID overrides     ----------------------------------------------*/
/*   If you want to make your own macro to replace module_id_x, make it here   */
//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------
//                          Parameter setting CAN IDS
//-------------------------------------------------------------------------------
//(0x300)
#define PARAMETER_SET_ID                    (CANR_0x600_ID | CANR_GRP_TELEMETRY_ID | CANR_MODULE_ID0_ID)
#define PARAMETER_SET_MODULE_ID_8pos        (0)
#define PARAMETER_SET_PARAM_ID_16pos        (1)
#define PARAMETER_SET_PARAM_val_32pos       (1)

//-------------------------------------------------------------------------------
//                          Status bit setting CAN IDS
//-------------------------------------------------------------------------------
//(0x301)
#define STATUS_BIT_SET_ID                   (CANR_0x600_ID | CANR_GRP_TELEMETRY_ID | CANR_MODULE_ID1_ID)
#define STATUS_BIT_SET_MODULE_ID_8pos       (0)
#define STATUS_BIT_SET_OPERATION_ID_8pos    (1)
#define STATUS_BIT_SET_PARAM_mask_32pos     (1)

#define STATUS_BIT_SET_OPERATION_INVALID    (0)
#define STATUS_BIT_SET_OPERATION_SET        (1)
#define STATUS_BIT_SET_OPERATION_CLEAR      (2)
#define STATUS_BIT_SET_OPERATION_FLIP       (4)


//-------------------------------------------------------------------------------
//                          DV CANopen RESERVED IDS
//-------------------------------------------------------------------------------

// RES (NODE-ID = 0x011) & EPOS4 (NODE-ID = 0x001)

#define NODE_ID                            (0x011)

#define DV_NMT_NODE_CONTROL				   (0x000)
// Slave nodes: Receive only
// General CANopen

#define DV_SYNC                            (0x080)
// Slave nodes: Receive only
// General CANopen

#define DV_RES_EMERGENCY                   (0x080 + NODE_ID)
// Slave nodes: Transmit

#define DV_TIME_STAMP					   (0x100)
// Slave nodes: Receive only
// General CANopen

#define DV_RES_PDO_TRANSMIT                (0x180 + NODE_ID)
// Slave nodes: Transmit

#define DV_RES_PDO_RECEIVE                 (0x200 + NODE_ID)
// Slave nodes: Receive

#define DV_RES_SDO_TRANSMIT                (0x580 + NODE_ID)
// Slave nodes: Transmit

#define DV_RES_SDO_RECEIVE                 (0x600 + NODE_ID)
// Slave nodes: Receive

#define DV_RES_NMT_NODE_MONITORING         (0x700 + NODE_ID)
// Slave nodes: Transmit

#define DV_LSS_TRANSMIT                    (0x7E4)
// Slave nodes: Transmit
// General CANopen

#define DV_RES_LSS_RECEIVE                 (0x7E5)
// Slave nodes: Receive
// General CANopen

#endif /* REVOLVE_CAN_DEFINITIONS_H_ */
