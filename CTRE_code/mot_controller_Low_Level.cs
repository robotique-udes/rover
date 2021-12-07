using System;
using System.Text;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.LowLevel;
using CTRE.Phoenix.Motion;

namespace CTRE.Phoenix.LowLevel
{
    using TALON_Control_6_MotProfAddTrajPoint_huff0_t = UInt64;
    using TALON_Control_6_MotProfAddTrajPoint_t = UInt64;

    /**
     * @brief CAN TALON SRX driver.
     *
     * The TALON SRX is designed to instrument all runtime signals periodically.
     * The default periods are chosen to support 16 TALONs with 10ms update rate
     * for control (throttle or setpoint).  However these can be overridden with
     * SetStatusFramePeriod. @see SetStatusFrameRate
     * The getters for these unsolicited signals are auto generated at the bottom
     * of this module.
     *
     * Likewise most control signals are sent periodically using the fire-and-forget
     * CAN API.  The setters for these unsolicited signals are auto generated at the
     * bottom of this module.
     *
     * Signals that are not available in an unsolicited fashion are the Close Loop
     * gains.  For teams that have a single profile for their TALON close loop they
     * can use either the webpage to configure their TALONs once or set the PIDF,
     * Izone, CloseLoopRampRate, etc... once in the robot application.  These
     * parameters are saved to flash so once they are loaded in the TALON, they
     * will persist through power cycles and mode changes.
     *
     * For teams that have one or two profiles to switch between, they can use the
     * same strategy since there are two slots to choose from and the
     * ProfileSlotSelect is periodically sent in the 10 ms control frame.
     *
     * For teams that require changing gains frequently, they can use the soliciting
     * API to get and set those parameters.  Most likely they will only need to set
     * them in a periodic fashion as a function of what motion the application is
     * attempting.  If this API is used, be mindful of the CAN utilization reported
     * in the driver station.
     *
     * If calling application has used the config routines to configure the
     * selected feedback sensor, then all positions are measured in floating point
     * precision rotations.  All sensor velocities are specified in floating point
     * precision RPM.
     * @see ConfigPotentiometerTurns
     * @see ConfigEncoderCodesPerRev
     * HOWEVER, if calling application has not called the config routine for
     * selected feedback sensor, then all getters/setters for position/velocity use
     * the native engineering units of the Talon SRX firm (just like in 2015).
     * Signals explained below.
     *
     * Encoder position is measured in encoder edges.  Every edge is counted
     * (similar to roboRIO 4X mode).  Analog position is 10 bits, meaning 1024
     * ticks per rotation (0V => 3.3V).  Use SetFeedbackDeviceSelect to select
     * which sensor type you need.  Once you do that you can use GetSensorPosition()
     * and GetSensorVelocity().  These signals are updated on CANBus every 20ms (by
     * default).  If a relative sensor is selected, you can zero (or change the
     * current value) using SetSensorPosition.
     *
     * Analog Input and quadrature position (and velocity) are also explicitly
     * reported in GetEncPosition, GetEncVel, GetAnalogInWithOv, GetAnalogInVel.
     * These signals are available all the time, regardless of what sensor is
     * selected at a rate of 100ms.  This allows easy instrumentation for "in the
     * pits" checking of all sensors regardless of modeselect.  The 100ms rate is
     * overridable for teams who want to acquire sensor data for processing, not
     * just instrumentation.  Or just select the sensor using
     * SetFeedbackDeviceSelect to get it at 20ms.
     *
     * Velocity is in position ticks / 100ms.
     *
     * All output units are in respect to duty cycle (throttle) which is -1023(full
     * reverse) to +1023 (full forward).  This includes demand (which specifies
     * duty cycle when in duty cycle mode) and rampRamp, which is in throttle units
     * per 10ms (if nonzero).
     *
     * Pos and velocity close loops are calc'd as
     *   err = target - posOrVel.
     *   iErr += err;
     *   if(   (IZone!=0)  and  abs(err) > IZone)
     *       ClearIaccum()
     *   output = P X err + I X iErr + D X dErr + F X target
     *   dErr = err - lastErr
     * P, I, and D gains are always positive. F can be negative.
     * Motor direction can be reversed using SetRevMotDuringCloseLoopEn if
     * sensor and motor are out of phase. Similarly feedback sensor can also be
     * reversed (multiplied by -1) if you prefer the sensor to be inverted.
     *
     * P gain is specified in throttle per error tick.  For example, a value of 102
     * is ~9.9% (which is 102/1023) throttle per 1 ADC unit(10bit) or 1 quadrature
     * encoder edge depending on selected sensor.
     *
     * I gain is specified in throttle per integrated error. For example, a value
     * of 10 equates to ~0.99% (which is 10/1023) for each accumulated ADC unit
     * (10 bit) or 1 quadrature encoder edge depending on selected sensor.
     * Close loop and integral accumulator runs every 1ms.
     *
     * D gain is specified in throttle per derivative error. For example a value of
     * 102 equates to ~9.9% (which is 102/1023) per change of 1 unit (ADC or
     * encoder) per ms.
     *
     * I Zone is specified in the same units as sensor position (ADC units or
     * quadrature edges).  If pos/vel error is outside of this value, the
     * integrated error will auto-clear...
     *   if(   (IZone!=0)  and  abs(err) > IZone)
     *       ClearIaccum()
     * ...this is very useful in preventing integral windup and is highly
     * recommended if using full PID to keep stability low.
     *
     * CloseLoopRampRate is in throttle units per 1ms.  Set to zero to disable
     * ramping.  Works the same as RampThrottle but only is in effect when a close
     * loop mode and profile slot is selected.
     *
     */
    public class MotController_LowLevel : Device_LowLevel
    {
        protected const UInt32 STATUS_01 = 0x041400;
        const UInt32 STATUS_02 = 0x041440;
        const UInt32 STATUS_03 = 0x041480;
        const UInt32 STATUS_04 = 0x0414C0;
        const UInt32 STATUS_05 = 0x041500;
        const UInt32 STATUS_06 = 0x041540;
        const UInt32 STATUS_07 = 0x041580;
        const UInt32 STATUS_08 = 0x0415C0;
        protected const UInt32 STATUS_09 = 0x041600;
        const UInt32 STATUS_10 = 0x041640;
        const UInt32 STATUS_11 = 0x041680;
        const UInt32 STATUS_12 = 0x0416C0;
        const UInt32 STATUS_13 = 0x041700;
        const UInt32 STATUS_14 = 0x041740;
        const UInt32 STATUS_15 = 0x041780;

        const UInt32 CONTROL_1 = 0x040000;
        const UInt32 CONTROL_2 = 0x040040;
        const UInt32 CONTROL_3 = 0x040080;
        const UInt32 CONTROL_5 = 0x040100;
        protected const UInt32 CONTROL_6 = 0x040140;

        const UInt32 PARAM_REQ = 0x041800;
        const UInt32 PARAM_RESP = 0x041840;
        const UInt32 PARAM_SET = 0x041880;

        const float FLOAT_TO_FXP_10_22 = (float)0x400000;
        const float FXP_TO_FLOAT_10_22 = 0.0000002384185791015625f;

        const float FLOAT_TO_FXP_0_8 = (float)0x100;
        const float FXP_TO_FLOAT_0_8 = 0.00390625f;

        /* Motion Profile status bits */
        const int kMotionProfileFlag_ActTraj_IsValid = 0x1;
        const int kMotionProfileFlag_HasUnderrun = 0x2;
        const int kMotionProfileFlag_IsUnderrun = 0x4;
        const int kMotionProfileFlag_ActTraj_IsLast = 0x8;
        const int kMotionProfileFlag_ActTraj_VelOnly = 0x10;

        const int kMinFirmwareVersionMajor = 11;
        const int kMinFirmwareVersionMinor = 0;

        /* Motion Profile Set Output */
        // Motor output is neutral, Motion Profile Executer is not running.
        const int kMotionProf_Disabled = 0;
        // Motor output is updated from Motion Profile Executer, MPE will
        // process the buffered points.
        const int kMotionProf_Enable = 1;
        // Motor output is updated from Motion Profile Executer, MPE will
        // stay processing current trajectory point.
        const int kMotionProf_Hold = 2;

        const int kDefaultControl6PeriodMs = 10;

        private UInt64 _cache;
        private UInt32 _len;

        private int _usingAdvancedFeatures = 0;
        private int _setPoint = 0;
        private ControlMode _appliedMode = ControlMode.Disabled;


        //--------------------- Constructors -----------------------------//
        /**
         * Constructor for the CAN Enabled CTRE motor controller.
         * @param deviceNumber The CAN ID of the CTRE motor controller.
         */
        public MotController_LowLevel(int baseArbId)
            : base((uint)baseArbId, (uint)baseArbId | STATUS_05, (uint)baseArbId | PARAM_REQ, (uint)baseArbId | PARAM_RESP, (uint)baseArbId | PARAM_SET, (uint)baseArbId | STATUS_15)
        {
            /* 
             * Talon SRX, Victor SPX, etc., are controlled via two frames...
             * - Enable frame (ArbID $401BF, first data byte is 0 for disable, 1 for enable, other bytes are zero, DLC=8).
             * - Control frame (ArbID uses CONTROL_3), each device gets a unique control frame.
             * ... Enable is taken care of by HERO firmware. Control is done in this class.
             * All CAN Bus framing uses 29bit arbIDs.
             */
            CTRE.Native.CAN.Send(CONTROL_3 | (uint)_baseArbId, 0x00, 8, 10); /* control frame */

            /* fill description */
            StringBuilder work = new StringBuilder();
            switch (_baseArbId & 0xFFFF0000)
            {
                case 0x02040000:
                    work.Append("Talon SRX ");
                    break;
                case 0x01040000:
                    work.Append("Victor SPX ");
                    break;
                default:
                    work.Append("Motor Controller ");
                    break;
            }
            work.Append(GetDeviceNumber());
            SetDescription(work.ToString());
        }

        /* wrapper so we can call with default parameters */
        new void CheckFirmVers(int minMajor = kMinFirmwareVersionMajor, int minMinor = kMinFirmwareVersionMinor, ErrorCode code = ErrorCode.FirmwareTooOld)
        {
            /* check firm */
            base.CheckFirmVers(minMajor, minMinor, code);
        }

        //------ Set output routines. ----------//
        public void SetDemand(ControlMode mode, int demand0, int demand1)
        {
            ErrorCode retval = ErrorCode.OK;

            /* check firm */
            CheckFirmVers();
            if (_usingAdvancedFeatures > 0)
            {
                --_usingAdvancedFeatures;
                CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            }
            /* feature below not implemented yet, just save the error code */
            switch (mode)
            {
                case ControlMode.MotionProfile:
                    CheckFirmVers(3, 2, ErrorCode.MotProfFirmThreshold); /* must use 3.2 or greater */
                    break;
                case ControlMode.MotionProfileArc:
                    CheckFirmVers(3, 4, ErrorCode.MotProfFirmThreshold2); /* must use 3.4 or greater */
                    break;
                case ControlMode.Current:
                case ControlMode.MotionMagic:
                case ControlMode.Position:
                case ControlMode.Velocity:
                case ControlMode.PercentOutput:
                case ControlMode.Follower:
                case ControlMode.Disabled:
                    /* supported */
                    break;

                default:
                    retval = ErrorCode.FeatureNotSupported;
                    break;
            }

            /* if closed looping to caller's target, save the target */
            switch (mode)
            {
                case ControlMode.Current:
                case ControlMode.MotionMagic:
                case ControlMode.Position:
                case ControlMode.Velocity:
                    /* save it for GetClosedLoopTarget */
                    _setPoint = demand0;
                    break;
                case ControlMode.PercentOutput:
                case ControlMode.Follower:
                case ControlMode.Disabled:
                    /* there is no target */
                    break;
                case ControlMode.MotionProfile:
                case ControlMode.MotionProfileArc:
                    /* just use the act traj pt */
                    break;
            }

            /* sterilize inputs */
            switch (mode)
            {
                case ControlMode.PercentOutput:
                    if (false) { }
                    else if (demand0 > +1023) { demand0 = +1023; }
                    else if (demand0 < -1023) { demand0 = -1023; }
                    break;
                case ControlMode.Position: /* if a mode is missing, compiler warning will catch it */
                case ControlMode.Velocity:
                case ControlMode.Current:
                case ControlMode.Follower:
                case ControlMode.MotionProfile:
                case ControlMode.MotionMagic:
                case ControlMode.MotionProfileArc:
                case ControlMode.Disabled:
                    break;
            }

            /* get the frame */
            retval = (ErrorCode)CTRE.Native.CAN.GetSendBuffer(CONTROL_3 | _baseArbId, ref _cache);
            if (retval != ErrorCode.OK) { return; }

            /* unpack */
            byte d0_h8 = (byte)(demand0 >> 0x10);
            byte d0_m8 = (byte)(demand0 >> 0x08);
            byte d0_l8 = (byte)(demand0);
            byte d1_h8 = (byte)(demand1 >> 10);
            byte d1_m8 = (byte)(demand1 >> 2);
            byte d1_l2 = (byte)(demand1 & 0x03);
            int mode_4b = (int)mode & 0xf;
            int EnableAuxPID1 = 0;
            int SelectDemandType = 0;

            /* clear */
            _cache &= ~(0xFFul << 0x00);    /* demand0 */
            _cache &= ~(0xFFul << 0x08);    /* demand0 */
            _cache &= ~(0xFFul << 0x10);    /* demand0 */
            _cache &= ~(0xFFul << 0x18);    /* demand1 */
            _cache &= ~(0xFFul << 0x20);    /* demand1 */
            _cache &= ~(0xE0ul << 0x28);    /* demand1 */
            _cache &= ~(0x0Ful << 0x28);    /* mode_4b */
            _cache &= ~(0x01ul << (0x30 + 5));    /* EnableAuxPID1 */
            _cache &= ~(0x01ul << (0x30 + 6));    /* SelectDemandType */

            /* shift in */
            _cache |= (UInt64)(d0_h8) << 0x00;
            _cache |= (UInt64)(d0_m8) << 0x08;
            _cache |= (UInt64)(d0_l8) << 0x10;
            _cache |= (UInt64)(d1_h8) << 0x18;
            _cache |= (UInt64)(d1_m8) << 0x20;
            _cache |= (UInt64)(d1_l2) << (0x28 + 6);
            _cache |= (UInt64)(mode_4b) << (0x28);
            _cache |= (UInt64)(EnableAuxPID1) << (0x30 + 5);
            _cache |= (UInt64)(SelectDemandType) << (0x30 + 6);

            /* save the mode */
            _appliedMode = mode;

            /* flush changes */
            CTRE.Native.CAN.Send(CONTROL_3 | _baseArbId, _cache, 8, 0xFFFFFFFF);
        }

        /**
 * @param mode
 * @param demand0	If open loop, [-1,+1]
 *					If closed loop, units or units/100ms.
 * @param demand1	if open-loop, [-1,+1]
 * @param demand1Type 0 for off, 1 for AuxiliaryPID, 2 for feedforward
 */
        public ErrorCode Set(ControlMode mode, double demand0, double demand1, int demand1Type)
        {
            ErrorCode retval = ErrorCode.OKAY;
            /* check for ship firm */
            CheckFirmVers();
            if (_usingAdvancedFeatures > 0)
            {
                --_usingAdvancedFeatures;
                CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            }
            /* feature below not implemented yet, just save the error code */
            switch (mode)
            {
                case ControlMode.MotionProfile:
                    CheckFirmVers(11, 2, ErrorCode.MotProfFirmThreshold); /* must use 3.2 or greater */
                    break;
                case ControlMode.MotionProfileArc:
                    CheckFirmVers(11, 4, ErrorCode.MotProfFirmThreshold2); /* must use 3.4 or greater */
                    break;
                case ControlMode.Current:
                case ControlMode.MotionMagic:
                case ControlMode.Position:
                case ControlMode.Velocity:
                case ControlMode.PercentOutput:
                case ControlMode.Follower:
                case ControlMode.Disabled:
                    /* supported */
                    break;

                default:
                    retval = ErrorCode.FeatureNotSupported;
                    break;
            }

            /* if closed looping to caller's target, save the target */
            switch (mode)
            {
                case ControlMode.Current:
                case ControlMode.MotionMagic:
                //case ControlMode.MotionMagicArc:
                case ControlMode.Position:
                case ControlMode.Velocity:
                    /* save it for GetClosedLoopTarget */
                    _setPoint = (int)demand0;
                    break;
                case ControlMode.PercentOutput:
                case ControlMode.Follower:
                case ControlMode.Disabled:
                    /* there is no target */
                    break;
                case ControlMode.MotionProfile:
                case ControlMode.MotionProfileArc:
                    /* just use the act traj pt */
                    break;
            }

            /* scale and sterilize inputs */
            switch (mode)
            {
                case ControlMode.PercentOutput:
                    /* coerce */
                    if (false) { }
                    else if (demand0 > +1) { demand0 = +1; }
                    else if (demand0 < -1) { demand0 = -1; }
                    /* scale [-1,+1] => [-1023,+1023] */
                    demand0 *= 1023;
                    break;
                case ControlMode.Position: /* if a mode is missing, compiler warning will catch it */
                case ControlMode.Velocity:
                case ControlMode.Current:
                case ControlMode.Follower:
                case ControlMode.MotionProfile:
                case ControlMode.MotionMagic:
                //case ControlMode.MotionMagicArc:
                case ControlMode.MotionProfileArc:
                case ControlMode.Disabled:
                    break;
            }

            /* get the frame */
            retval = (ErrorCode)CTRE.Native.CAN.GetSendBuffer(CONTROL_3 | _baseArbId, ref _cache);
            if (retval != ErrorCode.OK) { return retval; }

            int EnableAuxPID1 = 0;
            int SelectDemandType = 0;

            /* scale feedforward */
            switch (demand1Type)
            {
                case 0:
                    EnableAuxPID1 = 0;
                    SelectDemandType = 0;
                    break;
                case 1: /* PID[1] */
                    _usingAdvancedFeatures = 100;
                    EnableAuxPID1 = 1;
                    SelectDemandType = 0;
                    break;
                case 2: /* dem1 is throtBump */
                    _usingAdvancedFeatures = 100;
                    EnableAuxPID1 = 0;
                    SelectDemandType = 1;
                    /* coerce */
                    if (false) { }
                    else if (demand1 > +1) { demand1 = +1; }
                    else if (demand1 < -1) { demand1 = -1; }
                    /* scale [-1,+1] => [-1023,+1023] */
                    demand1 *= 1023;
                    break;
            }

            /* int casts */
            int idemand0 = (int)demand0;
            int idemand1 = (int)demand1;
            /* unpack */
            byte d0_h8 = (byte)(idemand0 >> 0x10);
            byte d0_m8 = (byte)(idemand0 >> 0x08);
            byte d0_l8 = (byte)(idemand0);
            byte d1_h8 = (byte)(idemand1 >> 10);
            byte d1_m8 = (byte)(idemand1 >> 2);
            byte d1_l2 = (byte)(idemand1 & 0x03);
            int mode_4b = (int)mode & 0xf;


            /* clear */
            _cache &= ~(0xFFul << 0x00);    /* demand0 */
            _cache &= ~(0xFFul << 0x08);    /* demand0 */
            _cache &= ~(0xFFul << 0x10);    /* demand0 */
            _cache &= ~(0xFFul << 0x18);    /* demand1 */
            _cache &= ~(0xFFul << 0x20);    /* demand1 */
            _cache &= ~(0xE0ul << 0x28);    /* demand1 */
            _cache &= ~(0x0Ful << 0x28);    /* mode_4b */
            _cache &= ~(0x01ul << (0x30 + 5));    /* EnableAuxPID1 */
            _cache &= ~(0x01ul << (0x30 + 6));    /* SelectDemandType */

            /* shift in */
            _cache |= (UInt64)(d0_h8) << 0x00;
            _cache |= (UInt64)(d0_m8) << 0x08;
            _cache |= (UInt64)(d0_l8) << 0x10;
            _cache |= (UInt64)(d1_h8) << 0x18;
            _cache |= (UInt64)(d1_m8) << 0x20;
            _cache |= (UInt64)(d1_l2) << (0x28 + 6);
            _cache |= (UInt64)(mode_4b) << (0x28);
            _cache |= (UInt64)(EnableAuxPID1) << (0x30 + 5);
            _cache |= (UInt64)(SelectDemandType) << (0x30 + 6);


            /* save the mode */
            _appliedMode = mode;

            /* flush changes */
            CTRE.Native.CAN.Send(CONTROL_3 | _baseArbId, _cache, 8, 0xFFFFFFFF);

            return SetLastError(retval);
        }

        public void SelectDemandType(bool enable)
        {
            SetClrBit(enable ? 1 : 0, 0x30 + 6, CONTROL_3); /* SelectDemandType */
        }

        public void SetNeutralMode(NeutralMode neutralMode)
        {
            byte sig_b2 = (byte)((int)neutralMode & 3);
            SetClrSmallVal(sig_b2, 2, 6, 0, CONTROL_3); /* OverrideBrakeEn */
        }

        //------ Invert behavior ----------//
        public void SetSensorPhase(bool PhaseSensor)
        {
            int aBit = PhaseSensor ? 1 : 0;
            SetClrSmallVal(aBit, 1, 7, 7, CONTROL_3);
        }

        public void SetInverted(bool invert)
        {
            int aBit = invert ? 1 : 0;
            SetClrSmallVal(aBit, 1, 7, 6, CONTROL_3);
        }
        //----- private utility ------------------//
        private int CalcPercPer10Ms(float secondsFromNeutralToFull)
        {
            /* if seconds is zero(or negative) that means disable ramp */
            if (secondsFromNeutralToFull <= 0) { return 0; }
            /* user wants to enable a ramp*/
            int percPer10ms;
            /* user wants to disable */
            percPer10ms = (int)(1023 / (secondsFromNeutralToFull * 100));
            /* if ramps is super slow, step size falls to zero, ensure we are as slow as can be */
            if (percPer10ms == 0) { percPer10ms = 1; }
            /* return [1,+inf] percent per 10ms*/
            return percPer10ms;

        }
        private int CalcMotorOutput(float percentOut)
        {
            return (int)(1023 * percentOut);
        }
        private byte CalcMotorDeadband(float percentOut)
        {
            int retval = (int)(1023 * percentOut);
            if (retval > byte.MaxValue)
                return byte.MaxValue;
            return (byte)retval;
        }
        private int CalcVoltage_8_8(float voltage)
        {
            return (int)(256.0 * voltage);
        }
        int BoundAboveOne(int param)
        {
            if (param < 1) { return 1; }
            return param;
        }

        //----- general output shaping ------------------//
        public ErrorCode ConfigOpenloopRamp(float secondsFromNeutralToFull, int timeoutMs)
        {
            int ramp = CalcPercPer10Ms(secondsFromNeutralToFull);
            return ConfigSetParameter(ParamEnum.eOpenloopRamp, ramp, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigClosedloopRamp(float secondsFromNeutralToFull, int timeoutMs)
        {
            int ramp = CalcPercPer10Ms(secondsFromNeutralToFull);
            return ConfigSetParameter(ParamEnum.eClosedloopRamp, ramp, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigPeakOutputForward(float percentOut, int timeoutMs)
        {
            int param = CalcMotorOutput(percentOut);
            return ConfigSetParameter(ParamEnum.ePeakPosOutput, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigPeakOutputReverse(float percentOut, int timeoutMs)
        {
            int param = CalcMotorOutput(percentOut);
            return ConfigSetParameter(ParamEnum.ePeakNegOutput, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigNominalOutputForward(float percentOut, int timeoutMs)
        {
            int param = CalcMotorOutput(percentOut);
            return ConfigSetParameter(ParamEnum.eNominalPosOutput, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigNominalOutputReverse(float percentOut, int timeoutMs)
        {
            int param = CalcMotorOutput(percentOut);
            return ConfigSetParameter(ParamEnum.eNominalNegOutput, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigNeutralDeadband(float percentDeadband, int timeoutMs)
        {
            byte param = CalcMotorDeadband(percentDeadband);
            return ConfigSetParameter(ParamEnum.eNeutralDeadband, param, 0, 0, timeoutMs);
        }

        //------ Voltage Compensation ----------//
        public ErrorCode ConfigVoltageCompSaturation(float voltage, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eNominalBatteryVoltage, voltage, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs)
        {
            int param = BoundAboveOne(filterWindowSamples);
            return ConfigSetParameter(ParamEnum.eBatteryVoltageFilterSize, param, 0, 0, timeoutMs);
        }
        public void EnableVoltageCompensation(bool enable)
        {
            SetClrBit(enable ? 1 : 0, 5 * 8 + 4, CONTROL_3); /* EnableVoltageCompen */
        }

        //------ General Status ----------//
        public ErrorCode GetBusVoltage(out float param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_04 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)(_cache);
            Int32 raw = 0;
            raw |= L;
            param = 0.05F * raw + 4F;
            return SetLastError(retval);
        }
        public ErrorCode GetMotorOutputPercent(out float param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_01 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 24);
            byte L = (byte)(_cache >> 32);
            H &= 0x7;
            L &= 0xff;
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 11);
            raw >>= (32 - 11);
            param = ((float)raw) / 1023.0F;
            return SetLastError(retval);
        }
        public ErrorCode GetOutputCurrent(out float param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_02 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 40);
            byte L = (byte)(_cache >> 48);
            H &= 0xff;
            L &= 0xc0;
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= L;
            raw >>= 6;
            param = 0.125F * raw + 0F;
            return SetLastError(retval);
        }
        public ErrorCode GetTemperature(out float param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_04 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)(_cache >> 56);
            L &= 0x3F;
            Int32 raw = 0;
            raw |= L;
            param = (float)raw;
            return SetLastError(retval);
        }
        //------ sensor selection ----------//
        public ErrorCode ConfigSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs)
        {
            switch (pidIdx)
            {
                case 0: break;
                case 1: break;
                default: return ErrorCode.InvalidParamValue;
            }

            /* check param */
            switch (feedbackDevice)
            {
                case FeedbackDevice.RemoteSensor0:
                case FeedbackDevice.RemoteSensor1:
                    CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm); //Must use latest firmware
                    _usingAdvancedFeatures = 100;
                    break;
                case FeedbackDevice.QuadEncoder:
                case FeedbackDevice.Analog:
                case FeedbackDevice.Tachometer:
                case FeedbackDevice.PulseWidthEncodedPosition:
                case FeedbackDevice.SoftwareEmulatedSensor:
                case FeedbackDevice.SensorSum:
                case FeedbackDevice.SensorDifference:
                    break;
                default:
                    break;
            }
            int param = (int)feedbackDevice;
            return ConfigSetParameter(ParamEnum.eFeedbackSensorType, param, 0, pidIdx, timeoutMs);
        }

        public ErrorCode ConfigSelectedFeedbackCoefficient(
        float coefficient, int pidIdx, int timeoutMs)
        {
            CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            _usingAdvancedFeatures = 100;

            return ConfigSetParameter(ParamEnum.eSelectedSensorCoefficient, coefficient, 0, pidIdx, timeoutMs);
        }

        public ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
        RemoteSensorSource remoteSensorSource, int remoteOrdinal, int timeoutMs)
        {

            CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            _usingAdvancedFeatures = 100;
            if (remoteOrdinal < 0) { return ErrorCode.InvalidParamValue; }
            if (remoteOrdinal > 1) { return ErrorCode.InvalidParamValue; }

            ErrorCode err1 = ConfigSetParameter(ParamEnum.eRemoteSensorSource, (int)remoteSensorSource, 0, remoteOrdinal, timeoutMs);
            ErrorCode err2 = ConfigSetParameter(ParamEnum.eRemoteSensorDeviceID, deviceID, 0, remoteOrdinal, timeoutMs);

            if (err1 != ErrorCode.OK)
            {
                return SetLastError(err1);
            }
            else
            {
                return SetLastError(err2);
            }
        }

        public ErrorCode ConfigSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs)
        {
            CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            _usingAdvancedFeatures = 100;

            int ordinal = (int)sensorTerm;

            ErrorCode err1 = ErrorCode.OKAY;
            switch (feedbackDevice)
            {
                case FeedbackDevice.SensorDifference:
                case FeedbackDevice.SensorSum:
                    /* it makes no sense to select these */
                    err1 = ErrorCode.InvalidParamValue;
                    break;
                default:
                    break;
            }

            ErrorCode err2 = ConfigSetParameter(ParamEnum.eSensorTerm, (int)feedbackDevice, 0, ordinal, timeoutMs);

            if (err1 != ErrorCode.OK)
            {
                return SetLastError(err1);
            }
            else
            {
                return SetLastError(err2);
            }
        }

        //------- sensor status --------- //
        public ErrorCode GetSelectedSensorPosition(out int param, int pidIdx)
        {
            if (pidIdx == 0)
            {
                int err = CTRE.Native.CAN.Receive(STATUS_02 | _baseArbId, ref _cache, ref _len);
                byte H = (byte)(_cache >> 0);
                byte M = (byte)(_cache >> 8);
                byte L = (byte)(_cache >> 16);
                int PosDiv8 = (int)((_cache >> (0x38 + 4)) & 1);
                param = 0;
                param |= H;
                param <<= 8;
                param |= M;
                param <<= 8;
                param |= L;
                param <<= (32 - 24); /* sign extend */
                param >>= (32 - 24); /* sign extend */
                if (PosDiv8 == 1) { param *= 8; }
                return SetLastError(err);
            }
            else if (pidIdx == 1)
            {
                int err = CTRE.Native.CAN.Receive(STATUS_12 | _baseArbId, ref _cache, ref _len);
                byte H = (byte)(_cache >> 0);
                byte M = (byte)(_cache >> 8);
                byte L = (byte)(_cache >> 16);
                int PosDiv8 = (int)((_cache >> (0x20 + 7)) & 1);
                param = 0;
                param |= H;
                param <<= 8;
                param |= M;
                param <<= 8;
                param |= L;
                param <<= (32 - 24); /* sign extend */
                param >>= (32 - 24); /* sign extend */
                if (PosDiv8 == 1) { param *= 8; }
                return SetLastError(err);
            }
            else
            {
                param = 0;
                return SetLastError(ErrorCode.InvalidParamValue);
            }
        }

        public ErrorCode GetSelectedSensorVelocity(out int param, int pidIdx)
        {
            if (pidIdx == 0)
            {
                int err = CTRE.Native.CAN.Receive(STATUS_02 | _baseArbId, ref _cache, ref _len);
                byte H = (byte)(_cache >> 24);
                byte L = (byte)(_cache >> 32);
                int velDiv4 = (int)((_cache >> (0x38 + 3)) & 1);
                param = 0;
                param |= H;
                param <<= 8;
                param |= L;
                param <<= (32 - 16); /* sign extend */
                param >>= (32 - 16); /* sign extend */
                if (velDiv4 == 1)
                    param *= 4;
                return SetLastError(err);
            }
            else if (pidIdx == 1)
            {
                int err = CTRE.Native.CAN.Receive(STATUS_12 | _baseArbId, ref _cache, ref _len);
                byte H = (byte)(_cache >> 24);
                byte L = (byte)(_cache >> 32);
                int velDiv4 = (int)((_cache >> (0x20 + 7)) & 1);
                param = 0;
                param |= H;
                param <<= 8;
                param |= L;
                param <<= (32 - 16); /* sign extend */
                param >>= (32 - 16); /* sign extend */
                if (velDiv4 == 1)
                    param *= 4;
                return SetLastError(err);
            }
            else
            {
                param = 0;
                return SetLastError(ErrorCode.InvalidParamValue);
            }
        }

        public ErrorCode SetSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs)
        {
            if (pidIdx == 0)
            {
                int param = (int)sensorPos;
                return ConfigSetParameter(ParamEnum.eSelectedSensorPosition, param, 0, 0, timeoutMs);
            }
            else if (pidIdx == 1)
            {
                /* teams can set the secondary feedback sensor
                 *  with the low level API for remote object API */
                return SetLastError(ErrorCode.AuxiliaryPIDNotSupportedYet);
            }
            /* invalid PID */
            return SetLastError(ErrorCode.InvalidParamValue);
        }
        //------ status frame period changes ----------//
        public ErrorCode SetControlFramePeriod(ControlFrame frame, int periodMs)
        {
            /* sterilize inputs */
            if (periodMs < 0) { periodMs = 0; }
            if (periodMs > 0xFF) { periodMs = 0xFF; }

            uint fullId = (uint)((int)_baseArbId | (int)frame); /* build ID */

            /* apply the change if frame is transmitting */
            int err = CTRE.Native.CAN.GetSendBuffer(fullId, ref _cache);
            if (err == 0)
            {
                err = CTRE.Native.CAN.Send(fullId, _cache, 8, (uint)periodMs);
            }

            return SetLastError(err);
        }
        public ErrorCode SetStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs)
        {
            int fullId = (int)((int)_baseArbId | (int)frame); /* build ID */

            return base.SetStatusFramePeriod(fullId, periodMs, timeoutMs);
        }
        public ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs)
        {
            int fullId = (int)((int)_baseArbId | (int)frame); /* build ID */

            return base.SetStatusFramePeriod(fullId, periodMs, timeoutMs);
        }
        public ErrorCode GetStatusFramePeriod(StatusFrame frame, out int periodMs, int timeoutMs)
        {
            int fullId = (int)((int)_baseArbId | (int)frame); /* build ID */

            return base.GetStatusFramePeriod(fullId, out periodMs, timeoutMs);
        }
        public ErrorCode GetStatusFramePeriod(StatusFrameEnhanced frame, out int periodMs, int timeoutMs)
        {
            int fullId = (int)((int)_baseArbId | (int)frame); /* build ID */

            return base.GetStatusFramePeriod(fullId, out periodMs, timeoutMs);
        }
        //----- velocity signal conditionaing ------//
        public ErrorCode ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs)
        {
            int param = (int)period;
            return ConfigSetParameter(ParamEnum.eSampleVelocityPeriod, param, 0, 0, timeoutMs);
        }

        public ErrorCode ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs)
        {
            int param = (int)windowSize;
            return ConfigSetParameter(ParamEnum.eSampleVelocityWindow, param, 0, 0, timeoutMs);
        }
        //------ ALL limit switch ----------//
        ErrorCode ConfigSingleLimitSwitchSource(
                LimitSwitchSource limitSwitchSource, LimitSwitchNormal normalOpenOrClose,
                int deviceIDIfApplicable, int timeoutMs, bool isForward)
        {

            int ordinal = isForward ? 0 : 1; /* forward or reverse */

            ErrorCode err1 = ErrorCode.InvalidParamValue;
            switch (limitSwitchSource)
            {
                case LimitSwitchSource.FeedbackConnector:
                case LimitSwitchSource.RemoteTalonSRX:
                case LimitSwitchSource.RemoteCANifier:
                case LimitSwitchSource.Deactivated:
                    err1 = ConfigSetParameter(ParamEnum.eLimitSwitchSource, (float)limitSwitchSource, 0, ordinal, timeoutMs);
                    break;
            };

            ErrorCode err2 = ErrorCode.InvalidParamValue;
            switch (normalOpenOrClose)
            {
                case LimitSwitchNormal.NormallyOpen:
                case LimitSwitchNormal.NormallyClosed:
                case LimitSwitchNormal.Disabled:
                    err2 = ConfigSetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, (float)normalOpenOrClose, 0, ordinal, timeoutMs);
                    break;
            };


            ErrorCode err3 = ConfigSetParameter(ParamEnum.eLimitSwitchRemoteDevID, deviceIDIfApplicable, 0, ordinal, timeoutMs);


            if (err1 != ErrorCode.OK)
            {
                SetLastError(err1);
            }
            else if (err2 != ErrorCode.OK)
            {
                SetLastError(err2);
            }
            else
            {
                SetLastError(err3);
            }

            return GetLastError();
        }
        public ErrorCode ConfigForwardLimitSwitchSource(
        LimitSwitchSource limitSwitchSource,
        LimitSwitchNormal normalOpenOrClose, int deviceIDIfApplicable,
        int timeoutMs)
        {

            return ConfigSingleLimitSwitchSource(limitSwitchSource, normalOpenOrClose,
                    deviceIDIfApplicable, timeoutMs, true);
        }

        public ErrorCode ConfigReverseLimitSwitchSource(
        LimitSwitchSource limitSwitchSource,
        LimitSwitchNormal normalOpenOrClose, int deviceIDIfApplicable,
        int timeoutMs)
        {

            return ConfigSingleLimitSwitchSource(limitSwitchSource, normalOpenOrClose,
                    deviceIDIfApplicable, timeoutMs, false);
        }
        public void OverrideLimitSwitchesEnable(bool enable)
        {
            SetClrBit(enable ? 0 : 1, 0x30 + 7, CONTROL_3); /* LimitSwitchDisable */
        }
        //------ soft limit ----------//
        public ErrorCode ConfigForwardSoftLimit(int forwardSensorLimit, int timeoutMs)
        {
            int param = (int)forwardSensorLimit;
            return ConfigSetParameter(ParamEnum.eForwardSoftLimitThreshold, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigReverseSoftLimit(int reverseSensorLimit, int timeoutMs)
        {
            int param = (int)reverseSensorLimit;
            return ConfigSetParameter(ParamEnum.eReverseSoftLimitThreshold, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigForwardSoftLimitEnable(bool enable, int timeoutMs)
        {
            int param = enable ? 1 : 0;
            return ConfigSetParameter(ParamEnum.eForwardSoftLimitEnable, param, 0, 0,
                    timeoutMs);
        }
        public ErrorCode ConfigReverseSoftLimitEnable(bool enable, int timeoutMs)
        {
            int param = enable ? 1 : 0;
            return ConfigSetParameter(ParamEnum.eReverseSoftLimitEnable, param, 0, 0,
                    timeoutMs);
        }
        public void OverrideSoftLimitsEnable(bool enable)
        {
            SetClrBit((!enable) ? 1 : 0, 8 * 5 + 5, CONTROL_3); /* DisableSoftLimits */
        }

        //------ Current Lim ----------//
        public ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs)
        {
            int param = BoundAboveOne(amps);
            return ConfigSetParameter(ParamEnum.ePeakCurrentLimitAmps, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigPeakCurrentDuration(int milliseconds, int timeoutMs)
        {
            int param = BoundAboveOne(milliseconds);
            return ConfigSetParameter(ParamEnum.ePeakCurrentLimitMs, param, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigContinuousCurrentLimit(int amps, int timeoutMs)
        {
            int param = BoundAboveOne(amps);
            return ConfigSetParameter(ParamEnum.eContinuousCurrentLimitAmps, param, 0, 0, timeoutMs);
        }
        public void EnableCurrentLimit(bool enable)
        {
            SetClrBit(enable ? 1 : 0, 0x38 + 4, CONTROL_3); /* EnCurrentLimit */
        }

        //------ General Close loop ----------//
        public ErrorCode Config_kP(int slotIdx, float value, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_P, value, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode Config_kI(int slotIdx, float value, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_I, value, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode Config_kD(int slotIdx, float value, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_D, value, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode Config_kF(int slotIdx, float value, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_F, value, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode Config_IntegralZone(int slotIdx, int izone, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_IZone, izone, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode ConfigAllowableClosedloopError(int slotIdx, int allowableCloseLoopError, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_AllowableErr, allowableCloseLoopError, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, float iaccum , int timeoutMs = 0)
        {
            CheckFirmVers(11, 2); /* must use 11.2 or greater */
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_MaxIAccum, iaccum, 0x00, slotIdx, timeoutMs);
        }

        public ErrorCode ConfigClosedLoopPeakOutput(int slotIdx, float percentOut, int timeoutMs)
        {
            CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            _usingAdvancedFeatures = 100;
            return ConfigSetParameter(ParamEnum.eProfileParamSlot_PeakOutput, percentOut, 0x00, slotIdx, timeoutMs);
        }
        public ErrorCode ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs)
        {
            CheckFirmVers(11, 8, ErrorCode.TalonFeatureRequiresHigherFirm);
            _usingAdvancedFeatures = 100;
            return ConfigSetParameter(ParamEnum.ePIDLoopPeriod, loopTimeMs, 0x00, slotIdx, timeoutMs);
        }

        public ErrorCode SetIntegralAccumulator(float iaccum = 0, int timeoutMs = 0)
        {
            return ConfigSetParameter(ParamEnum.eClosedLoopIAccum, iaccum, 0x00, 0x00, timeoutMs);
        }

        public ErrorCode GetClosedLoopError(out int error, int pidIdx)
        {
            uint statusID = (pidIdx == 0) ? STATUS_13 : STATUS_14;
            int err = CTRE.Native.CAN.Receive(statusID | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x00);
            byte M = (byte)(_cache >> 0x08);
            byte L = (byte)(_cache >> 0x10);
            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= M;
            param <<= 8;
            param |= L;
            param <<= (32 - 24); /* sign extend */
            param >>= (32 - 24); /* sign extend */
            error = param;
            return SetLastError(err);
        }
        public ErrorCode GetIntegralAccumulator(out float iaccum, int pidIdx)
        {
            uint statusID = (pidIdx == 0) ? STATUS_13 : STATUS_14;
            int err = CTRE.Native.CAN.Receive(statusID | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x18);
            byte M = (byte)(_cache >> 0x20);
            byte L = (byte)(_cache >> 0x28);
            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= M;
            param <<= 8;
            param |= L;
            param <<= (32 - 24); /* sign extend */
            param >>= (32 - 24); /* sign extend */
            iaccum = param;
            return SetLastError(err);
        }
        public ErrorCode GetErrorDerivative(out float derivError, int pidIdx)
        {
            uint statusID = (pidIdx == 0) ? STATUS_13 : STATUS_14;
            int err = CTRE.Native.CAN.Receive(statusID | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x30);
            byte L = (byte)(_cache >> 0x38);
            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= L;
            param <<= (32 - 16); /* sign extend */
            param >>= (32 - 16); /* sign extend */
            derivError = param;
            return SetLastError(err);
        }
        public ErrorCode SelectProfileSlot(int slotIdx, int pidIdx)
        {
            ErrorCode retval = ErrorCode.OK;
            /* sterilize inputs */
            if (slotIdx > 3) { slotIdx = 3; }
            else if (slotIdx < 0) { slotIdx = 0; }

            if (pidIdx == 0)
            {
                SetClrSmallVal(slotIdx, 2, 7, 0, CONTROL_3);
            }
            else if (pidIdx == 1)
            {
                SetClrSmallVal(slotIdx, 2, 7, 2, CONTROL_3);
            }
            else
            {
                /* no return code so nothing to pass to caller */
                retval = ErrorCode.InvalidParamValue;
            }
            
            return SetLastError(retval);
        }
        //------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//

        public ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eMotMag_VelCruise, sensorUnitsPer100ms, 0, 0, timeoutMs);
        }
        public ErrorCode ConfigMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eMotMag_Accel, sensorUnitsPer100msPerSec, 0, 0, timeoutMs);
        }

        public ErrorCode GetClosedLoopTarget(out int value, int pidIdx)
        {
            /* clear output */
            value = 0;

            if (pidIdx == 0)
            {
                /* look at applied mode to see if call is valid */
                switch (_appliedMode)
                {
                    case ControlMode.Current:
                    case ControlMode.MotionMagic:
                    //case ControlMode.MotionMagicArc:
                    case ControlMode.Position:
                    case ControlMode.Velocity:
                        /* update the variable */
                        value = _setPoint;
                        return ErrorCode.OK;
                    case ControlMode.Disabled:
                    case ControlMode.Follower:
                    case ControlMode.PercentOutput:
                        /* set point is not saved, use a different call */
                        return SetLastError(ErrorCode.ControlModeNotValid);
                    case ControlMode.MotionProfile:
                    case ControlMode.MotionProfileArc:
                        /* get the target of the current point */
                        return GetActiveTrajectoryPosition(out value);
                    default:
                        return SetLastError(ErrorCode.InvalidParamValue);
                }
            }
            if (pidIdx == 1)
            {
                return SetLastError(ErrorCode.NotImplemented);
            }
            return SetLastError(ErrorCode.InvalidParamValue);
        }

        public ErrorCode GetActiveTrajectoryVelocity(
        out int sensorUnitsPer100ms)
        {
            int err = CTRE.Native.CAN.Receive(STATUS_10 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache);
            byte L = (byte)(_cache >> 0x08);
            int veldiv4 = (int)((_cache >> (0x38 + 3)) & 1);
            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= L;
            param <<= (32 - 16); /* sign extend */
            param >>= (32 - 16); /* sign extend */

            sensorUnitsPer100ms = param;
            if (veldiv4 == 1)
                sensorUnitsPer100ms *= 4;
            return SetLastError(err);
        }

        public ErrorCode GetActiveTrajectoryPosition(
        out int sensorUnits)
        {
            int err = CTRE.Native.CAN.Receive(STATUS_10 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x10);
            byte M = (byte)(_cache >> 0x18);
            byte L = (byte)(_cache >> 0x20);
            int posdiv8 = (int)((_cache >> (0x38 + 2)) & 1);
            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= M;
            param <<= 8;
            param |= L;
            param <<= (32 - 24); /* sign extend */
            param >>= (32 - 24); /* sign extend */
            sensorUnits = param;
            if (posdiv8 == 1)
                sensorUnits *= 8;
            return SetLastError(err);
        }

        public ErrorCode GetActiveTrajectoryHeading(
        out double turnUnits)
        {
            int err = CTRE.Native.CAN.Receive(STATUS_10 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x28);
            byte M = (byte)(_cache >> 0x30);
            byte L = (byte)((_cache >> (0x38 + 4)) & 0xF);

            int param;
            param = 0;
            param |= H;
            param <<= 8;
            param |= M;
            param <<= 4;
            param |= L;
            param <<= (32 - 20); /* sign extend */
            param >>= (32 - 20); /* sign extend */
            turnUnits = param;
            return SetLastError(err);
        }

        //------ Motion Profile Buffer ----------//
        /* implemented in child class */


        //------ Faults ----------//
        public ErrorCode GetFaults(Faults toFill)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_01 | _baseArbId, ref _cache, ref _len);
            toFill.HardwareFailure =    ((_cache) & 1) == 1;
            toFill.ReverseLimitSwitch = ((_cache >> 1) & 1) == 1;
            toFill.ForwardLimitSwitch = ((_cache >> 2) & 1) == 1;
            toFill.UnderVoltage =       ((_cache >> 3) & 1) == 1;
            toFill.ResetDuringEn =      ((_cache >> 4) & 1) == 1;
            toFill.SensorOutOfPhase =   ((_cache >> 5) & 1) == 1;
            toFill.SensorOverflow =     ((_cache >> 6) & 1) == 1;
            toFill.ReverseSoftLimit = ((_cache >> (0x18)) & 1) == 1;
            toFill.ForwardSoftLimit = ((_cache >> (0x18 + 1)) & 1) == 1;
            toFill.HardwareESDReset = ((_cache >> (0x18 + 2)) & 1) == 1;
            toFill.RemoteLossOfSignal = ((_cache >> (0x30 + 4)) & 1) == 1;
            return SetLastError(retval);
        }
        public ErrorCode GetStickyFaults(Faults toFill)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_02 | _baseArbId, ref _cache, ref _len);
            toFill.RemoteLossOfSignal = ((_cache >> (0x38 + 0)) & 1) == 1;
            toFill.HardwareESDReset =   ((_cache >> (0x38 + 1)) & 1) == 1;
            toFill.ResetDuringEn =      ((_cache >> (0x38 + 2)) & 1) == 1;
            toFill.SensorOutOfPhase =   ((_cache >> (0x38 + 5)) & 1) == 1;
            toFill.SensorOverflow =     ((_cache >> (0x38 + 6)) & 1) == 1;
            toFill.ReverseSoftLimit =   ((_cache >> (0x30 + 0)) & 1) == 1;
            toFill.ForwardSoftLimit =   ((_cache >> (0x30 + 1)) & 1) == 1;
            toFill.ReverseLimitSwitch = ((_cache >> (0x30 + 2)) & 1) == 1;
            toFill.ForwardLimitSwitch = ((_cache >> (0x30 + 3)) & 1) == 1;
            toFill.UnderVoltage =       ((_cache >> (0x30 + 4)) & 1) == 1;
            return SetLastError(retval);
        }
        public ErrorCode ClearStickyFaults(int timeoutMs = 0)
        {
            return ConfigSetParameter(ParamEnum.eStickyFaults, 0, 0, 0, timeoutMs);
        }

        //------ Motor Controller Sensor Collection API ---------//
        public ErrorCode GetAnalogInWithOv(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_04 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x10);
            byte M = (byte)(_cache >> 0x18);
            byte L = (byte)(_cache >> 0x20);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= M;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 24); /* sign extend */
            raw >>= (32 - 24); /* sign extend */
            param = (int)raw;
            return SetLastError(retval);
        }
        public ErrorCode GetAnalogInVel(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_04 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)((_cache >> 0x18) & 3);
            byte L = (byte)(_cache >> 0x20);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 16); /* sign extend */
            raw >>= (32 - 16); /* sign extend */
            param = (int)raw;
            return SetLastError(retval);
        }

        
        public ErrorCode GetQuadraturePosition(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_03 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0);
            byte M = (byte)(_cache >> 8);
            byte L = (byte)(_cache >> 16);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= M;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 24); /* sign extend */
            raw >>= (32 - 24); /* sign extend */

            int posDiv8 = (int)((_cache >> (0x38 + 4)) & 1);
            if(posDiv8 == 1) { raw *= 8; }

            param = (int)raw;
            return SetLastError(retval);
        }

        
        public ErrorCode GetQuadratureVelocity(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_03 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 24);
            byte L = (byte)(_cache >> 32);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 16); /* sign extend */
            raw >>= (32 - 16); /* sign extend */

            int velDiv4 = (int)((_cache >> (0x38 + 3)) & 1);
            if (velDiv4 == 1) { raw *= 4; }

            param = (int)raw;
            return SetLastError(retval);
        }
        public ErrorCode GetPulseWidthPosition(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_08 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0);
            byte M = (byte)(_cache >> 8);
            byte L = (byte)(_cache >> 16);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= M;
            raw <<= 8;
            raw |= L;
            raw <<= (32 - 24); /* sign extend */
            raw >>= (32 - 24); /* sign extend */
            param = (int)raw;
            return SetLastError(retval);
        }
        public ErrorCode GetPulseWidthVelocity(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_08 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)((_cache >> 0x28) & 0x1F);
            byte M = (byte)(_cache >> 0x30);
            byte L = (byte)((_cache >> (0x38 + 5)) & 0x3);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= M;
            raw <<= 3;
            raw |= L;
            raw <<= (32 - 16); /* sign extend */
            raw >>= (32 - 16); /* sign extend */
            param = (int)raw;
            return SetLastError(retval);
        }
        public ErrorCode GetPulseWidthRiseToFallUs(out int param)
        {
            const int BIT12 = 1 << 12;
            int temp = 0;
            int periodUs = 0;
            /* first grab our 12.12 position */
            ErrorCode retval1 = GetPulseWidthPosition(out temp);
            /* mask off number of turns */
            temp &= 0xFFF;
            /* next grab the waveform period. This value
             * will be zero if we stop getting pulses **/
            ErrorCode retval2 = GetPulseWidthRiseToRiseUs(out periodUs);
            /* now we have 0.12 position that is scaled to the waveform period.
                    Use fixed pt multiply to scale our 0.16 period into us.*/
            param = (temp * periodUs) / BIT12;
            /* pass the first error code */
            if (retval1 == 0)
                retval1 = retval2;
            return SetLastError(retval1);
        }
        public ErrorCode GetPulseWidthRiseToRiseUs(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_08 | _baseArbId, ref _cache, ref _len);
            byte H = (byte)(_cache >> 0x18);
            byte M = (byte)(_cache >> 0x20);
            byte L = (byte)((_cache >> (0x28 + 5)) & 0x3);
            Int32 raw = 0;
            raw |= H;
            raw <<= 8;
            raw |= M;
            raw <<= 3;
            raw |= L;
            raw <<= (32 - 19); /* sign extend */
            raw >>= (32 - 19); /* sign extend */
            param = (int)raw;
            return SetLastError(retval);
        }
        public ErrorCode GetPinStateQuadA(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_03 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)(_cache >> 63);
            param = (L & 1);
            return SetLastError(retval);
        }
        /**
         * @return IO level of QUADB pin.
         */
        public ErrorCode GetPinStateQuadB(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_03 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)(_cache >> 62);
            param = (L & 1);
            return SetLastError(retval);
        }
        /**
         * @return IO level of QUAD Index pin.
         */
        public ErrorCode GetPinStateQuadIdx(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_03 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)(_cache >> 61);
            param = (L & 1);
            return SetLastError(retval);
        }
        /**
         * @return '1' iff forward limit switch is closed, 0 iff switch is open.
         * This function works regardless if limit switch feature is enabled.
         */
        public ErrorCode IsFwdLimitSwitchClosed(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_01 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)((_cache >> (0x18 + 7)) & 1);
            param = L == 0 ? 1 : 0;
            return SetLastError(retval);
        }
        /**
         * @return '1' iff reverse limit switch is closed, 0 iff switch is open.
         * This function works regardless if limit switch feature is enabled.
         */
        public ErrorCode IsRevLimitSwitchClosed(out int param)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_01 | _baseArbId, ref _cache, ref _len);
            byte L = (byte)((_cache >> (0x18 + 6)) & 1);
            param = L == 0 ? 1 : 0;
            return SetLastError(retval);
        }

        ErrorCode SetAnalogPosition(int newPosition,
        int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eAnalogPosition, newPosition, 0, 0, timeoutMs);
        }
        ErrorCode SetQuadraturePosition(int newPosition,
                int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.eQuadraturePosition, newPosition, 0, 0, timeoutMs);
        }
        ErrorCode SetPulseWidthPosition(int newPosition,
                int timeoutMs)
        {
            return ConfigSetParameter(ParamEnum.ePulseWidthPosition, newPosition, 0, 0, timeoutMs);
        }


        //------ Utility ----------//
        public void SetClrBit(int bit, int shift, uint baseId)
        {
            /* get the frame */
            int retval = CTRE.Native.CAN.GetSendBuffer(baseId | _baseArbId, ref _cache);
            if (retval != 0) { return; }

            /* clear */
            _cache &= ~(0x1ul << shift);

            /* shift in */
            if (bit != 0)
            {
                _cache |= (UInt64)(1) << (shift);
            }

            /* flush changes */
            CTRE.Native.CAN.Send(baseId | _baseArbId, _cache, 8, 0xFFFFFFFF);
        }
        public void SetClrSmallVal(int value, int bitTotal, int byteIdx, int bitShift_LE, uint baseId)
        {
            UInt64 valu = (UInt64)value;
            /* get the frame */
            int retval = CTRE.Native.CAN.GetSendBuffer(baseId | _baseArbId, ref _cache);
            if (retval != 0) { return; }

            /* make the mask */
            UInt64 mask = 1;
            mask <<= bitTotal;
            --mask;

            /* shfit byte mask and byt value to correct spot within byte */
            mask <<= bitShift_LE;
            valu <<= bitShift_LE;

            /* clear mask within byte*/
            _cache &= ~(mask << (byteIdx * 8));

            /* shift in value within byte */
            _cache |= valu << (byteIdx * 8);

            /* flush changes */
            CTRE.Native.CAN.Send(baseId | _baseArbId, _cache, 8, 0xFFFFFFFF);
        }

        //-------------------------------- Device_LowLevel requirements -----------------------//
        protected override void EnableFirmStatusFrame(bool enable)
        {
            SetClrBit(enable ? 0 : 1, 7 * 8 + 5, CONTROL_3); /* DisableFirmStatusFrame */
        }
        protected override ErrorCode SetLastError(ErrorCode errorCode)
        {
            _lastError.SetLastError(errorCode);
            return _lastError;
        }
        public ErrorCode GetLastError()
        {
            return _lastError.GetLastError();
        }

        protected ErrorCode SetLastError(int errorCode)
        {
            _lastError.SetLastError((ErrorCode)errorCode);
            return _lastError;
        }

        //------ Custom Persistent Params ----------//
        public new ErrorCode ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs = 0)
        {
            return base.ConfigSetCustomParam(newValue, paramIndex, timeoutMs);
        }
        public new ErrorCode ConfigGetCustomParam(out int readValue, int paramIndex, int timeoutMs = Constants.GetParamTimeoutMs)
        {
            return base.ConfigGetCustomParam(out readValue, paramIndex, timeoutMs);
        }
    }
}
