package frc.robot.FLYTLib.FLYTMotorLib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;















public class SparkController extends SuperController{

    /*
     * Rev Library
     */
    //motor controller
    SparkMax sparkMax; //General controller
    SparkMaxConfig config; //Controller configuration objects



    //encoders
    AbsoluteEncoder absEncoder; //absalute encoder
    RelativeEncoder relEncoder; //relative encoder
    EncoderConfig encoderConfig; //general encoder configuration
    AbsoluteEncoderConfig absEncoderConfig; //absalute


    //Motion contorler
    MAXMotionConfig motionConfig;



    //closed loop
    ClosedLoopConfig closedLoopCfg; //closed loop configuration
    SparkClosedLoopController closedLoopController; //closed loop control

 
    //private vars for internal calculation and specifications
    private boolean e_encoderAvailable = false; //check if enxternal encoder connected
    private boolean e_absalute = false; //check if specified encoder is absalute
    private boolean pidREADY =  false; //  checks and sees if pid setup was successfully used
    public boolean pidDisabled = true;
    private ControlType controlType;
    private double motorID;

    



    /**
     * Run this constructor for brushed and brushless motors with no connected external encoders.
     * You are required to run encoderCfg() if brushless is chosen.
     * @param m_id - motor id
     * @param m_brushless - motor type
     * @param m_break - motor idle mode
     */
    public SparkController(int m_id, boolean m_brushless, boolean m_break, boolean invert){
        //setup sparkmax object reference
        sparkMax = new SparkMax(m_id, m_brushless ? MotorType.kBrushless : MotorType.kBrushed);
        
        config = new SparkMaxConfig();
        config.inverted(invert);
        ControllerUpdate();
        encoderConfig = new EncoderConfig();
        motorID = m_id;
        //checks if brushless or not, since by defult brushless has encoder
        if(m_brushless){
            relEncoder = sparkMax.getEncoder(); 
            e_encoderAvailable = true;

        }

        //something should be added here later

    }



    /**
     * Run this constructor for brushed and brushless motors that have connected external encoder
     * @param m_id - motor id
     * @param m_brushless - motor type
     * @param m_break - motor idel mode
     * @param e_absalute - encoder type
     */
    public SparkController(int m_id, boolean m_brushless, boolean m_break, boolean invert, boolean e_absalute){
        //setup sparkmax object reference
        sparkMax = new SparkMax(m_id, m_brushless ? MotorType.kBrushless : MotorType.kBrushed);
        config = new SparkMaxConfig();
        config.inverted(invert);
        ControllerUpdate();

        //if motor is brushless and has connected absalute encoder
        if(m_brushless && e_absalute){
            e_absalute = true;
            absEncoder = sparkMax.getAbsoluteEncoder();
            e_encoderAvailable = true;
        //if motor is brushless and encoder is not absalute
        }else if(m_brushless && !e_absalute){
            e_absalute = false;
            relEncoder = sparkMax.getAlternateEncoder();
            e_encoderAvailable = true;
        //if motor is brushed and connected absalute encoder
        }else if(!m_brushless && e_absalute){
            e_absalute = true;
            absEncoder = sparkMax.getAbsoluteEncoder();
            e_encoderAvailable = true;
            //if motor is brushed and conected relative encoder
        }else if(!m_brushless && !e_absalute){
            e_absalute = false;
            relEncoder = sparkMax.getAlternateEncoder();
            e_encoderAvailable = true;
        }
    }


    /**
     * Set relative speed, multiplies factor by availble voltage. (conversion factored)
     * If pid is enabled, set postion, set velocity, etc.
     */
    public void set(double set){

        if(pidDisabled){
            sparkMax.set(set);
        }else if(ControlType.kCurrent == controlType || ControlType.kDutyCycle == controlType || ControlType.kVoltage == controlType){
            closedLoopController.setReference(set, controlType);
        }else{
            closedLoopController.setReference(set/conversionFactor, controlType);
        }

    }

    /**
     * Disable motor
     */
    public void disable(){
        sparkMax.disable();        
    }

    /**
     * Get current postion (multiplied by converstion factor)
     * If no encoder configured, function will return 0
     */
    public double getPos(){
        if(e_encoderAvailable){
            if(e_absalute){
                return absEncoder.getPosition()*conversionFactor;
            }else{
                return relEncoder.getPosition()*conversionFactor;
            }

        }else{
            return 0;
        }
        
    }

    /**
     * Get current Velocity (multiplied by conversion factor)
     */
    public double getVel(){
        if(e_encoderAvailable){
            if(e_absalute){
                return absEncoder.getVelocity()*conversionFactor;
            }else{
                return relEncoder.getVelocity()*conversionFactor;
            }

        }else{
            return 0;
        }
    }

    ///
    /**
     * Get current Acceleration (multiplied by conversion factor) (UNDER DEVELOPMENT) RETURNS ONLY ZERO
     */
    public double getAcc(){
        return 0;
    }
    ///
    /**
     * Get motor Temprature
     */
    public double getTemp(){
        return sparkMax.getMotorTemperature();
    }

    /** 
     * Get applied motor Current
     */
    public double getCurrent(){
        return sparkMax.getOutputCurrent();
    }

    /**
     * Get current bus velocity
     */
    public double getVol(){
        return sparkMax.getBusVoltage();
    }

    /**
     * Get motor can id
     */
    public double getMotorID(){
        return motorID;
    }
    

    /**
     * Tune pid values, (if p equals to zero - pid disabled) Run pidSetup First!!
     * @param p - proportional
     * @param i - integral
     * @param d - derivitive
     * @param ff - velocity feedfarward
     */
    public void pidTune(double p, double i, double d, double ff){

        if (pidREADY) {
            
            if(p != 0){
                closedLoopCfg.p(p);
                pidDisabled = false;
                closedLoopCfg.i(i);
                closedLoopCfg.d(d);
                closedLoopCfg.velocityFF(ff);
                config.apply(closedLoopCfg);
                ControllerUpdate();
            }else{
                pidDisabled = true;
            }
        }
        //ERROR IF PID SETUP WASN'T USED BEFORE
    }

    /**
     * PID setup, required to run before in implementing pid in code.
     * @param min - min pid output
     * @param max - max pid output
     * @param izone - integral zone
     * @param imax - integral max
     * @param primaryEnc - true for use of primary encoder for internal pid control
     * @param controlType - 0:position, 1:velocity, 2:kmaxPosition, 3:kmaxVelocity, 4:Voltage, 5:Current, 6:kDuty
     */
    public void pidSetup(double min, double max, double izone, double imax, boolean primaryEnc, int controlType){    
        if(pidDisabled)
        {

        }else{

        
            closedLoopCfg.outputRange(min, imax);
            closedLoopCfg.iZone(izone);
            closedLoopCfg.iMaxAccum(imax);
            closedLoopCfg.feedbackSensor(primaryEnc ? FeedbackSensor.kPrimaryEncoder : FeedbackSensor.kAlternateOrExternalEncoder);
            config.apply(closedLoopCfg);
            pidREADY = true;

            switch (controlType) {
                case 0:

                    this.controlType = ControlType.kPosition;
                    break;
                    
                case 1:
                    this.controlType = ControlType.kVelocity;
                    break;

                case 2:
                    this.controlType = ControlType.kMAXMotionPositionControl;
                    break;

                case 3:
                    this.controlType = ControlType.kMAXMotionVelocityControl;
                    break;

                case 4:
                    this.controlType = ControlType.kVoltage;
                    break;

                case 5:
                    this.controlType = ControlType.kCurrent;
                    break;

                case 6:
                    this.controlType = ControlType.kDutyCycle;
                    break;                          
            
                default:
                    break;
            }
            ControllerUpdate();
            //ADD MAX ALLOUD ERROR
            //ADD ERROR FOR INVALLID CONTROL TYPE NUM
        }

    }

    /**
     * Setup motion profile Run pidSetup First!!
     * @param maxVel - max velocity
     * @param maxAcc - max acceleration
     */
    public void motionProfile(double maxVel, double maxAcc){
        motionConfig.maxAcceleration(maxAcc);
        motionConfig.maxVelocity(maxVel);
        closedLoopCfg.apply(motionConfig);
        config.apply(closedLoopCfg);
        ControllerUpdate();
        //ERROR IF USED WITHOUT PID SETUP FIRST
    }

    /**
     * Setup followers
     * @param leaderID - id of motor to FOLLOW
     * @param invert - invert relative to the LEADER
     */
    public void followeMe(int leaderID, boolean invert){
        config.follow(leaderID, invert);
        ControllerUpdate();
        //MAKE IT AUTOMATICCALY DONE
    }

    /**
     * Setup encoder parameters
     * @param countsPerRev - number ot encoder counts per one encoder revolution (IGNORED WITH ABSALUTE ENCODERS)
     * @param setPos - new encoder pos, zero or anything else
     */
    public void encocderCfg(int countsPerRev, double setPos){
        //check which encoder params to configure
        if(e_absalute){
            absEncoderConfig.zeroOffset(setPos/conversionFactor);
        }else{
            relEncoder.setPosition(setPos/conversionFactor);
            encoderConfig.countsPerRevolution(countsPerRev);
        }
        
    }

    /**
     * Advanced controller configuration
     * @param voltageComp - compensate voltage when driving motor
     * @param currentStallLim - limit current at the stall (if disabled, currentFreeLim is also disabled)
     * @param currentFreeLim - limit current at motor max speed
     * @param converstionFactor - factor to convert convertion roations into degrees or radians (get postion, and set, etc) (0 is defult 1)
     */
    public void advanceControl(double voltageComp, int currentStallLim, int currentFreeLim, double conversionFactor){
 
        config.voltageCompensation(voltageComp);
        config.smartCurrentLimit(currentStallLim, currentFreeLim);
        config.smartCurrentLimit(currentStallLim);

        if(conversionFactor != 0){
            super.conversionFactor = conversionFactor;
        }else{
            super.conversionFactor = 1;
        }
        
        ControllerUpdate();
        //NO RPM LIMIT IMPLEMENTED

    }

    //Updates the controller with new config params
    private void ControllerUpdate(){
        sparkMax.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters); 
    }

}