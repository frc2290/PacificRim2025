package frc.robot.FLYTLib.FLYTMotorLib;

import frc.robot.FLYTLib.FLYTDashboard.FlytLogger;

public abstract class FlytMotorController {

    protected double conversionFactor = 1;
    protected FlytLogger controllerLogger;
    protected String controllerName;
    //pid stuff
    protected double p, i, d, ff;
    protected int controlType;
    
    public FlytMotorController(String m_controllerName) {
        controllerName = m_controllerName;//why?
        controllerLogger = new FlytLogger(controllerName);
        controllerLogger.addDoublePublisher("MotorID", () -> getMotorID());
        controllerLogger.addDoublePublisher("MotorPos", () -> getPos());
        controllerLogger.addDoublePublisher("P", () -> getP());

    }

    /**
     * Set relative speed, multiplies factor by availble voltage. (conversion factored)
     * If pid is enabled, set postion, set velocity, etc.
     */
    public abstract void set(double set);


    /**
     * Disable motor
     */
    public abstract void disable();

    /**
     * Get current postion (multiplied by converstion factor)
     * If no encoder configured, function will return 0
     */
    public abstract double getPos();

    /**
     * Get current Velocity (multiplied by conversion factor)
     */
    public abstract double getVel();


    public abstract double getAcc();

    /**
     * Get motor Temprature
     */
    public abstract double getTemp();

    /** 
     * Get applied motor Current
     */
    public abstract double getCurrent();

    /**
     * Get current bus velocity
     */
    public abstract double getVol();

    /**
     * Get motor can id
     */
    public abstract double getMotorID();
    


    /**
     * Tune pid values, (if p equals to zero - pid disabled) Run pidSetup First!!
     * @param p - proportional
     * @param i - integral
     * @param d - derivitive
     * @param ff - velocity feedfarward
     */
    public void pidTune(double m_p, double m_i, double m_d, double m_ff) {
        p = m_p;
        i = m_i;
        d = m_d;
        ff = m_ff;
    }

    public double getP() {
        return p;
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
    public abstract void pidSetup(double min, double max, double izone, double imax, boolean primaryEnc, int controlType);

    /**
     * Setup motion profile Run pidSetup First!!
     * @param maxVel - max velocity
     * @param maxAcc - max acceleration
     */
    public abstract void motionProfile(double maxVel, double maxAcc);

    /**
     * Setup followers
     * @param leaderID - id of motor to FOLLOW
     * @param invert - invert relative to the LEADER
     */
    public abstract void followeMe(int leaderID, boolean invert);

    /**
     * Setup encoder parameters
     * @param countsPerRev - number ot encoder counts per one encoder revolution (IGNORED WITH ABSALUTE ENCODERS)
     * @param setPos - new encoder pos, zero or anything else
     */
    public abstract void encocderCfg(int countsPerRev, double setPos);

    /**
     * Advanced controller configuration
     * @param voltageComp - compensate voltage when driving motor
     * @param currentStallLim - limit current at the stall (if disabled, currentFreeLim is also disabled)
     * @param currentFreeLim - limit current at motor max speed
     * @param converstionFactor - factor to convert convertion roations into degrees or radians (get postion, and set, etc) (0 is defult 1)
     */
    public abstract void advanceControl(double voltageComp, int currentStallLim, int currentFreeLim, double conversionFactor);

    /**
     * Method to return the friendly name of the motor. Name is given through constructor.
     * 
     * @return Friendly name of the motor controller.
     */
    public String getName() {
        return controllerName;
    }

    /**
     * Method to be called in periodic of Subsystem the Controller is used in.
     * This updates the dashboard values.
     */
    public void updateLogger() {
        controllerLogger.update();
    }

    public double getDouble(String doubleName) {
        return controllerLogger.getDouble(doubleName);
    }
}
