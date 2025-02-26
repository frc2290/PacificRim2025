package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
import frc.utils.FLYTLib.FLYTMotorLib.SparkMaxController;

public class DifferentialArmSubsystem extends SubsystemBase {

    //two motors
    //each controlled by trapesoidal control postion loop cascacaded with internal velocity loop
    // yea a lot

    private FlytMotorController motor1; //flyt motor
    private FlytMotorController motor2; //flyt motor

    private Endeffector endeffector;

    private PIDController pid_extension; //position loop extension
    private PIDController pid_rotation; //position loop rotation

    private double setExtension; //set extention position
    private double setRotation; //set rotation position

    private double extensionPos;
    private double rotationPos;



   public DifferentialArmSubsystem(Endeffector m_endeffector){
        endeffector = m_endeffector;
        motor1 = new SparkMaxController(getName(), 0, true, true, false);
        motor2 = new SparkMaxController(getName(), 0, true, true, false);
        motor1.advanceControl(Constants.DifferentialArm.voltageComp, Constants.DifferentialArm.currentStallLim, Constants.DifferentialArm.currentFreeLim, 0);
        //motor2.advanceControl(Constants.DifferentialArm.voltageComp, Constants.DifferentialArm.currentStallLim, Constants.DifferentialArm.currentFreeLim, 0);
        //motor1.pidSetup(-1, 1, 0, 1, true, 1); //setup p
        //motor2.pidSetup(-1, 1, 0, 1, true, 1); //setup pid
        //motor1.pidTune(Constants.DifferentialArm.v_kp, Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd, Constants.DifferentialArm.v_ff);
        //motor2.pidTune(Constants.DifferentialArm.v_kp, Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd, Constants.DifferentialArm.v_ff);

        //setup position loop ADD FEEDFARWARD
        //pid_extension = new PIDController(Constants.DifferentialArm.e_kp, Constants.DifferentialArm.e_ki, Constants.DifferentialArm.e_kp);
        //pid_rotation = new PIDController(Constants.DifferentialArm.r_kp, Constants.DifferentialArm.r_ki, Constants.DifferentialArm.r_kp);

    }

    public void Extend(double setpoint){
        motor2.setPower(setpoint);
        motor1.setPower(setpoint);
        //setExtension = setpoint

    }
    
    public void Rotate(double setpoint){
        motor2.setPower(-setpoint);
        motor1.setPower(setpoint);
        //setRotation = setpoint;
    }


    private void setPosition(double setPoint, double setOmega){
        motor1.set(setPoint-setOmega);
        motor2.set(setPoint+setOmega);
    }
    
    public double getExtensionPosition(){
        return (motor1.getPos()+motor2.getPos())/2;
    }

    public double getRotationPosition(){
        return endeffector.getWristPos();
    }

    
    @Override
    public void periodic() {
        //logging stuff
        motor1.updateLogger(Constants.debugMode);
        motor2.updateLogger(Constants.debugMode);

        //Position Extension PID Loop
        //setPosition(pid_extension.calculate(getExtensionPosition(), setExtension),pid_rotation.calculate(getRotationPosition(), setRotation));
        //GET ENCODER DISTANCE HAS TO BE FIGURED OUT LATER
    }
}
