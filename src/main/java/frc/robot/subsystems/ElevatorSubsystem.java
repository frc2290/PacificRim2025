package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
import frc.utils.FLYTLib.FLYTMotorLib.SparkFlexController;

public class ElevatorSubsystem extends SubsystemBase{

    //motor group
    FlytMotorController motor1; 
    FlytMotorController motor2;

    public ElevatorSubsystem() {
        motor1 = new SparkFlexController(getName(), Elevator.kLeftElevatorMotorId, true, true, true); //motor construct
        //motor1.advanceControl(0,0,0,0);//setup advace control
        //motor1.pidSetup(-1, 1, 0, 0, true, 0); //setup pid
        //motor1.motionProfile(0, 0); //create motion profile
        //motor1.pidTune(0, 0, 0, 0); //tune pid
        motor2 = new SparkFlexController(getName(), Elevator.kRightElevatorMotorId, true, true, false); //creat second motor
        motor2.followeMe(Elevator.kLeftElevatorMotorId, true); //make it follow motor 1
    }

    /**
     * Set lift position
     * @param setpoint
     */
    public void set(double setpoint){
        motor1.set(setpoint);
    }

    public void setPower(double power) {
        motor1.setPower(power);
    }

    @Override
    public void periodic() {
        motor1.updateLogger(Constants.debugMode);
        motor2.updateLogger(Constants.debugMode);
    }
}
