package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
import frc.utils.FLYTLib.FLYTMotorLib.SparkMaxController;

public class Elevator extends SubsystemBase{

    //motor group
    FlytMotorController motor1; 
    FlytMotorController motor2;



    public Elevator(){
        motor1 = new SparkMaxController(getName(), Constants.Lift.motorId, true, true, false); //motor construct
        motor1.advanceControl(0,0,0,0);//setup advace control
        motor1.pidSetup(-1, 1, 0, 0, true, 0); //setup pid
        motor1.motionProfile(0, 0); //create motion profile
        motor1.pidTune(0, 0, 0, 0); //tune pid
        motor2 = new SparkMaxController(getName(), Constants.Lift.motorId2, true, true, false); //creat second motor
        motor2.followeMe(Constants.Lift.motorId, true); //make it follow motor 1
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
        motor1.updateLogger();
        
    }
    
}
