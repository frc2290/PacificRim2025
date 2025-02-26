package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
import frc.utils.FLYTLib.FLYTMotorLib.SparkMaxController;

public class Endeffector extends SubsystemBase {

    //
    FlytMotorController motor;


    public Endeffector (){

        motor = new SparkMaxController(getName(), 0, false, false, false, true);
        //motor.advanceControl(0, 0, 0, 0);

    }


    
    public void Intake(double power){
        motor.setPower(power);
    }

    public double getWristPos(){
        return motor.getPos();
    }



    @Override
    public void periodic() {
        motor.updateLogger(Constants.debugMode);
    }
    
}
