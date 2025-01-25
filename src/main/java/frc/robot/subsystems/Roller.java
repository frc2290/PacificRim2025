package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors.RollerM;
import frc.robot.FLYTLib.FLYTDashboard.MotorDashboard;
import frc.robot.FLYTLib.FLYTMotorLib.SparkController;
import frc.robot.FLYTLib.FLYTMotorLib.SuperController;

public class Roller extends SubsystemBase {

    SuperController roller;
    MotorDashboard dashboard;

    //Roller for Kitbot using sparkmax brushless motor controller
    public Roller() {
        roller = new SparkController(RollerM.m_id, RollerM.m_brushless, RollerM.m_break_mode);
        roller.advanceControl(RollerM.m_voltage_compensation, RollerM.m_motor_current, 0, 0);
        roller.encocderCfg(100,0);
        dashboard = new MotorDashboard(roller);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    private Command exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }
  
    /**
     * Run the roller at a given speed
     */
    public void runRoller(double speed) {
        roller.set(speed);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
    
}


