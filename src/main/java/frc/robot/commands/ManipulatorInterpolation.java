package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;


public class ManipulatorInterpolation extends Command {

    private DifferentialSubsystem diffArm;

    //does not have any requirmennts, use carfuly inside other commands
    public ManipulatorInterpolation(DifferentialSubsystem diffArm) {
        this.diffArm = diffArm;

    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute(){

        interpolation();
    }

    private void interpolation(){


        /** Diff Arm Interpolation */
        if (diffArm.hasLaserCanDistance()) {
                diffArm.setExtensionSetpoint(diffArm.l4ExtensionInterpolate());
                diffArm.setRotationSetpoint(diffArm.l4RotationInterpolate());
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
    

