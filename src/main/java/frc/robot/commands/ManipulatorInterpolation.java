package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;


public class ManipulatorInterpolation extends Command {

    private DifferentialSubsystem diffArm;
    private ManipulatorStateMachine manipulatorSM;

    //does not have any requirmennts, use carfuly inside other commands
    public ManipulatorInterpolation(DifferentialSubsystem diffArm, ManipulatorStateMachine manipulatorSM) {
        this.diffArm = diffArm;
        this.manipulatorSM = manipulatorSM;
    
        addRequirements(diffArm);
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

        
        if(diffArm.atExtenstionSetpoint()  && diffArm.atRotationSetpoint()){
            manipulatorSM.setreadyToScore(true);
        } else {
            manipulatorSM.setreadyToScore(false);
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
    

