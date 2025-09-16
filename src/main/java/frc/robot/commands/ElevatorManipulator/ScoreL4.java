package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

public class ScoreL4 extends Command {

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diffArm;
    private ElevatorSubsystem elevator;
    //private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    //private int step = 0;
    private boolean atPosition = false;
    private boolean isInterpolated = false;
    private boolean isCoralScored = false;

    private double elevatorPos = 1.72;
    private double diffExt = 140;
    private double diffRot = 235;
    private double diffExt1 = 220;

    public ScoreL4(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator){

        manipulatorSm = m_state;
        diffArm = m_diff;
        elevator = m_elevator;
        //manipulator = m_manipulator;

        addRequirements(m_diff, elevator);
    }

    @Override
    public void initialize() {

        diffArm.setExtensionSetpoint(diffExt);
        elevator.setElevatorSetpoint(elevatorPos);
        diffArm.setRotationSetpoint(diffRot);
    }

    @Override
    public void execute(){

        //place where interpolation code would be imported
        //interpolation();
        //this commadn only ends if robot interpolated and scored the coral
        if(diffArm.atRotationSetpoint() && diffArm.atExtenstionSetpoint() && elevator.atPosition()){
            atPosition = true;
            manipulatorSm.atGoalState(true);
        }
            
        

        //safety code if it takes too long extend out out
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
        manipulatorSm.atGoalState(false);
        
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return atPosition && !manipulatorSm.getHasCoral();
    }

}

