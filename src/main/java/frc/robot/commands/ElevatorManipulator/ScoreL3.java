package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

public class ScoreL3 extends Command {

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diffArm;
    private ElevatorSubsystem elevator;
    //private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    //private int step = 0;
    private boolean atPosition = false;
    private boolean isInterpolated = false;
    private boolean isCoralScored = false;


    public ScoreL3(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator){

        manipulatorSm = m_state;
        diffArm = m_diff;
        elevator = m_elevator;
        //manipulator = m_manipulator;

        addRequirements(m_diff, elevator);
    }

    @Override
    public void initialize() {

        diffArm.setExtensionSetpoint(DifferentialArm.l2_3Ext);
        elevator.setElevatorSetpoint(Elevator.L3);
        diffArm.setRotationSetpoint(DifferentialArm.l2_3Rot);
    }

    @Override
    public void execute(){

        //place where interpolation code would be imported
        //interpolation();
        //this commadn only ends if robot interpolated and scored the coral
        if(diffArm.atRotationSetpoint() && diffArm.atExtenstionSetpoint() && elevator.atPosition()){
            atPosition = true;
            manipulatorSm.setatGoalState(true);
        }else{
            atPosition = false;
            manipulatorSm.setatGoalState(false);
        }

            
        

        //safety code if it takes too long extend out out
    }

    private void interpolation(){

        if (diffArm.hasLaserCanDistance() && manipulatorSm.getInterpolateActive()) {
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
        return atPosition && !manipulatorSm.getHasCoral();
    }

}