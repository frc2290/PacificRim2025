package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

public class IntakeCoral extends Command{

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diffArm;
    private ElevatorSubsystem elevator;
    //private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    //private int step = 0;
    private boolean atPosition = false;
    private boolean hasCoral = false;

    public IntakeCoral(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator){

        manipulatorSm = m_state;
        diffArm = m_diff;
        elevator = m_elevator;
        //manipulator = m_manipulator;

        addRequirements(m_diff, elevator);

    }

    @Override
    public void initialize() {
        diffArm.setExtensionSetpoint(DifferentialArm.intakeExtensionSetpoint);
        diffArm.setRotationSetpoint(DifferentialArm.intakeRotationSetpoint);
    }

    @Override
    public void execute(){

        if(diffArm.atExtenstionSetpoint() && diffArm.atRotationSetpoint()){
            atPosition = true;
        }
            
        hasCoral = manipulatorSm.getHasCoral();

        //safety code if it takes too long extend out out
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return atPosition && hasCoral;
    }
}
