package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

public class PrepCoralIntake extends Command {

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diffArm;
    private ElevatorSubsystem elevator;
    //private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    //private int step = 0;
    private boolean atPosition = false;

    public PrepCoralIntake(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator){

        manipulatorSm = m_state;
        diffArm = m_diff;
        elevator = m_elevator;
        //manipulator = m_manipulator;

        addRequirements(m_diff, elevator);

    }

    @Override
    public void initialize() {
        diffArm.setExtensionSetpoint(80);
        diffArm.setRotationSetpoint(225);
        elevator.setElevatorSetpoint(Elevator.intakeSetpoint);
    }

    @Override
    public void execute(){

        if(diffArm.atExtenstionSetpoint() && diffArm.atRotationSetpoint() && elevator.atPosition()){
            atPosition = true;
        }

        //safety code if it takes too long extend out out
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return atPosition;
    }

    
}
