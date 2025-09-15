package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

public class StartPosition extends Command{

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diff;
    private ElevatorSubsystem elevator;
    private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    private int step = 0;

    public StartPosition(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, ManipulatorSubsystem m_manipulator){
        manipulatorSm = m_state;
        diff = m_diff;
        elevator = m_elevator;
        manipulator = m_manipulator;

        addRequirements(m_diff, elevator, manipulator);
    }   
    
    @Override
    public void initialize() {}

    @Override
    public void execute(){}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
