package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Moves the manipulator near the intake pose while leaving room to confirm coral presence. */
public class PrepCoralIntake extends Command {

    private ManipulatorStateMachine manipulatorSm;
    private DifferentialSubsystem diffArm;
    private ElevatorSubsystem elevator;
    //private ManipulatorSubsystem manipulator;
    //private final Timer stepTimer = new Timer();
    //private int step = 0;
    private boolean atPosition = false;

    /**
     * Creates a command that stages the manipulator just outside of the full intake pose so the
     * sensors can confirm a coral is present before moving fully in.
     */
    public PrepCoralIntake(ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator){

        manipulatorSm = m_state;
        diffArm = m_diff;
        elevator = m_elevator;
        //manipulator = m_manipulator;

        addRequirements(m_diff, elevator);

    }

    @Override
    public void initialize() {
        // Extend the arm farther out so the manipulator is ready to grab an incoming coral.
        diffArm.setExtensionSetpoint(230);
        diffArm.setRotationSetpoint(DifferentialArm.intakeRotationSetpoint);
        elevator.setElevatorSetpoint(Elevator.intakeSetpoint);
    }

    @Override
    public void execute(){

        // Track whether the arm and elevator have reached their commands yet.
        if(diffArm.atExtenstionSetpoint() && diffArm.atRotationSetpoint()){
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
        // Once we're staged, the state machine can advance to the intake command.
        return atPosition;
    }


}
