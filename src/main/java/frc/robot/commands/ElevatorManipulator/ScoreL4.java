package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Score command for the L4 reef level using preset arm setpoints. */
public class ScoreL4 extends Command {

    private static final double ELEVATOR_SETPOINT = 1.72;
    private static final double EXTENSION_SETPOINT = 140;
    private static final double ROTATION_SETPOINT = 235;

    private final ManipulatorStateMachine manipulatorStateMachine;
    private final DifferentialSubsystem differentialSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean atPosition = false;

    public ScoreL4(
            ManipulatorStateMachine manipulatorStateMachine,
            DifferentialSubsystem differentialSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        this.manipulatorStateMachine = manipulatorStateMachine;
        this.differentialSubsystem = differentialSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(differentialSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        atPosition = false;
        manipulatorStateMachine.atGoalState(false);

        elevatorSubsystem.setElevatorSetpoint(ELEVATOR_SETPOINT);
        differentialSubsystem.setExtensionSetpoint(EXTENSION_SETPOINT);
        differentialSubsystem.setRotationSetpoint(ROTATION_SETPOINT);
    }

    @Override
    public void execute() {
        if (differentialSubsystem.atRotationSetpoint()
                && differentialSubsystem.atExtenstionSetpoint()
                && elevatorSubsystem.atPosition()) {
            atPosition = true;
            manipulatorStateMachine.atGoalState(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorStateMachine.atGoalState(false);
    }

    @Override
    public boolean isFinished() {
        if (differentialSubsystem.hasLaserCanDistance() && manipulatorStateMachine.getHasCoral()) {
            return true;
        }
        return atPosition && !manipulatorStateMachine.getHasCoral();
    }
}

