package frc.robot.commands.ElevatorManipulator;

import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Score command for the L4 reef level using interpolated arm setpoints. */
public class ScoreL4Interpolated extends InterpolatedScoreCommand {

    private static final double ELEVATOR_SETPOINT = 1.72;
    private static final double DEFAULT_EXTENSION = 140;
    private static final double DEFAULT_ROTATION = 235;

    public ScoreL4Interpolated(
            ManipulatorStateMachine manipulatorStateMachine,
            DifferentialSubsystem differentialSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        super(
                manipulatorStateMachine,
                differentialSubsystem,
                elevatorSubsystem,
                ELEVATOR_SETPOINT,
                DEFAULT_EXTENSION,
                DEFAULT_ROTATION,
                differentialSubsystem::l4ExtensionInterpolate,
                differentialSubsystem::l4RotationInterpolate);
    }
}
