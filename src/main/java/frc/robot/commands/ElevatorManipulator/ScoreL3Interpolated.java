package frc.robot.commands.ElevatorManipulator;

import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Score command for the L3 reef level using interpolated arm setpoints. */
public class ScoreL3Interpolated extends InterpolatedScoreCommand {

    private static final double ELEVATOR_SETPOINT = 1.14;
    private static final double DEFAULT_EXTENSION = 170;
    private static final double DEFAULT_ROTATION = 230;

    public ScoreL3Interpolated(
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
                differentialSubsystem::l2_3ExtensionInterpolate,
                differentialSubsystem::l2_3RotationInterpolate);
    }
}
