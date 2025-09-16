package frc.robot.commands.ElevatorManipulator;

import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Score command for the L2 reef level using interpolated arm setpoints. */
public class ScoreL2Interpolated extends InterpolatedScoreCommand {

    private static final double ELEVATOR_SETPOINT = 0.74;
    private static final double DEFAULT_EXTENSION = 170;
    private static final double DEFAULT_ROTATION = 230;

    public ScoreL2Interpolated(
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
