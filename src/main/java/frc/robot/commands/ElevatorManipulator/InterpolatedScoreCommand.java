package frc.robot.commands.ElevatorManipulator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/**
 * Base command for scoring positions that rely on the LaserCan distance sensor
 * to interpolate differential arm setpoints. Each concrete implementation
 * supplies the desired elevator height, default arm positions, and the
 * interpolation callbacks for its scoring level.
 */
public abstract class InterpolatedScoreCommand extends Command {

    private final ManipulatorStateMachine manipulatorStateMachine;
    private final DifferentialSubsystem differentialSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final double elevatorSetpoint;
    private final double defaultExtension;
    private final double defaultRotation;
    private final DoubleSupplier extensionInterpolation;
    private final DoubleSupplier rotationInterpolation;

    private boolean atPosition = false;
    private boolean usingInterpolation = false;

    protected InterpolatedScoreCommand(
            ManipulatorStateMachine manipulatorStateMachine,
            DifferentialSubsystem differentialSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            double elevatorSetpoint,
            double defaultExtension,
            double defaultRotation,
            DoubleSupplier extensionInterpolation,
            DoubleSupplier rotationInterpolation) {
        this.manipulatorStateMachine = manipulatorStateMachine;
        this.differentialSubsystem = differentialSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSetpoint = elevatorSetpoint;
        this.defaultExtension = defaultExtension;
        this.defaultRotation = defaultRotation;
        this.extensionInterpolation = extensionInterpolation;
        this.rotationInterpolation = rotationInterpolation;

        addRequirements(differentialSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        atPosition = false;
        usingInterpolation = false;
        manipulatorStateMachine.atGoalState(false);

        elevatorSubsystem.setElevatorSetpoint(elevatorSetpoint);
        differentialSubsystem.setExtensionSetpoint(defaultExtension);
        differentialSubsystem.setRotationSetpoint(defaultRotation);
    }

    @Override
    public void execute() {
        double extensionTarget = defaultExtension;
        double rotationTarget = defaultRotation;

        if (differentialSubsystem.hasLaserCanDistance()) {
            extensionTarget = extensionInterpolation.getAsDouble();
            rotationTarget = rotationInterpolation.getAsDouble();
            usingInterpolation = true;
        } else {
            usingInterpolation = false;
        }

        differentialSubsystem.setExtensionSetpoint(extensionTarget);
        differentialSubsystem.setRotationSetpoint(rotationTarget);

        if (differentialSubsystem.atRotationSetpoint()
                && differentialSubsystem.atExtenstionSetpoint()
                && elevatorSubsystem.atPosition()) {
            atPosition = true;
            manipulatorStateMachine.atGoalState(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!differentialSubsystem.hasLaserCanDistance()) {
            differentialSubsystem.setExtensionSetpoint(defaultExtension);
            differentialSubsystem.setRotationSetpoint(defaultRotation);
        }
        manipulatorStateMachine.atGoalState(false);
    }

    @Override
    public boolean isFinished() {
        if (!differentialSubsystem.hasLaserCanDistance() && manipulatorStateMachine.getHasCoral()) {
            return true;
        }
        return atPosition && !manipulatorStateMachine.getHasCoral();
    }

    /**
     * Indicates whether the command is currently using interpolated setpoints or
     * the static fallbacks.
     *
     * @return {@code true} when a valid LaserCan reading is being used for
     *         interpolation.
     */
    protected boolean isUsingInterpolation() {
        return usingInterpolation;
    }
}

