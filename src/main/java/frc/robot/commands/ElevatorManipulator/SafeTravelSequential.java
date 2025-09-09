package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SafeTravelSequential extends Command {
    private final DifferentialSubsystem diffArm;
    private final ElevatorSubsystem elevator;
    private int step = 0;
    private final Timer stepTimer = new Timer();
    
    public SafeTravelSequential(DifferentialSubsystem diffArm, ElevatorSubsystem elevator) {
        this.diffArm = diffArm;
        this.elevator = elevator;
        addRequirements(diffArm, elevator);
    }
    
    @Override
    public void initialize() {
        step = 0;
        stepTimer.restart();
    }
    
    @Override
    public void execute() {
        switch (step) {
            case 0: // Retract extension first
                diffArm.setExtensionSetpoint(220);
                if (diffArm.atExtenstionSetpoint() || stepTimer.hasElapsed(2.0)) {
                    step = 1;
                    stepTimer.restart();
                }
                break;
                
            case 1: // Lower elevator
                elevator.setElevatorSetpoint(Elevator.transportSetpoint);
                if (elevator.atPosition() || stepTimer.hasElapsed(3.0)) {
                    step = 2;
                    stepTimer.restart();
                }
                break;
                
            case 2: // Set final rotation
                diffArm.setRotationSetpoint(DifferentialArm.transportRotationSetpoint);
                if (diffArm.atRotationSetpoint() || stepTimer.hasElapsed(2.0)) {
                    step = 3;
                    stepTimer.restart();
                }
                break;
                
            case 3: // Set final extension
                diffArm.setExtensionSetpoint(DifferentialArm.transportExtensionSetpoint);
                if (diffArm.atExtenstionSetpoint() || stepTimer.hasElapsed(2.0)) {
                    step = 4;
                }
                break;
        }
    }
    
    @Override
    public boolean isFinished() {
        return step == 4;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Cleanup if needed
    }
}