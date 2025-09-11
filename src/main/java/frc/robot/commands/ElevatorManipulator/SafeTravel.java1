package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateMachine;

public class SafeTravel extends Command {


    private double diffExt1 = 220;
    DifferentialSubsystem diffArm;
    ElevatorSubsystem elevator;
    StateMachine stateMachine;
    Integer step = 0;

    public SafeTravel(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateMachine stateMachine) {
        this.diffArm = diffArm;
        this.elevator = elevator;
        this.stateMachine = stateMachine;

        addRequirements(diffArm, elevator);
    }

    //     //     Command moveAndRot = diffArm.setRotAndExtSetpointCommand(DifferentialArm.transportExtensionSetpoint, DifferentialArm.transportRotationSetpoint);
    //     //     Command moveElev = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
    //     //     result = new SequentialCommandGroup(moveAndRot, moveElev);
    //     // }

    

       @Override
    public void initialize() {
        // // Kick off your motions once
        // diffArm.setExtensionSetpoint(diffExt1);
        // elevator.setElevatorSetpoint(Elevator.transportSetpoint);
    }

    @Override
    public void execute() {

        switch (step){
            case 0:
                diffArm.setExtensionSetpointCommand(diffExt1);
                if(diffArm.atExtenstionSetpoint()){
                    step = 1;
                }
                break;
            case 1:
                elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
                if(elevator.atPosition()){
                    step = 2;
                }
                break;
            case 2:
                diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
                if(diffArm.atRotationSetpoint()){
                    step = 3;
                }
                break;
            case 3:
                diffArm.setExtensionSetpointCommand(DifferentialArm.transportExtensionSetpoint);
                if(diffArm.atRotationSetpoint()){
                    step = 4;
                }
                break;
            default:
                step = 0;
        }

    }



    @Override
    public void end(boolean interrupted) {
        // if (!interrupted) {
        //     stateMachine.setCurrentState(PositionState.TravelPosition);
        // }
    }

    @Override
    public boolean isFinished() {
        // Define your own stopping condition
        return step == 4;
    }

}



