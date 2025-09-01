package frc.robot.subsystems;

import java.lang.Thread.State;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.GraphCommand;
import frc.utils.GraphCommand.GraphCommandNode;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;


public class StateMachine extends SubsystemBase {

    //Place to import csubsystems and commmands
    private GraphCommand m_graphCommand = new GraphCommand();
    private ElevatorSubsystem elevator;
    private DifferentialSubsystem diff;
    private DriveSubsystem drive;
    private ManipulatorSubsystem manipulator;
    private PoseEstimatorSubsystem pose;
    private LEDUtility ledUtility;
    private XboxController driverController;


    /**
     * DriveTrain states - drive state machine
     */
    public enum DriveState {
        Teleop,            // Field oriented freerome
        FollowPath,        // Auto path following
        BargeRelative,     //FacesBarge
        ClimbRelative,     //FacesClimb
        ProcessorRelative, //Faces Processor
        CoralStation,    //FacesIntake based on half field
        ReefRelative,      //FacesReef based on robot position and angles as drives around
        ReefAlign,       //Locked to right or left reef, holding position
        Cancelled
    }

    /**
     * Arm and Elevator states part of other state machine
     */
    public enum ElevatorManipulatorState {
        StartPosition,     //Initial robot poisiton when turned on
        SafeCoralTravel,   //Safe travel with coral position
        IntakeCoral,
        L1,
        L2,
        L3,
        L4,
        AlgaeL2,
        AlgaeL3,
        Processor,
        Barge,
        Climb,
        Manual,
        Reset,
    }
        
    /**
     * Endeffector
     */
    public enum EndEffector{
        HasCoral,          //Robot has coral
        HasAlgae,          //Robot has Algae
        IntakeAlgae,       //Intake Algae
        IntakeCoral,       //Intake Coral
        ScoreCoral,        //Score Coral
        ScoreAlgae         //Score Algae
    }


    /*
     * Graph Command Nodes
     **/
    GraphCommandNode startPosition = m_graphCommand.new GraphCommandNode(
        "StartPosition", 
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));
    
    GraphCommandNode preCoralIntake = m_graphCommand.new GraphCommandNode(
        "PreCoralIntake", 
        new PrintCommand("SendCommand to move into this position, for elevator then when safe move the extension out and rotatation"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    //Set in the main statemachine
    GraphCommandNode intakeCoral = m_graphCommand.new GraphCommandNode(
        "IntakeCoral",
        new PrintCommand("Turn on the intake, wait until note is detected"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode safeCoralTravel = m_graphCommand.new GraphCommandNode(
        "SafeCoralTravel",
        new PrintCommand("Move into safe coral travel position, by this point all of the running commands have finished and robot is wainting for driver commands"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode l1Prep = m_graphCommand.new GraphCommandNode(
        "L1Prep",
        new PrintCommand("Move elevator to L1 prep position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode l2Prep = m_graphCommand.new GraphCommandNode(
        "L2Prep",
        new PrintCommand("Move elevator to L2 prep position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode l3Prep = m_graphCommand.new GraphCommandNode(
        "L3Prep",
        new PrintCommand("Move elevator to L3 prep position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode l4Prep = m_graphCommand.new GraphCommandNode(
        "L4Prep",
        new PrintCommand("Move elevator to L4 prep position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode scoreL1 = m_graphCommand.new GraphCommandNode(
        "ScoreL1",
        new PrintCommand("Get into final position and wait for the score command from driver"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode scoreL2 = m_graphCommand.new GraphCommandNode(
        "ScoreL2",
        new PrintCommand("Get into final position and wait for the score command from driver"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode scoreL3 = m_graphCommand.new GraphCommandNode(
        "ScoreL3",
        new PrintCommand("Get into final position and wait for the score command from driver"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode scoreL4 = m_graphCommand.new GraphCommandNode(
        "ScoreL4",
        new PrintCommand("Get into final position and wait for the score command from driver"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode l1PostScore = m_graphCommand.new GraphCommandNode(
        "L1PostScore",
        new PrintCommand("Get into safe position before going down to safe travel"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode l2PostScore = m_graphCommand.new GraphCommandNode(
        "L2PostScore",
        new PrintCommand("Get into safe position before going down to safe travel"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode l3PostScore = m_graphCommand.new GraphCommandNode(
        "L3PostScore",
        new PrintCommand("Get into safe position before going down to safe travel"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode l4PostScore = m_graphCommand.new GraphCommandNode( 
        "L4PostScore",
        new PrintCommand("Get into safe position before going down to safe travel"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode prepAlgaeIntake = m_graphCommand.new GraphCommandNode(
        "PrepAlgaeIntake",
        new PrintCommand("Get into position to intake algae"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode prepAlgaeL2 = m_graphCommand.new GraphCommandNode(
        "PrepAlgaeL2",
        new PrintCommand("Final position before intake algaeL2"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode prepAlgaeL3 = m_graphCommand.new GraphCommandNode(
        "PrepAlgaeL3",
        new PrintCommand("Final position before intake algaeL3"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));

    GraphCommandNode safeAlgaeTravel = m_graphCommand.new GraphCommandNode(
        "SafeAlgaeTravel",
        new PrintCommand("Get into safe travel with algae position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode scoreProssesor = m_graphCommand.new GraphCommandNode(
        "ScoreProssesor",
        new PrintCommand("Basicaly same as safe travel with algae postion"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode prepScoreBarge = m_graphCommand.new GraphCommandNode(
        "PrepScoreBarge",
        new PrintCommand("Get into position to score into barge"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode scoreBarge = m_graphCommand.new GraphCommandNode(
        "ScoreBarge",
        new PrintCommand("Final score barge position and wait for driver to hit score"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode climbPrep = m_graphCommand.new GraphCommandNode(
        "ClimbPrep",
        new PrintCommand("Get into position to preapare to engage climber out, after which it is enaged"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode climbReady = m_graphCommand.new GraphCommandNode(
        "ClimbReady",
        new PrintCommand("Once climber out robot is waiting for driver to hit climb command"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   

    GraphCommandNode cancelled = m_graphCommand.new GraphCommandNode(
        "Cancelled",
        new PrintCommand("Should cancel all of the running commands, stop the robot and get everything into safe travel position"),
        new PrintCommand("Nothing"),
        new PrintCommand("Nothing"));   
        

    //General State Variables
    private boolean rotLock = true; //Lock rotation when driving preventing user from overriding
    private boolean rightScore = false; //Selecting Branch for scoring
    private boolean driveTransitioning = false; //If drivetrain statemachine transitioning
    private boolean elevManiTransitioning = false; //If elevatorManipulator is transitioning between states
    private boolean isAuto = false; //If robot is in auto mode
    private boolean isDisabled = false; //If robot is disabled

    private Command currentCommand = null;

    private EndEffector endEffector = EndEffector.HasCoral; //When robot starts, it knows, it has coral

    // State storage and queueing
    // private RobotState prevRobotState = RobotState.StartPosition;
    // private RobotState robotState = RobotState.StartPosition; //current main robot state
    // private RobotState goalRobotState = RobotState.StartPosition; //PositionState.TravelPosition;
    private DriveState prevDriveState = DriveState.Teleop;
    private DriveState driveState = DriveState.Teleop;
    private DriveState goalDriveState = DriveState.Teleop;
    private ElevatorManipulatorState prevElevManiState = ElevatorManipulatorState.StartPosition;
    private ElevatorManipulatorState elevManiState = ElevatorManipulatorState.StartPosition; //When robot is turned on, this isthe starting state
    private ElevatorManipulatorState goalElevManiState = ElevatorManipulatorState.StartPosition;


    /*
     * Creat a new StateSubsystem
     **/
    public StateMachine(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, DriveSubsystem m_drive, ManipulatorSubsystem m_manipulator, PoseEstimatorSubsystem m_pose, LEDUtility m_ledUtility, XboxController m_driverController) {
        diff = m_diff;
        elevator = m_elevator;
        drive = m_drive;
        manipulator = m_manipulator;
        pose = m_pose;
        ledUtility = m_ledUtility;
        driverController = m_driverController;

        //Graph Command setup
        m_graphCommand.setGraphRootNode(startPosition); //rood node
        startPosition.AddNode(safeCoralTravel, 1.0); //connections
        safeCoralTravel.AddNode(preCoralIntake, 1.0);
        preCoralIntake.AddNode(intakeCoral, 1.0);
        intakeCoral.AddNode(safeCoralTravel, 1.0);
        safeCoralTravel.AddNode(l1Prep, 1.0);
        safeCoralTravel.AddNode(l2Prep, 1.0);
        safeCoralTravel.AddNode(l3Prep, 1.0);
        safeCoralTravel.AddNode(l4Prep, 1.0);
        l1Prep.AddNode(l2Prep, 1.0);
        l1Prep.AddNode(l3Prep, 1.0);
        l1Prep.AddNode(l4Prep, 1.0);
        l2Prep.AddNode(l1Prep, 1.0);
        l2Prep.AddNode(l3Prep, 1.0);
        l2Prep.AddNode(l4Prep, 1.0);
        l3Prep.AddNode(l1Prep, 1.0);
        l3Prep.AddNode(l2Prep, 1.0);
        l3Prep.AddNode(l4Prep, 1.0);
        l4Prep.AddNode(l1Prep, 1.0);
        l4Prep.AddNode(l2Prep, 1.0);
        l4Prep.AddNode(l3Prep, 1.0);
        l1Prep.AddNode(scoreL1, 1.0);
        l2Prep.AddNode(scoreL2, 1.0);
        l3Prep.AddNode(scoreL3, 1.0);
        l4Prep.AddNode(scoreL4, 1.0);
        scoreL1.AddNode(l1Prep, 1.0);
        scoreL2.AddNode(l2Prep, 1.0);
        scoreL3.AddNode(l3Prep, 1.0);
        scoreL4.AddNode(l4Prep, 1.0);
        scoreL1.AddNode(l1PostScore, 1.0);
        scoreL2.AddNode(l2PostScore, 1.0);
        scoreL3.AddNode(l3PostScore, 1.0);
        scoreL4.AddNode(l4PostScore, 1.0);
        l1PostScore.AddNode(safeCoralTravel, 1.0);
        l2PostScore.AddNode(safeCoralTravel, 1.0);
        l3PostScore.AddNode(safeCoralTravel, 1.0);
        l4PostScore.AddNode(safeCoralTravel, 1.0);
        safeCoralTravel.AddNode(prepAlgaeIntake, 1.0);
        prepAlgaeIntake.AddNode(safeAlgaeTravel, 1.0);
        prepAlgaeIntake.AddNode(prepAlgaeL2, 1.0);
        prepAlgaeIntake.AddNode(prepAlgaeL3, 1.0);
        prepAlgaeL2.AddNode(prepAlgaeIntake, 1.0);
        prepAlgaeL3.AddNode(prepAlgaeIntake, 1.0);
        prepAlgaeL2.AddNode(safeAlgaeTravel, 1.0);
        prepAlgaeL3.AddNode(safeAlgaeTravel, 1.0);
        safeAlgaeTravel.AddNode(scoreProssesor, 1.0);
        safeAlgaeTravel.AddNode(prepScoreBarge, 1.0);
        prepScoreBarge.AddNode(scoreBarge, 1.0);
        scoreProssesor.AddNode(safeCoralTravel, 1.0);
        scoreBarge.AddNode(safeCoralTravel, 1.0);
        safeCoralTravel.AddNode(climbPrep, 1.0);
        climbPrep.AddNode(climbReady, 1.0);
        climbReady.AddNode(climbPrep, 1.0);
        climbPrep.AddNode(safeCoralTravel, 1.0);

        //Any position to cancelled
        //Rest stuff is basicaly tie every node to special node that can cancell and reset robot at anypoint

        //Rest of the setup
        m_graphCommand.addRequirements(this); //this subsystem required by graph command
        this.setDefaultCommand(m_graphCommand); //Set graph command as root and keep it constantly running
        m_graphCommand.setCurrentNode(startPosition); //same as root node
        m_graphCommand.initialize(); //calculate "cheapest" paths to each node from each node

    }


     /** Triggers? */
      public Trigger atDriveTarget() {
         return new Trigger(() -> pose.atTargetPose() && atCurrentDriveState());
     }
     public Trigger atElevManiTarget() {
         return new Trigger(() -> elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint() && atCurrentElevManiState());
     }

    //  //QUESTIONED
    //  public boolean atAlgaePosition() {
    //      return (elevManiState == ElevatorManipulatorState.PrepAlgaeL2 || elevManiState == ElevatorManipulatorState.PrepAlgaeL3);
    //  }
    //  public Trigger atAlgaePositionTrigger() {
    //      return new Trigger(() -> atAlgaePosition());
    //  }



     /** Robot State Section */

    /**
     * Get if robot is transitioning between states
     * @return True if transitioning
     */
    // public boolean robotIsTransitioning() {
    //     return robotTransitioning;
    // }
    public boolean driveIsTransitioning() {
        return driveTransitioning;
    }
    public boolean elevManiIsTransitioning() {
        return elevManiTransitioning;
    }

     //LATER SHUOLD BE MODIFIED, PASTED FROM ORIGINAL ONE
     public boolean atSafeState() {
        return (getCurrentElevManiState() == ElevatorManipulatorState.StartPosition);
    }

    /**
     * Get current state robot is in
     * @return Current state of the robot
     */
    // public RobotState getCurrentState() {
    //     return robotState;
    // }
    public DriveState getCurrentDriveState() {
        return driveState;
    }
    public ElevatorManipulatorState getCurrentElevManiState() {
        return elevManiState;
    }  

    /**
     * Get if robot is at current state based on all subsystems being at their setpoint
     * @return True if at current state
     */
    // public boolean atCurrentRobotState() {
    //     return atCurrentDriveState() && atCurrentElevManiState() && !robotIsTransitioning();
    // }
    public boolean atCurrentDriveState() { //FIND THE RIGHT CODE CHECK
        return pose.atTargetPose() && !driveIsTransitioning();
    }
    public boolean atCurrentElevManiState() {
        return elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint() && !elevManiIsTransitioning();
    }

    /**
     * Get previous state of the robot
     * @return Previous state of the robot
     */
    // public RobotState getPrevRobotState() {
    //     return prevRobotState;
    // }
    public DriveState getPrevDriveState() {
        return prevDriveState;
    }
    public ElevatorManipulatorState getPrevElevManiState() {
        return prevElevManiState;
    }

    /**
     * Get goal state of the robot
     * @return Goal state of the robot
     */
    // public RobotState getGoalRobotState() {
    //     return goalRobotState;
    // }
    public DriveState getGoalDriveState() {
        return goalDriveState;
    }
    public ElevatorManipulatorState getGoalElevManiState() {
        return goalElevManiState;
    }

    /**
     * Set the current state of the robot. Also sets the previous state to where it was before
     * @param curState State the robot is currently at
     */
    // public void setCurrentRobotState(RobotState curState) {
    //     robotTransitioning = false;
    //     prevRobotState = robotState;
    //     robotState = curState;
    //     System.out.println("Current: " + robotState.toString() + " Prev: " + prevRobotState.toString() + " Goal: "
    //             + goalRobotState.toString());
    // }
    public void setCurrentDriveState(DriveState curState) {
        driveTransitioning = false;
        prevDriveState = driveState;
        driveState = curState;
        System.out.println("Current: " + driveState.toString() + " Prev: " + prevDriveState.toString() + " Goal: "
                + goalDriveState.toString());
    }
    public void setCurrentElevManiState(ElevatorManipulatorState curState) {
        elevManiTransitioning = false;
        prevElevManiState = elevManiState;
        elevManiState = curState;
        System.out.println("Current: " + elevManiState.toString() + " Prev: " + prevElevManiState.toString() + " Goal: "
                + goalElevManiState.toString());
    }

    // public Command setCurrentRobotStateCommand(RobotState curState) {
    //     return Commands.runOnce(() -> {
    //         setCurrentRobotState(curState);
    //     });
    // }
    public Command setCurrentDriveStateCommand(DriveState curState) {
        return Commands.runOnce(() -> {
            setCurrentDriveState(curState);
        });
    }
    public Command setCurrentElevManiStateCommand(ElevatorManipulatorState curState) {
        return Commands.runOnce(() -> {
            setCurrentElevManiState(curState);
        });
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newState Goal state for the robot to go to
     */
    // public void setRobotGoal(RobotState newState) {
    //     // if (currentCommand != null) {
    //     //     currentCommand.cancel();
    //     // }
    //     goalRobotState = newState;
    //     robotTransitioning = false;
    //     System.out.println("New Goal: " + newState.toString());
    //     // currentState = newState;
    // }
    public void setDriveGoal(DriveState newState) {
        // if (currentCommand != null) {
        //     currentCommand.cancel();
        // }
        goalDriveState = newState;
        driveTransitioning = false;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }
    public void setElevManiGoal(ElevatorManipulatorState newState) {
        // if (currentCommand != null) {
        //     currentCommand.cancel();
        // }
        goalElevManiState = newState;
        elevManiTransitioning = false;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }

    // public boolean atRobotGoal() {
    //     return goalRobotState == robotState;
    // }
    public boolean atDriveGoal() {
        return goalDriveState == driveState;
    }
    public boolean atElevManiGoal() {
        return goalElevManiState == elevManiState;
    }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     */
    //LATER SHUOLD BE MODIFIED, PASTED FROM ORIGINAL ONE
    // public void cancelCurrentCommand() {
    //     currentCommand.cancel();
    //     //setCurrentState(PositionState.Cancelled);
    //     //setGoal(PositionState.Cancelled);
    //     //goalState = PositionState.Cancelled;
    //     elevator.setElevatorSetpoint(elevator.getPosition());
    //     diff.setExtensionSetpoint(diff.getExtensionPosition());
    //     diff.setRotationSetpoint(diff.getRotationPosition());
    // }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     * @return Instant command to cancel current running command
     */
    // public Command cancelCommand() {
    //     return Commands.runOnce(() -> cancelCurrentCommand());
    // }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newGoal Goal state for the robot to go to
     * @return Instant command to set goal state
     */
    // public Command setGoalCommand(RobotState newGoal) {
    //     return Commands.runOnce(() -> setRobotGoal(newGoal));
    // }
    public Command setGoalDriveCommand(DriveState newGoal){
        return Commands.runOnce(() -> setDriveGoal(newGoal));
    }
    public Command setGoalElevManiCommand(ElevatorManipulatorState newGoal){
        return Commands.runOnce(() -> setElevManiGoal(newGoal));
    }


    /** ----- Branch Selection ----- */
    /**
     * Get if robot wants to score on right branch
     * @return True if right branch
     */
    public boolean getRightScore() {
        return rightScore;
    }

    private void setRightScore(boolean right) {
        rightScore = right;
    }

    /**
     * Set if robot wants to score on right branch
     * @param right True if right branch
     * @return Instant command to set right branch
     */
    public Command setRightScoreCommand(boolean right) {
        return Commands.runOnce(() -> setRightScore(right));
    }
     /** ------------------------- */


    /**
     * Get if robot has coral
     * @return True if has coral
     */
    public boolean hasCoral() {
        return manipulator.hasCoral();
    }

    /**
     * Get if robot has algae
     * @return True if has algae
     */
    public boolean hasAlgae() {
        return manipulator.hasAlgae();
    }

    /**
     * Check if elevator is at a safe height to move
     * @return True if elevator is at a safe height
     */
    public boolean safeToMove() {
        return elevator.getPosition() < 1;
    }

    /** ----- Drive State Section ----- */
    
    /**
     * Get if robot drive is rotation locked
     * @return True if rotation locked
     */
    public boolean getRotationLock() {
        return rotLock;
    }

    /**
     * Set robot drive rotation lock
     * @param lock True if locked
     */
    public void setRotationLock(boolean lock) {
        rotLock = lock;
    }

    /**
     * Toggle robot drive rotation lock
     * @return Instant command to toggle rotation lock
     */
    public Command toggleRotationLock() {
        return Commands.runOnce(() -> rotLock = !rotLock);
    }

    /** Other */

    public boolean isAuto() {
        return isAuto;
    }

    public Trigger isAutoTrigger() {
        return new Trigger(() -> isAuto);
    }

    public void setAuto(boolean auto) {
        isAuto = auto;
    }

    public boolean isDisabled() {
        return isDisabled;
    }

    public void setDisabled(boolean disabled) {
        isDisabled = disabled;
    }


    public Command safeSwitch(Command oldCmd, Command newCmd) {
    return new InstantCommand(() -> {
        if (oldCmd.isScheduled()) {
            oldCmd.cancel();
        }
        newCmd.schedule();
    });
}

    public void driveStateMachine(){
        //Manage Drive State Machine
            switch (goalDriveState) {
        case Teleop:
                //free roam, just field oriented driving
            break;
        case FollowPath:
                //path to follow
            break;
        case BargeRelative:
                //command to tell robot to face barge
            break;
        case ClimbRelative:
                //command to tell robot to face climb
            break;
        case ProcessorRelative:
                //Commadn to tell robot to dynamically face processor (angel towards it)
            break;
        case CoralStation:
                //command to tell robot to face intake (rest is managed by AutomatedDrive)
            break;
        case ReefRelative:
                //command to tell robot to face towards closest reef (rest is managed by AutomatedDrive)
            break;
        case ReefAlign:         
               //still faces reef but locks angle to either left or right side based on what branch we are scoring on and does not drive, locked in position waiting to score
            break;
        case Cancelled:
                //make sure all of the drive commands are cancelled and robot is stopped
            break;
        default:

            System.out.println("Unknown State!!!!!!!!!!");
            break;
        }
    }

    

    @Override
    public void periodic() {
        /**
         * State management system
         * Check if its at the goal, otherwise run the command needed to reach the goal
         * Inside each case for each state is specific controls to add moves in case it needs to go somewhere else first (Mostly been moved to individual commands)
         * Finally schedules command(s) at the bottom to be executed
         */
        

        if(DriverStation.isEnabled()){

            switch (goalElevManiState) {
                case StartPosition:
                        //Initial robot poisiton when turned on
                    break;
                case IntakeCoral:
                        //Intake Coral
                    break;
                case L1:
                        //Safe Travel with Coral
                    break;
                case L2:
                        //Prepare for L1 score
                    break;
                case L3:
                        //Prepare for L2 score
                    break;
                case L4:
                        //Prepare for L3 score
                    break;
                case AlgaeL2:
                        //Prepare for L4 score
                    break;
                case AlgaeL3:
                        //Final position to score L1
                    break;
                case Processor:
                        //Final position to score L2
                    break;
                case Barge:
                        //Final position to score L3
                    break;
                case Climb:
                        //Final position to score L4
                    break;
                case Manual:
                        //Get into safe position before going down to safe travel
                    break;
                case Reset:

                    break;
                default:
                    System.out.println("Unknown State!!!!!!!!!!");
                    break;
                }
            
            //Manage Elevator and Manipulator State Machine
            if (true){
                driveStateMachine();
            }

            currentCommand.schedule();
        }
       
       
    }

}



