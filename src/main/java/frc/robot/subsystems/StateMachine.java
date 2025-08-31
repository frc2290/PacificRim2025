package frc.robot.subsystems;

import java.lang.Thread.State;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.Positions.AlgaeL2Position;
import frc.robot.commands.Positions.AlgaeL3Position;
import frc.robot.commands.Positions.ClimbPosition;
import frc.robot.commands.Positions.IntakePosition;
import frc.robot.commands.Positions.L1Position;
import frc.robot.commands.Positions.L2Position;
import frc.robot.commands.Positions.L3Position;
import frc.robot.commands.Positions.L4Position;
import frc.robot.commands.Positions.ProcessorPosition;
import frc.robot.commands.Positions.TravelPosition;
import frc.robot.commands.Waits.SetGoalWait;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.LEDEffects;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;


public class StateMachine extends SubsystemBase {

    //Place to import csubsystems and commmands
    private ElevatorSubsystem elevator;
    private DifferentialSubsystem diff;
    private DriveSubsystem drive;
    private ManipulatorSubsystem manipulator;
    private PoseEstimatorSubsystem pose;
    private LEDUtility ledUtility;


    /**
     * Main Robot State Options - General comand state caleld by controller
     */
    public enum RobotState{
        StartPosition,  //Initial startin position
        Intake,
        Travel,
        L1,
        L2,
        L3,
        L4,
        AlgaeL2,
        AlgaeL3,
        Prossesor,
        Barge,
        Climb,
        Manual,      //setRobot to manual mode override
        Resseting    //Resets Robot if robot got stuck at some command or state
    }


    /**
     * DriveTrain states - drive state machine
     */
    public enum DriveState {
        Travel,            // Field oriented freerome
        FollowPath,        // Auto path following
        BargeRelative,     //FacesBarge
        ClimbRelative,     //FacesClimb
        ProcessorRelative, //Faces Processor
        IntakeRelative,    //FacesIntake based on half field
        ReefRelative,      //FacesReef based on robot position and angles as drives around
        ReefPreScore,       //Locked to right or left reef, holding position
        Cancelled
    }

    /**
     * Arm and Elevator states part of other state machine
     */
    public enum ElevatorManipulatorState {
        StartPosition,     //Initial robot poisiton when turned on
        TravelPosition,    //TravelPosition Reset, basicaly PreCoralPosition just intake is not running
        PreCoralIntake,    //Prepare for CoralIntake
        IntakeCoral,       //Intake Coral
        SafeCoralTravel,   //Safe Travel with Coral
        L1Prep,            //Prepare for L1 score
        L2Prep,            //Prepare for L2 score
        L3Prep,            //Prepare for L3 score
        L4Prep,            //Prepare for L4 score
        ScoreL1,           //Final position to score L1
        ScoreL2,           //Final position to score L2
        ScoreL3,           //Final position to score L3
        ScoreL4,           //Final position to score L4
        L1PostScore,       //Get into safe position before going down to safe travel
        L2PostScore,       //Get into safe position before going down to safe travel
        L3PostScore,
        L4PostScore,
        PrepAlgaeIntake,   //Prepare to intake Algae
        PrepAlgaeL2,       //Prepare to intake Algae from L2
        PrepAlgaeL3,       //Prepare to intake Algae from L3
        SafeAlgaeTravel,   //Safe travel with Algae
        ScoreProssesor,    //Score Algae into prossesor final position
        PrepScoreBarge,    //Prepage to score Algae into barge
        ScoreBarge,        //Score Algae into barge
        ClimbPrep,         //Get into safe position before engagin climber
        ClimbRead,         //RetractClimber - Ready to Climb
        Cancelled          //Cancell all the running commands and states, can also be called using manual override, to reset robot to travel position
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

    //General State Variables
    private boolean rotLock = true; //Lock rotation when driving preventing user from overriding
    private boolean rightScore = false; //Selecting Branch for scoring
    private boolean driveTransitioning = false; //If drivetrain statemachine transitioning
    private boolean elevManiTransitioning = false; //If elevatorManipulator is transitioning between states
    private boolean robotTransitioning = false; //If robot is transitioning between main states
    private boolean isAuto = false; //If robot is in auto mode
    private boolean isDisabled = false; //If robot is disabled

    private Command currentCommand = null;

    private EndEffector endEffector = EndEffector.HasCoral; //When robot starts, it knows, it has coral

    // State storage and queueing
    private RobotState prevRobotState = RobotState.StartPosition;
    private RobotState robotState = RobotState.StartPosition; //current main robot state
    private RobotState goalRobotState = RobotState.StartPosition; //PositionState.TravelPosition;
    private DriveState prevDriveState = DriveState.Travel;
    private DriveState driveState = DriveState.Travel;
    private DriveState goalDriveState = DriveState.Travel;
    private ElevatorManipulatorState prevElevManiState = ElevatorManipulatorState.StartPosition;
    private ElevatorManipulatorState elevManiState = ElevatorManipulatorState.StartPosition; //When robot is turned on, this isthe starting state
    private ElevatorManipulatorState goalElevManiState = ElevatorManipulatorState.StartPosition;

    //LATER SHUOLD BE MODIFIED, PASTED FROM ORIGINAL ONE
    private FlytLogger stateDash = new FlytLogger("State");

    /*
     * Creat a new StateSubsystem
     */
    public StateMachine(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, DriveSubsystem m_drive, ManipulatorSubsystem m_manipulator, PoseEstimatorSubsystem m_pose, LEDUtility m_ledUtility) {
        diff = m_diff;
        elevator = m_elevator;
        drive = m_drive;
        manipulator = m_manipulator;
        pose = m_pose;
        ledUtility = m_ledUtility;

        //Dashboard
        stateDash.addIntegerPublisher("RobotState", false, () -> robotState.ordinal());
        stateDash.addIntegerPublisher("DriveState", false, () -> driveState.ordinal());
        stateDash.addIntegerPublisher("ElevManiState", false, () -> elevManiState.ordinal());
        stateDash.addIntegerPublisher("EndEffector", false, () -> endEffector.ordinal());
        stateDash.addBoolPublisher("RotLock", false, () -> rotLock);
        stateDash.addBoolPublisher("RightScore", false, () -> rightScore);
        //stateDash.addBoolPublisher("Transitioning", false, () -> transitioning);
        stateDash.addBoolPublisher("isAuto", false, () -> isAuto);
        stateDash.addBoolPublisher("isDisabled", false, () -> isDisabled);
    }

     /** Triggers? */
      public Trigger atDriveTarget() {
         return new Trigger(() -> pose.atTargetPose() && atCurrentDriveState());
     }
     public Trigger atElevManiTarget() {
         return new Trigger(() -> elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint() && atCurrentElevManiState());
     }

     //QUESTIONED
     public boolean atAlgaePosition() {
         return (elevManiState == ElevatorManipulatorState.PrepAlgaeL2 || elevManiState == ElevatorManipulatorState.PrepAlgaeL3);
     }
     public Trigger atAlgaePositionTrigger() {
         return new Trigger(() -> atAlgaePosition());
     }



     /** Robot State Section */

    /**
     * Get if robot is transitioning between states
     * @return True if transitioning
     */
    public boolean robotIsTransitioning() {
        return robotTransitioning;
    }
    public boolean driveIsTransitioning() {
        return driveTransitioning;
    }
    public boolean elevManiIsTransitioning() {
        return elevManiTransitioning;
    }

     //LATER SHUOLD BE MODIFIED, PASTED FROM ORIGINAL ONE
     public boolean atSafeState() {
        return (getCurrentElevManiState() == ElevatorManipulatorState.TravelPosition ||
                getCurrentElevManiState() == ElevatorManipulatorState.StartPosition);
    }

    /**
     * Get current state robot is in
     * @return Current state of the robot
     */
    public RobotState getCurrentState() {
        return robotState;
    }
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
    public boolean atCurrentRobotState() {
        return atCurrentDriveState() && atCurrentElevManiState() && !robotIsTransitioning();
    }
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
    public RobotState getPrevRobotState() {
        return prevRobotState;
    }
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
    public RobotState getGoalRobotState() {
        return goalRobotState;
    }
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
    public void setCurrentRobotState(RobotState curState) {
        robotTransitioning = false;
        prevRobotState = robotState;
        robotState = curState;
        System.out.println("Current: " + robotState.toString() + " Prev: " + prevRobotState.toString() + " Goal: "
                + goalRobotState.toString());
    }
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

    public Command setCurrentRobotStateCommand(RobotState curState) {
        return Commands.runOnce(() -> {
            setCurrentRobotState(curState);
        });
    }
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
    public void setRobotGoal(RobotState newState) {
        // if (currentCommand != null) {
        //     currentCommand.cancel();
        // }
        goalRobotState = newState;
        robotTransitioning = false;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }
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

    public boolean atRobotGoal() {
        return goalRobotState == robotState;
    }
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
    public Command setGoalCommand(RobotState newGoal) {
        return Commands.runOnce(() -> setRobotGoal(newGoal));
    }


    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newGoal Goal state for the robot to go to
     * @param wait Boolean to determine if we want to wait for it to finish (True will wait)
     * @return Instant command to set goal state or command with a wait until at goal
     */
    // public Command setGoalCommand(PositionState newGoal, boolean wait) {
    //     if (wait) {
    //         return new SetGoalWait(this, newGoal);
    //     } else {
    //         return setGoalCommand(newGoal);
    //     }
    // }

    /**
     * Get if robot wants to score on right branch
     * @return True if right branch
     */
    public boolean getRightScore() {
        return rightScore;
    }

    /**
     * Set if robot wants to score on right branch
     * @param right True if right branch
     */
    public void setRightScore(boolean right) {
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

    public void driveStateMachine(){
        //Manage Drive State Machine
            switch (goalDriveState) {
        case Travel:
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
        case IntakeRelative:
                //command to tell robot to face intake (rest is managed by AutomatedDrive)
            break;
        case ReefRelative:
                //command to tell robot to face towards closest reef (rest is managed by AutomatedDrive)
            break;
        case ReefPreScore:         
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

    public void elevatorManipulatorStateMachine(){
        //Manage Elevator and Manipulator State Machine
            switch (goalElevManiState) {
        case StartPosition:
                //Initial robot poisiton when turned on
            break;
        case TravelPosition:
                //TravelPosition Reset, basicaly PreCoralPosition just intake is not running
            break;
        case PreCoralIntake:
                //Prepare for CoralIntake
            break;
        case IntakeCoral:
                //Intake Coral
            break;
        case SafeCoralTravel:
                //Safe Travel with Coral
            break;
        case L1Prep:
                //Prepare for L1 score
            break;
        case L2Prep:
                //Prepare for L2 score
            break;
        case L3Prep:
                //Prepare for L3 score
            break;
        case L4Prep:
                //Prepare for L4 score
            break;
        case ScoreL1:
                //Final position to score L1
            break;
        case ScoreL2:
                //Final position to score L2
            break;
        case ScoreL3:
                //Final position to score L3
            break;
        case ScoreL4:
                //Final position to score L4
            break;
        case L1PostScore:
                //Get into safe position before going down to safe travel
            break;
        case L2PostScore:
                //Get into safe position before going down to safe travel
            break;
        case L3PostScore:
                //
            break;
        case L4PostScore:
                //
            break;
        case PrepAlgaeIntake:
                //Prepare to intake Algae
            break;
        case PrepAlgaeL2:
                //Prepare to intake Algae from L2
            break;
        case PrepAlgaeL3:
                //Prepare to intake Algae from L3
            break;
        case SafeAlgaeTravel:
                //Safe travel with Algae
            break;
        case ScoreProssesor:
                //Score Algae into prossesor final position
            break;
        case PrepScoreBarge:
                //Prepage to score Algae into barge
            break;
        case ScoreBarge:
                //Score Algae into barge
            break;
        case ClimbPrep:
                //Get into safe position before engagin climber
            break;
        case ClimbRead:
                //RetractClimber - Ready to Climb
            break;
        case Cancelled:
                //Cancell all the running commands and states, can also be called using manual override, to reset robot to travel position
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

        if (goalRobotState != robotState && !robotIsTransitioning() && atCurrentRobotState() && DriverStation.isEnabled())
        {
            switch (goalRobotState) {
                case StartPosition:
                    //Nothing
                    //currentCommand = new TravelPosition(diff, elevator, this);
                    //robotTransitioning = false;
                    break;
                case Intake:
                    //set drivetrainstatemachine to IntakeRelative awaiting to get into reefRelative
                    //when that is done
                    //elevatorManipulatorStateMachine() to PreCoralIntake
                    //after few internal cheks it will go to intakecoral and wait until it has coral
                    //after which it will go to safe coral travel position (robot state will be set to Travel)
                    //once it got into that position drivetrainstatemachine will be set to reefRelative
                    
                    break;
                case Travel:
                    //This is internal state where it has a game piece and is in safe travel position before going into any other state
                    
                    break;
                case L1:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to L1Prep
                    //when that is done it will be set to ScoreL1 and at the same time drive statemachine will be set to ReefPreScore 
                    //which mean, it will drive towards closest reef and lock angle to either left or right based on what branch we are scoring on
                    //once it is at scoreL1 position and at reef prescore position stopped moving or almost stopped moving
                    // it will kick in the interpolation and wait for driver to hit score button
                   
                    break;
                case L2:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to L1Prep
                    //when that is done it will be set to ScoreL2 and at the same time drive statemachine will be set to ReefPreScore 
                    //which mean, it will drive towards closest reef and lock angle to either left or right based on what branch we are scoring on
                    //once it is at scoreL2 position and at reef prescore position stopped moving or almost stopped moving
                    // it will kick in the interpolation and wait for driver to hit score button
            
                    break;
                case L3:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to L1Prep
                    //when that is done it will be set to ScoreL3 and at the same time drive statemachine will be set to ReefPreScore 
                    //which mean, it will drive towards closest reef and lock angle to either left or right based on what branch we are scoring on
                    //once it is at scoreL3 position and at reef prescore position stopped moving or almost stopped moving
                    // it will kick in the interpolation and wait for driver to hit score button
                  
                    break;
                case L4:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to L1Prep
                    //when that is done it will be set to ScoreL4 and at the same time drive statemachine will be set to ReefPreScore 
                    //which mean, it will drive towards closest reef and lock angle to either left or right based on what branch we are scoring on
                    //once it is at scoreL4 position and at reef prescore position stopped moving or almost stopped moving
                    // it will kick in the interpolation and wait for driver to hit score button
                 
                    break;
                case AlgaeL2:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to PrepAlgaeL2 and double make sure 
                    //drivetrain is set to reefRelative
                    //when that is done it will be set to IntakeAlgae and wait until it has algae
                    //when it detects algae, it will be set to SafeAlgaeTravel (robot state will be set to Travel)
                    //once it got into that position drivetrainstatemachine will be set to prossesor unless driver requests barge (Robot state will be set to Prossesor unless driver specifies barge)
               
                    break;
                case AlgaeL3:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to PrepAlgaeL2 and double make sure 
                    //drivetrain is set to reefRelative
                    //when that is done it will be set to IntakeAlgae and wait until it has algae
                    //when it detects algae, it will be set to SafeAlgaeTravel (robot state will be set to Travel)
                    //once it got into that position drivetrainstatemachine will be set to Scoreprossesor unless driver requests barge
            
                    break;
                case Prossesor:
                    //Waits untill driver scores or requests barge
                  
                    break;
                case Barge:
                    //Waits untill driver scores
       
                    break;
                case Climb:
                    //Once this state has been set by driver, elevatorManipulatorStateMachine() will be set to ClimbPrep and drivetrain will be set to ClimbRelative
                    //when that is done it will be set to ClimbRead and wait until driver hits climb button
                     
                    break;
                case Manual:
                    //When driver wants to take over manual control of elevator and manipulator, it will set robot state to manual, 
                    //state machines will freez at current position and listen for direct commands from the driver
                    //once disabled robot will follow next state commadn from driver
                    break;  
                case Resseting:
                    //When robot got stuck or something went wrong, driver can resset robot, it will cancel all running commands and set everything to safe travel position
                    //Also will be done automaticaly if robot is not at goal after certain time or robot needs to change too many states to reach the goal

                    break;
                default:
                    System.out.println("Unknown State!!!!!!!!!!");
                    break;
            }

            
                    
        }

        

        //Manage Elevator and Manipulator State Machine
        if (true){
            elevatorManipulatorStateMachine();
            currentCommand.schedule();
        }

        //Manage Drive State Machine
        if (true){
            driveStateMachine();
            currentCommand.schedule();
        }



  
    

        // // LED Management?
        // if (atCurrentState()) {
        //     if (isAuto()) {
        //         ledUtility.setAll(LEDEffect.PULSE, LEDEffects.flytBlue);
        //     } else if (isDisabled()) {
        //         ledUtility.getStrip("TopLeft").setEffect(LEDEffect.NAVLIGHTS, Color.kRed);
        //         ledUtility.getStrip("TopRight").setEffect(LEDEffect.NAVLIGHTS, Color.kGreen);
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //     } else if (getCurrentState() == PositionState.IntakePosition && !manipulator.hasCoral() && getDriveState() == DriveState.CoralStation) {
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, Color.kGreen);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, Color.kGreen);
        //     } else if (getCurrentState() == PositionState.IntakePosition && manipulator.hasCoral() && getDriveState() == DriveState.CoralStation) {
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
        //     } else if (!getRotationLock()) {
        //         ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kRed);
        //         ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kRed);
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //     } else if (getDriveState() == DriveState.Teleop) {
        //         if (getRightScore()) {
        //             ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kPurple);
        //             ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kPurple);
        //         } else {
        //             ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kYellow);
        //             ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kYellow);
        //         }
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        //     } else if (getDriveState() == DriveState.ReefScoreMove) {
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
        //     } else if (getDriveState() == DriveState.ReefScore) {
        //         ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
        //         ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
        //     } 
        // }

        // /** Diff Arm Interpolation */
        // if (atInterpolateScoreState() 
        //     && diff.hasLaserCanDistance() 
        //     && !isAuto() 
        //     && manipulator.hasCoral()) 
        //     {
        //     //if (atInterpolateScoreState() && diff.hasLaserCanDistance() && manipulator.hasCoral()) {
        //         if (currentState == PositionState.L4Position) {
        //         diff.setExtensionSetpoint(diff.l4ExtensionInterpolate());
        //         diff.setRotationSetpoint(diff.l4RotationInterpolate());
        //     } else {
        //         diff.setExtensionSetpoint(diff.l2_3ExtensionInterpolate());
        //         diff.setRotationSetpoint(diff.l2_3RotationInterpolate());
        //     }
        // }

        stateDash.update(Constants.debugMode);
    }

    // private boolean atInterpolateScoreState() {
    //     return currentState == PositionState.L2Position
    //             || currentState == PositionState.L3Position || currentState == PositionState.L4Position;
    // }
}



