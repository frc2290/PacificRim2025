package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.robot.commands.DriveCommands.ManualDrive;
import frc.robot.commands.ElevatorManipulator.BargePrep;
import frc.robot.commands.ElevatorManipulator.IntakeCoral;
import frc.robot.commands.ElevatorManipulator.L2Prep;
import frc.robot.commands.ElevatorManipulator.L3PostScore;
import frc.robot.commands.ElevatorManipulator.L3Prep;
import frc.robot.commands.ElevatorManipulator.L4PostScore;
import frc.robot.commands.ElevatorManipulator.L4Prep;
import frc.robot.commands.ElevatorManipulator.PrepCoralIntake;
import frc.robot.commands.ElevatorManipulator.SafeTravelSequential;
import frc.robot.commands.ElevatorManipulator.ScoreL2;
import frc.robot.commands.ElevatorManipulator.ScoreL3;
import frc.robot.commands.ElevatorManipulator.ScoreL4;
import frc.robot.commands.EndEffector.ManipulatorIntakeCoral;
import frc.robot.commands.EndEffector.ScoreCoral;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;

public class ManipulatorStateMachine extends SubsystemBase {

    private GraphCommand m_graphCommand = new GraphCommand();
    private FlytDashboardV2 dashboard = new FlytDashboardV2("ManipulatorStateMachine");
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;
    private ElevatorSubsystem m_elevator;
    private DifferentialSubsystem m_diff;
    private ManipulatorSubsystem m_manipulator;
    private ClimbSubsystem m_climb;

    /**
    * Manipulator and Elevator states - Manipulator State Machine
    */
    public enum ElevatorManipulatorState {
        START_POSITION,
        SAFE_CORAL_TRAVEL,
        INTAKE_CORAL,
        L1,
        L2,
        L3,
        L4,
        ALGAE_L2,
        ALGAE_L3,
        PROCESSOR,
        BARGE,
        CLIMB,
        MANUAL,
        RESET,

        PrepL1,
        PrepL2,
        PrepL3,
        PrepL4,

        PostL1,
        PostL2,
        PostL3,
        PostL4,

    }

    /*
     * Graph Command Nodes for Drive State Machine
     */
    GraphCommandNode startPositionNode;
    GraphCommandNode preCoralIntakeNode;
    GraphCommandNode intakeCoralNode;
    GraphCommandNode safeCoralTravelNode;
    GraphCommandNode l1PrepNode;
    GraphCommandNode l2PrepNode;
    GraphCommandNode l3PrepNode;
    GraphCommandNode l4PrepNode;
    GraphCommandNode scoreL1Node;
    GraphCommandNode scoreL2Node;
    GraphCommandNode scoreL3Node;
    GraphCommandNode scoreL4Node;
    GraphCommandNode l1PostScoreNode;
    GraphCommandNode l2PostScoreNode;
    GraphCommandNode l3PostScoreNode;
    GraphCommandNode l4PostScoreNode;
    GraphCommandNode prepAlgaeIntakeNode;
    GraphCommandNode prepAlgaeL2Node;
    GraphCommandNode prepAlgaeL3Node;
    GraphCommandNode safeAlgaeTravelNode;
    GraphCommandNode scoreProssesorNode;
    GraphCommandNode prepScoreBargeNode;
    GraphCommandNode scoreBargeNode;
    GraphCommandNode climbPrepNode;
    GraphCommandNode climbReadyNode;
    GraphCommandNode cancelledNode;

    //variables
    private boolean atGoalState = false; //checks if it reached the final state
    private boolean canScore = false; //checks final state is reached and command aproves the score
    private boolean score = false; //drive sends command to score
    private boolean interpolate = false;

    private boolean driveAtPose = false;


    private boolean reachGoalStateFailed = false;
    private double stateEntryTime = 0.0;


    /**
     * ManipulatorStateMachine Constructor
     * @param diff
     * @param elevator
     * @param manipulator
     * @param climb
     */
    public ManipulatorStateMachine(DifferentialSubsystem diff, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator, ClimbSubsystem climb) {
        m_diff = diff;
        m_elevator = elevator;
        m_manipulator = manipulator;
        m_climb = climb;
    
        initializeGraphCommand();

        // Set the root/current BEFORE scheduling it as default
        m_graphCommand.setGraphRootNode(startPositionNode);
        m_graphCommand.setCurrentNode(startPositionNode);
        
        // Now register requirements and set default
        m_graphCommand.addRequirements(this);
        this.setDefaultCommand(m_graphCommand);
        
    }

    private void initializeGraphCommand(){

            startPositionNode = m_graphCommand.new GraphCommandNode(
                "StartPosition", 
                new PrintCommand("At Start Position"),
                new PrintCommand(""),
                new PrintCommand(""));

            preCoralIntakeNode = m_graphCommand.new GraphCommandNode(
                "PreCoralIntake", 
                new PrepCoralIntake(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            intakeCoralNode = m_graphCommand.new GraphCommandNode(
                "IntakeCoral", 
                new IntakeCoral(this, m_diff, m_elevator),
                new PrintCommand(""),
                new ManipulatorIntakeCoral(m_manipulator));

            safeCoralTravelNode = m_graphCommand.new GraphCommandNode(
                "SafeCoralTravelPos", 
                new SafeTravelSequential(m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));
            
            l1PrepNode = m_graphCommand.new GraphCommandNode(
                "L1Prep", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            l2PrepNode = m_graphCommand.new GraphCommandNode(
                "L2Prep", 
                new L2Prep(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            l3PrepNode = m_graphCommand.new GraphCommandNode(
                "L3Prep", 
                new L2Prep(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            l4PrepNode = m_graphCommand.new GraphCommandNode(
                "L4Prep", 
                new L4Prep(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            scoreL1Node = m_graphCommand.new GraphCommandNode(
                "ScoreL1", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            scoreL2Node = m_graphCommand.new GraphCommandNode(
                "ScoreL2", 
                new ParallelCommandGroup(
                    new ScoreL2(this, m_diff, m_elevator),
                    new ScoreCoral(this,m_manipulator)),
                new PrintCommand("Hi this is l2"),
                new PrintCommand("Hi this is l2"));

            scoreL3Node = m_graphCommand.new GraphCommandNode(
                "ScoreL3", 
                new ParallelCommandGroup(
                    new ScoreL3(this, m_diff, m_elevator),
                    new ScoreCoral(this,m_manipulator)),
                new PrintCommand(""),
                new PrintCommand(""));


            scoreL4Node = m_graphCommand.new GraphCommandNode(
                "ScoreL4", 
                new ParallelCommandGroup(
                    new ScoreL4(this, m_diff, m_elevator),
                    new ScoreCoral(this,m_manipulator)),
                new PrintCommand(""),
                new PrintCommand(""));

            l1PostScoreNode = m_graphCommand.new GraphCommandNode(
                "L1PostScore", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            l2PostScoreNode = m_graphCommand.new GraphCommandNode(
                "L2PostScore", 
                new L3PostScore(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            l3PostScoreNode = m_graphCommand.new GraphCommandNode(
                "L3PostScore", 
                new L3PostScore(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));  

            l4PostScoreNode = m_graphCommand.new GraphCommandNode(
                "L4PostScore", 
                new L4PostScore(this, m_diff, m_elevator),
                new PrintCommand(""),
                new PrintCommand(""));

            prepAlgaeIntakeNode = m_graphCommand.new GraphCommandNode(
                "PrepAlgaeIntake", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            prepAlgaeL2Node = m_graphCommand.new GraphCommandNode(
                "PrepAlgaeL2", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            prepAlgaeL3Node = m_graphCommand.new GraphCommandNode(      
                "PrepAlgaeL3", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            safeAlgaeTravelNode = m_graphCommand.new GraphCommandNode(
                "SafeAlgaeTravelPos", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            scoreProssesorNode = m_graphCommand.new GraphCommandNode(
                "ScoreProcessor", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            prepScoreBargeNode = m_graphCommand.new GraphCommandNode(
                "PrepScoreBarge", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));  

            scoreBargeNode = m_graphCommand.new GraphCommandNode(
                "ScoreBarge", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));  

            climbPrepNode = m_graphCommand.new GraphCommandNode(
                "ClimbPrep", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            climbReadyNode = m_graphCommand.new GraphCommandNode(
                "ClimbReady", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));

            cancelledNode = m_graphCommand.new GraphCommandNode(
                "Cancelled", 
                new PrintCommand(""),
                new PrintCommand(""),
                new PrintCommand(""));


        // Safe travel connections
        startPositionNode.AddNode(safeCoralTravelNode, 1.0); //start position to safe travel
        safeCoralTravelNode.AddNode(preCoralIntakeNode, 1.0); //safe travel to pre coral intake
        safeCoralTravelNode.AddNode(l1PrepNode, 1.0); //safe travel to l1 prep
        safeCoralTravelNode.AddNode(l2PrepNode, 1.0); //safe travel to l2 prep
        safeCoralTravelNode.AddNode(l3PrepNode, 1.0); //safe travel to l3 prep
        safeCoralTravelNode.AddNode(l4PrepNode, 1.0); //safe travel to l4 prep
        preCoralIntakeNode.AddNode(intakeCoralNode, 1.0); //pre coral intake to coral intake
        intakeCoralNode.AddNode(safeCoralTravelNode, 1.0); //coral intake to safe travel
        //l1PrepNode.AddNode(l2PrepNode, 1.0); //l1 prep to l2 prep
        //l1PrepNode.AddNode(l3PrepNode, 1.0); //l1 prep to l3 prep
        //l1PrepNode.AddNode(l4PrepNode, 1.0); //l1 prep to l4 prep
        //l2PrepNode.AddNode(l1PrepNode, 1.0); //l2 prep to l1 prep
        //l2PrepNode.AddNode(l3PrepNode, 1.0); //l2 prep to l3 prep
        //l2PrepNode.AddNode(l4PrepNode, 1.0); //l2 prep to l4 prep
        //l3PrepNode.AddNode(l1PrepNode, 1.0); //l3 prep to l1 prep
        //l3PrepNode.AddNode(l2PrepNode, 1.0); //l3 prep to l2 prep
        //l3PrepNode.AddNode(l4PrepNode, 1.0); //l3 prep to l4 prep
        //l4PrepNode.AddNode(l1PrepNode, 1.0); //l4 prep to l1 prep
        //l4PrepNode.AddNode(l2PrepNode, 1.0); //l4 prep to l2 prep
        //l4PrepNode.AddNode(l3PrepNode, 1.0); //l4 prep to l3 prep
        l1PrepNode.AddNode(scoreL1Node, 1.0); //l1 prep to score l1
        l2PrepNode.AddNode(scoreL2Node, 1.0); //l2 prep to score l2
        l3PrepNode.AddNode(scoreL3Node, 1.0); //l3 prep to score l3
        l4PrepNode.AddNode(scoreL4Node, 1.0); //l4 prep to score l4
        //scoreL1Node.AddNode(l1PrepNode, 1.0); //score l1 to l1 prep
        //scoreL2Node.AddNode(l2PrepNode, 1.0); //score l2 to l2 prep
        //scoreL3Node.AddNode(l3PrepNode, 1.0); //score l3 to l3 prep
        //scoreL4Node.AddNode(l4PrepNode, 1.0); //score l4 to l4 prep
        scoreL1Node.AddNode(l1PostScoreNode, 1.0); //score l1 to l1 post score
        scoreL2Node.AddNode(l2PostScoreNode, 1.0); //score l2 to l2 post score
        scoreL3Node.AddNode(l3PostScoreNode, 1.0); //score l3 to l3 post score
        scoreL4Node .AddNode(l4PostScoreNode, 1.0); //score l4 to l4 post score
        l1PostScoreNode.AddNode(safeCoralTravelNode, 1.0); //l1 post score to safe travel
        l2PostScoreNode.AddNode(safeCoralTravelNode, 1.0); //l2 post score to safe travel
        l3PostScoreNode.AddNode(safeCoralTravelNode, 1.0); //l3 post score to safe travel
        l4PostScoreNode.AddNode(safeCoralTravelNode, 1.0); //l4 post score to safe travel
        safeCoralTravelNode.AddNode(prepAlgaeIntakeNode, 1.0); //safe travel to prep algae intake
        prepAlgaeIntakeNode.AddNode(safeAlgaeTravelNode, 1.0); //prep algae intake to safe algae travel
        prepAlgaeIntakeNode.AddNode(prepAlgaeL2Node, 1.0); //prep algae intake to prep algae l2
        prepAlgaeIntakeNode.AddNode(prepAlgaeL3Node, 1.0); //prep algae intake to prep algae l3
        prepAlgaeL2Node.AddNode(prepAlgaeIntakeNode, 1.0); //prep algae l2 to prep algae intake
        prepAlgaeL3Node.AddNode(prepAlgaeIntakeNode, 1.0); //prep algae l3 to prep algae intake
        prepAlgaeL2Node.AddNode(safeAlgaeTravelNode, 1.0); //prep algae l2 to safe algae travel 
        prepAlgaeL3Node.AddNode(safeAlgaeTravelNode, 1.0); //prep algae l3 to safe algae travel
        safeAlgaeTravelNode.AddNode(scoreProssesorNode, 1.0); //safe algae travel to score processor 
        safeAlgaeTravelNode.AddNode(prepScoreBargeNode, 1.0); //safe algae travel to prep score barge
        prepScoreBargeNode.AddNode(scoreBargeNode, 1.0); //prep score barge to score barge  
        scoreProssesorNode.AddNode(safeCoralTravelNode, 1.0); //score processor to safe coral travel
        scoreBargeNode.AddNode(safeCoralTravelNode, 1.0); //score barge to safe coral travel
        safeCoralTravelNode.AddNode(climbPrepNode, 1.0); //safe coral travel to climb prep
        climbPrepNode.AddNode(climbReadyNode, 1.0); //climb prep to climb ready
        climbReadyNode.AddNode(climbPrepNode, 1.0); //climb ready to climb prep
        climbPrepNode.AddNode(safeCoralTravelNode, 1.0); //climb prep to safe coral travel
                

                
        
    }
    
    //need for scoring, make sure drive subsystem is at pose before scoring
    public void setDriveAtPose(boolean atPose){
        driveAtPose = atPose;
    }

    public void setInterpolation(boolean m_interpolate){
        interpolate = m_interpolate;
    }
    /**
     * Tell state machine that command finished succesfully (Shuold only be set by commands)
     * @param state
     */
    public void setatGoalState(boolean state){
        atGoalState = state;
    }
    
    //should only be used by commands, lets scoring command know that systems are ready to score
    public void setreadyToScore(boolean canscore){
        canScore = canscore;
    }
    
    /**
     * Can only be used by commands, tell statemachine that command too to long to complete, needs reset, or revert to prep node
     */
    public void failedToReachGoal(boolean isFailed){
        reachGoalStateFailed = isFailed;
    }

    //used by a coardinator to tell statemachine that it needs to score if it can
    public void score(boolean m_score){
        score = m_score;
    }

    public boolean getInterpolateActive(){
        return interpolate;
    }

    public boolean getHasCoral(){
        return m_manipulator.hasCoral();   
    }

    
    public boolean readyToScore(){
        return canScore;
    }

    //driver made a call to score now
    public boolean scoreNow(){
        return score;
    }
    
    /**
     * GraphCommand is still reaching setgoalnode
     * @return 
     */
    public boolean isTransitioning() {
        return m_graphCommand.isTransitioning();
    }

    private GraphCommandNode getGraphState(){
        return m_graphCommand.getCurrentNode();
    }

    /**
     * Check if all all subsystem are at position, atGoalState is updated by a last command
     * @return
     */
    public boolean atGoalState(){
        return atGoalState;
    }

    public boolean reachGoalStateFailed(){
        return reachGoalStateFailed;
    }

    /** ----- State Transition Commands ----- */

    /**
     * Sets the goal state for the manipulator state machine
     * @param m_state
     */
    public void setElevatorManipulatorCommand(ElevatorManipulatorState m_state){
        switch (m_state){
            case SAFE_CORAL_TRAVEL:
                m_graphCommand.setTargetNode(safeCoralTravelNode);
                break;  
            case INTAKE_CORAL:
                m_graphCommand.setTargetNode(intakeCoralNode);
                break;
            case L1:
                m_graphCommand.setTargetNode(scoreL1Node);
                break;
            case L2:
                m_graphCommand.setTargetNode(scoreL2Node);
                break;
            case L3:
                m_graphCommand.setTargetNode(scoreL3Node);
                break;
            case L4:
                m_graphCommand.setTargetNode(scoreL4Node);
                break;
            case ALGAE_L2:
                m_graphCommand.setTargetNode(prepAlgaeL2Node);
                break;
            case ALGAE_L3:
                m_graphCommand.setTargetNode(prepAlgaeL3Node);
                break;
            case PROCESSOR:
                m_graphCommand.setTargetNode(scoreProssesorNode);
                break;
            case BARGE:
                m_graphCommand.setTargetNode(scoreBargeNode);
                break;
            case CLIMB:
                m_graphCommand.setTargetNode(climbPrepNode);
                break;
            case MANUAL:
                break;
            default:
                break;
        }
    }

    /**     
     * Return current state
     * @return
     */
    public ElevatorManipulatorState getCurrentState(){
        GraphCommandNode currentNode = m_graphCommand.getCurrentNode();
        if(currentNode == startPositionNode) return ElevatorManipulatorState.START_POSITION;
        if(currentNode == safeCoralTravelNode) return ElevatorManipulatorState.SAFE_CORAL_TRAVEL;
        if(currentNode == intakeCoralNode) return ElevatorManipulatorState.INTAKE_CORAL;
        if(currentNode == l1PrepNode) return ElevatorManipulatorState.PrepL1 ;
        if(currentNode == scoreL1Node) return ElevatorManipulatorState.L1;
        if(currentNode == l1PostScoreNode) return ElevatorManipulatorState.PostL1;
        if(currentNode == l2PrepNode) return ElevatorManipulatorState.PrepL2 ;
        if(currentNode == scoreL2Node) return ElevatorManipulatorState.L2;
        if(currentNode == l2PostScoreNode) return ElevatorManipulatorState.PostL2;
        if(currentNode == l3PrepNode) return ElevatorManipulatorState.PrepL3 ;
        if(currentNode == scoreL3Node) return ElevatorManipulatorState.L3;
        if(currentNode == l3PostScoreNode) return ElevatorManipulatorState.PostL3;
        if(currentNode == l4PrepNode) return ElevatorManipulatorState.PrepL4 ;
        if(currentNode == scoreL4Node) return ElevatorManipulatorState.L4;
        if(currentNode == l4PostScoreNode) return ElevatorManipulatorState.PostL4;
        if(currentNode == prepAlgaeL2Node) return ElevatorManipulatorState.ALGAE_L2;
        if(currentNode == prepAlgaeL3Node) return ElevatorManipulatorState.ALGAE_L3;
        if(currentNode == scoreProssesorNode) return ElevatorManipulatorState.PROCESSOR;
        if(currentNode == prepScoreBargeNode || currentNode == scoreBargeNode) return ElevatorManipulatorState.BARGE;
        if(currentNode == climbPrepNode || currentNode == climbReadyNode) return ElevatorManipulatorState.CLIMB;
        if(currentNode == cancelledNode) return ElevatorManipulatorState.RESET;
        return ElevatorManipulatorState.MANUAL;
    }





    /** ----- Periodic ----- */
    @Override
    public void periodic() {
        //Update dashboard
        dashboard.putString("Goal State", getCurrentState().toString());
        dashboard.putBoolean("Transitioning", isTransitioning());
        dashboard.putBoolean("DriveTrainLocked", atGoalState());
        dashboard.putString("Current GraphState State", getGraphState().toString());
        dashboard.putBoolean("At state graphcommand", !isTransitioning());

        
    }

    
}
