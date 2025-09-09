package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorManipulator.SafeTravel;
import frc.utils.GraphCommand;
import frc.utils.GraphCommand.GraphCommandNode;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.LEDUtility;

/**
 * Subsystem that manages the elevator and manipulator state machine.  This
 * class began as a mirror of a legacy monolithic manager but now owns the
 * logic directly.
 */
public class ArmStateManager extends SubsystemBase {

    // Subsystems and utilities used by the state machine.  These are passed in
    // so that this class can schedule commands in the same manner as the
    // original implementation.
    private DifferentialSubsystem diff;
    private ElevatorSubsystem elevator;
    private DriveSubsystem drive;
    private ManipulatorSubsystem manipulator;
    private PoseEstimatorSubsystem pose;
    private LEDUtility ledUtility;
    private XboxController driverController;

    // Graph based command runner used for complex state transitions.
    private GraphCommand m_graphCommand = new GraphCommand();

    /**
     * Elevator and manipulator high level states.
     */
    public enum ElevatorManipulatorState {
        StartPosition,
        SafeCoralTravel,
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
     * Simple enumeration describing what game piece the end effector currently
     * holds.  Only the values needed for the current logic are included.
     */
    public enum EndEffector {
        HasCoral,
        HasAlgae,
        IntakeAlgae,
        IntakeCoral,
        ScoreCoral,
        ScoreAlgae
    }

    // GraphCommand nodes describing arm state transitions.
    private final GraphCommandNode startPosition = m_graphCommand.new GraphCommandNode(
            "StartPosition",
            new PrintCommand("Robot is at start position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode preCoralIntake = m_graphCommand.new GraphCommandNode(
            "PreCoralIntake",
            new PrintCommand(
                    "SendCommand to move into this position, for elevator then when safe move the extension out and rotation"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode intakeCoral = m_graphCommand.new GraphCommandNode(
            "IntakeCoral",
            new PrintCommand("Turn on the intake, wait until coral is detected"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode safeCoralTravel = m_graphCommand.new GraphCommandNode(
            "SafeCoralTravel",
            new SafeTravel(diff, elevator),
            new PrintCommand("Nothing"),
            reachedElevManiGoalCommand());

    private final GraphCommandNode l1Prep = m_graphCommand.new GraphCommandNode(
            "L1Prep",
            new PrintCommand("Move elevator to L1 prep position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l2Prep = m_graphCommand.new GraphCommandNode(
            "L2Prep",
            new PrintCommand("Move elevator to L2 prep position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l3Prep = m_graphCommand.new GraphCommandNode(
            "L3Prep",
            new PrintCommand("Move elevator to L3 prep position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l4Prep = m_graphCommand.new GraphCommandNode(
            "L4Prep",
            new PrintCommand("Move elevator to L4 prep position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreL1 = m_graphCommand.new GraphCommandNode(
            "ScoreL1",
            new PrintCommand(
                    "Get into final position and wait for the score command from driver"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreL2 = m_graphCommand.new GraphCommandNode(
            "ScoreL2",
            new PrintCommand(
                    "Get into final position and wait for the score command from driver"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreL3 = m_graphCommand.new GraphCommandNode(
            "ScoreL3",
            new PrintCommand(
                    "Get into final position and wait for the score command from driver"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreL4 = m_graphCommand.new GraphCommandNode(
            "ScoreL4",
            new PrintCommand(
                    "Get into final position and wait for the score command from driver"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l1PostScore = m_graphCommand.new GraphCommandNode(
            "L1PostScore",
            new PrintCommand("Get into safe position before going down to safe travel"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l2PostScore = m_graphCommand.new GraphCommandNode(
            "L2PostScore",
            new PrintCommand("Get into safe position before going down to safe travel"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l3PostScore = m_graphCommand.new GraphCommandNode(
            "L3PostScore",
            new PrintCommand("Get into safe position before going down to safe travel"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode l4PostScore = m_graphCommand.new GraphCommandNode(
            "L4PostScore",
            new PrintCommand("Get into safe position before going down to safe travel"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode prepAlgaeIntake = m_graphCommand.new GraphCommandNode(
            "PrepAlgaeIntake",
            new PrintCommand("Get into position to intake algae"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode prepAlgaeL2 = m_graphCommand.new GraphCommandNode(
            "PrepAlgaeL2",
            new PrintCommand("Final position before intake algaeL2"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode prepAlgaeL3 = m_graphCommand.new GraphCommandNode(
            "PrepAlgaeL3",
            new PrintCommand("Final position before intake algaeL3"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode safeAlgaeTravel = m_graphCommand.new GraphCommandNode(
            "SafeAlgaeTravel",
            new PrintCommand("Get into safe travel with algae position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreProssesor = m_graphCommand.new GraphCommandNode(
            "ScoreProssesor",
            new PrintCommand("Basicaly same as safe travel with algae postion"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode prepScoreBarge = m_graphCommand.new GraphCommandNode(
            "PrepScoreBarge",
            new PrintCommand("Get into position to score into barge"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode scoreBarge = m_graphCommand.new GraphCommandNode(
            "ScoreBarge",
            new PrintCommand("Final score barge position and wait for driver to hit score"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode climbPrep = m_graphCommand.new GraphCommandNode(
            "ClimbPrep",
            new PrintCommand(
                    "Get into position to preapare to engage climber out, after which it is enaged"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode climbReady = m_graphCommand.new GraphCommandNode(
            "ClimbReady",
            new PrintCommand(
                    "Once climber out robot is waiting for driver to hit climb command"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    private final GraphCommandNode cancelled = m_graphCommand.new GraphCommandNode(
            "Cancelled",
            new PrintCommand(
                    "Should cancel all of the running commands, stop the robot and get everything into safe travel position"),
            new PrintCommand("Nothing"),
            new PrintCommand("Nothing"));

    // ----- General state variables -----
    private EndEffector endEffector = EndEffector.HasCoral;
    private ElevatorManipulatorState prevElevManiState = ElevatorManipulatorState.StartPosition;
    private ElevatorManipulatorState elevManiState = ElevatorManipulatorState.StartPosition;
    private ElevatorManipulatorState goalElevManiState = ElevatorManipulatorState.StartPosition;
    private boolean elevManiTransitioning = false;
    private boolean isDisabled = false;

    /**
     * Creates the manager and wires up the minimal graph used for state
     * transitions.  The GraphCommand is registered as the default command so it
     * will be scheduled automatically when the subsystem is idle.
     */
    public ArmStateManager(DifferentialSubsystem diff,
                           ElevatorSubsystem elevator,
                           DriveSubsystem drive,
                           ManipulatorSubsystem manipulator,
                           PoseEstimatorSubsystem pose,
                           LEDUtility ledUtility,
                           XboxController driverController) {
        this.diff = diff;
        this.elevator = elevator;
        this.drive = drive;
        this.manipulator = manipulator;
        this.pose = pose;
        this.ledUtility = ledUtility;
        this.driverController = driverController;

        // Setup the full graph describing arm transitions.  This mirrors the
        // original layout.
        m_graphCommand.setGraphRootNode(startPosition);
        startPosition.AddNode(safeCoralTravel, 1.0);
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

        // Register and initialize the graph command so it runs continuously.
        m_graphCommand.addRequirements(this);
        setDefaultCommand(m_graphCommand);
        m_graphCommand.setCurrentNode(startPosition);
        m_graphCommand.initialize();
    }

    // ----------- Goal management -----------
    public void setElevManiGoal(ElevatorManipulatorState newState) {
        goalElevManiState = newState;
        elevManiTransitioning = true;
    }

    public Command setGoalElevManiCommand(ElevatorManipulatorState newGoal) {
        return Commands.runOnce(() -> setElevManiGoal(newGoal));
    }

    public ElevatorManipulatorState getGoalElevManiState() {
        return goalElevManiState;
    }

    // ----------- Current state management -----------
    public void setCurrentElevManiState(ElevatorManipulatorState curState) {
        elevManiTransitioning = false;
        prevElevManiState = elevManiState;
        elevManiState = curState;
    }

    public Command setCurrentElevManiStateCommand(ElevatorManipulatorState curState) {
        return Commands.runOnce(() -> setCurrentElevManiState(curState));
    }

    public ElevatorManipulatorState getCurrentElevManiState() {
        return elevManiState;
    }

    // ----------- Goal checks -----------
    public void reachedElevManiGoal() {
        goalElevManiState = elevManiState;
    }

    public Command reachedElevManiGoalCommand() {
        return Commands.runOnce(this::reachedElevManiGoal);
    }

    public boolean atElevManiGoal() {
        return elevManiState == goalElevManiState;
    }

    // ----------- Utility helpers -----------
    public boolean hasCoral() {
        return endEffector == EndEffector.HasCoral;
    }

    public boolean isDisabled() {
        return isDisabled;
    }

    public void setDisabled(boolean disabled) {
        isDisabled = disabled;
    }

    // ----------- Triggers -----------
    public Trigger hasCoralTrigger() {
        return manipulator.hasCoralTrigger();
    }

    public Trigger hasAlgaeTrigger() {
        return manipulator.hasAlgaeTrigger();
    }

    public Trigger atElevManiTarget() {
        return new Trigger(() -> elevator.atPosition() &&
                diff.atExtenstionSetpoint() &&
                diff.atRotationSetpoint() &&
                atElevManiGoal());
    }

    public boolean atAlgaePosition() {
        return elevManiState == ElevatorManipulatorState.AlgaeL2 ||
               elevManiState == ElevatorManipulatorState.AlgaeL3;
    }

    public Trigger atAlgaePositionTrigger() {
        return new Trigger(this::atAlgaePosition);
    }

    /**
     * Basic elevator/manipulator state machine. Only a subset of the original
     * transitions are included but the structure mirrors the original code so
     * that future work can expand it.
     */
    public void process() {
        switch (goalElevManiState) {
            case StartPosition:
                setCurrentElevManiState(ElevatorManipulatorState.StartPosition);
                if (!isDisabled() && getCurrentElevManiState() == ElevatorManipulatorState.StartPosition) {
                    setElevManiGoal(ElevatorManipulatorState.SafeCoralTravel);
                }
                break;
            case IntakeCoral:
                m_graphCommand.setTargetNode(intakeCoral);
                break;
            case L1:
                m_graphCommand.setTargetNode(scoreL1);
                break;
            case L2:
                m_graphCommand.setTargetNode(scoreL2);
                break;
            case L3:
                m_graphCommand.setTargetNode(scoreL3);
                break;
            case L4:
                m_graphCommand.setTargetNode(scoreL4);
                break;
            case SafeCoralTravel:
                m_graphCommand.setTargetNode(safeCoralTravel);
                if (!hasCoral() && atElevManiGoal()) {
                    setElevManiGoal(ElevatorManipulatorState.IntakeCoral);
                }
                break;
            default:
                // Other states are not yet implemented in this standalone manager
                break;
        }
    }

    @Override
    public void periodic() {
        // Run the internal state machine each loop so the default command can
        // react to changes in desired state.
        process();
    }
}

