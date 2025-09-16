package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;
import frc.utils.FlytDashboardV2;

public class StateMachineCoardinator extends SubsystemBase {

  // attributes
  private boolean hasAlgae = false;
  private boolean isDisabled = true;
  private boolean isAuto = false;
  private boolean reefAligned = false;
  private ControllerProfile currentProfile = null;
  private RobotState goalState = null;

  private DriveStateMachine driveSM;
  private ManipulatorStateMachine manipulatorSM;
  private FlytDashboardV2 dashboard = new FlytDashboardV2("Coardinator");

  public enum RobotState {
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
    RESET;
  }

  public enum ControllerProfile {
    DEFAULT_CORAL,
    ALGAE,
    MANUAL;
  }

  public StateMachineCoardinator(
      ManipulatorStateMachine m_manipulatorStateMachine, DriveStateMachine m_driveStateMachine) {
    manipulatorSM = m_manipulatorStateMachine;
    driveSM = m_driveStateMachine;
    currentProfile = ControllerProfile.DEFAULT_CORAL;
    goalState = RobotState.START_POSITION;
  }

  // Triggers

  /*
   * Setters
   */
  public void setRightScore(boolean isRight) {
    driveSM.setRightScore(isRight);
  }

  public void score(boolean canScore) {
    manipulatorSM.canScore(canScore);
  }

  public void robotDisabled(boolean disabled) {
    isDisabled = disabled;
  }

  public void robotAuto(boolean auto) {
    isAuto = auto;
  }

  // if driver pushes trigger to make robot align, set var to true
  public void setReefAlign(boolean align) {
    reefAligned = align;
  }

  /*
   * Getters
   */
  public boolean getRightScore() {
    return driveSM.getRightScore();
  }

  /**
   * Check which reef to align to
   *
   * @return
   */
  public boolean getReefAlign() {
    return reefAligned;
  }

  /**
   * Find current Controller profile
   *
   * @return
   */
  public ControllerProfile getCurrentControllerProfile() {
    return currentProfile;
  }

  /**
   * Check if we have coral inside the robot
   *
   * @return
   */
  public boolean gethasCoral() {
    return manipulatorSM.getHasCoral();
  }

  public boolean getManipulatorAtGoalState() {
    return manipulatorSM.atGoalState();
  }

  // set elevator goal
  private void setElevatorManipulatorGoal(ElevatorManipulatorState m_elevmanistate) {
    manipulatorSM.setElevatorManipulatorCommand(m_elevmanistate);
  }

  // set drive goal
  private void setDriveGoal(DriveState m_drivestate) {
    driveSM.setDriveCommand(m_drivestate);
  }

  /**
   * Set Global state goal
   *
   * @param state
   */
  public void setRobotGoal(RobotState state) {
    // safety checks before requesting state changes
    goalState = state;
    // state change
    switch (state) {
      case START_POSITION:
        setElevatorManipulatorGoal(ElevatorManipulatorState.START_POSITION);
        setDriveGoal(DriveState.CANCELLED);
        break;
      case SAFE_CORAL_TRAVEL:
        setElevatorManipulatorGoal(ElevatorManipulatorState.SAFE_CORAL_TRAVEL);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case INTAKE_CORAL:
        // setElevatorManipulatorGoal(ElevatorManipulatorState.INTAKE_CORAL);
        setDriveGoal(DriveState.CORAL_STATION);
        break;
      case L1:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L1);
        break;
      case L2:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L2);
        break;
      case L3:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L3);
        break;
      case L4:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L4);

        break;
      case ALGAE_L2:
        setElevatorManipulatorGoal(ElevatorManipulatorState.ALGAE_L2);
        break;
      case ALGAE_L3:
        setElevatorManipulatorGoal(ElevatorManipulatorState.ALGAE_L3);
        break;
      case PROCESSOR:
        setElevatorManipulatorGoal(ElevatorManipulatorState.PROCESSOR);
        setDriveGoal(DriveState.PROCESSOR_RELATIVE);
        break;
      case BARGE:
        setElevatorManipulatorGoal(ElevatorManipulatorState.BARGE);
        setDriveGoal(DriveState.BARGE_RELATIVE);
        break;
      case CLIMB:
        setElevatorManipulatorGoal(ElevatorManipulatorState.CLIMB);
        setDriveGoal(DriveState.CLIMB_RELATIVE);
        break;
      case MANUAL:
        setElevatorManipulatorGoal(ElevatorManipulatorState.MANUAL);
        setDriveGoal(DriveState.MANUAL);
        break;
      case RESET:
        setElevatorManipulatorGoal(ElevatorManipulatorState.RESET);
        setDriveGoal(DriveState.CANCELLED);
        break;
    }
  }

  /** Handle automatic state transitions based on current states and state conditions */
  private void handleAutomaticTransitions() {

    // robot is not disabled, and driver station is enabled
    if (!isDisabled && DriverStation.isEnabled()) {

      // this should only run once and at the beggining to automaticaly take robot out of the start
      // position
      if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.START_POSITION) {
        setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
      }

      // run following command only if manipulator state machine is completed its moves and at final
      // state
      if (manipulatorSM.atGoalState()
          && !manipulatorSM.isTransitioning()
          && !manipulatorSM.reachGoalStateFailed()) {

        if (gethasCoral()) {
          if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL) {
            setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
            manipulatorSM.atGoalState(false);
          }
        } else {
          if (getCurrentControllerProfile() == ControllerProfile.DEFAULT_CORAL) {
            setRobotGoal(RobotState.INTAKE_CORAL);
            manipulatorSM.atGoalState(false);
          }
        }
      }
    }
  }

  @Override
  public void periodic() {

    if (driveSM.getCurrentState() == DriveState.REEF_RELATIVE && getReefAlign()) {
      driveSM.setDriveCommand(DriveState.REEF_ALIGN);
    } else if (driveSM.getCurrentState() == DriveState.REEF_ALIGN && !getReefAlign()) {
      driveSM.setDriveCommand(DriveState.REEF_RELATIVE);
    }

    dashboard.putString("Current State", goalState.toString());
    dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
    dashboard.putBoolean("RobotDisabled", isDisabled);
    dashboard.putBoolean("Reef Align", getReefAlign());
    dashboard.putString("Controller Profile", getCurrentControllerProfile().toString());
    dashboard.putBoolean("Has Coral", gethasCoral());
    // This method will be called once per scheduler run
    // handleAutomaticTransitions();
  }
}
