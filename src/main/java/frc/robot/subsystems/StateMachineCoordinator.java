// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;
import frc.utils.FlytDashboardV2;
import frc.utils.LEDEffects;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.LEDUtility;

/** Bridges the drive and manipulator state machines while updating LEDs for feedback. */
public class StateMachineCoordinator extends SubsystemBase {

  // attributes
  /** Mirrors the robot enable state so LEDs can be updated appropriately. */
  private boolean isDisabled = true;

  /** Indicates whether autonomous mode is active. */
  private boolean isAuto = false;

  /** True while the driver is holding the reef alignment trigger. */
  private boolean reefAligned = false;

  /** Active controller profile that dictates button bindings. */
  private ControllerProfile currentProfile = ControllerProfile.DEFAULT_CORAL;

  /** Latest global goalState requested by the driver or auto routine. */
  private RobotState goalState = RobotState.START_POSITION;

  /** Active algae intake goal to revisit when algae is not secured. */
  private RobotState algaeIntakeGoal = RobotState.ALGAE_L2;

  private DriveStateMachine driveSM;
  private ManipulatorStateMachine manipulatorSM;
  private FlytDashboardV2 dashboard = new FlytDashboardV2("Coordinator");

  /** Helper for setting global LED patterns based on robot state. */
  private LEDUtility ledUtility;

  public enum RobotState {
    START_POSITION,
    SAFE_CORAL_TRANSPORT,
    SAFE_ALGAE_TRANSPORT,
    INTAKE_CORAL,
    L1,
    L2,
    L3,
    L4,
    ALGAE_L2,
    ALGAE_L3,
    PROCESSOR,
    BARGE,
    CLIMB_READY,
    CLIMB,
    CLIMB_ABORT,
    MANUAL,
    RESET;
  }

  public enum ControllerProfile {
    DEFAULT_CORAL,
    ALGAE,
    MANUAL,
    Climb;
  }

  public StateMachineCoordinator(
      ManipulatorStateMachine m_manipulatorStateMachine,
      DriveStateMachine m_driveStateMachine,
      LEDUtility m_ledUtility) {
    manipulatorSM = m_manipulatorStateMachine;
    driveSM = m_driveStateMachine;
    ledUtility = m_ledUtility;
  }

  // Triggers

/**
 * Sets which side of the reef to score on.
 * @param isRight
 */
  public void setRightScore(boolean isRight) {
    driveSM.setRightScore(isRight);
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
   * Checks which reef to align to.
   *
   * @return
   */
  public boolean getReefAlign() {
    return reefAligned;
  }

  /**
   * Returns the current controller profile.
   *
   * @return
   */
  public ControllerProfile getCurrentControllerProfile() {
    return currentProfile;
  }

  public void setControllerProfile(ControllerProfile profile) {
    // Drivers can swap profiles to expose different button mappings and LED themes.
    currentProfile = profile;
    // Prevent stale transitions from leaving the robot in a weird previous state if it gets stuck.
  }

  /**
   * Checks if the robot currently holds coral.
   *
   * @return
   */
  public boolean gethasCoral() {
    return manipulatorSM.getHasCoral();
  }

  public boolean gethasAlgae() {
    return manipulatorSM.getHasAlgae();
  }

  public boolean getManipulatorAtGoalState() {
    return manipulatorSM.atGoalState();
  }

  // Set elevator goal.
  private void setElevatorManipulatorGoal(ElevatorManipulatorState m_elevmanistate) {
    manipulatorSM.setElevatorManipulatorCommand(m_elevmanistate);
  }

  // Set drive goal.
  private void setDriveGoal(DriveState m_drivestate) {
    driveSM.setDriveCommand(m_drivestate);
  }

  /**
   * Sets the global state goal.
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
        case SAFE_CORAL_TRANSPORT:
        setElevatorManipulatorGoal(ElevatorManipulatorState.SAFE_CORAL_TRAVEL);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case SAFE_ALGAE_TRANSPORT:
        setElevatorManipulatorGoal(ElevatorManipulatorState.SAFE_ALGAE_TRAVEL);
        setDriveGoal(DriveState.PROCESSOR_RELATIVE);
        break;
      case INTAKE_CORAL:
        setElevatorManipulatorGoal(ElevatorManipulatorState.INTAKE_CORAL);
        setDriveGoal(DriveState.CORAL_STATION);
        break;
      case L1:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L1);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case L2:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L2);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case L3:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L3);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case L4:
        setElevatorManipulatorGoal(ElevatorManipulatorState.L4);
        setDriveGoal(DriveState.REEF_RELATIVE);

        break;
      case ALGAE_L2:
        setElevatorManipulatorGoal(ElevatorManipulatorState.ALGAE_L2);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case ALGAE_L3:
        setElevatorManipulatorGoal(ElevatorManipulatorState.ALGAE_L3);
        setDriveGoal(DriveState.REEF_RELATIVE);
        break;
      case PROCESSOR:
        setElevatorManipulatorGoal(ElevatorManipulatorState.PROCESSOR);
        setDriveGoal(DriveState.PROCESSOR_RELATIVE);
        break;
      case BARGE:
        setElevatorManipulatorGoal(ElevatorManipulatorState.BARGE);
        setDriveGoal(DriveState.BARGE_RELATIVE);
        break;
      case CLIMB_READY:
        setElevatorManipulatorGoal(ElevatorManipulatorState.CLIMB_READY);
        setDriveGoal(DriveState.MANUAL);
        break;
      case CLIMB:
        setElevatorManipulatorGoal(ElevatorManipulatorState.CLIMBED);
        setDriveGoal(DriveState.MANUAL);
        break;
      case CLIMB_ABORT:
        setElevatorManipulatorGoal(ElevatorManipulatorState.CLIMB_ABORT);
        setDriveGoal(DriveState.MANUAL);
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

  public void requestToScore(boolean score) {
    manipulatorSM.score(score);
  }

  /** Handles automatic state transitions based on current subsystem states. */
  private void handleAutomaticTransitions() {

    //tell manipulator when drive is at position
    manipulatorSM.setDriveAtPose(driveSM.atPosition());

    manipulatorSM.isAuto(isAuto);
   

    // Only process automatic transitions while the robot is enabled and not in manual mode.
    if (!isAuto &&!isDisabled && DriverStation.isEnabled() && (getCurrentControllerProfile() != ControllerProfile.MANUAL) && getCurrentControllerProfile() != ControllerProfile.ALGAE) {

      // Automatically leave the start position on the first iteration so the robot exits the start
      // pose immediately.
      // This prevents lingering in the pre-match configuration.
      if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.START_POSITION) {
        setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT);
      }

      // Manage Reef_ALIGN transitions based on the alignment trigger.
      if (driveSM.getCurrentState() == DriveState.REEF_RELATIVE && getReefAlign()) {
        driveSM.setDriveCommand(DriveState.REEF_ALIGN);
      } else if (driveSM.getCurrentState() == DriveState.REEF_ALIGN && !getReefAlign()) {
        driveSM.setDriveCommand(DriveState.REEF_RELATIVE);
      }

      if (!manipulatorSM.isTransitioning() && ControllerProfile.DEFAULT_CORAL == getCurrentControllerProfile() && goalState != RobotState.PROCESSOR) {
 
        if (gethasCoral()) {
          if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL) {
            setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT);
          }

        } else {
          if (getCurrentControllerProfile() == ControllerProfile.DEFAULT_CORAL) {
            setRobotGoal(RobotState.INTAKE_CORAL);
          }
        }
      }

      if (!manipulatorSM.isTransitioning() && ControllerProfile.ALGAE == getCurrentControllerProfile()) {

        boolean hasAlgaeNow = manipulatorSM.getHasAlgae();
        ElevatorManipulatorState currentManipulatorState = manipulatorSM.getCurrentState();

        if (hasAlgaeNow){

          if ((currentManipulatorState == ElevatorManipulatorState.ALGAE_L2 || currentManipulatorState == ElevatorManipulatorState.ALGAE_L3) && goalState != RobotState.SAFE_ALGAE_TRANSPORT && goalState != RobotState.BARGE && goalState != RobotState.PROCESSOR) {
                   setRobotGoal(RobotState.SAFE_ALGAE_TRANSPORT);
          }

        }else{
          if (goalState == RobotState.BARGE || goalState == RobotState.PROCESSOR) {
            setRobotGoal(RobotState.SAFE_ALGAE_TRANSPORT);
   }
        }

      //   if (hasAlgaeNow) {

      //     if ((currentManipulatorState == ElevatorManipulatorState.ALGAE_L2 || currentManipulatorState == ElevatorManipulatorState.ALGAE_L3) && goalState != RobotState.SAFE_ALGAE_TRANSPORT && goalState != RobotState.BARGE && goalState != RobotState.PROCESSOR) {
      //       setRobotGoal(RobotState.SAFE_ALGAE_TRANSPORT);
      //     }

      //   } else if ((goalState == RobotState.SAFE_ALGAE_TRANSPORT || currentManipulatorState == ElevatorManipulatorState.SAFE_ALGAE_TRAVEL) && goalState != algaeIntakeGoal) {
      //     setRobotGoal(algaeIntakeGoal);

      //   } else if (currentManipulatorState != ElevatorManipulatorState.ALGAE_L2 && currentManipulatorState != ElevatorManipulatorState.ALGAE_L3 && goalState != algaeIntakeGoal) {
      //     setRobotGoal(algaeIntakeGoal);
      //   }
      }


      // Successful commands mark the manipulator goal as reached.
      // When the driver calls a new goal, the graph command moves toward the next target
      // node and clears the goal flag while it transitions.
      if (manipulatorSM.isTransitioning()) {
        manipulatorSM.setatGoalState(false);
      }
    }

    if (manipulatorSM.atGoalState()) {
      if (isAuto) {
        ledUtility.setAll(LEDEffect.PULSE, LEDEffects.flytBlue);
      } else if (isDisabled) {
        ledUtility.getStrip("TopLeft").setEffect(LEDEffect.NAVLIGHTS, Color.kRed);
        ledUtility.getStrip("TopRight").setEffect(LEDEffect.NAVLIGHTS, Color.kGreen);
        ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
        ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
      } else if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL
          && !gethasCoral()
          && driveSM.getCurrentState() == DriveState.CORAL_STATION) {
        ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, Color.kGreen);
        ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, Color.kGreen);
      }
      } else if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL && manipulatorSM.getHasCoral() &&
       driveSM.getCurrentState() == DriveState.CORAL_STATION) {
           ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
           ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
       } else if (getCurrentControllerProfile() == ControllerProfile.MANUAL) {
          ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kRed);
          ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kRed);
          ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
          ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
      } else if (driveSM.getCurrentState() == DriveState.REEF_RELATIVE) {
          if (getRightScore()) {
             ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kPurple);
             ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kPurple);
         } else {
             ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kYellow);
             ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kYellow);
          }
          ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
          ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
      } else if (driveSM.getCurrentState() == DriveState.REEF_ALIGN) {
          ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
          ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
      } else if (driveSM.atPosition() && manipulatorSM.atGoalState()) {
          ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
          ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
      }
    }
  
  @Override
  public void periodic() {

    dashboard.putString("Current State", goalState.toString());
    dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
    dashboard.putBoolean("RobotDisabled", isDisabled);
    dashboard.putBoolean("Reef Align", getReefAlign());
    dashboard.putString("Controller Profile", getCurrentControllerProfile().toString());
    dashboard.putBoolean("Has Coral", gethasCoral());
    dashboard.putBoolean("Has Algae", gethasAlgae());
    // This method will be called once per scheduler run
    handleAutomaticTransitions();
  }
}

