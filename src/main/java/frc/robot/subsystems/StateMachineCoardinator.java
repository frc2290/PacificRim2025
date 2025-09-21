package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;
import frc.utils.FlytDashboardV2;
import frc.utils.LEDEffects;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.LEDUtility;

/** Bridges the drive and manipulator state machines while updating LEDs for feedback. */
public class StateMachineCoardinator extends SubsystemBase {

        //attributes
        /** Tracks whether the manipulator currently holds algae. */
        private boolean hasAlgae = false;
        /** Mirrors the robot enable state so LEDs can be updated appropriately. */
        private boolean isDisabled = true;
        /** Indicates whether autonomous mode is active. */
        private boolean isAuto = false;
        /** True while the driver is holding the reef alignment trigger. */
        private boolean reefAligned = false;
        /** Active controller profile that dictates button bindings. */
        private ControllerProfile currentProfile = ControllerProfile.DEFAULT_CORAL;
        /** Latest global goal state requested by the driver or auto routine. */
        private RobotState goalState = RobotState.START_POSITION;
        

        private DriveStateMachine driveSM;
        private ManipulatorStateMachine manipulatorSM;
        private FlytDashboardV2 dashboard = new FlytDashboardV2("Coardinator");
        /** Helper for setting global LED patterns based on robot state. */
        private LEDUtility ledUtility;

        public enum RobotState{
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

        public enum ControllerProfile{
                DEFAULT_CORAL,
                ALGAE,
                MANUAL;
        }
        
        public StateMachineCoardinator(ManipulatorStateMachine m_manipulatorStateMachine, DriveStateMachine m_driveStateMachine, LEDUtility m_ledUtility) {
                manipulatorSM = m_manipulatorStateMachine;
                driveSM = m_driveStateMachine;
                ledUtility = m_ledUtility;
                
        }

        //Triggers

        /*
         * Setters
         */
        public void setRightScore(boolean isRight){
                driveSM.setRightScore(isRight);
        }

        public void robotDisabled(boolean disabled){
                isDisabled = disabled;
        }

        public void robotAuto(boolean auto){
                isAuto = auto;
        }

        //if driver pushes trigger to make robot align, set var to true
        public void setReefAlign(boolean align){
                reefAligned = align;
        }

        /*
         * Getters
         */
        public boolean getRightScore(){
                return driveSM.getRightScore();
        }

        /**
         * Check which reef to align to
         * @return
         */
        public boolean getReefAlign(){
                return reefAligned;
        }

        /**
         * Find current Controller profile
         * @return
         */
        public ControllerProfile getCurrentControllerProfile(){
                return currentProfile;
        }

        public void setControllerProfile(ControllerProfile profile){
                // Drivers can swap profiles to expose different button mappings and LED themes.
                currentProfile = profile;
                //also runs a command do robot does not weirdly go into werid previouse state if got stuck
        }
        /**
         * Check if we have coral inside the robot
         * @return
         */
        public boolean gethasCoral(){
                return manipulatorSM.getHasCoral(); 
        }

        public boolean getManipulatorAtGoalState(){
                return manipulatorSM.atGoalState();
        }
        //set elevator goal
        private void setElevatorManipulatorGoal(ElevatorManipulatorState m_elevmanistate){
                manipulatorSM.setElevatorManipulatorCommand(m_elevmanistate);

        }

        //set drive goal
        private void setDriveGoal(DriveState m_drivestate){
                driveSM.setDriveCommand(m_drivestate);
                
        }

        /**
         * Set Global state goal
         * @param state
         */
        public void setRobotGoal(RobotState state){
                //safety checks before requesting state changes
                goalState = state;
                //state change
                switch(state){
                        case START_POSITION:
                                setElevatorManipulatorGoal(ElevatorManipulatorState.START_POSITION);
                                setDriveGoal(DriveState.CANCELLED);
                                break;
                        case SAFE_CORAL_TRAVEL:
                                setElevatorManipulatorGoal(ElevatorManipulatorState.SAFE_CORAL_TRAVEL);
                                setDriveGoal(DriveState.REEF_RELATIVE);
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
                        case CLIMB:
                                setElevatorManipulatorGoal(ElevatorManipulatorState.CLIMB_READY);
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
        
        public void requestToScore(boolean score){
                manipulatorSM.score(score);
        }

    /**
     * Handle automatic state transitions based on current states and state conditions
     */
    private void handleAutomaticTransitions() {

        //robot is not disabled, and driver station is enabled
        if(!isDisabled && DriverStation.isEnabled() && (getCurrentControllerProfile() != ControllerProfile.MANUAL)){

                //this should only run once and at the beggining to automaticaly take robot out of the start position
                if(manipulatorSM.getCurrentState() == ElevatorManipulatorState.START_POSITION){
                        setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
                }
                   
                //Manages Reef_ALIGN
                if(driveSM.getCurrentState() == DriveState.REEF_RELATIVE && getReefAlign()){
                        driveSM.setDriveCommand(DriveState.REEF_ALIGN);
                }else if(driveSM.getCurrentState() == DriveState.REEF_ALIGN && !getReefAlign()){
                        driveSM.setDriveCommand(DriveState.REEF_RELATIVE);
                }
                

                
                 if(!manipulatorSM.isTransitioning()){

                         if(gethasCoral()){
                                // if(manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL){
                                //          setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
                                // }

                         }else{
                                 if(getCurrentControllerProfile() == ControllerProfile.DEFAULT_CORAL){
                                         setRobotGoal(RobotState.INTAKE_CORAL);
                                 }
                         }



                        
                 }


                //Sets elevator state machine ready to score when drivestatemachine is at position (add state check later)
                if(driveSM.atPosition()){
                        manipulatorSM.setreadyToScore(true);
                }else{
                        manipulatorSM.setreadyToScore(false);
                }

                // when command is succesful it will set goalstate for manipulator state machine
                //as true as being reached, when driver call new goal, graph command changes tearget 
                //node and goes into transition thus not at goal state
                if(manipulatorSM.isTransitioning()){
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
                } else if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL && !gethasCoral() && driveSM.getCurrentState() == DriveState.CORAL_STATION) {
                    ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, Color.kGreen);
                    ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, Color.kGreen);}
                // } else if (getCurrentState() == PositionState.IntakePosition && manipulator.hasCoral() && getDriveState() == DriveState.CoralStation) {
                //     ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
                //     ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
                // } else if (!getRotationLock()) {
                //     ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kRed);
                //     ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kRed);
                //     ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                //     ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                // } else if (getDriveState() == DriveState.Teleop) {
                //     if (getRightScore()) {
                //         ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kPurple);
                //         ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kPurple);
                //     } else {
                //         ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kYellow);
                //         ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kYellow);
                //     }
                //     ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                //     ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                // } else if (getDriveState() == DriveState.ReefScoreMove) {
                //     ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
                //     ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
                // } else if (getDriveState() == DriveState.ReefScore) {
                //     ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
                //     ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
                // } 
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
                dashboard.putBoolean("Command Ready to score", manipulatorSM.readyToScore());
                dashboard.putBoolean("Score Request", manipulatorSM.scoreNow());
                // This method will be called once per scheduler run
                handleAutomaticTransitions();
        } 

        
}
