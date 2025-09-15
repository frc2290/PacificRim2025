package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;

public class StateMachineCoardinator extends SubsystemBase {

        //attributes
        private boolean hasCoral = false;
        private boolean hasAlgae = false;
        private boolean isDisabled = true;
        private boolean isAuto = false;
        private boolean isRightBranch = true;
        private boolean reefAligned = false;
        private ControllerProfile currentProfile = null;

        private DriveStateMachine driveSM;
        private ManipulatorStateMachine manipulatorSM;

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
        
        public StateMachineCoardinator(ManipulatorStateMachine m_manipulatorStateMachine, DriveStateMachine m_driveStateMachine) {
                manipulatorSM = m_manipulatorStateMachine;
                driveSM = m_driveStateMachine;
                currentProfile = ControllerProfile.DEFAULT_CORAL;
                
                //since we always have coral at start
                setHasCoral(true);
        }
    
        /*
         * Setters
         */
        public void setRightScore(boolean isRight){
                isRightBranch = isRight;
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

        public void setHasCoral(boolean coral){
                hasCoral = coral;
                manipulatorSM.setHasCoral(coral);
        }

        /*
         * Getters
         */
        public boolean getRightScore(){
                return isRightBranch;
        }

        public boolean getReefAlign(){
                return reefAligned;
        }

        public ControllerProfile getCurrentControllerProfile(){
                return currentProfile;
        }

        //set elevator goal
        private void setElevatorManipulatorGoal(ElevatorManipulatorState m_elevmanistate){
                manipulatorSM.setElevatorManipulatorCommand(m_elevmanistate);
        }

        //set drive goal
        private void setDriveGoal(DriveState m_drivestate){
                driveSM.setDriveCommand(m_drivestate);
        }

        //main state change function
        public void setRobotGoal(RobotState state){
                //safety checks before requesting state changes

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
        
        @Override
        public void periodic() {
                // This method will be called once per scheduler run
        } 
}
