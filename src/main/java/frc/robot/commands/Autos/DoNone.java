package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
import frc.robot.subsystems.StateMachineCoordinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/** One piece autonomous that drives to the right branch and scores a single coral. */
public class DoNone extends Command {


  DriveSubsystem drive;

  public DoNone(DriveSubsystem m_drive)
  {
    drive = m_drive;

  }

  @Override
  public void initialize() {
    drive.drive(0,0,0,false);
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}