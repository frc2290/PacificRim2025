package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;

public class ManipulatorSubsystem extends SubsystemBase {

    SparkMax manipulatorMotor;
    SparkMaxConfig manipulatorConfig = new SparkMaxConfig();

    SparkAbsoluteEncoder manipulatorAbsEncoder;
    RelativeEncoder relEncoder;

    FlytLogger manipDash = new FlytLogger("Manipulator");

    public ManipulatorSubsystem (){
        manipulatorMotor = new SparkMax(Manipulator.kManipulatorMotorId, MotorType.kBrushless);

        manipulatorConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(40);
        manipulatorMotor.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        manipulatorAbsEncoder = manipulatorMotor.getAbsoluteEncoder();
        relEncoder = manipulatorMotor.getEncoder();

        manipDash.addDoublePublisher("Motor Pos", true, () -> getMotorPos());
        manipDash.addBoolPublisher("Got Coral", false, () -> gotCoral());
        manipDash.addDoublePublisher("Manip Current", true, () -> manipulatorMotor.getOutputCurrent());
    }
    
    public void intake(double power){
        manipulatorMotor.set(power);
    }

    public double getWristPos(){
        return manipulatorAbsEncoder.getPosition();

    }

    public double getMotorPos() {
        return relEncoder.getPosition();
    }

    public void resetMotorPos() {
        relEncoder.setPosition(0);
    }

    public boolean gotCoral() {
        return manipulatorAbsEncoder.getVelocity() < 10;
        //return true;
    }

    @Override
    public void periodic() {
        manipDash.update(Constants.debugMode);
    }
    
}
