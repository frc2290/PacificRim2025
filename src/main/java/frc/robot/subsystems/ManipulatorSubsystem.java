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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class ManipulatorSubsystem extends SubsystemBase {

    private SparkMax manipulatorMotor;
    private SparkMaxConfig manipulatorConfig = new SparkMaxConfig();

    private SparkAbsoluteEncoder manipulatorAbsEncoder;
    private RelativeEncoder relEncoder;

    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private FlytLogger manipDash = new FlytLogger("Manipulator");

    public ManipulatorSubsystem (){
        manipulatorMotor = new SparkMax(Manipulator.kManipulatorMotorId, MotorType.kBrushless);

        manipulatorConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(40);
        manipulatorMotor.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        manipulatorAbsEncoder = manipulatorMotor.getAbsoluteEncoder();
        relEncoder = manipulatorMotor.getEncoder();

        manipDash.addDoublePublisher("Motor Pos", true, () -> getMotorPos());
        manipDash.addBoolPublisher("Got Coral", false, () -> hasCoral());
        manipDash.addDoublePublisher("Manip Current", true, () -> manipulatorMotor.getOutputCurrent());
    }
    
    public void intake(double power){
        manipulatorMotor.set(power);
    }

    public double getWristPos(){
        return manipulatorAbsEncoder.getPosition();

    }

    public double getOutputCurrent() {
        return manipulatorMotor.getOutputCurrent();
    }

    public double getMotorPos() {
        return relEncoder.getPosition();
    }

    public void resetMotorPos() {
        relEncoder.setPosition(0);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setCoral(boolean coral) {
        hasCoral = coral;
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }

    public void setAlgae(boolean algae) {
        hasAlgae = algae;
    }

    public Trigger hasCoralTrigger() {
        return new Trigger(() -> hasCoral());
    }

    @Override
    public void periodic() {
        manipDash.update(Constants.debugMode);
    }
    
}
