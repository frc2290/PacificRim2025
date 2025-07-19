package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class ManipulatorSubsystem extends SubsystemBase {

    private SparkFlex manipulatorMotor;
    private SparkFlexConfig manipulatorConfig = new SparkFlexConfig();

    private SparkAbsoluteEncoder manipulatorAbsEncoder;
    private RelativeEncoder relEncoder;

    private SparkLimitSwitch manipulatorLimitSwitch;

    Debouncer coralDebounce = new Debouncer(0.05);

    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private FlytLogger manipDash = new FlytLogger("Manipulator");

    public ManipulatorSubsystem (){
        manipulatorMotor = new SparkFlex(Manipulator.kManipulatorMotorId, MotorType.kBrushless);

        manipulatorConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(40);
        manipulatorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        manipulatorMotor.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        manipulatorAbsEncoder = manipulatorMotor.getAbsoluteEncoder();
        relEncoder = manipulatorMotor.getEncoder();
        manipulatorLimitSwitch = manipulatorMotor.getForwardLimitSwitch();

        manipDash.addDoublePublisher("Motor Pos", true, () -> getMotorPos());
        manipDash.addBoolPublisher("Got Coral", false, () -> hasCoral());
        manipDash.addDoublePublisher("Manip Current", true, () -> manipulatorMotor.getOutputCurrent());
        manipDash.addBoolPublisher("Sees Coral", true, () -> seesCoral());
    }
    
    public void intake(double power){
        manipulatorMotor.set(power);
    }

    public Command runIntake(double power) {
        return Commands.runOnce(() -> intake(power));
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

    public boolean seesCoral() {
        return coralDebounce.calculate(manipulatorLimitSwitch.isPressed());
    }

    @Override
    public void periodic() {
        manipDash.update(Constants.debugMode);
    }
    
}
