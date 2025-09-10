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
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkLimitSwitchSim;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.FieldConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private SparkFlex manipulatorMotor;
    private SparkFlexConfig manipulatorConfig = new SparkFlexConfig();

    private SparkAbsoluteEncoder manipulatorAbsEncoder;
    private RelativeEncoder relEncoder;

    private SparkLimitSwitch manipulatorLimitSwitch;

    // Reference to drivetrain for pose checks in simulation
    private final DriveSubsystem drive;

    // Simulation members
    private SparkFlexSim manipSim;
    private SparkRelativeEncoderSim encoderSim;
    private SparkLimitSwitchSim limitSwitchSim;

    Debouncer coralDebounce = new Debouncer(0.05);

    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private FlytLogger manipDash = new FlytLogger("Manipulator");

    public ManipulatorSubsystem (DriveSubsystem drive){
        this.drive = drive;
        manipulatorMotor = new SparkFlex(Manipulator.kManipulatorMotorId, MotorType.kBrushless);

        manipulatorConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(50);
        manipulatorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        manipulatorMotor.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        manipulatorAbsEncoder = manipulatorMotor.getAbsoluteEncoder();
        relEncoder = manipulatorMotor.getEncoder();
        manipulatorLimitSwitch = manipulatorMotor.getForwardLimitSwitch();

        if (RobotBase.isSimulation()) {
            manipSim = new SparkFlexSim(manipulatorMotor, DCMotor.getNEO(1));
            encoderSim = manipSim.getRelativeEncoderSim();
            limitSwitchSim = new SparkLimitSwitchSim(manipulatorMotor, true);
        }

        manipDash.addDoublePublisher("Motor Pos", true, () -> getMotorPos());
        manipDash.addBoolPublisher("Got Coral", false, () -> hasCoral());
        manipDash.addBoolPublisher("Got Algae", true, () -> hasAlgae());
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

    public Trigger hasAlgaeTrigger() {
        return new Trigger(() -> hasAlgae());
    }

    public boolean seesCoral() {
        return coralDebounce.calculate(manipulatorLimitSwitch.isPressed());
    }

    @Override
    public void periodic() {
        manipDash.update(Constants.debugMode);
    }

    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation() && manipSim != null) {
            manipSim.iterate(manipulatorMotor.getAppliedOutput(), 0.02, manipulatorMotor.getBusVoltage());
            encoderSim.setPosition(manipSim.getPosition());
            encoderSim.setVelocity(manipSim.getVelocity());

            // Use robot pose to determine game-piece acquisition
            if (drive != null && drive.getField() != null) {
                Pose2d pose = drive.getField().getRobotPose();

                boolean atCoralStation =
                        pose.getTranslation().getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation())
                                        <= Constants.SIM_INTAKE_TOLERANCE_METERS
                                || pose.getTranslation().getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation())
                                        <= Constants.SIM_INTAKE_TOLERANCE_METERS;

                boolean atReef =
                        pose.getTranslation().getDistance(FieldConstants.Reef.center)
                                <= Constants.SIM_INTAKE_TOLERANCE_METERS;

                if (!hasCoral && atCoralStation) {
                    hasCoral = true;
                    limitSwitchSim.setPressed(true);
                } else if (!hasAlgae && atReef) {
                    hasAlgae = true;
                }
            }

            SmartDashboard.putNumber("Manipulator RPM", relEncoder.getVelocity());
            SmartDashboard.putBoolean("Has Coral", hasCoral());
        }
    }

}
