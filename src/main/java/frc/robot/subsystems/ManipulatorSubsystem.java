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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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

    // Simulation members
    private SparkFlexSim manipSim;
    private SparkRelativeEncoderSim encoderSim;
    private SparkLimitSwitchSim limitSwitchSim;

    private final Timer coralIntakeTimer = new Timer();

    Debouncer coralDebounce = new Debouncer(0.05);

    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private FlytLogger manipDash = new FlytLogger("Manipulator");

    public ManipulatorSubsystem (){
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

        manipDash.addDoublePublisher("Motor Pos", true, this::getMotorPos);
        manipDash.addBoolPublisher("Got Coral", false, this::hasCoral);
        manipDash.addBoolPublisher("Got Algae", true, this::hasAlgae);
        manipDash.addDoublePublisher("Manip Current", true, this::getOutputCurrent);
        manipDash.addBoolPublisher("Sees Coral", true, this::seesCoral);
        manipDash.addDoublePublisher("RPM", true, this::getRPM);
    }

    /**
     * Test-oriented constructor allowing hardware dependencies to be supplied
     * externally for unit testing.
     */
    public ManipulatorSubsystem(
            SparkFlex manipulatorMotor,
            SparkAbsoluteEncoder manipulatorAbsEncoder,
            RelativeEncoder relEncoder,
            SparkLimitSwitch manipulatorLimitSwitch,
            Debouncer coralDebounce) {
        this.manipulatorMotor = manipulatorMotor;
        this.manipulatorAbsEncoder = manipulatorAbsEncoder;
        this.relEncoder = relEncoder;
        this.manipulatorLimitSwitch = manipulatorLimitSwitch;
        this.coralDebounce = coralDebounce;
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
        if (RobotBase.isSimulation() && manipSim != null) {
            return manipSim.getMotorCurrent();
        }
        return manipulatorMotor.getOutputCurrent();
    }

    /**
     * Returns the current draw of the manipulator motor.
     *
     * @return Manipulator motor output current.
     */
    public double getCurrentDraw() {
        return getOutputCurrent();
    }

    public double getMotorPos() {
        if (RobotBase.isSimulation() && encoderSim != null) {
            return encoderSim.getPosition();
        }
        return relEncoder.getPosition();
    }

    public double getRPM() {
        if (RobotBase.isSimulation() && encoderSim != null) {
            return encoderSim.getVelocity();
        }
        return relEncoder.getVelocity();
    }

    public void resetMotorPos() {
        if (RobotBase.isSimulation() && encoderSim != null) {
            encoderSim.setPosition(0);
        } else {
            relEncoder.setPosition(0);
        }
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
        boolean pressed;
        if (RobotBase.isSimulation() && limitSwitchSim != null) {
            pressed = limitSwitchSim.getPressed();
        } else {
            pressed = manipulatorLimitSwitch.isPressed();
        }
        return coralDebounce.calculate(pressed);
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

            // Time-based simulation of coral intake
            if (!hasCoral && Math.abs(manipulatorMotor.get()) > 0.05) {
                if (!coralIntakeTimer.isRunning()) {
                    coralIntakeTimer.restart();
                    limitSwitchSim.setPressed(true);
                } else if (coralIntakeTimer.hasElapsed(0.1)) {
                    hasCoral = true;
                    limitSwitchSim.setPressed(false);
                    coralIntakeTimer.stop();
                }
            } else {
                limitSwitchSim.setPressed(false);
                coralIntakeTimer.stop();
            }
        }
    }

}
