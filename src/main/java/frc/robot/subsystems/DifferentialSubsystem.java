package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class DifferentialSubsystem extends SubsystemBase {

    // two motors
    // each controlled by trapesoidal control postion loop cascacaded with internal
    // velocity loop
    // yea a lot

    // private FlytMotorController motor1; //flyt motor
    // private FlytMotorController motor2; //flyt motor

    private ProfiledPIDController extensionPid = new ProfiledPIDController(30, 0, 1.5, new Constraints(3000, 12000));
    private ProfiledPIDController rotationPid = new ProfiledPIDController(60, 0, 4, new Constraints(1400, 5600));

    private ElevatorFeedforward extFeedforward = new ElevatorFeedforward(DifferentialArm.e_kS, DifferentialArm.e_kG,
            DifferentialArm.e_kV, DifferentialArm.e_kA);
    private ArmFeedforward rotFeedforward = new ArmFeedforward(DifferentialArm.r_kS, DifferentialArm.r_kG,
            DifferentialArm.r_kV, DifferentialArm.r_kA);

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private RelativeEncoder leftEnc;
    private RelativeEncoder rightEnc;

    private SparkClosedLoopController leftArm;
    private SparkClosedLoopController rightArm;

    private FlytLogger differentialDash = new FlytLogger("Differential");

    private double extensionSetpoint = 0;
    private double rotationSetpoint = 0;

    private SlewRateLimiter extendSlew = new SlewRateLimiter(700);
    private SlewRateLimiter rotateSlew = new SlewRateLimiter(120);

    double extensionVelocity;
    double rotationVelocity;
    double leftCommand;
    double rightCommand;

    public DifferentialSubsystem() {
        // motor1 = new SparkMaxController(getName(), DifferentialArm.kLeftMotorId,
        // true, true, false);
        // motor2 = new SparkMaxController(getName(), DifferentialArm.kRightMotorId,
        // true, true, false);
        // //motor1.advanceControl(Constants.DifferentialArm.voltageComp,
        // Constants.DifferentialArm.currentStallLim,
        // Constants.DifferentialArm.currentFreeLim, 0);
        // //motor2.advanceControl(Constants.DifferentialArm.voltageComp,
        // Constants.DifferentialArm.currentStallLim,
        // Constants.DifferentialArm.currentFreeLim, 0);
        // //motor1.pidSetup(-1, 1, 0, 1, true, 1); //setup p
        // //motor2.pidSetup(-1, 1, 0, 1, true, 1); //setup pid
        // //motor1.pidTune(Constants.DifferentialArm.v_kp,
        // Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd,
        // Constants.DifferentialArm.v_ff);
        // //motor2.pidTune(Constants.DifferentialArm.v_kp,
        // Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd,
        // Constants.DifferentialArm.v_ff);

        // setup position loop ADD FEEDFARWARD
        // pid_extension = new PIDController(Constants.DifferentialArm.e_kp,
        // Constants.DifferentialArm.e_ki, Constants.DifferentialArm.e_kp);
        // pid_rotation = new PIDController(Constants.DifferentialArm.r_kp,
        // Constants.DifferentialArm.r_ki, Constants.DifferentialArm.r_kp);

        leftMotor = new SparkMax(DifferentialArm.kLeftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(DifferentialArm.kRightMotorId, MotorType.kBrushless);

        leftConfig.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .encoder
                    .positionConversionFactor(34.2857)
                    .velocityConversionFactor(0.5714);
        leftConfig.closedLoop
                .p(DifferentialArm.v_kp, ClosedLoopSlot.kSlot0)
                .i(DifferentialArm.v_ki, ClosedLoopSlot.kSlot0)
                .d(DifferentialArm.v_kd, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig.apply(leftConfig);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEnc = leftMotor.getEncoder();
        leftEnc.setPosition(0);
        rightEnc = rightMotor.getEncoder();
        rightEnc.setPosition(0);

        leftArm = leftMotor.getClosedLoopController();
        rightArm = rightMotor.getClosedLoopController();

        extensionPid.reset(extensionSetpoint);
        rotationPid.reset(rotationSetpoint);

        differentialDash.addDoublePublisher("Left POS", true, () -> getLeftPos());
        differentialDash.addDoublePublisher("Right POS", true, () -> getRightPos());
        differentialDash.addDoublePublisher("Extension POS", false, () -> getExtensionPosition());
        differentialDash.addDoublePublisher("Rotation POS", false, () -> getRotationPosition());
        differentialDash.addBoolPublisher("At Extension", false, () -> atExtenstionSetpoint());
        differentialDash.addBoolPublisher("At Rotation", false, () -> atRotationSetpoint());
        differentialDash.addDoublePublisher("Ext Setpoint", false, () -> getExtensionSetpoint());
        differentialDash.addDoublePublisher("Rot Setpoint", false, () -> getRotationSetpoint());
        differentialDash.addDoublePublisher("Left Output", true, () -> leftMotor.getAppliedOutput());
        differentialDash.addDoublePublisher("Left Current", true, () -> leftMotor.getOutputCurrent());
        differentialDash.addDoublePublisher("Right Output", true, () -> rightMotor.getAppliedOutput());
        differentialDash.addDoublePublisher("Right Current", true, () -> rightMotor.getOutputCurrent());
        differentialDash.addDoublePublisher("Voltage", true, () -> leftMotor.getBusVoltage());
        differentialDash.addDoublePublisher("Ext Command Vel", true, () -> extensionVelocity);
        differentialDash.addDoublePublisher("Ext Vel", true, () -> (leftEnc.getVelocity() + rightEnc.getVelocity()) / 2);
        differentialDash.addDoublePublisher("Rot Command Vel", true, () -> rotationVelocity);
        differentialDash.addDoublePublisher("Rot Vel", true, () -> (leftEnc.getVelocity() - rightEnc.getVelocity()) / 2);
        differentialDash.addDoublePublisher("Left Command", true, () -> leftCommand);
        differentialDash.addDoublePublisher("Right Command", true, () -> rightCommand);
    }

    public void extend(double setpoint) {
        leftMotor.set(setpoint);
        rightMotor.set(setpoint);
    }

    public void rotate(double setpoint) {
        leftMotor.set(setpoint);
        rightMotor.set(-setpoint);
    }

    public void setExtensionSetpoint(double setpoint) {
        extensionSetpoint = setpoint;
    }

    public Command setExtensionSetpointCommand(double setpoint) {
        return Commands.run(() -> setExtensionSetpoint(setpoint)).until(() -> atExtenstionSetpoint());
    }

    public void setRotationSetpoint(double setpoint) {
        rotationSetpoint = setpoint;
    }

    public Command setRotationSetpointCommand(double setpoint) {
        return Commands.run(() -> setRotationSetpoint(setpoint)).until(() -> atRotationSetpoint());
    }

    public Command setRotAndExtSetpointCommand(double ext, double rot) {
        return Commands.run(() -> {
            setExtensionSetpoint(ext);
            setRotationSetpoint(rot);
        }).until(() -> {
            return atRotationSetpoint() && atExtenstionSetpoint();
        });
    }

    public double getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public double getRotationSetpoint() {
        return rotationSetpoint;
    }

    public Command incrementExtensionSetpoint(double increment) {
        return Commands.runOnce(() -> extensionSetpoint += increment);
    }

    public Command incrementRotationSetpoint(double increment) {
        return Commands.runOnce(() -> rotationSetpoint += increment);
    }

    public double getLeftPos() {
        return leftEnc.getPosition();
    }

    public double getRightPos() {
        return rightEnc.getPosition();
    }

    public double getExtensionPosition() {
        return (getLeftPos() + getRightPos()) / 2;
        // return (motor1.getPos()+motor2.getPos())/2;
    }

    public double getRotationPosition() {
        return -(((getLeftPos() - getRightPos()) / 2) / 200) * 360;
        // return endeffector.getWristPos();
    }

    public boolean atExtenstionSetpoint() {
        return (extensionSetpoint - 5) <= getExtensionPosition() && getExtensionPosition() <= (extensionSetpoint + 5);
    }

    public boolean atRotationSetpoint() {
        return (rotationSetpoint - 5) <= getRotationPosition() && getRotationPosition() <= (rotationSetpoint + 5);
    }

    private double degreesToMM(double degrees) {
        return (degrees / 360) * 200;
    }

    @Override
    public void periodic() {
        // logging stuff
        // motor1.updateLogger(Constants.debugMode);
        // motor2.updateLogger(Constants.debugMode);

        // Position Extension PID Loop
        // setPosition(pid_extension.calculate(getExtensionPosition(),
        // setExtension),pid_rotation.calculate(getRotationPosition(), setRotation));
        // GET ENCODER DISTANCE HAS TO BE FIGURED OUT LATER
        // if (Constants.debugMode) {
        // double tempSetpoint = differentialDash.getDouble("Ext Setpoint");
        // if (extensionSetpoint != tempSetpoint) {
        // extensionSetpoint = tempSetpoint;
        // }
        // double tempRot = differentialDash.getDouble("Rot Setpoint");
        // if (rotationSetpoint != tempRot) {
        // rotationSetpoint = tempRot;
        // }
        // }
        extensionVelocity = extensionPid.calculate(getExtensionPosition(), extensionSetpoint);
        rotationVelocity = rotationPid.calculate(getRotationPosition(), rotationSetpoint);
        double extFeed = extFeedforward.calculate(extensionVelocity);
        double rotFeed = rotFeedforward.calculate((degreesToRadians(getRotationPosition()) - 0.139), rotationVelocity);
        leftCommand = extensionVelocity - degreesToMM(rotationVelocity);
        rightCommand = extensionVelocity + degreesToMM(rotationVelocity);
        leftArm.setReference(extensionVelocity - degreesToMM(rotationVelocity), ControlType.kVelocity,
                ClosedLoopSlot.kSlot0, (extFeed - rotFeed));
        rightArm.setReference(extensionVelocity + degreesToMM(rotationVelocity), ControlType.kVelocity,
                ClosedLoopSlot.kSlot0, (extFeed + rotFeed));
        // leftArm.setReference(extendSlew.calculate(extensionSetpoint) -
        // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        // rightArm.setReference(extendSlew.calculate(extensionSetpoint) +
        // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        differentialDash.update(Constants.debugMode);
    }
}
