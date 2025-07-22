package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.robot.commands.Waits.ExtensionAndRotationWait;
import frc.robot.commands.Waits.ExtensionSetWait;
import frc.robot.commands.Waits.RotationSetWait;
import frc.utils.LinearInterpolator;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class DifferentialSubsystem extends SubsystemBase {

    private ProfiledPIDController extensionPid = new ProfiledPIDController(30, 0, 1.5, new Constraints(3000, 12000));
    private ProfiledPIDController rotationPid = new ProfiledPIDController(60, 0, 4, new Constraints(1400, 5600));

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

    private LaserCan lc;
    private int laserCanDistance = 0;
    private boolean hasLaserCanDistance = false;

    private LinearInterpolator l4RotationInterpolator = new LinearInterpolator(DifferentialArm.l4RotationData);
    private LinearInterpolator l4ExtensionInterpolator = new LinearInterpolator(DifferentialArm.l4ExtensionData);
    private LinearInterpolator l2_3RotationInterpolator = new LinearInterpolator(DifferentialArm.l2_3RotationData);
    private LinearInterpolator l2_3ExtensionInterpolator = new LinearInterpolator(DifferentialArm.l2_3ExtensionData);

    double extensionVelocity;
    double rotationVelocity;
    double leftCommand;
    double rightCommand;

    public DifferentialSubsystem() {
        leftMotor = new SparkMax(DifferentialArm.kLeftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(DifferentialArm.kRightMotorId, MotorType.kBrushless);

        leftConfig.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .encoder
                    .positionConversionFactor(34.2857)
                    .velocityConversionFactor(0.5714)
                    .quadratureMeasurementPeriod(10)
                    .quadratureAverageDepth(2);
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

        lc = new LaserCan(DifferentialArm.kLaserCanId);

        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan Configuration failed! " + e);
        }

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
        differentialDash.addIntegerPublisher("LaserCan Distance", true, () -> laserCanDistance);
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
        return new ExtensionSetWait(this, setpoint);
        //return Commands.run(() -> setExtensionSetpoint(setpoint)).until(() -> atExtenstionSetpoint());
    }

    public void setRotationSetpoint(double setpoint) {
        rotationSetpoint = setpoint;
    }

    public Command setRotationSetpointCommand(double setpoint) {
        return new RotationSetWait(this, setpoint);
        //return Commands.run(() -> setRotationSetpoint(setpoint)).until(() -> atRotationSetpoint());
    }

    public Command setRotAndExtSetpointCommand(double ext, double rot) {
        return new ExtensionAndRotationWait(this, ext, rot);
        // return Commands.run(() -> {
        //     setExtensionSetpoint(ext);
        //     setRotationSetpoint(rot);
        // }).until(() -> {
        //     return atRotationSetpoint() && atExtenstionSetpoint();
        // });
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

    public int getLaserCanDistance() {
        return laserCanDistance;
    }

    public boolean hasLaserCanDistance() {
        return hasLaserCanDistance;
    }

    public double l4RotationInterpolate() {
        return l4RotationInterpolator.getInterpolatedValue(laserCanDistance);
    }

    public double l4ExtensionInterpolate() {
        return l4ExtensionInterpolator.getInterpolatedValue(laserCanDistance);
    }

    public double l2_3RotationInterpolate() {
        return l2_3RotationInterpolator.getInterpolatedValue(laserCanDistance);
    }

    public double l2_3ExtensionInterpolate() {
        return l2_3ExtensionInterpolator.getInterpolatedValue(laserCanDistance);
    }

    private double degreesToMM(double degrees) {
        return (degrees / 360) * 200;
    }

    @Override
    public void periodic() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < 430) {
            laserCanDistance = measurement.distance_mm;
            hasLaserCanDistance = true;
            //System.out.println("The target is " + measurement.distance_mm + "mm away!");
        } else {
            hasLaserCanDistance = false;
        }

        extensionVelocity = extensionPid.calculate(getExtensionPosition(), extensionSetpoint);
        rotationVelocity = rotationPid.calculate(getRotationPosition(), rotationSetpoint);
        //double extFeed = extFeedforward.calculate(extensionVelocity);
        //double rotFeed = rotFeedforward.calculate((degreesToRadians(getRotationPosition()) - 0.139), rotationVelocity);
        leftCommand = extensionVelocity - degreesToMM(rotationVelocity);
        rightCommand = extensionVelocity + degreesToMM(rotationVelocity);
        leftArm.setReference(extensionVelocity - degreesToMM(rotationVelocity), ControlType.kVelocity);
        rightArm.setReference(extensionVelocity + degreesToMM(rotationVelocity), ControlType.kVelocity);
        // leftArm.setReference(extendSlew.calculate(extensionSetpoint) -
        // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        // rightArm.setReference(extendSlew.calculate(extensionSetpoint) +
        // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        differentialDash.update(Constants.debugMode);
    }
}
