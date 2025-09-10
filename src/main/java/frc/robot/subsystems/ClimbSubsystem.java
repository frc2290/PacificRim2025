// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.Constants.DriveConstants;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax leftMotor;

    private Servo servo;
    private double servoOpen = 90;
    private double servoClosed = 180;

    private SparkClosedLoopController climber;

    private RelativeEncoder leftEnc;

    private SparkMaxConfig leftConfig = new SparkMaxConfig();

    // Simulation members
    private SparkMaxSim climberSim;
    private SparkRelativeEncoderSim climberEncoderSim;

    private double climberSetpoint = 0;

    public double tClimb = 2;
    public double vSlewrate = Math.abs((Climber.climberOutSetpoint - Climber.climberInSetpoint) / tClimb);
    public double climbDistance = (Units.inchesToMeters(20)/DriveConstants.kMaxSpeedMetersPerSecond);

    private SlewRateLimiter climberSlew = new SlewRateLimiter(vSlewrate);

    private boolean climbing = false;

    private FlytLogger climbDash = new FlytLogger("Climb");

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
        leftMotor = new SparkMax(Climber.kLeftClimberMotorId, MotorType.kBrushless);

        climber = leftMotor.getClosedLoopController();

        leftEnc = leftMotor.getEncoder();
        leftEnc.setPosition(0);

        if (RobotBase.isSimulation()) {
            climberSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
            climberEncoderSim = climberSim.getRelativeEncoderSim();
        }

        leftConfig.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60).encoder
                .positionConversionFactor(Climber.kPositionConversion)
                .velocityConversionFactor(Climber.kVelocityConversion);
        leftConfig.closedLoop
                .p(Climber.kP, ClosedLoopSlot.kSlot0)
                .i(Climber.kI, ClosedLoopSlot.kSlot0)
                .d(Climber.kD, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        servo = new Servo(1);

        setServoOpen();

        climbDash.addDoublePublisher("Climb Pos", true, () -> getClimberPos());
        climbDash.addDoublePublisher("Climb Setpoint", true, () -> getClimberSetpoint());
        climbDash.addDoublePublisher("Climb Servo Pos", true, () -> getServoPos());
        climbDash.addDoublePublisher("Climb Current", true, () -> leftMotor.getOutputCurrent());
    }

    /**
     * Test-oriented constructor that allows hardware dependencies to be supplied
     * externally for unit testing.
     */
    public ClimbSubsystem(
            SparkMax leftMotor,
            SparkClosedLoopController controller,
            RelativeEncoder leftEnc,
            Servo servo) {
        this.leftMotor = leftMotor;
        this.climber = controller;
        this.leftEnc = leftEnc;
        this.servo = servo;
    }

    public boolean getClimbing() {
        return climbing;
    }

    public void setClimbing(boolean hold) {
        climbing = hold;
    }

    public void setClimberSpeed(double speed) {
        leftMotor.set(speed);
    }

    public void stopClimberMotor() {
        leftMotor.stopMotor();
    }

    public Command setClimberSpeedCommand(double speed) {
        return this.runOnce(() -> setClimberSpeed(speed));
    }

    public double getClimberPos() {
        return leftEnc.getPosition();
    }

    public double getClimberVel() {
        return leftEnc.getVelocity();
    }

    public double getClimberSetpoint() {
        return climberSetpoint;
    }

    public void setClimberSetpoint(double setpoint) {
        climberSetpoint = setpoint;
    }

    public boolean atClimberSetpoint() {
        return (climberSetpoint - 5) < getClimberPos() && getClimberPos() < (climberSetpoint + 5);
    }

    public boolean climberOut() {
        return (Climber.climberOutSetpoint - 5) < getClimberPos() && getClimberPos() < (Climber.climberInSetpoint + 5);
    }

    /**
     * Returns the current draw of the climber motor.
     *
     * @return Climber motor output current.
     */
    public double getCurrentDraw() {
        return leftMotor.getOutputCurrent();
    }

    public double getServoPos() {
        return servo.getPosition();
    }

    public void setServoPos(double pos) {
        servo.setAngle(pos);
    }

    public void setServoClose() {
        servo.setAngle(servoClosed);
    }

    public void setServoOpen() {
        servo.setAngle(servoOpen);
    }

    @Override
    public void periodic() {
        if (!climbing) {
            climber.setReference(climberSlew.calculate(climberSetpoint), ControlType.kPosition);
        }
        // This method will be called once per scheduler run
        climbDash.update(Constants.debugMode);
    }

    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation() && climberSim != null) {
            climberSim.iterate(leftMotor.getAppliedOutput(), 0.02, leftMotor.getBusVoltage());
            climberEncoderSim.setPosition(climberSim.getPosition() * Climber.kPositionConversion);
            climberEncoderSim.setVelocity(climberSim.getVelocity() * Climber.kVelocityConversion);
            SmartDashboard.putNumber("Climber Sim Pos", getClimberPos());
        }
    }
}
