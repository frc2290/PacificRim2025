// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private Servo servo;

    private SparkClosedLoopController climber;

    private RelativeEncoder leftEnc;

    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private double climberSetpoint = 0;

    private FlytLogger climbDash = new FlytLogger("Climb");

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
        leftMotor = new SparkMax(Climber.kLeftClimberMotorId, MotorType.kBrushless);
        //rightMotor = new SparkMax(Climber.kRightClimberMotorId, MotorType.kBrushless);

        climber = leftMotor.getClosedLoopController();

        leftEnc = leftMotor.getEncoder();
        leftEnc.setPosition(0);

        leftConfig.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40).encoder
                .positionConversionFactor(Climber.kPositionConversion)
                .velocityConversionFactor(Climber.kVelocityConversion);
        leftConfig.closedLoop
                .p(Climber.kP, ClosedLoopSlot.kSlot0)
                .i(Climber.kI, ClosedLoopSlot.kSlot0)
                .d(Climber.kD, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        servo = new Servo(0);

        //rightConfig.follow(Climber.kLeftClimberMotorId, true)
        //        .idleMode(IdleMode.kBrake);

        //rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbDash.addDoublePublisher("Climb Pos", true, () -> getClimberPos());
        climbDash.addDoublePublisher("Climb Servo Pos", true, () -> getServoPos());
        climbDash.addDoublePublisher("Climb Current", true, () -> leftMotor.getOutputCurrent());
    }

    public void setClimberSpeed(double speed) {
        leftMotor.set(speed);
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

    public double getServoPos() {
        return servo.getPosition();
    }

    public void setServoPos(double pos) {
        servo.set(pos);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climbDash.update(Constants.debugMode);
    }
}
