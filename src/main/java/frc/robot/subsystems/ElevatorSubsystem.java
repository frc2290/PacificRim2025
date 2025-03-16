package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
//import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
//import frc.utils.FLYTLib.FLYTMotorLib.SparkFlexController;

public class ElevatorSubsystem extends SubsystemBase {

    //motor group
    //FlytMotorController leftMotor; 
    //FlytMotorController rightMotor;

    private ProfiledPIDController traPidController = new ProfiledPIDController(64, 0, 0, new Constraints(2, 5));
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);

    private SparkFlex leftMotor;
    private SparkFlex rightMotor;

    private SparkFlexConfig leftConfig = new SparkFlexConfig();
    private SparkFlexConfig rightConfig = new SparkFlexConfig();

    private SparkClosedLoopController elevator;

    private RelativeEncoder leftEnc;

    private FlytLogger elevatorDash = new FlytLogger("Elevator");

    private double elevatorSetpoint = 0;

    //private SlewRateLimiter elevatorSlew = new SlewRateLimiter(3);

    public ElevatorSubsystem() {
        // leftMotor = new SparkFlexController(getName(), Elevator.kLeftElevatorMotorId, true, true, true); //motor construct
        // //motor1.advanceControl(0,0,0,0);//setup advace control
        // //motor1.pidSetup(-1, 1, 0, 0, true, 0); //setup pid
        // //motor1.motionProfile(0, 0); //create motion profile
        // //motor1.pidTune(0, 0, 0, 0); //tune pid
        // rightMotor = new SparkFlexController(getName(), Elevator.kRightElevatorMotorId, true, true, false); //creat second motor
        // rightMotor.followeMe(Elevator.kLeftElevatorMotorId, true); //make it follow motor 1

        leftMotor = new SparkFlex(Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
        rightMotor = new SparkFlex(Elevator.kRightElevatorMotorId, MotorType.kBrushless);

        elevator = leftMotor.getClosedLoopController();

        leftEnc = leftMotor.getEncoder();
        leftEnc.setPosition(0);

        leftConfig.inverted(true)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .encoder
                        .positionConversionFactor(Elevator.kPositionConversion)
                        .velocityConversionFactor(Elevator.kVelocityConversion);
            leftConfig.closedLoop
                        .p(Elevator.kP, ClosedLoopSlot.kSlot0)
                        .i(Elevator.kI, ClosedLoopSlot.kSlot0)
                        .d(Elevator.kD, ClosedLoopSlot.kSlot0)
                        .outputRange(-1, 1);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        rightConfig.follow(Elevator.kLeftElevatorMotorId, true)
                    .idleMode(IdleMode.kBrake);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        traPidController.reset(elevatorSetpoint);

        elevatorDash.addDoublePublisher("Elevator POS", false, () -> getPosition());
        elevatorDash.addDoublePublisher("Elevator Setpoint", false, () -> getElevatorSetpoint());
        elevatorDash.addBoolPublisher("At Position", false, () -> atPosition());
        elevatorDash.addDoublePublisher("Duty Cycle", true, () -> leftMotor.getAppliedOutput());
        elevatorDash.addDoublePublisher("Voltage", true, () -> leftMotor.getBusVoltage());
        elevatorDash.addDoublePublisher("Current", false, () -> leftMotor.getOutputCurrent());
    }

    /**
     * Change the current setpoint for the Elevator
     * @param setpoint - Desired position for elevator
     */
    public void setElevatorSetpoint(double setpoint){
        //leftMotor.set(setpoint);
        elevatorSetpoint = setpoint;
    }

    public Command setElevatorSetpointCommand(double setpoint) {
        return Commands.run(() -> setElevatorSetpoint(setpoint)).until(() -> atPosition());
    }

    public Command incrementElevatorSetpoint(double increment) {
        return Commands.runOnce(() -> elevatorSetpoint += increment);
    }

    public double getElevatorSetpoint() {
        return elevatorSetpoint;
    }

    /**
     * Set speed of elevator (value between -1 and 1)
     * @param power - Speed setting (between -1 and 1)
     */
    public void setSpeed(double power) {
        leftMotor.set(power);
    }

    /**
     * Returns current pos of Elevator
     * @return Pos of Elevator
     */
    public double getPosition() {
        return leftEnc.getPosition();
    }

    /**
     * Determines if Elevator is at current setpoint
     * @return True if at setpoint
     */
    public boolean atPosition() {
        return (elevatorSetpoint - 0.04) <= getPosition() && getPosition() <= (elevatorSetpoint + 0.04);
    }

    @Override
    public void periodic() {
        // if (Constants.debugMode) {
        //     double tempSetpoint = elevatorDash.getDouble("Elevator Setpoint");
        //     if (elevatorSetpoint != tempSetpoint) {
        //         elevatorSetpoint = tempSetpoint > 1.3 ? 1.3 : tempSetpoint;
        //     }
        // }
        double velocity = traPidController.calculate(getPosition(), elevatorSetpoint);
        double elevFeed = feedforward.calculate(velocity);
        elevator.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);//, elevFeed);
        //elevator.setReference(elevatorSlew.calculate(elevatorSetpoint), ControlType.kPosition, ClosedLoopSlot.kSlot0, Elevator.kKG);
        elevatorDash.update(Constants.debugMode);
    }
}
