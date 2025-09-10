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
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.commands.Waits.ElevatorSetWait;
import frc.robot.Constants;
import frc.utils.ExponentialProfiledPIDController;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
//import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
//import frc.utils.FLYTLib.FLYTMotorLib.SparkFlexController;

public class ElevatorSubsystem extends SubsystemBase {

    //motor group
    //FlytMotorController leftMotor; 
    //FlytMotorController rightMotor;

    private ProfiledPIDController traPidController = new ProfiledPIDController(64, 0, 1, new TrapezoidProfile.Constraints(2.5, 9));
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);

    private SparkFlex leftMotor;
    private SparkFlex rightMotor;

    private SparkFlexConfig leftConfig = new SparkFlexConfig();
    private SparkFlexConfig rightConfig = new SparkFlexConfig();

    private SparkClosedLoopController elevator;

    private RelativeEncoder leftEnc;

    private FlytLogger elevatorDash = new FlytLogger("Elevator");

    // Simulation members
    private ElevatorSim elevatorSim;
    private SparkFlexSim leftSim;
    private SparkRelativeEncoderSim leftEncoderSim;

    private double elevatorSetpoint = 0;

    private ExponentialProfiledPIDController expPidController = new ExponentialProfiledPIDController(0.0, 0.0, 0.0, ExponentialProfile.Constraints.fromCharacteristics(0.0, 0.0, 0.0));

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

        if (RobotBase.isSimulation()) {
            elevatorSim = new ElevatorSim(
                    DCMotor.getNEO(1),
                    Elevator.kSimGearing,
                    Elevator.kSimCarriageMassKg,
                    Elevator.kSimDrumRadiusMeters,
                    Elevator.kSimMinHeightMeters,
                    Elevator.kSimMaxHeightMeters,
                    true,
                    0.0);
            leftSim = new SparkFlexSim(leftMotor, DCMotor.getNEO(1));
            leftEncoderSim = leftSim.getRelativeEncoderSim();
        }

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

        expPidController.setTolerance(0.0, 0.0);


        elevatorDash.addDoublePublisher("Elevator POS", false, () -> getPosition());
        elevatorDash.addDoublePublisher("Elevator Setpoint", false, () -> getElevatorSetpoint());
        elevatorDash.addBoolPublisher("At Position", false, () -> atPosition());
        elevatorDash.addDoublePublisher("Duty Cycle", true, () -> leftMotor.getAppliedOutput());
        elevatorDash.addDoublePublisher("Voltage", true, () -> leftMotor.getBusVoltage());
        elevatorDash.addDoublePublisher("Current", true, () -> leftMotor.getOutputCurrent());
        elevatorDash.addDoublePublisher("Velocity", true, () -> leftEnc.getVelocity());
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
        return new ElevatorSetWait(this, setpoint);
        //return Commands.run(() -> setElevatorSetpoint(setpoint)).until(() -> atPosition());
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

    /**
     * Returns the total current draw of the elevator motors.
     *
     * @return Combined output current of left and right motors.
     */
    public double getCurrentDraw() {
        return leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent();
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

    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation() && elevatorSim != null) {
            double batteryVoltage = RobotController.getBatteryVoltage();
            leftSim.setBusVoltage(batteryVoltage);
            leftSim.iterate(leftMotor.getAppliedOutput(), 0.02, batteryVoltage);

            double volts = leftMotor.getAppliedOutput() * batteryVoltage;
            elevatorSim.setInputVoltage(volts);
            elevatorSim.update(0.02);

            leftEncoderSim.setPosition(elevatorSim.getPositionMeters());
            leftEncoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());

            SmartDashboard.putNumber("Elevator Height", elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("Elevator Velocity", elevatorSim.getVelocityMetersPerSecond());
            SmartDashboard.putNumber("Elevator Voltage", volts);
            SmartDashboard.putNumber("Elevator Current", getCurrentDraw());
            SmartDashboard.putBoolean("Elevator At Setpoint", atPosition());
        }
    }
}
