package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class DifferentialSubsystem extends SubsystemBase {

    //two motors
    //each controlled by trapesoidal control postion loop cascacaded with internal velocity loop
    // yea a lot

    //private FlytMotorController motor1; //flyt motor
    //private FlytMotorController motor2; //flyt motor

    private ProfiledPIDController extensionPid = new ProfiledPIDController(400, 0, 0, new Constraints(5000, 10000));
    private ProfiledPIDController rotationPid = new ProfiledPIDController(800, 0, 40, new Constraints(2000, 4000));

    private ManipulatorSubsystem endeffector;

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

   public DifferentialSubsystem(ManipulatorSubsystem m_endeffector){
        endeffector = m_endeffector;
        // motor1 = new SparkMaxController(getName(), DifferentialArm.kLeftMotorId, true, true, false);
        // motor2 = new SparkMaxController(getName(), DifferentialArm.kRightMotorId, true, true, false);
        // //motor1.advanceControl(Constants.DifferentialArm.voltageComp, Constants.DifferentialArm.currentStallLim, Constants.DifferentialArm.currentFreeLim, 0);
        // //motor2.advanceControl(Constants.DifferentialArm.voltageComp, Constants.DifferentialArm.currentStallLim, Constants.DifferentialArm.currentFreeLim, 0);
        // //motor1.pidSetup(-1, 1, 0, 1, true, 1); //setup p
        // //motor2.pidSetup(-1, 1, 0, 1, true, 1); //setup pid
        // //motor1.pidTune(Constants.DifferentialArm.v_kp, Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd, Constants.DifferentialArm.v_ff);
        // //motor2.pidTune(Constants.DifferentialArm.v_kp, Constants.DifferentialArm.v_ki, Constants.DifferentialArm.v_kd, Constants.DifferentialArm.v_ff);

        //setup position loop ADD FEEDFARWARD
        //pid_extension = new PIDController(Constants.DifferentialArm.e_kp, Constants.DifferentialArm.e_ki, Constants.DifferentialArm.e_kp);
        //pid_rotation = new PIDController(Constants.DifferentialArm.r_kp, Constants.DifferentialArm.r_ki, Constants.DifferentialArm.r_kp);

        leftMotor = new SparkMax(DifferentialArm.kLeftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(DifferentialArm.kRightMotorId, MotorType.kBrushless);

        leftConfig.inverted(true)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .encoder
                        .positionConversionFactor(60)
                        .velocityConversionFactor(1);
            leftConfig.closedLoop
                        .p(DifferentialArm.v_kp)
                        .i(DifferentialArm.v_ki)
                        .d(DifferentialArm.v_kd)
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
    }

    public void extend(double setpoint){
        leftMotor.set(setpoint);
        rightMotor.set(setpoint);
    }
    
    public void rotate(double setpoint){
        leftMotor.set(setpoint);
        rightMotor.set(-setpoint);
    }

    public void setExtensionSetpoint(double setpoint) {
        extensionSetpoint = setpoint;
    }

    public void setRotationSetpoint(double setpoint) {
        rotationSetpoint = setpoint;
    }

    public double getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public double getRotationSetpoint() {
        return rotationSetpoint;
    }

    public Command incrementExtensionSetpoint(double increment) {
        return this.runOnce(() -> extensionSetpoint += increment);
    }

    public Command incrementRotationSetpoint(double increment) {
        return this.runOnce(() -> rotationSetpoint += increment);
    }

    public double getLeftPos() {
        return leftEnc.getPosition();
    }

    public double getRightPos() {
        return rightEnc.getPosition();
    }
    
    public double getExtensionPosition(){
        return (getLeftPos() + getRightPos()) / 2;
        //return (motor1.getPos()+motor2.getPos())/2;
    }

    public double getRotationPosition(){
        return -(((getLeftPos() - getRightPos()) /2 )/ 200) * 360;
        //return endeffector.getWristPos();
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
        //logging stuff
        //motor1.updateLogger(Constants.debugMode);
        //motor2.updateLogger(Constants.debugMode);

        //Position Extension PID Loop
        //setPosition(pid_extension.calculate(getExtensionPosition(), setExtension),pid_rotation.calculate(getRotationPosition(), setRotation));
        //GET ENCODER DISTANCE HAS TO BE FIGURED OUT LATER
        // if (Constants.debugMode) {
        //     double tempSetpoint = differentialDash.getDouble("Ext Setpoint");
        //     if (extensionSetpoint != tempSetpoint) {
        //         extensionSetpoint = tempSetpoint;
        //     }
        //     double tempRot = differentialDash.getDouble("Rot Setpoint");
        //     if (rotationSetpoint != tempRot) {
        //         rotationSetpoint = tempRot;
        //     }
        // }
        double extensionVelocity = extensionPid.calculate(getExtensionPosition(), extensionSetpoint);
        double rotationVelocity = rotationPid.calculate(getRotationPosition(), rotationSetpoint);
        //leftArm.setReference(extensionVelocity - degreesToMM(rotationVelocity), ControlType.kVelocity);
        //rightArm.setReference(extensionVelocity + degreesToMM(rotationVelocity), ControlType.kVelocity);
        leftArm.setReference(extendSlew.calculate(extensionSetpoint) - degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        rightArm.setReference(extendSlew.calculate(extensionSetpoint) + degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
        differentialDash.update(Constants.debugMode);
    }
}
