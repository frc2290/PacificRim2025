package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.reset;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.ArgumentCaptor;

import frc.robot.Configs;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

class MAXSwerveModuleTest {
    @Mock private SparkFlex drivingSpark;
    @Mock private SparkMax turningSpark;
    @Mock private RelativeEncoder drivingEncoder;
    @Mock private AbsoluteEncoder turningEncoder;
    @Mock private SparkClosedLoopController driveController;
    @Mock private SparkClosedLoopController turnController;

    private AutoCloseable mocks;

    @BeforeEach
    void setUp() {
        mocks = MockitoAnnotations.openMocks(this);
        when(turningEncoder.getPosition()).thenReturn(0.0);
    }

    @AfterEach
    void tearDown() throws Exception {
        mocks.close();
        Configs.MAXSwerveModule.drivingConfig.idleMode(IdleMode.kBrake);
    }

    private MAXSwerveModule newModule(double offset) {
        return new MAXSwerveModule(
                drivingSpark,
                turningSpark,
                drivingEncoder,
                turningEncoder,
                driveController,
                turnController,
                offset);
    }

    @Test
    void getStateAppliesChassisOffset() {
        when(drivingEncoder.getVelocity()).thenReturn(4.2);
        when(turningEncoder.getPosition()).thenReturn(1.0);

        MAXSwerveModule module = newModule(0.25);
        SwerveModuleState state = module.getState();

        assertEquals(4.2, state.speedMetersPerSecond);
        assertEquals(0.75, state.angle.getRadians(), 1e-9);
    }

    @Test
    void getPositionAppliesChassisOffset() {
        when(drivingEncoder.getPosition()).thenReturn(5.0);
        when(turningEncoder.getPosition()).thenReturn(2.0);

        MAXSwerveModule module = newModule(0.5);
        SwerveModulePosition pos = module.getPosition();

        assertEquals(5.0, pos.distanceMeters);
        assertEquals(1.5, pos.angle.getRadians(), 1e-9);
    }

    @Test
    void setDesiredStateCommandsControllersWithOffset() {
        MAXSwerveModule module = newModule(0.1);
        SwerveModuleState desired = new SwerveModuleState(3.0, Rotation2d.fromRadians(0.2));

        module.setDesiredState(desired);

        verify(driveController).setReference(3.0, ControlType.kVelocity);
        verify(turnController).setReference(0.3, ControlType.kPosition);
    }

    @Test
    void resetEncodersZerosDriveEncoder() {
        MAXSwerveModule module = newModule(0.0);
        reset(drivingEncoder);
        module.resetEncoders();
        verify(drivingEncoder).setPosition(0);
    }

    @Test
    void getCurrentDrawSumsMotorCurrents() {
        when(drivingSpark.getOutputCurrent()).thenReturn(10.0);
        when(turningSpark.getOutputCurrent()).thenReturn(1.5);

        MAXSwerveModule module = newModule(0.0);
        assertEquals(11.5, module.getCurrentDraw(), 1e-9);
    }

    @Test
    void runDriveCharacterizationSetsVoltageAndHoldsAngle() {
        MAXSwerveModule module = newModule(0.1);

        module.runDriveCharacterization(2.0);

        verify(driveController).setReference(2.0, ControlType.kVoltage);
        ArgumentCaptor<Double> angleCaptor = ArgumentCaptor.forClass(Double.class);
        verify(turnController).setReference(angleCaptor.capture(), eq(ControlType.kPosition));
        assertEquals(0.1, angleCaptor.getValue(), 1e-9);
    }

    @Test
    void runTurnCharacterizationSetsVoltage() {
        MAXSwerveModule module = newModule(0.0);
        module.runTurnCharacterization(1.0);
        verify(turnController).setReference(1.0, ControlType.kVoltage);
    }

    @Test
    void setDriveCoastConfiguresDriveSpark() {
        MAXSwerveModule module = newModule(0.0);

        module.setDriveCoast();

        verify(drivingSpark).configure(any(SparkFlexConfig.class), eq(ResetMode.kNoResetSafeParameters), eq(PersistMode.kNoPersistParameters));
    }
}

