package talonfx;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import data.Length;
import java.util.List;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.With;
import talonfx.SubsystemSimConfiguration.ArmSimConfig;
import talonfx.SubsystemSimConfiguration.ElevatorSimConfig;
import talonfx.SubsystemSimConfiguration.FlywheelSimConfig;
import talonfx.SubsystemSimConfiguration.GenericSimConfig;

public class SubsystemConfiguration {

  enum SimulationType {
    GENERIC,
    ELEVATOR,
    ARM,
    FLYWHEEL;
  }

  public enum SimulationMode {
    AUTO,
    FORCED,
    DISABLED;
  }

  public record CANDevice(Integer id, String canbus) {
    public CANDevice(Integer id) {
      this(id, "");
    }
  }

  /**
   * Constants for a TalonFX motor controller.
   *
   * @param device CANDevice constants.
   * @param invert InvertedValue of the device.
   */
  public record TalonFXConstants(CANDevice device, InvertedValue invert) {}

  @AllArgsConstructor(access = AccessLevel.PRIVATE)
  @With
  public static class CANCoderConfig {
    @With(value = AccessLevel.PRIVATE)
    public CANDevice device;

    public CANCoderConfig(CANDevice device) {
      this.device = device;
    }

    public double absoluteSensorDiscontinuityPoint = 0.0;
    public double magnetOffset = 0.0;
    public SensorDirectionValue sensorDirection = null;
  }

  @AllArgsConstructor(access = AccessLevel.PROTECTED)
  @With
  static class BaseConfig {
    @With(value = AccessLevel.PRIVATE)
    public final String name;

    @With(value = AccessLevel.PRIVATE)
    final TalonFXConstants masterConstants;

    @With(value = AccessLevel.PRIVATE)
    final TalonFXConstants[] slaveConstants;

    public BaseConfig(
        String name, TalonFXConstants masterConstants, List<TalonFXConstants> slaveConstants) {
      this.name = name;
      this.masterConstants = masterConstants;
      this.slaveConstants = slaveConstants.toArray(TalonFXConstants[]::new);
    }

    public final boolean enabled = true;
    public final boolean logAdvancedTelemetry = false;
    public final SimulationMode simMode = SimulationMode.AUTO;

    public final double loopPeriodSec = 0.02;
    public final double simulationPeriodSec = 0.001;

    public final CANCoderConfig canCoderConfig = null;

    public final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public final FeedbackSensorSourceValue feedbackSensorSource =
        FeedbackSensorSourceValue.RotorSensor;
    public final int feedbackRemoteSensorId = 99;
    public final double feedbackRotorOffset = 0.0; // rotations
    public final double rotorToSensorRatio = 1.0;
    public final double sensorToMechRatio = 1.0;
    public final double softLimitDeadband = 0.0;
    public final double velocityThreshold = 0.0; // rps
    public final double positionThreshold = 0.0; // rotations

    public final Slot0Configs slot0 = new Slot0Configs();
    public final Slot1Configs slot1 = new Slot1Configs();
    public final Slot2Configs slot2 = new Slot2Configs();

    public final double motionMagicCruiseVelocity = 0.0; // units/s
    public final double motionMagicAcceleration = 0.0; // units/s^2
    public final double motionMagicJerk = 0.0; // units/s^3
    public final double motionMagicExpo_kV = 0.0;
    public final double motionMagicExpo_kA = 0.0;

    public final double supplyCurrentLimit = 0.0; // amps
    public final double supplyCurrentLowerLimit = 0.0; // amps
    public final double supplyCurrentLowerTime = 0.0; // sec

    public final double statorCurrentLimit = 80; // amps

    public final double maxUnitsLimit = 0.0;
    public final double minUnitsLimit = 0.0;
  }

  public static class GenericConfig extends BaseConfig {

    public GenericConfig(
        String name, TalonFXConstants masterConstants, List<TalonFXConstants> slaveConstants) {
      super(name, masterConstants, slaveConstants);
    }

    public GenericSimConfig simConfig = null;

    	public GenericConfig withSimConfig(GenericSimConfig simConfig) {
        	this.simConfig = simConfig;
			return this;
    	}
  }

  public static class ArmConfig extends BaseConfig {
    public ArmConfig(
        String name, TalonFXConstants masterConstants, List<TalonFXConstants> slaveConstants) {
      super(name, masterConstants, slaveConstants);
    }

    public ArmSimConfig simConfig = null;

    public ArmConfig withSimConfig(ArmSimConfig simConfig) {
      this.simConfig = simConfig;
      return this;
    }
  }

  public static class ElevatorConfig extends BaseConfig {

    public Length spoolCircumference;

    public ElevatorConfig(
        String name,
        TalonFXConstants masterConstants,
        List<TalonFXConstants> slaveConstants,
        Length spoolCircumference) {
      super(name, masterConstants, slaveConstants);
      this.spoolCircumference = spoolCircumference;
    }

    public ElevatorConfig(
        String name,
        TalonFXConstants masterConstants,
        List<TalonFXConstants> slaveConstants,
        Length maxHeight,
        Length minHeight,
        double maxHeightRotations) {
      super(name, masterConstants, slaveConstants);
      this.spoolCircumference = maxHeight.minus(minHeight).div(maxHeightRotations);
    }

    public ElevatorSimConfig simConfig = null;

    public ElevatorConfig withSimConfig(ElevatorSimConfig simConfig) {
      this.simConfig = simConfig;
      return this;
    }

    public ElevatorConfig withSpoolCircumference(Length spoolCircumference) {
      this.spoolCircumference = spoolCircumference;
      return this;
    }
  }

  public static class FlywheelConfig extends BaseConfig {
    public FlywheelConfig(
        String name, TalonFXConstants masterConstants, List<TalonFXConstants> slaveConstants) {
      super(name, masterConstants, slaveConstants);
    }

    public FlywheelSimConfig simConfig = null;

    public FlywheelConfig withSimConfig(FlywheelSimConfig simConfig) {
      this.simConfig = simConfig;
      return this;
    }
  }
}
