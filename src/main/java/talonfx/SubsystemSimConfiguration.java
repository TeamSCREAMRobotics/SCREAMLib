package talonfx;

import data.Length;
import data.Mass;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.IntFunction;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.With;
import talonfx.SubsystemConfiguration.SimulationType;

public class SubsystemSimConfiguration {

  public static enum MotorType {
    KRAKEN_X60((num) -> DCMotor.getKrakenX60(num)),
    KRAKEN_X44((num) -> DCMotor.getKrakenX60(num)),
    FALCON_500((num) -> DCMotor.getFalcon500(num)),
    NEO((num) -> DCMotor.getNEO(num)),
    NEO_VORTEX((num) -> DCMotor.getNeoVortex(num)),
    NEO_550((num) -> DCMotor.getNeo550(num)),
    BAG((num) -> DCMotor.getBag(num));

    public final IntFunction<DCMotor> motor;

    private MotorType(IntFunction<DCMotor> motor) {
      this.motor = motor;
    }

    public DCMotor get(int numMotors) {
      return motor.apply(numMotors);
    }
  }

  @AllArgsConstructor
  @With
  public static class GenericSimConfig {
    @With(value = AccessLevel.PRIVATE)
    SimulationType simType = SimulationType.GENERIC;

    public final MotorType motorType;
    public final double JKgMetersSquaredMOI;
  }

  @AllArgsConstructor
  @With
  public static class ArmSimConfig {
    @With(value = AccessLevel.PRIVATE)
    SimulationType simType = SimulationType.ARM;

    public final MotorType motorType;
    public final double JKgMetersSquaredMOI;
    public final Length armLength;
    public final Rotation2d maxAngle;
    public final Rotation2d minAngle;
    public final boolean simulateGravity;
    public final Rotation2d startingAngle;
  }

  @AllArgsConstructor
  @With
  public static class ElevatorSimConfig {
    @With(value = AccessLevel.PRIVATE)
    SimulationType simType = SimulationType.ELEVATOR;

    public final MotorType motorType;
    public final Mass carriageMass;
    public final Length maxHeight;
    public final Length minHeight;
    public final boolean simulateGravity;
    public final Length startingHeight;
  }

  @AllArgsConstructor
  @With
  public static class FlywheelSimConfig {
    @With(value = AccessLevel.PRIVATE)
    SimulationType simType = SimulationType.FLYWHEEL;

    public final MotorType motorType;
    public final double JKgMetersSquaredMOI;
  }
}
