package com.SCREAMLib.dashboard;

import com.SCREAMLib.data.Length;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.Supplier;

public class Mechanism {

  public Mechanism2d measured;
  public Mechanism2d setpoint;

  public String key;

  private MechanismRoot2d measuredRoot;
  private MechanismRoot2d setpointRoot;

  private MechanismLigament2d measuredLig;
  private MechanismLigament2d setpointLig;

  private Supplier<Length> measuredLength = () -> new Length();
  private Supplier<Rotation2d> measuredAngle = () -> new Rotation2d();
  private Supplier<Length> setpointLength = () -> new Length();
  private Supplier<Rotation2d> setpointAngle = () -> new Rotation2d();

  private Supplier<Translation2d> position = () -> new Translation2d();

  public Mechanism(String key) {
    this.key = key;
  }

  protected void initialize(Mechanism2d measured, Mechanism2d setpoint) {
    this.measured = measured;
    this.setpoint = setpoint;

    measuredRoot = measured.getRoot(key + " Actual", 0, 0);
    setpointRoot = setpoint.getRoot(key + " Setpoint", 0, 0);

    measuredLig =
        measuredRoot.append(
            new MechanismLigament2d("Actual", 0.0, 0.0, 6, new Color8Bit(Color.kRed)));
    setpointLig =
        setpointRoot.append(
            new MechanismLigament2d("Setpoint", 0.0, 0.0, 6, new Color8Bit(Color.kGreen)));
  }

  public Mechanism withStaticAngle(Rotation2d angle) {
    this.measuredAngle = () -> angle;
    this.setpointAngle = () -> angle;
    return this;
  }

  public Mechanism withDynamicAngle(Supplier<Rotation2d> measured, Supplier<Rotation2d> setpoint) {
    this.measuredAngle = measured;
    this.setpointAngle = setpoint;
    return this;
  }

  public Mechanism withDynamicAngle(Supplier<Rotation2d> angle) {
    this.measuredAngle = angle;
    this.setpointAngle = angle;
    return this;
  }

  public Mechanism withStaticLength(Length length) {
    this.measuredLength = () -> length;
    this.setpointLength = () -> length;
    return this;
  }

  public Mechanism withDynamicLength(Supplier<Length> measured, Supplier<Length> setpoint) {
    this.measuredLength = measured;
    this.setpointLength = setpoint;
    return this;
  }

  public Mechanism withDynamicLength(Supplier<Length> length) {
    this.measuredLength = length;
    this.setpointLength = length;
    return this;
  }

  public Mechanism withStaticPosition(Translation2d position) {
    this.position = () -> position;
    return this;
  }

  public Mechanism withDynamicPosition(Supplier<Translation2d> position) {
    this.position = position;
    return this;
  }

  public void setAngle(Rotation2d angle) {
    this.measuredAngle = () -> angle;
    this.setpointAngle = () -> angle;
  }

  public void setAngle(Rotation2d measured, Rotation2d setpoint) {
    this.measuredAngle = () -> measured;
    this.setpointAngle = () -> setpoint;
  }

  public void setLength(Length length) {
    this.measuredLength = () -> length;
    this.setpointLength = () -> length;
  }

  public void setLength(Length measured, Length setpoint) {
    this.measuredLength = () -> measured;
    this.setpointLength = () -> setpoint;
  }

  public void setPosition(Translation2d position) {
    this.position = () -> position;
  }

  protected void update() {
    this.setpointLig.setLength(setpointLength.get().getMeters());
    this.setpointLig.setAngle(setpointAngle.get().getDegrees());
    this.measuredLig.setLength(measuredLength.get().getMeters());
    this.measuredLig.setAngle(measuredAngle.get().getDegrees());
    this.setpointRoot.setPosition(position.get().getX(), position.get().getY());
    this.measuredRoot.setPosition(position.get().getX(), position.get().getY());
  }
}
