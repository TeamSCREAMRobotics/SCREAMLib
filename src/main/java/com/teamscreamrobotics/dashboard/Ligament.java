package com.teamscreamrobotics.dashboard;

import java.util.function.Supplier;

import com.teamscreamrobotics.data.Length;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Represents a single joint segment in a {@link Mechanism}, tracking both a measured (red) and
 * setpoint (green) {@link MechanismLigament2d}. Use the {@code with*} builder methods to configure
 * angle and length sources, then attach to a {@link Mechanism}.
 */
public class Ligament {

  protected MechanismLigament2d measuredLig;
  protected MechanismLigament2d setpointLig;

  private Supplier<Length> measuredLength = () -> new Length();
  private Supplier<Rotation2d> measuredAngle = () -> Rotation2d.kZero;
  private Supplier<Length> setpointLength = () -> new Length();
  private Supplier<Rotation2d> setpointAngle = () -> Rotation2d.kZero;

  protected boolean overrideAppend;

  /** Creates a Ligament with default zero angle and length; configure with {@code with*} methods. */
  public Ligament(){}

  protected void initialize(int index, MechanismRoot2d measured, MechanismRoot2d setpoint){
    measuredLig = measured.append(
        new MechanismLigament2d("Joint " + (index + 1),  0.0, 0.0, 6, new Color8Bit(Color.kRed)));
    setpointLig = setpoint.append(
        new MechanismLigament2d("Joint " + (index + 1), 0.0, 0.0, 6, new Color8Bit(Color.kGreen)));
  }

  protected void initialize(int index, MechanismLigament2d previousMeasured, MechanismLigament2d previousSetpoint){
    measuredLig = previousMeasured.append(
        new MechanismLigament2d("Joint " + (index + 1), 0.0, 0.0, 6, new Color8Bit(Color.kRed)));
    setpointLig = previousSetpoint.append(
        new MechanismLigament2d("Joint " + (index + 1), 0.0, 0.0, 6, new Color8Bit(Color.kGreen)));
  }

  /**
   * When {@code true}, this ligament is appended directly to the root instead of its predecessor.
   * Useful for multi-branch mechanisms that share a common root but diverge.
   */
  public Ligament withOverrideAppend(boolean override){
    overrideAppend = override;
    return this;
  }
    
  /**
   * Sets a fixed angle for both the measured and setpoint ligaments.
   *
   * @param angle the constant angle to display
   */
  public Ligament withStaticAngle(Rotation2d angle) {
    this.measuredAngle = () -> angle;
    this.setpointAngle = () -> angle;
    return this;
  }

  /**
   * Sets independent angle suppliers for the measured and setpoint ligaments.
   *
   * @param measured supplier for the actual/measured angle
   * @param setpoint supplier for the goal/setpoint angle
   */
  public Ligament withDynamicAngle(Supplier<Rotation2d> measured, Supplier<Rotation2d> setpoint) {
    this.measuredAngle = measured;
    this.setpointAngle = setpoint;
    return this;
  }

  /**
   * Sets a single angle supplier used for both measured and setpoint ligaments.
   *
   * @param angle supplier for the angle to display
   */
  public Ligament withDynamicAngle(Supplier<Rotation2d> angle) {
    this.measuredAngle = angle;
    this.setpointAngle = angle;
    return this;
  }

  /**
   * Sets a fixed length for both the measured and setpoint ligaments.
   *
   * @param length the constant length to display
   */
  public Ligament withStaticLength(Length length) {
    this.measuredLength = () -> length;
    this.setpointLength = () -> length;
    return this;
  }

  /**
   * Sets independent length suppliers for the measured and setpoint ligaments.
   *
   * @param measured supplier for the actual/measured length
   * @param setpoint supplier for the goal/setpoint length
   */
  public Ligament withDynamicLength(Supplier<Length> measured, Supplier<Length> setpoint) {
    this.measuredLength = measured;
    this.setpointLength = setpoint;
    return this;
  }

  /**
   * Sets a single length supplier used for both measured and setpoint ligaments.
   *
   * @param length supplier for the length to display
   */
  public Ligament withDynamicLength(Supplier<Length> length) {
    this.measuredLength = length;
    this.setpointLength = length;
    return this;
  }

  /**
   * Imperatively sets the same fixed angle for both measured and setpoint ligaments.
   *
   * @param angle the angle to display
   */
  public void setAngle(Rotation2d angle) {
    this.measuredAngle = () -> angle;
    this.setpointAngle = () -> angle;
  }

  /**
   * Imperatively sets independent fixed angles for the measured and setpoint ligaments.
   *
   * @param measured the actual/measured angle
   * @param setpoint the goal/setpoint angle
   */
  public void setAngle(Rotation2d measured, Rotation2d setpoint) {
    this.measuredAngle = () -> measured;
    this.setpointAngle = () -> setpoint;
  }

  /**
   * Imperatively sets the same fixed length for both measured and setpoint ligaments.
   *
   * @param length the length to display
   */
  public void setLength(Length length) {
    this.measuredLength = () -> length;
    this.setpointLength = () -> length;
  }

  /**
   * Imperatively sets independent fixed lengths for the measured and setpoint ligaments.
   *
   * @param measured the actual/measured length
   * @param setpoint the goal/setpoint length
   */
  public void setLength(Length measured, Length setpoint) {
    this.measuredLength = () -> measured;
    this.setpointLength = () -> setpoint;
  }

  protected void update(){
    this.setpointLig.setLength(setpointLength.get().getMeters());
    this.setpointLig.setAngle(setpointAngle.get().getDegrees());
    this.measuredLig.setLength(measuredLength.get().getMeters());
    this.measuredLig.setAngle(measuredAngle.get().getDegrees());
  }
}
