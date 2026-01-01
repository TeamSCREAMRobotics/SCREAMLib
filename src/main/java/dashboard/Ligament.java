package dashboard;

import java.util.function.Supplier;

import data.Length;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Ligament {

  protected MechanismLigament2d measuredLig;
  protected MechanismLigament2d setpointLig;

  private Supplier<Length> measuredLength = () -> new Length();
  private Supplier<Rotation2d> measuredAngle = () -> Rotation2d.kZero;
  private Supplier<Length> setpointLength = () -> new Length();
  private Supplier<Rotation2d> setpointAngle = () -> Rotation2d.kZero;

  protected boolean overrideAppend;

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

  public Ligament withOverrideAppend(boolean override){
    overrideAppend = override;
    return this;
  }
    
  public Ligament withStaticAngle(Rotation2d angle) {
    this.measuredAngle = () -> angle;
    this.setpointAngle = () -> angle;
    return this;
  }

  public Ligament withDynamicAngle(Supplier<Rotation2d> measured, Supplier<Rotation2d> setpoint) {
    this.measuredAngle = measured;
    this.setpointAngle = setpoint;
    return this;
  }

  public Ligament withDynamicAngle(Supplier<Rotation2d> angle) {
    this.measuredAngle = angle;
    this.setpointAngle = angle;
    return this;
  }

  public Ligament withStaticLength(Length length) {
    this.measuredLength = () -> length;
    this.setpointLength = () -> length;
    return this;
  }

  public Ligament withDynamicLength(Supplier<Length> measured, Supplier<Length> setpoint) {
    this.measuredLength = measured;
    this.setpointLength = setpoint;
    return this;
  }

  public Ligament withDynamicLength(Supplier<Length> length) {
    this.measuredLength = length;
    this.setpointLength = length;
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

  protected void update(){
    this.setpointLig.setLength(setpointLength.get().getMeters());
    this.setpointLig.setAngle(setpointAngle.get().getDegrees());
    this.measuredLig.setLength(measuredLength.get().getMeters());
    this.measuredLig.setAngle(measuredAngle.get().getDegrees());
  }
}
