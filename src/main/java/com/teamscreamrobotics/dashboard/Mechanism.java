package com.teamscreamrobotics.dashboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import java.util.function.Supplier;

/**
 * A named group of {@link Ligament}s forming a kinematic chain for {@link MechanismVisualizer}.
 * The first ligament is anchored at a root; subsequent ones chain from their predecessor unless
 * {@link Ligament#withOverrideAppend(boolean)} is set.
 */
public class Mechanism {

  public Mechanism2d measured;
  public Mechanism2d setpoint;

  public String key;

  private MechanismRoot2d measuredRoot;
  private MechanismRoot2d setpointRoot;

  private Supplier<Translation2d> position = () -> Translation2d.kZero;

  private Ligament[] ligaments;

  /**
   * Creates a mechanism with the given dashboard key and ordered ligament chain.
   *
   * @param key       identifier used as the root name in the {@link Mechanism2d} widget
   * @param ligaments one or more ligaments, in chain order from root outward
   */
  public Mechanism(String key, Ligament... ligaments) {
    this.key = key;
    this.ligaments = ligaments;
  }

  protected void initialize(Mechanism2d measured, Mechanism2d setpoint) {
    this.measured = measured;
    this.setpoint = setpoint;

    measuredRoot = measured.getRoot(key + " Actual", 0, 0);
    setpointRoot = setpoint.getRoot(key + " Setpoint", 0, 0);

    ligaments[0].initialize(0, measuredRoot, setpointRoot);
    if(ligaments.length > 1){
      for(int i = 1; i < ligaments.length; i++){
        if(ligaments[i].overrideAppend){
          ligaments[i].initialize(i, measuredRoot, setpointRoot);
          continue;
        }
        ligaments[i].initialize(i, ligaments[i - 1].measuredLig, ligaments[i - 1].setpointLig);
      }
    }
  }

  /**
   * Sets a fixed root position for the mechanism in the 2D widget.
   *
   * @param position the constant (x, y) position of the root
   */
  public Mechanism withStaticPosition(Translation2d position) {
    this.position = () -> position;
    return this;
  }

  /**
   * Sets a dynamic root position supplier, updated each periodic cycle.
   *
   * @param position supplier for the (x, y) root position
   */
  public Mechanism withDynamicPosition(Supplier<Translation2d> position) {
    this.position = position;
    return this;
  }

  /**
   * Imperatively overrides the root position with a new fixed value.
   *
   * @param position the new (x, y) root position
   */
  public void setPosition(Translation2d position) {
    this.position = () -> position;
  }

  protected void update() {
    for(Ligament lig : ligaments){
      lig.update();
    }
    this.setpointRoot.setPosition(position.get().getX(), position.get().getY());
    this.measuredRoot.setPosition(position.get().getX(), position.get().getY());
  }
}
