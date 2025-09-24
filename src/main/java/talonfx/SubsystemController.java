package talonfx;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class SubsystemController {

  TalonFX master;
  TalonFX[] slaves;

  SubsystemSimulator simulator;
  ProfiledPIDController simController;
  double simOutput;

  SubsystemController(TalonFX master, TalonFX[] slaves, SubsystemSimulator simulator) {
    this.master = master;
    this.slaves = slaves;
  }
}
