// 就傳動那裡辣
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;
import frc.robot.modules.NeoMotorPIDmodule;

// neo*2
public class IndexSubsystem extends SubsystemBase {

  NeoMotorPIDmodule index1 = new NeoMotorPIDmodule(PortID.index1_neo, IdleMode.kBrake);
  NeoMotorPIDmodule index2 = new NeoMotorPIDmodule(PortID.index2_neo, IdleMode.kBrake);

  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void eat() {
    index1.setPercentOutput(-0.5);
    index2.setPercentOutput(-0.5);
  }

  public void put() {
    index1.setPercentOutput(0.5);
    index2.setPercentOutput(0.5);
  }

  public void indexStop() {
    index1.setPercentOutput(0);
    index2.setPercentOutput(0);
  }

  public void set_NeutralMode(IdleMode mode) {
    index1.motor.setIdleMode(mode);
    index2.motor.setIdleMode(mode);
  }
}
