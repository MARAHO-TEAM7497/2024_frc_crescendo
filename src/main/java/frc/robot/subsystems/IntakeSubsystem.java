package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.*;
import frc.robot.Constants.PortID;

// neo*2

public class IntakeSubsystem extends SubsystemBase {

  NeoMotorPIDmodule suck1 = new NeoMotorPIDmodule(PortID.intake_suck1_neo, IdleMode.kBrake);
  NeoMotorPIDmodule suck2 = new NeoMotorPIDmodule(PortID.intake_suck2_neo, IdleMode.kBrake);
  SlewRateLimiter filter = new SlewRateLimiter(0.85);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

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
    suck1.setPercentOutput(-0.5);
    suck2.setPercentOutput(-0.5);
  }

  public void put() {
    suck1.setPercentOutput(0.5);
    suck2.setPercentOutput(0.5);
  }

  public void suckStop() {
    suck1.setPercentOutput(0);
    suck2.setPercentOutput(0);
  }

}
