package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;
import frc.robot.Constants.shooterConstant;
import frc.robot.modules.*;
//2*falcon
//應該還要有一顆調仰角的 我就當他是neo了

public class ShooterSubsystem extends SubsystemBase {

  public TalonFxMotorPIDmodule shooterUp = new TalonFxMotorPIDmodule(PortID.shooter_up_kraken, NeutralModeValue.Coast, 10, -10);
  public TalonFxMotorPIDmodule shooterDown = new TalonFxMotorPIDmodule(PortID.shooter_down_kraken,
      NeutralModeValue.Coast, 10, -10);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void shoot() {
    shooterUp.set_voltage(-10);
    shooterDown.set_voltage(-10);
  }

  public void amp() {
    shooterUp.set_voltage(4);
    shooterDown.set_voltage(-4);
  }

  public void stop() {
    shooterUp.stop();
    shooterDown.stop();
  }

  public void set_NeutralMode(NeutralModeValue mode){
    shooterUp.motor.setNeutralMode(mode);
    shooterDown.motor.setNeutralMode(mode);
  }
}