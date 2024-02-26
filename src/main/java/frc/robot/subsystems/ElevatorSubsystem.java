package frc.robot.subsystems;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;
import frc.robot.modules.*;

// 2*falcon

public class ElevatorSubsystem extends SubsystemBase {

  NeoMotorPIDmodule elevatorl  = new NeoMotorPIDmodule(PortID.elevator_l_neo, IdleMode.kBrake);
  NeoMotorPIDmodule elevatorr = new NeoMotorPIDmodule(PortID.elevator_r_neo, IdleMode.kBrake);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void elevator_stupid_up() {
    elevatorl.setPercentOutput(-0.5);
    elevatorr.setPercentOutput(0.5);
  }

  public void elevator_stupid_down() {
    elevatorl.setPercentOutput(0.3);
    elevatorr.setPercentOutput(-0.3);
  }

  public void elevator_stop() {
    elevatorl.setPercentOutput(0);
    elevatorr.setPercentOutput(0);
  }
}
