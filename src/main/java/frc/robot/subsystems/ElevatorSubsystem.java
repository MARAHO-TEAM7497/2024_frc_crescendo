package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;
import frc.robot.modules.*;

// 2*falcon

public class ElevatorSubsystem extends SubsystemBase {

  public TalonFxMotorPIDmodule elevatorl = new TalonFxMotorPIDmodule(PortID.elevator_elongation_l_falcon500,
      NeutralMode.Brake, FeedbackDevice.IntegratedSensor);
  public TalonFxMotorPIDmodule elevatorr = new TalonFxMotorPIDmodule(PortID.elevator_elongation_r_falcon500,
      NeutralMode.Brake, FeedbackDevice.IntegratedSensor);

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
