package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;
import frc.robot.modules.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;
// 2*falcon

public class ElevatorSubsystem extends SubsystemBase {

  NeoMotorPIDmodule elevatorl = new NeoMotorPIDmodule(PortID.elevator_l_neo, IdleMode.kBrake);
  NeoMotorPIDmodule elevatorr = new NeoMotorPIDmodule(PortID.elevator_r_neo, IdleMode.kBrake);
  // public TalonFX left_motor = new TalonFX(IDConstants.elevator_l);
  // public TalonFX right_motor = new TalonFX(IDConstants.elevator_r);
  // public DutyCycleOut left = new DutyCycleOut(0);
  // public DutyCycleOut right = new DutyCycleOut(0);
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

  public void elevator_l_stupid_up() {
    elevatorl.setPercentOutput(-1);
  }

  public void elevator_r_stupid_up() {
    elevatorr.setPercentOutput(-1);
  }

  public void elevator_l_stupid_down() {
    elevatorl.setPercentOutput(0.5);
  }

  public void elevator_r_stupid_down() {
    elevatorr.setPercentOutput(0.5);
  }
  public void elevator_l_stop() {
    elevatorl.setPercentOutput(0);
  }
  public void elevator_r_stop() {
    elevatorr.setPercentOutput(0);
  }





  // public void elevator_stop() {
  //   elevatorl.setPercentOutput(0);
  //   elevatorr.setPercentOutput(0);
  // }
  // public void stop_left() {
  //   left_motor.setControl(left.withOutput(0));
  // }
  // public void stop_right() {
  //   right_motor.setControl(right.withOutput(0));
  // }
  // public void up_left() {
  //   left_motor.setControl(left.withOutput(0.4));
  // }
  // public void up_right() {
  //   right_motor.setControl(right.withOutput(0.4));
  // }
  // public void down_left() {
  //   left_motor.setControl(left.withOutput(-0.4));
  // }
  // public void down_right() {
  //   right_motor.setControl(right.withOutput(-0.4));
  // }
}
