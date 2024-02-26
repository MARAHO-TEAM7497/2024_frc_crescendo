// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.PortID;

public class TalonFxMotorPIDmodule {
  public TalonFX motor;
  public TalonFXConfiguration Config = new TalonFXConfiguration();
  DutyCycleOut DutyCycle = new DutyCycleOut(0);
  VelocityVoltage Velocity = new VelocityVoltage(0);
  PositionVoltage Position = new PositionVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public TalonFxMotorPIDmodule(PortID id, NeutralModeValue NeutralMode) {
    Config.Slot0.kP = id.kP;
    Config.Slot0.kI = id.kI;
    Config.Slot0.kD = id.kD;

    Config.CurrentLimits.SupplyCurrentLimitEnable = id.LIMIT_CURRENT_ENABLE;
    Config.CurrentLimits.SupplyCurrentLimit = id.CONTINUOS_CURRENT_LIMIT;
    Config.CurrentLimits.SupplyCurrentThreshold = id.PEAK_CURRENT_LIMIT;
    Config.CurrentLimits.SupplyTimeThreshold = 0.1;

    motor.setNeutralMode(NeutralMode);
    motor.setInverted(id.reversed);
    motor.setVoltage(id.Voltage);

    Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = (id.ramp_rate);
    Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = (id.ramp_rate);
    Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = (id.ramp_rate);
    Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = (id.ramp_rate);

    Config.Feedback.SensorToMechanismRatio = 1;
    motor.getConfigurator().apply(Config);
  }

  public void cv2Ticks(double cvtTicks) {
    Config.Feedback.SensorToMechanismRatio = cvtTicks;
    motor.getConfigurator().apply(Config);
  }

  public void setVelocity(double Unit) {
    motor.setControl(Velocity.withVelocity(Unit));
  }

  public void stop() {
    motor.setControl(DutyCycle.withOutput(0));
  }

  public void set_position(double Unit) {
    motor.setControl(Position.withPosition(Unit));
  }

  public double get_Velocity() {
    return motor.getVelocity().getValue();
  }

  public void setPercentOutput(double PercentOutput) {
    if (Math.abs(PercentOutput) < 0.05)
      PercentOutput = 0;
    motor.setControl(DutyCycle.withOutput(PercentOutput));
  }

  public void reset_position() {
    motor.setPosition(0);
  }

  public void reset_current_position(double Unit) {
    motor.setPosition(Unit);
  }

  public double get_position() {
    return motor.getPosition().getValue();
  }

  public double get_OutputPercent() {
    return motor.getDutyCycle().getValue();
  }

}
