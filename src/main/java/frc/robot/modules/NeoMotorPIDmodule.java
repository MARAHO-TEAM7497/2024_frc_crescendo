// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import frc.robot.Constants.PortID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class NeoMotorPIDmodule {
    public CANSparkMax motor;
    public SparkPIDController pidController;
    public RelativeEncoder encoder;

    /** Creates a new ExampleSubsystem. */

    public NeoMotorPIDmodule(PortID id, IdleMode idle_mode) {
        motor = new CANSparkMax(id.port, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(id.reversed);
        motor.setIdleMode(idle_mode);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        pidController.setP(id.kP);
        pidController.setI(id.kI);
        pidController.setD(id.kD);
        pidController.setIZone(id.I_Zone);
        pidController.setFF(id.kF);
        motor.burnFlash();
    }

    public void cv2Ticks_velocity(double cvtTicks_velocity) {
        encoder.setVelocityConversionFactor(cvtTicks_velocity);
    }

    public void cv2Ticks_position(double cvtTicks_position) {
        encoder.setPositionConversionFactor(cvtTicks_position);
    }

    public void setVelocity(double Unit) {
        pidController.setReference(Unit, CANSparkMax.ControlType.kVelocity);
    }

    public void stop() {
        motor.set(0);
    }

    public void set_position(double Unit) {
        pidController.setReference(Unit, CANSparkMax.ControlType.kPosition);
    }

    public double get_Velocity() {
        return encoder.getVelocity();
    }

    public void setPercentOutput(double PercentOutput) {
        if (Math.abs(PercentOutput) < 0.05)
            PercentOutput = 0;
        motor.set(PercentOutput);
    }

    public void reset_position() {
        encoder.setPosition(0);
    }

    public void reset_current_position(double Unit) {
        encoder.setPosition((int) (Unit));
    }

    public double get_position() {
        return encoder.getPosition();
    }
}
