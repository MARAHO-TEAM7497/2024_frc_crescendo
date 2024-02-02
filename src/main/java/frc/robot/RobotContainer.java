// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystick_Cmd;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final IndexSubsystem indexSubsystem = new IndexSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final Limelight limelight = new Limelight();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(0);
  private final CommandXboxController copilotJoystick = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem.setDefaultCommand(new SwerveJoystick_Cmd(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawButton(9) ? 0 : driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverSpeedAxis),
        () -> copilotJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value),
        () -> copilotJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoystick.getRawButton(OIConstants.kDriverBrakeButtonIdx),
        () -> driverJoystick.getRawButton(OIConstants.kAimCon)
            || driverJoystick.getRawButton(OIConstants.kAimCube) ||
            driverJoystick.getRawButton(OIConstants.kAimAprilTag)
            || driverJoystick.getRawButton(OIConstants.kAimReflective)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // 全手動
    copilotJoystick.povDown().whileTrue(Commands.run(() -> {
    }));
    copilotJoystick.povDown().onFalse(Commands.runOnce(() -> shooterSubsystem.stop_shooter()));
    copilotJoystick.povUp().whileTrue(Commands.run(() -> {
    }));
    copilotJoystick.povUp().onFalse(Commands.runOnce(() -> shooterSubsystem.stop_shooter()));

    copilotJoystick.rightBumper().whileTrue(Commands.run(() -> elevatorSubsystem.elevator_stupid_up()));
    copilotJoystick.rightBumper().onFalse(Commands.runOnce(() -> elevatorSubsystem.elevator_stop()));
    copilotJoystick.leftBumper().whileTrue(Commands.run(() -> elevatorSubsystem.elevator_stupid_down()));
    copilotJoystick.leftBumper().onFalse(Commands.runOnce(() -> elevatorSubsystem.elevator_stop()));

    copilotJoystick.y().onTrue(Commands.runOnce(() -> {
      intakeSubsystem.eat();
    }));
    copilotJoystick.y().onFalse(Commands.runOnce(() -> {
      intakeSubsystem.suckStop();
    }));
    copilotJoystick.x().onTrue(Commands.runOnce(() -> {
      intakeSubsystem.put();
    }));
    copilotJoystick.x().onFalse(Commands.runOnce(() -> {
      intakeSubsystem.suckStop();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
