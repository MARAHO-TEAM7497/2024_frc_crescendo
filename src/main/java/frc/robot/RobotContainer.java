// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystick_Cmd;
import frc.robot.commands.autoSet_cmd;
import frc.robot.commands.autoShoot;
import frc.robot.commands.autoTake_cmd;
import frc.robot.commands.elevator_cmd;
import frc.robot.commands.rotate_cmd;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  // public final IndexSubsystem indexSubsystem = new IndexSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final Limelight limelight = new Limelight();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(0);
  private final CommandXboxController copilotJoystick = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser;
  // private final XboxController xbox = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("autotake", new autoTake_cmd(swerveSubsystem, limelight, intakeSubsystem));
    NamedCommands.registerCommand("shootSet", new autoSet_cmd(shooterSubsystem));
    NamedCommands.registerCommand("shoot", new autoShoot(intakeSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("rotate_2_180", new rotate_cmd(swerveSubsystem, 180));
    NamedCommands.registerCommand("reset_gyro", Commands.runOnce(()->swerveSubsystem.zeroHeading()));
    NamedCommands.registerCommand("rotate_2_0", new rotate_cmd(swerveSubsystem, 0));
    NamedCommands.registerCommand("rotate_2_120", new rotate_cmd(swerveSubsystem, 120));
    NamedCommands.registerCommand("rotate_2_-120", new rotate_cmd(swerveSubsystem, -120));




    


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem.setDefaultCommand(new SwerveJoystick_Cmd(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawButton(9) ? 0 : -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverSpeedAxis),
        () -> copilotJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value),
        () -> copilotJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoystick.getRawButton(OIConstants.kDriverBrakeButtonIdx),
        () -> driverJoystick.getRawButton(OIConstants.kAimCon)
            || driverJoystick.getRawButton(OIConstants.kAimCube)
            || driverJoystick.getRawButton(OIConstants.kAimAprilTag)
            || driverJoystick.getRawButton(OIConstants.kAimReflective)));

    elevatorSubsystem.setDefaultCommand(new elevator_cmd(elevatorSubsystem,
        () -> copilotJoystick.getLeftY(), () -> copilotJoystick.getRightY()));

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

    copilotJoystick.x().whileTrue(Commands.run(() -> intakeSubsystem.take()));
    copilotJoystick.x().onFalse(Commands.runOnce(() -> intakeSubsystem.stop()));
    copilotJoystick.rightBumper().whileTrue(Commands.run(() -> intakeSubsystem.shoot()));
    copilotJoystick.rightBumper().onFalse(Commands.runOnce(() -> intakeSubsystem.stop()));
    copilotJoystick.y().whileTrue(Commands.run(() -> shooterSubsystem.shoot()));
    copilotJoystick.y().onFalse(Commands.runOnce(() -> shooterSubsystem.stop()));
    copilotJoystick.a().whileTrue(Commands.run(() -> shooterSubsystem.amp()));
    copilotJoystick.a().onFalse(Commands.runOnce(() -> shooterSubsystem.stop()));

    copilotJoystick.b().whileTrue(new autoTake_cmd(swerveSubsystem, limelight, intakeSubsystem));
    copilotJoystick.b().whileFalse(Commands.run(() -> autoTake_cmd.finish()));

    copilotJoystick.back().whileTrue(Commands.run(() -> intakeSubsystem.eject()));
    copilotJoystick.back().onFalse(Commands.runOnce(() -> intakeSubsystem.stop()));

    copilotJoystick.povUp().whileTrue(new rotate_cmd(swerveSubsystem, 0));
    copilotJoystick.povUp().onFalse(Commands.runOnce(() -> rotate_cmd.finish()));
    copilotJoystick.povRight().whileTrue(new rotate_cmd(swerveSubsystem, 90));
    copilotJoystick.povRight().onFalse(Commands.runOnce(() -> rotate_cmd.finish()));
    copilotJoystick.povDown().whileTrue(new rotate_cmd(swerveSubsystem, 180));
    copilotJoystick.povDown().onFalse(Commands.runOnce(() -> rotate_cmd.finish()));
    copilotJoystick.povLeft().whileTrue(new rotate_cmd(swerveSubsystem, 270));
    copilotJoystick.povLeft().onFalse(Commands.runOnce(() -> rotate_cmd.finish()));

    new JoystickButton(driverJoystick, 11).onTrue(Commands.runOnce(()->swerveSubsystem.zeroHeading()));
    // new JoystickButton(driverJoystick, 10).onTrue(Commands.runOnce(()->swerveSubsystem.setHeading(90)));
    
    // copilotJoystick.povDown().whileTrue(Commands.run(() -> {
    // }));
    // copilotJoystick.povDown().onFalse(Commands.runOnce(() ->
    // shooterSubsystem.stop_shooter()));
    // copilotJoystick.povUp().whileTrue(Commands.run(() -> {
    // }));
    // copilotJoystick.povUp().onFalse(Commands.runOnce(() ->
    // shooterSubsystem.stop_shooter()));

    // copilotJoystick.rightBumper().whileTrue(Commands.run(() ->
    // elevatorSubsystem.elevator_stupid_up()));
    // copilotJoystick.rightBumper().onFalse(Commands.runOnce(() ->
    // elevatorSubsystem.elevator_stop()));
    // copilotJoystick.leftBumper().whileTrue(Commands.run(() ->
    // elevatorSubsystem.elevator_stupid_down()));
    // copilotJoystick.leftBumper().onFalse(Commands.runOnce(() ->
    // elevatorSubsystem.elevator_stop()));

    // copilotJoystick.y().onTrue(Commands.runOnce(() -> {
    // intakeSubsystem.eat();
    // }));
    // copilotJoystick.y().onFalse(Commands.runOnce(() -> {
    // intakeSubsystem.suckStop();
    // }));
    // copilotJoystick.x().onTrue(Commands.runOnce(() -> {
    // intakeSubsystem.put();
    // }));
    // copilotJoystick.x().onFalse(Commands.runOnce(() -> {
    // intakeSubsystem.suckStop();
    // }));
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
