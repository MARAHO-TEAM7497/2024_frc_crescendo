/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick_Cmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> copilotJoystick_ySpdFunction_left, copilotJoystick_ySpdFunction_right;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, SpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction, BrakeFunction, AutoAimFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    double steering_adjust = 0.0d;

    double xSpeed, ySpeed, turningSpeed;

    ChassisSpeeds chassisSpeeds;

    public SwerveJoystick_Cmd(SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
    Supplier<Double> SpeedFunction, Supplier<Double> copilotJoystick_ySpdFunction_left, Supplier<Double> copilotJoystick_ySpdFunction_right,
    Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> BrakeFunction,
    Supplier<Boolean> AutoAimFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.copilotJoystick_ySpdFunction_left = copilotJoystick_ySpdFunction_left;
        this.copilotJoystick_ySpdFunction_right = copilotJoystick_ySpdFunction_right;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.SpeedFunction = SpeedFunction;
        this.BrakeFunction = BrakeFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.AutoAimFunction = AutoAimFunction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        if (!AutoAimFunction.get()) {
            // 1. 獲取即時輸入
            xSpeed = xSpdFunction.get();
            ySpeed = ySpdFunction.get();
            turningSpeed = turningSpdFunction.get();

            // 2. 處理死區
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. 加入濾波器
            if (BrakeFunction.get()) {
                xSpeed = 0;
                ySpeed = 0;
                turningSpeed = 0;
            } else {
                xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
                        * (-SpeedFunction.get() + 1.1) / 2;
                ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
                        * (-SpeedFunction.get() + 1.1) / 2;

                if (copilotJoystick_ySpdFunction_right.get() != 0 || copilotJoystick_ySpdFunction_left.get() != 0) {
                    ySpeed = copilotJoystick_ySpdFunction_right.get() - copilotJoystick_ySpdFunction_left.get();
                    ySpeed /= 2;
                }
                turningSpeed = turningLimiter.calculate(turningSpeed) * 0.8
                        * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
                        * (-SpeedFunction.get() + 1.1) / 2;
            }

            // 4. 設定底盤目標速度
            SmartDashboard.putBoolean("fieldOrientedFunction", fieldOrientedFunction.get());
            if (fieldOrientedFunction.get()) {
                // 相對場地
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            } else {
                // 相對機器人
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }

            // 5. 轉換成各SwerveModule狀態
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. 將各SwerveModule輸出
            swerveSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
