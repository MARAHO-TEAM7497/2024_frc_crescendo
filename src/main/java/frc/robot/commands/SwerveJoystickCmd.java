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
 
 public class SwerveJoystickCmd extends Command {
 
     private final SwerveSubsystem swerveSubsystem;
     private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
     private final Supplier<Boolean> fieldOrientedFunction;
     private final Supplier<Boolean> SpeedHalfFunction;
     private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
 
     public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
             Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
             Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> SpeedHalfFunction) {
         this.swerveSubsystem = swerveSubsystem;
         this.xSpdFunction = xSpdFunction;
         this.ySpdFunction = ySpdFunction;
         this.turningSpdFunction = turningSpdFunction;
         this.fieldOrientedFunction = fieldOrientedFunction;
         this.SpeedHalfFunction = SpeedHalfFunction;
         this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
         this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
         this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
         addRequirements(swerveSubsystem);
     }
 
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
         // 1. 獲取即時輸入
         double xSpeed = xSpdFunction.get();
         double ySpeed = ySpdFunction.get();
         double turningSpeed = turningSpdFunction.get();
 
         // 2. 處理死區
         xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
         ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
         turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
 
         // 3. 加入濾波器
         if(SpeedHalfFunction.get()){
             xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/2;
             ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/2;
         }
         else{
             xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
             ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
         }
         turningSpeed = turningLimiter.calculate(turningSpeed)
                 * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
 
         // 4. 設定底盤目標速度
         ChassisSpeeds chassisSpeeds;
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
 
         // // 6. 將各SwerveModule輸出
         swerveSubsystem.setModuleStates(moduleStates);
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
 