/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.modules.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveReversed,
            DriveConstants.kFrontLeftTurningReversed,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderPort,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveReversed,
            DriveConstants.kFrontRightTurningReversed,
            DriveConstants.kFrontRightTurningAbsoluteEncoderPort,
            DriveConstants.kFrontRightTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kFrontRightTurningAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveReversed,
            DriveConstants.kBackLeftTurningReversed,
            DriveConstants.kBackLeftTurningAbsoluteEncoderPort,
            DriveConstants.kBackLeftTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kBackLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveReversed,
            DriveConstants.kBackRightTurningReversed,
            DriveConstants.kBackRightTurningAbsoluteEncoderPort,
            DriveConstants.kBackRightTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kBackRightTurningAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getSwervePosition());
    private Field2d field = new Field2d();

    public SwerveSubsystem() {
        /**
         * 10:11
         * 1.因gyro啟動時需時間校準
         * 2.等一秒後再重製
         * 3.為避免產生錯誤用try...catch
         * 4.為讓程式不被等的那一秒delay 使用Thread讓程式在另一個核心執行
         */
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwervePosition());
        SmartDashboard.putData("Robot Heading", gyro);
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        field.setRobotPose(getPose());
    }

    public void zeroHeading() {
        System.err.println("reset gyro...");
        gyro.setAngleAdjustment(0);
        gyro.reset();
    }

    public void setHeading(double offset){
        System.err.println("set offset..." + offset);
        gyro.setAngleAdjustment(offset);
    }

    /**
     * @return 取得車頭角度
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);    //轉成同位角(>=0deg <=360deg)
    }

    /**
     * 11:04
     * 陀螺儀要得到機器航向所用
     * @return rotation2d(角度)
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
            return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()};
    }

    public SwerveModulePosition[] getSwervePosition(){
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()};
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition( getRotation2d(),getSwervePosition(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //如果大於最大速度 全部等比例縮小
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        frontLeft.output(desiredStates[0]);
        frontRight.output(desiredStates[1]);
        backLeft.output(desiredStates[2]);
        backRight.output(desiredStates[3]);
    }

    public void resetAllEncoder(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
}
