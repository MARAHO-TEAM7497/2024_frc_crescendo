// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static enum PortID {
        elevator_r_neo(0, false, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        elevator_l_neo(1, false, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        shooter_down_kraken(20, false, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        shooter_up_kraken(21, false, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        intake_suck_up_kraken(22, true, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0.2, 0),
        intake_suck_down_kraken(23, true, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0.2, 0),
        digital_intake(0, false, 10.0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0),;

        public final int port;
        public final boolean reversed;
        public final double Voltage;
        public final boolean LIMIT_CURRENT_ENABLE;
        public final double CONTINUOS_CURRENT_LIMIT;
        public final double PEAK_CURRENT_LIMIT;
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int I_Zone;
        public final double ramp_rate;
        public final int allow_error;

        /**
         * port, reversed, kP, kI, kD, kF, I_Zone, ramp_rate, allow_error
         */
        PortID(int port, boolean reversed, double Voltage, boolean LIMIT_CURRENT_ENABLE, double CONTINUOS_CURRENT_LIMIT,
                double PEAK_CURRENT_LIMIT, double kP, double kI, double kD, double kF, int I_Zone, double ramp_rate,
                int allow_error) {
            this.port = port;
            this.reversed = reversed;
            this.Voltage = Voltage;
            this.LIMIT_CURRENT_ENABLE = LIMIT_CURRENT_ENABLE;
            this.PEAK_CURRENT_LIMIT = PEAK_CURRENT_LIMIT;
            this.CONTINUOS_CURRENT_LIMIT = CONTINUOS_CURRENT_LIMIT;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.I_Zone = I_Zone;
            this.ramp_rate = ramp_rate;
            this.allow_error = allow_error;
        }
    }

    public final static int kTIMEOUT = 10;

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kTurningEncoderTicksPerPulse2RPS = 1.0 / 409.6;
        public static final double kDriveEncoderTicksPerPulse2RPS = 1.0 / 204.8;
        public static final double kTurningEncoderTicks2Rot = 1.0 / 4096;
        public static final double kDriveMotorGearRatio = 1 / 6.12;
        public static final double kTurningMotorGearRatio = 1;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPS2MeterPerSec = kDriveEncoderRot2Meter;
        public static final double kTurningEncoderRPS2RadPerSec = kTurningEncoderRot2Rad;
        public static final double kPTurning = 0.3;

        public static final double kP = 0.15;
        public static final double kI = 0.0001;
        public static final double kD = 3;
        public static final double kF = 0.06;
        public static final double I_Zone = 0;
        public static final double allow_error = 3000;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(23.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 5;

        public static final int kFrontLeftTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 6;

        public static final int kFrontLeftTurningAbsoluteEncoderPort = 10;
        public static final int kBackLeftTurningAbsoluteEncoderPort = 12;
        public static final int kFrontRightTurningAbsoluteEncoderPort = 9;
        public static final int kBackRightTurningAbsoluteEncoderPort = 11;

        public static final boolean kFrontLeftTurningReversed = false;
        public static final boolean kBackLeftTurningReversed = false;
        public static final boolean kFrontRightTurningReversed = false;
        public static final boolean kBackRightTurningReversed = false;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kBackRightDriveReversed = true;

        public static final boolean kFrontLeftTurningAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftTurningAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightTurningAbsoluteEncoderReversed = true;
        public static final boolean kBackRightTurningAbsoluteEncoderReversed = true;

        public static final double kFrontLeftTurningAbsoluteEncoderOffsetTicks = 1040;
        public static final double kBackLeftTurningAbsoluteEncoderOffsetTicks = 413;
        public static final double kFrontRightTurningAbsoluteEncoderOffsetTicks = 1014;
        public static final double kBackRightTurningAbsoluteEncoderOffsetTicks = 2888;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm(), // Drive  base radius (distance from center to furthest module)
                new ReplanningConfig());
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverSpeedAxis = 3;
        public static final int kDriverFieldOrientedButtonIdx = 2;
        public static final int kResetHeading = 11;
        public static final int kDriverBrakeButtonIdx = 1;

        public static final int kAimCon = 3;
        public static final int kAimCube = 4;
        public static final int kAimAprilTag = 5;
        public static final int kAimReflective = 6;

        public static final double kDeadband = 0.1;
    }

    public static enum limelight_pipeline {
        aprilTag(0),
        reflective(1),
        cube(2),
        con(3),
        driver(4);

        public final int value;

        /**
         * port, reversed, kP, kI, kD, kF, I_Zone, ramp_rate, allow_error
         */
        limelight_pipeline(int value) {
            this.value = value;
        }
    }

    public static final class shooterConstant {
        public static final double meter2velocity(double meter) {
            return meter * (15 + 0 + 13 + 3 + 0); // 彩蛋
        }

        public static final double meter2angle(double meter) {
            return meter * (2 + 17 + 4 + 18 + 2 + 4 + 13 + 3 + 14); // 彩蛋
        }

        public static final int eat_position = 12 + 0 + 17 + 0 + 7 + 14; // 我不知道怎麼設計 就是調到可以吃note的位置
    }
}
