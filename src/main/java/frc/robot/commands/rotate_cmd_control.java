package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class rotate_cmd_control extends Command {

    double deg;
    ChassisSpeeds chassisSpeeds;
    SwerveSubsystem swerveSubsystem;
    static boolean finished = false;
    double kP = -0.1;

    public rotate_cmd_control(SwerveSubsystem swerveSubsystem, double deg) {
        this.swerveSubsystem = swerveSubsystem;
        this.deg = Math.IEEEremainder(deg, 360);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        double err = deg - swerveSubsystem.getHeading();
        if (Math.abs(err) > Math.abs((deg - swerveSubsystem.getHeading()) - 360)) {
            err = (deg - swerveSubsystem.getHeading()) - 360;
        }if (Math.abs(err) > Math.abs((deg - swerveSubsystem.getHeading()) + 360)) {
            err = (deg - swerveSubsystem.getHeading()) + 360;
        }
        System.err.println("deg err:" + err);
        if (Math.abs(err) > 5) {
            double output = err * kP;
            // if (Math.abs(output) > 2)
            //     output = Math.abs(output) / output * 2;
            SwerveJoystick_Cmd.rotate_cmd_turningSpeed = output;
        } else {
            finish();
        }
    }

    public static void finish() {
        finished = true;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
