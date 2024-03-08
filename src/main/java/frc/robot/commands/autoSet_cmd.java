/*
* https://youtu.be/0Xi9yb1IMyA
* learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
* thanks team 6814
*/

package frc.robot.commands;

import java.util.Timer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class autoSet_cmd extends Command {

    static boolean finished=false;
    ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();

    public autoSet_cmd(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.shoot();
        finished=true;
    }

    @Override
    public void execute() {
    }

    public static void finish(){
        finished=true;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
