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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class autoShoot extends Command {

    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();
    static boolean finished=false;

    public autoShoot(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooterSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(intakeSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() { 
        finished=false;
        intakeSubsystem.shoot();
        shooterSubsystem.shoot();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() > 2.0)
            finished = true;
    }

    public static void finish(){
         finished=true;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
