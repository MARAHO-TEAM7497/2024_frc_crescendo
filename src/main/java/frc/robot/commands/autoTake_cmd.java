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
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.IntakeSubsystem;


public class autoTake_cmd extends Command {

    SwerveSubsystem swerveSubsystem;
    Limelight limelight;
    IntakeSubsystem intakeSubsystem;
    ChassisSpeeds chassisSpeeds;
    static boolean finished=false;

    public autoTake_cmd(SwerveSubsystem swerveSubsystem,Limelight limelight,IntakeSubsystem intakeSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this.intakeSubsystem = intakeSubsystem;
        
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
         finished=false;
        if(!limelight.limelighttv()){
            finished=true;
            System.err.println("no cargo finding");
        }
    }

    @Override
    public void execute() {
        System.err.println("take...");

        double x_speed=1.5;
        //double x_speed=0;
        double y_speed=0;
        double x = limelight.limelighttx();
        //double y = limelight.limelightty();

        double turn = x*LimelightConstants.turn_percent;

        //if(y>LimelightConstants.distance||x<LimelightConstants.allow_error) x_speed=1.5;

        if(!intakeSubsystem.press()){
            intakeSubsystem.take();

            chassisSpeeds = new ChassisSpeeds(x_speed,y_speed,-turn);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }
        else{
            finished=true;
            intakeSubsystem.stop();
            swerveSubsystem.stopModules();
        }

    }

    public static void finish(){
         finished=true;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
