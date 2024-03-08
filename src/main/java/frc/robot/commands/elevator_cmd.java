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
import frc.robot.subsystems.ElevatorSubsystem;

public class elevator_cmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> ly, ry;

    double l, r;


    public elevator_cmd(ElevatorSubsystem elevatorSubsystem,Supplier<Double> ly,Supplier<Double> ry
    ) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.ly = ly;
        this.ry = ry;
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        l = ly.get();
        r = ry.get();
        
        if(l<0.5&&l>-0.5){
            elevatorSubsystem.elevator_l_stop();
        }
        else if(l<=0.5){
            elevatorSubsystem.elevator_l_stupid_up();
        }
        else{
            elevatorSubsystem.elevator_l_stupid_down();
        }

        if(r<0.5&&r>-0.5){
            elevatorSubsystem.elevator_r_stop();
        }
        else if(r<=0.5){
            elevatorSubsystem.elevator_r_stupid_up();
        }
        else{
            elevatorSubsystem.elevator_r_stupid_down();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
