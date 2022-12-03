package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final Double xMove;
    private final Double yMove;
    private final Double rotationMove;
    private final Timer timer;

    private final ChassisSpeeds movement;

    public AutoCommand(DrivetrainSubsystem drivetrainSubsystem,
                               Double xMovement,
                               Double yMovement,
                               Double rotationMovement) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.xMove = xMovement;
        this.yMove = yMovement;
        this.rotationMove = rotationMovement;
        this.movement = new ChassisSpeeds(xMove, yMove, rotationMove);
        timer = new Timer();

        addRequirements(drivetrainSubsystem);
    }

    public void init(){
        timer.reset();
    }
    public void execute() {
        timer.start();

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        SmartDashboard.putNumber("timer: ", timer.get());
        if(timer.get() < 2){
            m_drivetrainSubsystem.drive(movement);
        }
        else{
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 10));
        }
    }

    
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}