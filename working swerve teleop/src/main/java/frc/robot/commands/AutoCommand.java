package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// autonomous stuff
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;//configures the trajectory 
import java.util.List;
import edu.wpi.first.math.trajectory.Trajectory; 
import edu.wpi.first.math.trajectory.TrajectoryGenerator; 




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


    public AutoCommand(TrajectoryConfig config, double nX, double nY, double nR, double fX, double fY, double fR, double distance1, double rotation1, double distance2, double rotation2){
    // IDK!!! IM SORRY ELIMY 
    // CREATING CONFIGURATION FOR THE TRAJECTORY 
    //TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared); //constraints
    //Pose2d poseGurlStart = new Pose2d(0, 0, new Rotation2d(0)); // starting location of the wheels???
   // Pose2d poseGurlStop = new Pose2d(2, 3, new Rotation2d(0)); // ending position
    Trajectory moveGurl = TrajectoryGenerator.generateTrajectory(new Pose2d(nX, nY, new Rotation2d(nR)), List.of(new Translation2d(distance1, new Rotation2d(rotation1)), new Translation2d(distance2, new Rotation2d(rotation2))), new Pose2d(fX, fY, new Rotation2d(fR)), config);
    }

    // DEAR EMILY I AM ALMOST DONE IM SORRY THERE WERE SOME ROADBLOCKS IF YOU SEE THIS DURING 7TH PERIOD CAN YOU TRY RUN THIS AUTOCOMMAND IN ROBOTCONTAINER AND ROBOT
 

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