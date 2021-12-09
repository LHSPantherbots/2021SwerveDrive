package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


public class TestPathAuto {
DriveSubsystem m_robotDrive; 

// public TestPathAuto(DriveSubsystem drive){
//     m_robotDrive = drive;
// }


public static Command runPath(DriveSubsystem m_robotDrive){

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, -0.1), new Translation2d(1, -0.8)),
        new Pose2d(2.4, -1.2, Rotation2d.fromDegrees(-90)),config);
    

    Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.4, -1.2, Rotation2d.fromDegrees(-90)),
        List.of(new Translation2d(2.8, -0.8), new Translation2d(3.4, -0.4)),
        new Pose2d(3.4, -0.2, Rotation2d.fromDegrees(90)),config);


   
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1.0, -1.0), 
                  new Translation2d(2.0, -1.0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.0, 0, new Rotation2d(Math.PI/2)),
            config);

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            //exampleTrajectory,
            trajectoryOne,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                //exampleTrajectory,
                trajectoryTwo,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);



    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(swerveControllerCommand2).andThen(() -> m_robotDrive.drive(0, 0, 0, true));

}


}
