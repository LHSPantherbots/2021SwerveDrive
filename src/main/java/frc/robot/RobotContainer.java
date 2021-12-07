// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TestModuleSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final TestModuleSubsystem test_module = new TestModuleSubsystem();
 
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final Command justMove = new RunCommand(()-> m_robotDrive.drive(0.0, 1.0, 0.0, true)).withTimeout(3.0);
 
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_driverController.getY(GenericHID.Hand.kLeft)
            * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_driverController.getX(GenericHID.Hand.kLeft)
            * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_driverController.getX(GenericHID.Hand.kRight)
            * DriveConstants.kMaxSpeedMetersPerSecond;

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                  -m_driverController.getY(GenericHID.Hand.kLeft)
                  * DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_driverController.getX(GenericHID.Hand.kLeft)
            * DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_driverController.getX(GenericHID.Hand.kRight)
            * DriveConstants.kMaxSpeedMetersPerSecond,
                    
                    true), m_robotDrive));
    // test_module.setDefaultCommand(
    //   new RunCommand(
    //       () ->
    //          test_module.drive(
    //             -m_driverController.getY(GenericHID.Hand.kLeft),
    //             -m_driverController.getX(GenericHID.Hand.kLeft),
    //             -m_driverController.getX(GenericHID.Hand.kRight),
    //             true),
    //             test_module
    //          ));

    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return justMove;

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    var trajectoryOne =
  TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    List.of(new Translation2d(1, -0.1), new Translation2d(1, -0.8)),
    new Pose2d(2.4, -1.2, Rotation2d.fromDegrees(-90)),config);
    

  var trajectoryTwo =
  TrajectoryGenerator.generateTrajectory(
    new Pose2d(2.4, -1.2, Rotation2d.fromDegrees(-90)),
    List.of(new Translation2d(2.8, -0.8), new Translation2d(3.4, -0.4)),
    new Pose2d(3.4, -0.2, Rotation2d.fromDegrees(90)),config);


  var concatTraj = trajectoryOne.concatenate(trajectoryTwo);
    
    
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

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
