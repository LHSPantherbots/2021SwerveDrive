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
import frc.robot.commands.Auto.Straight_1_meter;
import frc.robot.commands.Auto.TestPathAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TestModuleSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  

  //Autonomous path commands
  private final Command doNothing = new RunCommand(()-> m_robotDrive.drive(0, 0, 0, true)).withTimeout(1);
  private final Command straight_1_m = Straight_1_meter.runPath(m_robotDrive);
  private final Command testPathAuto = TestPathAuto.runPath(m_robotDrive);
  //private final Command barrelRace = new BarrelRace(driveTrain, trajectories);
  //private final Command slalom = new Slalom(driveTrain, trajectories)


  //Sendable chooser to select autonomos routine
  SendableChooser<Command> autoChooser = new SendableChooser<>();










  /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();



    //Build Autonomus Chooser
    autoChooser.setDefaultOption("Do Nothing", doNothing);

    Shuffleboard.getTab("Autonomous").add(autoChooser);
    autoChooser.addOption("Straight 1 Meter", straight_1_m);
    autoChooser.addOption("Test path auto", testPathAuto);
    //autoChooser.addOption("Move Backward", midAutoB);

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
    return autoChooser.getSelected();
  }
}
