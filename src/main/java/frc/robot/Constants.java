// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 21;
    public static final int kRearLeftTurningMotorPort = 22;
    public static final int kFrontRightTurningMotorPort = 23;
    public static final int kRearRightTurningMotorPort = 24;

    public static final int kFrontLeftTurningEncoderPort = 11;
    public static final int kRearLeftTurningEncoderPort = 12;
    public static final int kFrontRightTurningEncoderPort = 13;
    public static final int kRearRightTurningEncoderPort = 14;

    public static final double kFrontLeftAngleZero = 353.9;
    public static final double kRearLeftAngleZero = 130.6;
    public static final double kFrontRightAngleZero = 245.6;
    public static final double kRearRightAngleZero = 180.7;


    //public static final boolean kFrontLeftTurningEncoderReversed = false;
    //public static final boolean kRearLeftTurningEncoderReversed = true;
    //public static final boolean kFrontRightTurningEncoderReversed = false;
    //public static final boolean kRearRightTurningEncoderReversed = true;



    //public static final boolean kFrontLeftDriveEncoderReversed = false;
    //public static final boolean kRearLeftDriveEncoderReversed = true;
    //public static final boolean kFrontRightDriveEncoderReversed = false;
    //public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 25 * 0.0254;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 25 * 0.0254;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kEncoderCPR = 1.0; //default unit for spark max distance is 1 revolution
    public static final double driveGearReduction = 50/14 * 16/28 * 60/15; //Gear tooth counts train on MK3 swerve module
    public static final double kWheelDiameterMeters = 6 * .0254; //6" wheel
    public static final double kDriveEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR *driveGearReduction); //converts motor rpm to meters wheel traveled

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.4;//1

    public static final double kPModuleDriveController = 0.2;//1
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
