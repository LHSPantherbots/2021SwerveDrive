// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private double moduleAngleOffset, moduleAngleZero = 0;
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANEncoder m_driveEncoder;

  private CANPIDController m_drivePidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private final CANCoder m_turningEncoder;
  //private final CANCoderConfiguration cancoderConfig;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  //private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( 0.0, 0.26);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPort,
      double angleZero){//, // This is the absolute angle of the module pointing forward 0deg
      //boolean driveEncoderReversed,
      //boolean turningEncoderReversed) {

    moduleAngleZero = angleZero;
    moduleAngleOffset = moduleAngleZero; //This is for possible logic needed to adjust the zero to an offset angle

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_drivePidController = m_driveMotor.getPIDController();


    this.m_driveEncoder = m_driveMotor.getEncoder();

    this.m_turningEncoder = new CANCoder(turningEncoderPort);
    this.m_turningEncoder.configMagnetOffset(-angleZero);

//*********************************************** */

//*
        // PID coefficients
        kP = 0.0002;//5e-5; 
        kI = 0.0;//1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156;//0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
    
        // Smart Motion Coefficients
        maxVel = 5700; // rpm
        maxAcc = 3000;
    
        // set PID coefficients
        m_drivePidController.setP(kP);
        m_drivePidController.setI(kI);
        m_drivePidController.setD(kD);
        m_drivePidController.setIZone(kIz);
        m_drivePidController.setFF(kFF);
        m_drivePidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        m_drivePidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_drivePidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_drivePidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_drivePidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

//************************************************************** */

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveEncoderVelocityMeterPerSec(), new Rotation2d(getModuleAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    //System.out.println("Original State: " + state.toString());
    state = SwerveModuleState.optimize(state, new Rotation2d(getModuleAngleRadians()));
    //System.out.println("Optimized State: " + state.toString());
    // Calculate the drive output from the drive PID controller.
    //final var driveOutput =
    //    driveFeedforward.calculate(state.speedMetersPerSecond) +
    //    m_drivePIDController.calculate(getDriveEncoderVelocityMeterPerSec(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getModuleAngleRadians(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    double motorRpm = getMotorRpmFromDriveVelocity(state.speedMetersPerSecond);

//************ */

    if (Math.abs(state.speedMetersPerSecond)>0.07){
        m_drivePidController.setReference(motorRpm, ControlType.kSmartVelocity);
        //m_drivePidController.setReference(state.speedMetersPerSecond*0, ControlType.kSmartVelocity);
    }else{
        m_drivePidController.setReference(0, ControlType.kSmartVelocity); // adds deadband
    }

  //**************************** */
    m_turningMotor.set(turnOutput);
  }

  public void manualDrive(double drive, double turn){
    m_driveMotor.set(drive);
    m_turningMotor.set(turn);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }


  public double getDriveEncoderPosition(){  //devault unit revolution
    return m_driveEncoder.getPosition();
  }

  public double getDriveEncoderVelocity(){  //defualt unit is rpm
    return m_driveEncoder.getVelocity();
  }

  public double getTurnEncoderPosition(){
    return m_turningEncoder.getAbsolutePosition();
  }

  public double getDriveEncoderPositionMeter(){
    return m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderDistancePerPulse ;
  }

  public double getMotorRpmFromDriveVelocity(double velocity){

    return velocity * 60 / ModuleConstants.kDriveEncoderDistancePerPulse;
  }


  public double getDriveEncoderVelocityMeterPerSec(){
    return getDriveEncoderVelocity() * ModuleConstants.kDriveEncoderDistancePerPulse / 60; //converts rpm to meters/second
  }

  public double getModuleAbsoluteAngle(){
     return m_turningEncoder.getAbsolutePosition();  //returns absolute module angle from 0 to 360
  }
  
  public double getModuleAngle(){
    double rawAngle = getModuleAbsoluteAngle();
    double angle;
    if (rawAngle > 180.0 && rawAngle < 360.0){
      angle = -180 + rawAngle % 180.0;
    }else{
      angle = rawAngle;
    }

    return angle;
  }

  public double getModuleAngleRadians(){
    return getModuleAngle() * Math.PI / 180;
  }

}
