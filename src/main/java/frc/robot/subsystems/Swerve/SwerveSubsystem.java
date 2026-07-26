// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeftModule = new SwerveModule(
    "Front Left",
    DriveChassisConstants.kFrontLeftDriveMotorID,
    DriveChassisConstants.kFrontLeftTurnMotorID,
    DriveChassisConstants.kFrontLeftDriveMotorReversed,
    DriveChassisConstants.kFrontLeftTurnMotorReversed,
    DriveChassisConstants.kFrontLeftAbsoluteEncoderID,
    DriveChassisConstants.kFrontLeftAbsoluteEncoderOffsetRadians,
    DriveChassisConstants.kFrontLeftAbsoluteEncoderReversed);

  private final SwerveModule frontRightModule = new SwerveModule(
    "Front Right",
    DriveChassisConstants.kFrontRightDriveMotorID,
    DriveChassisConstants.kFrontRightTurnMotorID,
    DriveChassisConstants.kFrontRightDriveMotorReversed,
    DriveChassisConstants.kFrontRightTurnMotorReversed,
    DriveChassisConstants.kFrontRightAbsoluteEncoderID,
    DriveChassisConstants.kFrontRightAbsoluteEncoderOffsetRadians,
    DriveChassisConstants.kFrontRightAbsoluteEncoderReversed);

  private final SwerveModule backLeftModule = new SwerveModule(
    "Back Left",
    DriveChassisConstants.kBackLeftDriveMotorID,
    DriveChassisConstants.kBackLeftTurnMotorID,
    DriveChassisConstants.kBackLeftDriveMotorReversed,
    DriveChassisConstants.kBackLeftTurnMotorReversed,
    DriveChassisConstants.kBackLeftAbsoluteEncoderID,
    DriveChassisConstants.kBackLeftAbsoluteEncoderOffsetRadians,
    DriveChassisConstants.kBackLeftAbsoluteEncoderReversed);
    
  private final SwerveModule backRightModule = new SwerveModule(
    "Back Right",
    DriveChassisConstants.kBackRightDriveMotorID,
    DriveChassisConstants.kBackRightTurnMotorID,
    DriveChassisConstants.kBackRightDriveMotorReversed,
    DriveChassisConstants.kBackRightTurnMotorReversed,
    DriveChassisConstants.kBackRightAbsoluteEncoderID,
    DriveChassisConstants.kBackRightAbsoluteEncoderOffsetRadians,
    DriveChassisConstants.kBackRightAbsoluteEncoderReversed);
  
    // private final ADIS16448_IMU gyro = new ADIS16448_IMU();
    private final Pigeon2 pidgy = new Pigeon2(4);
    private final Field2d field = new Field2d();
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      SwerveDriveConstants.kDriveKinematics, 
      getGyroRotation2d(), 
      getModulePositions(), 
      new Pose2d(0, 0, getGyroRotation2d())); //initial pose

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> { 
      try {
        // gyro.calibrate();
    
        Thread.sleep(5000); // Wait for gyro to calibrate,
        zeroGyro();         // then zero it
      } catch (Exception e){
        System.out.println(e);
      }
    }).start();

    brakeModules(); //brake modules on startup

    odometry.update(getGyroRotation2d(), getModulePositions()); //update initial odometry
    field.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getGyroRotation2d()); //put robot on field widget
    SmartDashboard.putData("Field", field); //send field to smartdashboard
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> zeroGyro()).ignoringDisable(true));
    SmartDashboard.putData("Calibrate Gyro", new InstantCommand(() -> calibrateGyro()).ignoringDisable(true));


    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getAbsoluteEncoderRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getAbsoluteEncoderRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getAbsoluteEncoderRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getAbsoluteEncoderRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(getGyroHeading()), null);
      }
    });

    // TODO: this RobotConfig is a placeholder (mass/MOI/wheel COF/current limit are guesses,
    // not measured). Re-export the real values from the PathPlanner GUI
    // (src/main/deploy/pathplanner/settings.json) once this robot's actual mass is known, and
    // switch this back to RobotConfig.fromGUISettings().
    RobotConfig placeholderRobotConfig = new RobotConfig(
      54.0, // robot mass (kg) - TODO: not measured, placeholder
      6.0, // robot MOI (kg*m^2) - TODO: not measured, placeholder
      new ModuleConfig(
        ModuleConstants.kWheelDiaMeterMeters / 2, // wheel radius (m)
        SwerveDriveConstants.kMaxSpeedMetersPerSecond, // max module speed (m/s)
        1.0, // wheel coefficient of friction - TODO: not measured, placeholder
        DCMotor.getNEO(1), // drive motor
        60.0, // drive current limit (amps) - TODO: not confirmed against real breaker/config
        1 // motors per module
      ),
      new Translation2d(SwerveDriveConstants.kWheelBase / 2, SwerveDriveConstants.kTrackWidth / 2), // front left
      new Translation2d(SwerveDriveConstants.kWheelBase / 2, -SwerveDriveConstants.kTrackWidth / 2), // front right
      new Translation2d(-SwerveDriveConstants.kWheelBase / 2, SwerveDriveConstants.kTrackWidth / 2), // back left
      new Translation2d(-SwerveDriveConstants.kWheelBase / 2, -SwerveDriveConstants.kTrackWidth / 2) // back right
    );

    //Configure autobuilder last
    AutoBuilder.configure(
      this::getPose2d, //Position supplier
      this::resetPose, //reset position
      this::getChassisSpeeds, //robot chassisspeeds supplier
      this::swerveDriveChassisSpeedsConsumer, //chassisspeeds consumer (command to drive robot)
      new PPHolonomicDriveController(
        new PIDConstants(1.7, 0.0, 0.0), // robot translation PID
        new PIDConstants(.3, 0.0, 0.0) // robot rotation PID
      ),
      placeholderRobotConfig,
      () -> {
        //mirror auto path for red side
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this); //swewrve subsystem
  }

  public double getGyroHeading(){
    // return Math.IEEEremainder(-gyro.getGyroAngleZ(), 360);
    // Pigeon2.getAngle() was removed; getYaw() is the StatusSignal-based replacement.
    return Math.IEEEremainder(-pidgy.getYaw().refresh().getValueAsDouble(), 360);
  }
  public Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(getGyroHeading());
  }
  public void zeroGyro(){
    // gyro.reset();
    pidgy.reset();
  }
  public void calibrateGyro(){
    // gyro.calibrate();
    // pidgy.setControl(new Calib3d());
  }


  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }
  public void resetPose(Pose2d pose){
    odometry.resetPosition(
      getGyroRotation2d(),
      getModulePositions(),
      pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean ignoreSpeedCheck){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond); //hyperdrive prevention lol
    frontLeftModule.setDesiredState(desiredStates[0], ignoreSpeedCheck);
    frontRightModule.setDesiredState(desiredStates[1], ignoreSpeedCheck);
    backLeftModule.setDesiredState(desiredStates[2], ignoreSpeedCheck);
    backRightModule.setDesiredState(desiredStates[3], ignoreSpeedCheck);
  }

  public void swerveDriveRobotRelative(double xSpdFB, double ySpdLR, double rotSpd){
    double xSpeedCommanded = xSpdFB;
    double ySpeedCommanded = ySpdLR;
    double rotSpeedCommanded = rotSpd;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotSpeedCommanded);

    SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, false);
  }
  public void swerveDriveChassisSpeedsConsumer(ChassisSpeeds speeds){ //for autobuilder, autobuilder gives a chassisspeed, and we extract the x, y, and rot speeds from it
    swerveDriveRobotRelative(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public void brakeModules(){
    frontLeftModule.brakeMotors();
    frontRightModule.brakeMotors();
    backLeftModule.brakeMotors();
    backRightModule.brakeMotors();
  }
  public void coastModules(){
    frontLeftModule.coastMotors();
    frontRightModule.coastMotors();
    backLeftModule.coastMotors();
    backRightModule.coastMotors();
  }
  public Boolean isBrakeMode(){
    return frontLeftModule.isBrakeMode();
  }
  public void switchIdleMode(){
    if (isBrakeMode()){
      coastModules();
    } else {
      brakeModules();
    }
  }

  public void stopModules(){
    frontLeftModule.stopMotors();
    frontRightModule.stopMotors();
    backLeftModule.stopMotors();
    backRightModule.stopMotors();
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      frontLeftModule.getSwerveModuleState(),
      frontRightModule.getSwerveModuleState(),
      backLeftModule.getSwerveModuleState(),
      backRightModule.getSwerveModuleState()
    };
  }
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      frontLeftModule.getSwerveModulePosition(),
      frontRightModule.getSwerveModulePosition(),
      backLeftModule.getSwerveModulePosition(),
      backRightModule.getSwerveModulePosition()
    };
  }

  public ChassisSpeeds getChassisSpeeds(){
    return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  public ChassisSpeeds getFieldRelativeChassisSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(
      getChassisSpeeds().vxMetersPerSecond, 
      getChassisSpeeds().vyMetersPerSecond, 
      getChassisSpeeds().omegaRadiansPerSecond, 
      getGyroRotation2d());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Robot Heading:", getGyroHeading());
    // SmartDashboard.putData("Gyro", gyro);
    SmartDashboard.putData("Gyro", pidgy);
    // SmartDashboard.putBoolean("Gyro Connected", gyro.isConnected());

    odometry.update(getGyroRotation2d(), getModulePositions());
    field.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getGyroRotation2d());

    SmartDashboard.putBoolean("Idle Mode", !isBrakeMode());

    SmartDashboard.putNumber("ChassisSpeeds X", getFieldRelativeChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds Y", getFieldRelativeChassisSpeeds().vyMetersPerSecond);
  }
}
