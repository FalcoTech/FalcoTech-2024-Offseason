// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private final String moduleName;

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;
  private final boolean driveMotorReversed;
  private final boolean turnMotorReversed;

  private final RelativeEncoder driveEncoder;

  private final PIDController turnPID;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  /** Creates a new SwerveModule. */
    public SwerveModule(
        String moduleName,
        int driveMotorID, int turnMotorID, boolean isDriveMotorReversed, boolean isTurnMotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed
    ){
    
    this.moduleName = moduleName;

    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

    this.driveMotorReversed = isDriveMotorReversed;
    this.turnMotorReversed = isTurnMotorReversed;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveMotorReversed);
    driveConfig.encoder
      .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter) // Encoder ticks to meters
      .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); // Encoder RPM to meters per second
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.inverted(turnMotorReversed);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset; //magnet offset
    this.absoluteEncoderReversed = isAbsoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderID);
    
    turnPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    turnPID.enableContinuousInput(-Math.PI, Math.PI); // Tells PID that the input is continuous

    resetEncoders();
  }

  public double getDriveMeters(){
    return driveEncoder.getPosition();
  }
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRadians(){
    double angle = absoluteEncoder.getPosition().refresh().getValueAsDouble() * 2 * Math.PI; // Encoder rotations * 2 * pi = radians
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  public double getAbsoluteEncoderVelocity(){
    double velocity = absoluteEncoder.getVelocity().refresh().getValueAsDouble() * 2 * Math.PI; 
    return velocity * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
  }


  public SwerveModuleState getSwerveModuleState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRadians()));
  }
  public SwerveModulePosition getSwerveModulePosition(){
    return new SwerveModulePosition(getDriveMeters(), new Rotation2d(getAbsoluteEncoderRadians()));
  }
  public void setDesiredState(SwerveModuleState state, boolean ignoreSpeedCheck){
    if (Math.abs(state.speedMetersPerSecond) < 0.01 && !ignoreSpeedCheck){ // If the speed is less than 0.01 m/s, and we aren't purposely trying to do that stop the module
      stopMotors();
      return;
    }

    state = SwerveModuleState.optimize(state, getSwerveModuleState().angle); // Calculate the shortest path to the desired angle
    driveMotor.set(state.speedMetersPerSecond / SwerveDriveConstants.kMaxSpeedMetersPerSecond); // Set the drive motor to the desired speed
    turnMotor.set(turnPID.calculate(getAbsoluteEncoderRadians(), state.angle.getRadians())); // Set the turn motor to the desired angle

    SmartDashboard.putString(moduleName + " state", state.toString()); // Desired state debug info
  }



  public void brakeMotors(){
    setIdleMode(IdleMode.kBrake);
  }
  public void coastMotors(){
    setIdleMode(IdleMode.kCoast);
  }
  private void setIdleMode(IdleMode idleMode){
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode);
    driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public Boolean isBrakeMode(){
    return driveMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
  }
  public void stopMotors(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(moduleName + " AbsEnc Rad", getAbsoluteEncoderRadians());
    SmartDashboard.putNumber(moduleName + " Velocity", getDriveVelocity());
  }
}
