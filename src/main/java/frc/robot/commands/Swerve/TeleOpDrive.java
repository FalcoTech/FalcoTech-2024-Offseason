// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.SwerveSubsystem;



public class TeleOpDrive extends Command {
  private final SwerveSubsystem m_swerveSubsystem = RobotContainer.m_swerveSubsystem;
  private final Vision m_visionSubsystem = RobotContainer.m_visionSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
  private final Supplier<Boolean> slowSpdFunction;
  private final Supplier<Boolean> fieldRelativeFunction;
  private final Supplier<Boolean> alignFunction;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  private final PIDController m_visionRotationPID = new PIDController(.075, 0, 0);
  private boolean kidMode = false;
  private double kidSpeed;

  /** Creates a new SwerveJoystickCommand */
  public TeleOpDrive( 
      Supplier<Double> xSpdFB, Supplier<Double> ySpdLR, Supplier<Double> rotSpd,
      Supplier<Boolean> slowSpeed,
      Supplier<Boolean> fieldRelative, Supplier<Boolean> alignRobot, Supplier<Boolean> kidModeSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.xSpdFunction = xSpdFB;
    this.ySpdFunction = ySpdLR;
    this.rotSpdFunction = rotSpd;
    this.slowSpdFunction = slowSpeed;
    this.fieldRelativeFunction = fieldRelative;
    this.alignFunction = alignRobot;

    kidMode = kidModeSupplier.get();
    
    kidSpeed = 0.25;

    this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond); //Dampen the acceleration
    this.rotLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(RobotContainer.m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the x, y, and rotation speeds from the joystick and apply the scale factor
    //TODO: Change x and y back and test because I'm stupid - Gavin
    double ySpeed = (xSpdFunction.get() * (slowSpdFunction.get() || alignFunction.get() ? SwerveDriveConstants.kTeleopDriveSlowSpeedScale : 1)) * SwerveDriveConstants.kTeleopDriveSpeedScale;
    double xSpeed = (ySpdFunction.get() * (slowSpdFunction.get() || alignFunction.get() ? SwerveDriveConstants.kTeleopDriveSlowSpeedScale : 1)) * SwerveDriveConstants.kTeleopDriveSpeedScale;
    double rotSpeed = (rotSpdFunction.get() * (slowSpdFunction.get() || alignFunction.get() ? SwerveDriveConstants.kTeleopTurnSlowSpeedScale : 1)) * SwerveDriveConstants.kTeleopTurnSpeedScale;
    // Calculate the rotation speed from the triggers and the joystick (will use 1 if you try to hyperdrive) 

    if (kidMode)
    {
     ySpeed = kidSpeed * ySpeed;
     xSpeed = kidSpeed * xSpeed;
     rotSpeed = kidSpeed * rotSpeed;
    }
    // Apply a deadband to the x, y, and rotation speeds
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kPilotDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kPilotDeadband ? ySpeed : 0; 
    rotSpeed = Math.abs(rotSpeed) > OperatorConstants.kPilotDeadband ? rotSpeed : 0;

    // Calculate speeds in meters per second
    xSpeed = xLimiter.calculate(xSpeed) * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    rotSpeed = rotLimiter.calculate(rotSpeed) * SwerveDriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // If the vision align button is pressed, use the vision subsystem to align the robot
    if (alignFunction.get() && m_visionSubsystem.getTV()) { //Checks for align button
      double error = m_visionSubsystem.getTX(); //maybe add or subtract the x speed to it for shooting on the move.
      double alignSpeed = m_visionRotationPID.calculate(error, 0);

      double alignCommanded = Math.min((rotSpeed + alignSpeed), 1);
      rotSpeed = alignCommanded;
    }

    // Create a speed command to send to the drivetrain
    ChassisSpeeds chassisSpeeds;
    if (fieldRelativeFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, rotSpeed, m_swerveSubsystem.getGyroRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    // Send the command to the drivetrain
    SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_swerveSubsystem.setModuleStates(moduleStates, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
