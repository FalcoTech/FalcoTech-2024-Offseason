// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.Vision;

// TODO: this command aims at the 2024 Crescendo speaker specifically (via Tilt's tiltMap, which
// is calibrated to that target's height/geometry). Meaningless outside that game's context -
// revisit once this robot's demo/practice purpose is decided.
public class TiltAimToSpeaker extends Command {
  private final Tilt m_tiltSubsystem = RobotContainer.m_tiltSubsystem;
  private final Vision m_visionSubsystem = RobotContainer.m_visionSubsystem;

  private double tiltMapAngle;
  /** Creates a new TiltAimToSpeaker. */
  public TiltAimToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tiltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.getTV()){
      double distanceToSpeaker = m_visionSubsystem.getDistanceToTarget();
      this.tiltMapAngle = m_tiltSubsystem.findTiltMapAngle(distanceToSpeaker);

      m_tiltSubsystem.setTiltToSetpoint(tiltMapAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tiltSubsystem.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tiltSubsystem.getTiltAngle() == tiltMapAngle;
  }
}
