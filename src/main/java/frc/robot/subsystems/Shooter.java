// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShootMotor = new TalonFX(ShooterConstants.kLeftShootMotorID);
  private final TalonFX rightShootMotor = new TalonFX(ShooterConstants.kRightShootMotorID);

  /** Creates a new Shooter. */

  public Shooter() {
    // TalonFX has no setInverted(boolean) in the current Phoenix6 API; inversion is set via
    // config instead. Clockwise_Positive is the equivalent of the old setInverted(true).
    var leftMotorConfigs = new MotorOutputConfigs();
    leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leftShootMotor.getConfigurator().apply(leftMotorConfigs);

    leftShootMotor.setControl(new CoastOut());
    rightShootMotor.setControl(new CoastOut());

    rightShootMotor.setControl(new Follower(leftShootMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setShooterSpeed(double speed){
    leftShootMotor.set(speed);
  }
  public void stopShooter(){
    leftShootMotor.set(0);
  }

  public double getShooterSpeed(){
    double leftShooterSpeed = leftShootMotor.getVelocity().refresh().getValueAsDouble();
    double rightShooterSpeed = rightShootMotor.getVelocity().refresh().getValueAsDouble();
    
    return (leftShooterSpeed + rightShooterSpeed) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed() * 60);
    SmartDashboard.putNumber("Left Shooter Speed", leftShootMotor.getVelocity().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Speed", rightShootMotor.getVelocity().refresh().getValueAsDouble());

    SmartDashboard.putNumber("Left Shooter Voltage Draw", leftShootMotor.getMotorVoltage().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Voltage Draw", rightShootMotor.getMotorVoltage().refresh().getValueAsDouble());
  }
}
