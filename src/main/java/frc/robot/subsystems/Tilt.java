// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Tilt extends SubsystemBase {
  private final SparkMax leftTiltMotor = new SparkMax(TiltConstants.kLeftTiltMotorID, MotorType.kBrushless);
  private final SparkMax rightTiltMotor = new SparkMax(TiltConstants.kRightTiltMotorID, MotorType.kBrushless);

  private final DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(TiltConstants.kTiltEncoderDIOPort);

  private final PIDController m_tiltPID = new PIDController(TiltConstants.kLiftPID_P, TiltConstants.kLiftPID_I, TiltConstants.kLiftPID_D);

  private final InterpolatingDoubleTreeMap tiltMap = new InterpolatingDoubleTreeMap();

  public boolean HANGSPEED = false;

  /** Creates a new Tilt. */
  public Tilt() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftTiltMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.follow(leftTiltMotor, true);
    rightTiltMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("Reset Tilt Encoder", new InstantCommand(() -> resetTiltEncoder()).ignoringDisable(true));
    // SmartDashboard.putBoolean("FAST TILT (HANG SPEED)", false);

    // TODO: this map is calibrated to shoot at the 2024 Crescendo speaker specifically
    // (distance-to-target -> tilt angle). Meaningless outside that game's context - revisit
    // once this robot's demo/practice purpose is decided.
    tiltMap.put(1.44, .03);
    tiltMap.put(1.6, .03);
    tiltMap.put(1.8, .04);
    tiltMap.put(2.0, .05);
    tiltMap.put(2.2, .06);
    tiltMap.put(2.4, .06);
    tiltMap.put(2.6, .06);

    tiltMap.put(2.8, .07); //maybe
    tiltMap.put(3.0, .07); //def not lol
  }
  
  public void moveTilt(double speed){
    leftTiltMotor.set(speed);
  }
  public void stopTilt(){
    leftTiltMotor.set(0);
  }

  public double getTiltAngle(){
    // getDistance() was removed from DutyCycleEncoder; get() returns the same raw rotations
    // this code always effectively used since distance-per-rotation was never configured.
    return tiltEncoder.get() + .01;
  }
  public void setTiltToSetpoint(double setpoint){
    double pidOutput = m_tiltPID.calculate(getTiltAngle(), setpoint); 
    double pidCommanded = 0;
    if (pidOutput > 0){
      pidCommanded = Math.min(pidOutput, .3);
    } else if (pidOutput < 0){
      pidCommanded = Math.max(pidOutput, -.22);
    }
    
    leftTiltMotor.set(pidCommanded);
  }
  public double findTiltMapAngle(double distance){
    return tiltMap.get(distance);
  }

  public void resetTiltEncoder(){
    // TODO: DutyCycleEncoder.reset() no longer exists and has no direct replacement in the
    // 2026 API. This is now a no-op; the "Reset Tilt Encoder" dashboard button does nothing.
    // Revisit if tilt zeroing is needed again (e.g. re-zero via setAssumedFrequency or a
    // software offset tracked separately).
  }

  public void toggleHangSpeed(){
    if (HANGSPEED == true){
      HANGSPEED = false;
    } else {
      HANGSPEED = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Encoder Angle", getTiltAngle());
    SmartDashboard.putNumber("Tilt Encoder Setpoint", m_tiltPID.getSetpoint());
    SmartDashboard.putBoolean("Tilt Encoder Connected", tiltEncoder.isConnected());

    SmartDashboard.putNumber("LT Output %", leftTiltMotor.getAppliedOutput());
    SmartDashboard.putNumber("RT Output %", rightTiltMotor.getAppliedOutput());
    SmartDashboard.putNumber("LT Amp Draw ", leftTiltMotor.getOutputCurrent());
    SmartDashboard.putNumber("RT Amp Draw ", rightTiltMotor.getOutputCurrent());
  }
}
