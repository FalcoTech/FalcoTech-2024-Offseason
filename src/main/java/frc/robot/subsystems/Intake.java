// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.CANSparkMax;

// import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final VictorSPX frontIntakeMotor;
  private final VictorSPX rearIntakeMotor;

  private final VictorSPX transferMotor;

  // private final LaserCan lowerLaserCan;
  // private final LaserCan upperLaserCan;
  
  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new VictorSPX(IntakeConstants.kFrontIntakeMotorID);
    rearIntakeMotor = new VictorSPX(IntakeConstants.kRearIntakeMotorID);
    frontIntakeMotor.setNeutralMode(NeutralMode.Coast);
    rearIntakeMotor.setNeutralMode(NeutralMode.Coast);
    frontIntakeMotor.setInverted(true);
    rearIntakeMotor.setInverted(true);

    rearIntakeMotor.follow(frontIntakeMotor);

    transferMotor = new VictorSPX(IntakeConstants.kTransferMotorID);
    transferMotor.setNeutralMode(NeutralMode.Coast);
    transferMotor.setInverted(true);

    // lowerLaserCan = new LaserCan(IntakeConstants.kLowerLaserCanID);
    // upperLaserCan = new LaserCan(IntakeConstants.kUpperLaserCanID);
    SmartDashboard.putBoolean("LASERCAN CHECKS", true);
  }

  public void setIntakeSpeed(double speed){
    frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void stopIntake(){
    frontIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setTransferSpeed(double speed){
    transferMotor.set(ControlMode.PercentOutput, speed);
  }
  public void stopTransfer(){
    transferMotor.set(ControlMode.PercentOutput, 0);
  }

  // public boolean getTransferReady(){
  //   if (lowerLaserCan.getMeasurement() != null){
  //       return lowerLaserCan.getMeasurement().distance_mm < 229; 
  //   } else {
  //     return false;
  //   }
  // }
  // public boolean getNoteReady(){
  //   if (upperLaserCan.getMeasurement() != null){
  //     return upperLaserCan.getMeasurement().distance_mm < 229;
  //   } else {
  //     return false;
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lower Intake Output %", frontIntakeMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Transfer Motor Output %", transferMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Lower Intake Power Consumption", frontIntakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Transfer Motor Power Consumption", transferMotor.getMotorOutputVoltage());

    // SmartDashboard.putBoolean("Transfer Ready", getTransferReady());
    // SmartDashboard.putBoolean("Note Ready", getNoteReady());
  }
}
