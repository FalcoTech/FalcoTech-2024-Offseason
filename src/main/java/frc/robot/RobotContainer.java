// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.IntakeAutos.RunBothIntakes;
import frc.robot.commands.Intake.IntakeAutos.RunIntakeForTime;
import frc.robot.commands.Intake.IntakeAutos.StopBothIntakes;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShooterAutos.RunShooterForTime;
import frc.robot.commands.Shooter.ShooterAutos.RunShooterWithSpeed;
import frc.robot.commands.Shooter.ShooterAutos.SpinUpShooter;
import frc.robot.commands.Shooter.ShooterAutos.StopShooter;
import frc.robot.commands.Swerve.LockWheels;
import frc.robot.commands.Swerve.TeleOpDrive;
import frc.robot.commands.Tilt.ManualTilt;
import frc.robot.commands.Tilt.SetTiltAngleDegrees;
import frc.robot.commands.Tilt.TiltAimToSpeaker;
import frc.robot.commands.Tilt.TiltAutos.AimTiltToSpeakerAuto;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined and initialized here
  public static final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public static final Vision m_visionSubsystem = new Vision();
  public static final Intake m_intakeSubsystem = new Intake();
  public static final Tilt m_tiltSubsystem = new Tilt();
  public static final Shooter m_shooterSubsystem = new Shooter();

  
  private final XboxController Pilot = new XboxController(OperatorConstants.kPilotPort);
  // public final PS4Controller Pilot = new PS4Controller(OperatorConstants.kPilotPort);
  private final XboxController CoPilot = new XboxController(OperatorConstants.kCoPilotPort);

  // The chooser for the autonomous routines
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private boolean kidMode = true;
     
  public RobotContainer() {
    // Pathplanner Warning:
      // Named commands must be registered before the creation of any PathPlanner Autos or Paths. 
      // It is recommended to do this in RobotContainer, after subsystem initialization, but before the creation 
      // of any other commands
    registerNamedCommands();

    configurePilotBindings();
    configureCoPilotBindings();
    configureSmartDashboard();
  }

  // new Trigger(() -> OPERATOR.GETSOMEBUTTON()).onTrue(new COMMAND(ARGUMENTS)); //Template for creating a new command binding
  private void configurePilotBindings() {
    m_swerveSubsystem.setDefaultCommand(new TeleOpDrive( 
      //TODO: Change x and y back and test because I'm stupid - Gavin
      () -> -Pilot.getLeftX(),
      () -> -Pilot.getLeftY(),
      () -> -Pilot.getRightX(),
      () -> Pilot.getLeftBumper(), //Slow Speed
      () -> !Pilot.getRightBumper(), //Field Relative
      () -> Pilot.getXButton(), //Vision Align
      () -> kidMode)); 

    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //XBOX CONTROLLER
    // new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.brakeModules())); //XBOX CONTROLLER
    // new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.coastModules())); //XBOX CONTROLLER
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.switchIdleMode())); //XBOX CONTROLLER
    
    new Trigger(() -> Pilot.getYButton()).whileTrue(new LockWheels()); //XBOX
    // new Trigger(() -> Pilot.getTriangleButton()).whileTrue(new SwerveLockWheels()); //PS4
  }



  // TODO: these bindings target 2024 Crescendo scoring (speaker/amp/note handling) and don't
  // make sense outside the context of that match anymore. Revisit once this robot's demo/practice
  // purpose is decided.
  private void configureCoPilotBindings(){
    m_intakeSubsystem.setDefaultCommand(new RunIntake((() -> (CoPilot.getRightTriggerAxis() - CoPilot.getLeftTriggerAxis()))));

    m_tiltSubsystem.setDefaultCommand(new ManualTilt(() -> (-CoPilot.getRightY() - CoPilot.getLeftY()) < 0 ? Math.min((-CoPilot.getRightY() - CoPilot.getLeftY()), 1) : Math.max((-CoPilot.getRightY() - CoPilot.getLeftY()), -1)));
    // new Trigger(() -> CoPilot.getYButton()).onTrue(new SetTiltAngleDegrees(.04)); //Shoot to speaker
    new Trigger(() -> CoPilot.getYButton()).whileTrue(new TiltAimToSpeaker()); //Auto Aim Tilt
    new Trigger(() -> CoPilot.getXButton()).onTrue(new SetTiltAngleDegrees(.30)); //Shoot to amp
    new Trigger(() -> CoPilot.getPOV() == 180).onTrue(new SetTiltAngleDegrees(0.01)); //tilt to intake

    new Trigger(() -> CoPilot.getPOV() == 0).onTrue(new InstantCommand(() -> m_tiltSubsystem.toggleHangSpeed())); //toggle fast tilt speed for hanging the robot at the endgame of a match which is the last 20 seconds signified by the guitar riff at the end of the match when it is quote unquote: "time to hang"
    

    new Trigger(() -> CoPilot.getStartButton()).whileTrue(new ManualTilt(() -> -.5)); 


    new Trigger(() -> CoPilot.getBButton()).whileTrue(new RunShooter(.7)); 
    new Trigger(() -> CoPilot.getAButton()).whileTrue(new RunShooter(.35)); 
    //Multiple commands can be bound to the same button using command groups
    new Trigger(() -> CoPilot.getLeftStickButton()).onTrue(new InstantCommand(() -> kidMode = !kidMode)); //XBOX CONTROLLER
  }


  // TODO: named commands and the "Amp Auto" chooser option below are all 2024 Crescendo auto
  // routines (speaker/amp/note scoring). They still work, but are meaningless outside that game
  // and should be replaced with routines that make sense for this robot's demo/practice purpose.
  private void registerNamedCommands(){
    //SWERVE
    NamedCommands.registerCommand("Lock Wheels", new LockWheels());

    //INTAKE
    NamedCommands.registerCommand("Start Intake", new RunBothIntakes(1));
    NamedCommands.registerCommand("Stop Intake", new StopBothIntakes());
    NamedCommands.registerCommand("Reverse Intake", new RunBothIntakes(-.75));

    NamedCommands.registerCommand("Intake For 1.5s", new RunIntakeForTime(1, 1.5));
    NamedCommands.registerCommand("Intake For 3s", new RunIntakeForTime(1, 3));
    NamedCommands.registerCommand("Unjam Note", new RunIntakeForTime(-.9, .15));

    //SHOOTER
    NamedCommands.registerCommand("Shoot Speaker", new RunShooterWithSpeed(.35));
    NamedCommands.registerCommand("Stop Shooter", new StopShooter());

    NamedCommands.registerCommand("Spin Up Speaker", new SpinUpShooter(.35));
    NamedCommands.registerCommand("Shoot For 5s", new RunShooterForTime(.35, 5));
    NamedCommands.registerCommand("Spin Up Auto Aim", new SpinUpShooter(.7));

    //TILT
    NamedCommands.registerCommand("Tilt To Intake", new SetTiltAngleDegrees(0.01));
    NamedCommands.registerCommand("Tilt To Speaker", new SetTiltAngleDegrees(.05));
    NamedCommands.registerCommand("Auto Aim Tilt", new AimTiltToSpeakerAuto());
    NamedCommands.registerCommand("Tilt To Amp", new SetTiltAngleDegrees(0.30));
  }

  private void configureSmartDashboard(){
    m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser.addOption("Amp Auto", new PathPlannerAuto("Amp Auto"));
    SmartDashboard.putData("Auto Chooser", m_autoChooser); //send auto chooser to smartdashboard
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //return the selected auto command
  }
  
}
