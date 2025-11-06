// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.JoystickMap;
import frc.robot.commands.climber.ClimbSequence;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.ShootNoteTargetVisible;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.ClimberR2;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.AmpShot;
import frc.robot.commands.intake.CollectNoteV2;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public SwerveSubsystem swerveSubsystem;
  public Joystick baseJoystick;
  public Joystick towerJoystick;
  public VisionSubsystem m_visionSubsystem;

  private Shooter m_shooter = new Shooter();
  private Angle m_angle = new Angle();
  private Intake m_intake = new Intake();
  private TurretSubsystem m_turret = new TurretSubsystem();
  private ClimberR2 climber_r2 = new ClimberR2();
  private SendableChooser<Command> chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
    m_visionSubsystem = new VisionSubsystem();
    baseJoystick = new Joystick(0);
    towerJoystick = new Joystick(1);
    this.chooser = new SendableChooser<>();

    this.chooser.addOption("None", Commands.none());

    SmartDashboard.putData(chooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    swerveSubsystem
        .setDefaultCommand(
            new JoystickDrive(swerveSubsystem,
                () -> baseJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS),
                () -> baseJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS),
                () -> baseJoystick.getRawAxis(JoystickMap.RIGHT_X_AXIS)));

    new JoystickButton(baseJoystick, JoystickMap.BUTTON_BACK)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.resetGyro()));
    new JoystickButton(baseJoystick, JoystickMap.BUTTON_B).onTrue(new AmpShot(m_intake, swerveSubsystem));
    new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER)
        .whileTrue(new CollectNoteV2(m_intake, m_shooter, m_angle, m_turret)).onFalse(
            m_intake.setCollectorPower(0));
    new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).onTrue(m_intake.setAnglePosition(0));
    new Trigger(() -> baseJoystick.getRawAxis(JoystickMap.RIGHT_TRIGGER) > 0.05)
        .whileTrue(m_intake.setCollectorPower(
            -0.95))
        .onFalse(m_intake.setCollectorPower(0));

    // Tower
    // A BUTTON without the conditional check on a visible apriltag
    // new JoystickButton(towerJoystick, JoystickMap.BUTTON_A)
    // .onTrue(new ParallelCommandGroup(
    // new TurretVectoring(m_turret, m_visionSubsystem, () ->
    // swerveSubsystem.getHeading()),
    // new ShootNote(m_shooter, m_angle, m_turret,
    // () -> m_visionSubsystem.UncorrectedDistance()))
    // .andThen(
    // m_turret.setTargetPositionRaw(0)
    // ));

    // Alternative to non-target visible method
    new JoystickButton(towerJoystick, JoystickMap.BUTTON_A).onTrue(new ShootNoteTargetVisible(
        m_shooter, m_angle, m_turret, m_visionSubsystem, swerveSubsystem,
        () -> m_visionSubsystem.UncorrectedDistance()).andThen(m_turret.setTargetPositionRaw(0)));

    new JoystickButton(towerJoystick, JoystickMap.BUTTON_X)
        .whileTrue(new ShootNoteAuto(
            31.5, -4100, m_shooter, m_angle,
            m_visionSubsystem).compose());
    new JoystickButton(towerJoystick, JoystickMap.BUTTON_Y)
        .whileTrue(new ShootNoteAuto(45, -2800, m_shooter, m_angle,
            m_visionSubsystem).compose());
    new JoystickButton(towerJoystick, JoystickMap.BUTTON_B)
        .whileTrue(new ShootNoteAuto(49.5, -3800, m_shooter, m_angle,
            m_visionSubsystem).compose());

    // Climber Sequence - assumes driver has already extended the climber and
    // position the hooks over the chain
    new JoystickButton(towerJoystick, JoystickMap.RIGHT_BUMPER)
        .onTrue(new ClimbSequence(m_intake, m_turret, climber_r2));

    new JoystickButton(towerJoystick, JoystickMap.LEFT_BUMPER)
        .onTrue(climber_r2.travelToClimberPos(64, 74));

    climber_r2.setDefaultCommand(Commands.run(() -> {
      double left = towerJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS) * -1;
      double right = towerJoystick.getRawAxis(JoystickMap.RIGHT_Y_AXIS) * -1;

      if (Math.abs(right) > 0.5) {
        right = 0.5;
      }

      if (Math.abs(left) > 0.5) {
        left = 0.5;
      }

      this.climber_r2.testPower(right, left);
    }, climber_r2));

    new Trigger(() -> towerJoystick.getRawAxis(
        JoystickMap.RIGHT_TRIGGER) >= 0.05).onTrue(m_angle.setAngleCommandNew(2));
    new Trigger(() -> towerJoystick.getRawAxis(
        JoystickMap.LEFT_TRIGGER) >= 0.05)
        .whileTrue(m_shooter.loadNoteUntilFound2(1000)).onFalse(m_shooter.stopShooter());

    new POVButton(towerJoystick, JoystickMap.POV_LEFT).onTrue(m_turret.setTargetPosition(0));
    new POVButton(towerJoystick, JoystickMap.POV_RIGHT).onTrue(m_turret.setTargetPosition(180));

  }

  public void onDisabled() {
    this.m_turret.setBrakeMode(IdleMode.kCoast);
    m_shooter.stop();
    m_intake.stop();
    m_shooter.stopShooter().initialize();
  }

  public void setupTeleop() {
    this.m_turret.setBrakeMode(IdleMode.kBrake);
    swerveSubsystem.resetGyro();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected().handleInterrupt(() -> {
      m_shooter.stop();
      m_intake.stop();
      m_shooter.stopShooter().initialize();
    });
  }
}
