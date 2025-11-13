// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.ShootNoteAuto2;
import frc.robot.commands.shooter.ShootNoteTargetVisible;
import frc.robot.commands.shooter.SmartLuanch;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.intake.AmpShot;
import frc.robot.commands.intake.CollectNoteV2;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import static edu.wpi.first.units.Units.*;

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

  // public SwerveSubsystem swerveSubsystem;
  public Joystick baseJoystick;
  public Joystick towerJoystick;
  public VisionSubsystem m_visionSubsystem;

  private Shooter m_shooter = new Shooter();
  private Angle m_angle = new Angle();
  private Intake m_intake = new Intake();
  private TurretSubsystem m_turret = new TurretSubsystem();
  private SendableChooser<Command> chooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withDesaturateWheelSpeeds(true) // Add
                                                                                                                 // a
                                                                                                                 // 10%
                                                                                                                 // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final double SPEED_PROPORTION = 0.25;
  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        /**
         * The joystick axies are swapped with drivetrain ones becayse wpi treats the +y
         * as left-side+ and +x as +forward
         * 
         * Keep in mind because the drivetrain is using this new coordinate system the
         * vision subsystem might not work.
         * 
         * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
         */
        drivetrain.applyRequest(
            () -> drive.withVelocityX((-joystick.getRawAxis(JoystickMap.LEFT_Y_AXIS) * MaxSpeed) * SPEED_PROPORTION) // Drive
                // forward
                // with
                // negative Y (forward)
                .withVelocityY((-joystick.getRawAxis(JoystickMap.LEFT_X_AXIS) * MaxSpeed) * SPEED_PROPORTION) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                            // (left)
        ));

    new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER)
        .whileTrue(m_angle.setAngleCommandNew(15).andThen(m_shooter.loadNoteUntilFound2(1500))).onFalse(
            m_intake.setCollectorPower(0));
    new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).whileTrue(m_angle.setAngleCommandNew(2));

    // 4100
    new JoystickButton(baseJoystick, JoystickMap.BUTTON_X)
        .whileTrue(new ShootNoteAuto2(
            31.5, -1200, m_shooter, m_angle,
            m_visionSubsystem).compose());
    new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
        .whileTrue(new ShootNoteAuto2(45, -1200, m_shooter, m_angle,
            m_visionSubsystem).compose());
    new JoystickButton(baseJoystick, JoystickMap.BUTTON_B)
        .whileTrue(new ShootNoteAuto2(49.5, -1200, m_shooter, m_angle,
            m_visionSubsystem).compose());

    new JoystickButton(baseJoystick, JoystickMap.BUTTON_A)
        .whileTrue(new SmartLuanch(m_shooter).andThen(m_angle.setAngleCommandNew(2)));

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
    // swerveSubsystem.resetGyro();
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
