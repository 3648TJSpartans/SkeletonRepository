// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumber;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;
import frc.robot.subsystems.absoluteEncoder.AbsEncoderConstants;
import frc.robot.subsystems.absoluteEncoder.AbsEncoderSparkMax;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;
// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntakeIO;
// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntakeIOSparkMax;
// import frc.robot.subsystems.coralSubsystems.elevator.Elevator;
// import frc.robot.subsystems.coralSubsystems.CoralConstants;
// import frc.robot.subsystems.coralSubsystems.elevator.ElevatorIO;
// import frc.robot.subsystems.coralSubsystems.elevator.ElevatorIOSparkMax;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private boolean override;
  private boolean endgameClosed = true;

  // Controller
  private final CommandXboxController m_driveController = new CommandXboxController(0);
  private final CommandXboxController m_copilotController = new CommandXboxController(1);
  private final CommandXboxController m_controllerTwo = new CommandXboxController(2);
  private final CommandXboxController m_ledController = new CommandXboxController(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1",
      30.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2",
      15.0);
  private final LoggedNetworkNumber endgameAlert3 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #3",
      5.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // m_led = new LedSubsystem();
    Logger.recordOutput("Poses/shouldFlip", AllianceFlipUtil.shouldFlip());
    Logger.recordOutput("Override", override);
    override = false;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
        // m_vision = new Vision(
        // m_drive::addVisionMeasurement,
        // new VisionIOLimelight("limelight-three", m_drive::getRotation),
        // new VisionIOLimelight("limelight-twoplus", m_drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        m_drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        // m_vision = new Vision(
        // m_drive::addVisionMeasurement,
        // // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0,
        // // m_drive::getPose),
        // new VisionIOPhotonVisionSim(VisionConstants.camera1Name,
        // VisionConstants.robotToCamera1,
        // m_drive::getPose));
        // To later be replaced with CoralIntakeIOSim
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        // m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {
        // }, new VisionIO() {
        // });
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutoChooser();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // configureAutos();

    configureLeds();
    // configureClimber();
    // configureCoralIntake();
    // configureDrive();
    // configureElevator();
    // configureSetpoints();
    // configureEndgameTriggers();
    // configureSft();

    m_copilotController.rightTrigger().onTrue(new InstantCommand(() -> toggleOverride()));
    /*
     * m_led.setLedPattern(LedConstants.elevatorHeight, m_led.elevatorBuffer);
     * m_led.setLedPattern(LedConstants.teal, m_led.leftGuideBuffer);
     * m_led.setLedPattern(LedConstants.yellow, m_led.rightGuideBuffer);
     */
  }

  private void configureAlerts() {
    new Trigger(
        () -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0
            && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .andThen(Commands.waitSeconds(4.75))
                .repeatedly()
                .withTimeout(15)

        // .beforeStarting(() -> leds.endgameAlert = true)
        // .finallyDo(() -> leds.endgameAlert = false)
        );
    new Trigger(
        () -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0
            && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(8)
        // .beforeStarting(() -> leds.endgameAlert = true)
        // .finallyDo(() -> leds.endgameAlert = false)
        );
    new Trigger(
        () -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0
            && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.3))
                .repeatedly()
                .withTimeout(10)
        // .beforeStarting(() -> leds.endgameAlert = true)
        // .finallyDo(() -> leds.endgameAlert = false)
        );
    // Countdown
    new Trigger(
        () -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0
            && DriverStation.getMatchTime() <= Math.round(endgameAlert3.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.8)
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly()
                .withTimeout(5)
        // .beforeStarting(() -> leds.endgameAlert = true)
        // .finallyDo(() -> leds.endgameAlert = false)
        );

  }

  public void configureAutoChooser() {

    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(m_drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Micah's test",
        AutoBuilder.buildAuto("src\\main\\deploy\\pathplanner\\autos\\test.auto"));
  }

  public void configureAbsoluteEncoder() {

  }

  public void configureLeds() {
    // define commands
    // Command ledAutnomousIndicator = new autonoumousIndicator(m_led);
    // Command ledTeleopIndicator = new teleopStatesIndicators(m_led, m_coral);
    // create triggers
    Trigger autonomous = new Trigger(() -> DriverStation.isAutonomousEnabled());
    Trigger teleop = new Trigger(() -> DriverStation.isTeleopEnabled());
    // apply triggers
    // autonomous.onTrue(ledAutnomousIndicator);
    // teleop.onTrue(ledTeleopIndicator);
    // teleleop >done
    // autonomous >done
    // algea intak running
    // elevator level
    // coral intake >done
    // and placing running

  }

  public void configureDrive() {
    // Default command, normal field-relative drive
    // m_drive.setDefaultCommand(
    // DriveCommands.joystickDrive(
    // m_drive,

    // () -> -m_driveController.getLeftY(),
    // () -> -m_driveController.getLeftX(),
    // () -> -m_driveController.getRightX(),
    // m_driveController.leftBumper(),
    // () -> m_vision.getTx(),
    // m_driveController.leftBumper(),
    // m_driveController.rightBumper(),
    // () -> !endgameClosed));

    // // Lock to 0° when A button is held
    // // m_driveController
    // // .b()
    // // .whileTrue(
    // // DriveCommands.joystickDriveAtAngle(
    // // m_drive,
    // // () -> m_driveController.getLeftY(),
    // // () -> m_driveController.getLeftX(),
    // // () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // m_driveController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // // Reset gyro to 0° when B button is pressed
    // m_driveController
    // .a()
    // .onTrue(
    // Commands.runOnce(
    // () -> m_drive.setPose(
    // new Pose2d(m_drive.getPose()
    // .getTranslation(),
    // new Rotation2d())),
    // m_drive)
    // .ignoringDisable(true));
  }

  public void configureRelativeEncoder() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void toggleOverride() {
    override = !override;
    Logger.recordOutput("Override", override);
  }

  private void setEndgamePoseState(boolean state) {

    endgameClosed = state;
    Logger.recordOutput("SFT/endgameClosed", endgameClosed);
  }

  @AutoLogOutput(key = "SFT/getEndgameClosed")
  private boolean getEndgamePoseState() {
    return endgameClosed;
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          m_driveController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          m_driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

}
