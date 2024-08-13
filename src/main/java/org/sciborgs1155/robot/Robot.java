package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.forklift.Forklift;
import org.sciborgs1155.robot.hanger.Hanger;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.wrist.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  private final Drive drive = Drive.create();
  private final Elevator elevator = Elevator.create();
  private final Wrist wrist = Wrist.create();
  private final Intake intake = Intake.create();
  private final Forklift forklift = Forklift.create();
  private final Hanger hanger = Hanger.create();

  // COMMANDS
  @Log.NT private final Autos autos = new Autos();

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, 1);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /** Creates an input stream for a joystick. */
  private InputStream createJoystickStream(InputStream input, double maxSpeed, double maxRate) {
    return input
        .deadband(Constants.DEADBAND, 1)
        .scale(maxSpeed)
        .signedPow(2)
        .rateLimit(maxRate);
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive.drive(
            createJoystickStream(
                driver::getLeftY,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                driver::getRightY,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                MAX_ACCEL.in(MetersPerSecondPerSecond))));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    // autonomous().whileTrue(new ProxyCommand(autos::get));
    FaultLogger.onFailing(f -> Commands.print(f.toString()));
    operator
        .b()
        .onTrue(
            wrist
                .goTo(() -> Math.PI / 8)
                .alongWith(elevator.goTo(() -> MAX_HEIGHT.in(Meters) / 2)));
    operator.a().onTrue(wrist.goTo(() -> Math.PI / 1.5).alongWith(elevator.goTo(() -> 0)));
  }
}
