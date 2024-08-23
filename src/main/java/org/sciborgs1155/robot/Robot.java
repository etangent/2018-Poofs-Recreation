package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ACCEL;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Set;
import java.util.function.DoubleSupplier;
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
  @Log.NT
  private final SendableChooser<Command> autos =
      Autos.configureAutos(drive, elevator, wrist, intake);

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
    return input.signedPow(2).scale(maxSpeed).rateLimit(maxRate);
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    DoubleSupplier scalingFactor = () -> 1 - (Math.abs(driver.getLeftY() - driver.getRightY()) / 8);

    InputStream leftSupplier = () -> scalingFactor.getAsDouble() * driver.getLeftY();
    InputStream rightSupplier = () -> scalingFactor.getAsDouble() * driver.getRightY();

    drive.setDefaultCommand(
        drive.drive(
            createJoystickStream(
                leftSupplier,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                rightSupplier,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                MAX_ACCEL.in(MetersPerSecondPerSecond))));
    // gives operator manual control of the elevator
    elevator.setDefaultCommand(
        elevator.manualElevator(
            InputStream.of(operator::getLeftY).deadband(Constants.DEADBAND, 1)));
    intake.setDefaultCommand(
        new DeferredCommand(
                () ->
                    intake.hasCube()
                        ? intake.clamp().alongWith(intake.stop())
                        : intake.tighten().alongWith(intake.stop()),
                Set.of(intake))
            .withName("intake default"));
    wrist.setDefaultCommand(wrist.hold());
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().onTrue(Commands.deferredProxy(autos::getSelected));

    operator.a().onTrue(wrist.toggle());

    // dpad up
    operator.povUp().whileTrue(elevator.fullExtend());

    // dpad down
    operator.povDown().whileTrue(elevator.stow());

    operator.rightTrigger().onTrue(intake.intakeAndKeep());

    /*
     * can cancel above command and go straight to clamping
     * if rollers are running to long
     */
    operator.rightBumper().onTrue(intake.clamp());

    operator.leftBumper().onTrue(hanger.deploy());

    operator.leftTrigger().onTrue(forklift.deploy());

    // climb sequence
    operator.leftTrigger().and(operator.leftBumper()).onTrue(elevator.pullUp());

    driver.rightTrigger().onTrue(intake.shoot());

    driver.leftTrigger().onTrue(intake.drop());
  }
}
