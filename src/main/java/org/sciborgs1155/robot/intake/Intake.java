package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.intake.IntakeConstants.INTAKE_SPEED;
import static org.sciborgs1155.robot.intake.IntakeConstants.SHOOTING_SPEED;
import static org.sciborgs1155.robot.intake.IntakeIO.ClampState.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.intake.IntakeIO.ClampState;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : none();
  }

  public static Intake none() {
    return new Intake(new NoIntake());
  }

  private final IntakeIO hardware;

  public Intake(IntakeIO hardware) {
    this.hardware = hardware;
  }

  public Command intakeAndKeep() {
    return backward().until(this::hasCube).andThen(clamp()).withName("intaking and keeping");
  }

  public Command drop() {
    return open()
    .until(() -> !hasCube())
    .andThen(tighten());
  }

  // I may do more stuff with this
  public Command shoot() {
    return runIntake(SHOOTING_SPEED).until(() -> !hasCube()).withName("shooting");
  }

  public Command runIntake(double power) {
    return runOnce(() -> hardware.setRollerPower(power))
        .andThen(Commands.idle(this))
        .finallyDo(() -> hardware.setRollerPower(0));
  }

  public Command forward() {
    return runIntake(INTAKE_SPEED).withName("forwards");
  }

  public Command backward() {
    return runIntake(INTAKE_SPEED).withName("backwards");
  }

  public Command stop() {
    return runIntake(0).withName("stopping");
  }

  public Command toggleClamp(ClampState clamp) {
    return runOnce(() -> hardware.setClamp(clamp)).asProxy();
  }

  public Command clamp() {
    return toggleClamp(CLAMP).withName("clamping");
  }

  public Command tighten() {
    return toggleClamp(DEFAULT).withName("tightening");
  }

  public Command open() {
    return toggleClamp(OPEN).withName("opening");
  }

  @Log.NT
  public boolean stalling() {
    return hardware.rollerCurrent() > DCMotor.getVex775Pro(2).stallCurrentAmps;
  }

  @Log.NT
  public double rollerVelocity() {
    return hardware.getVelocity();
  }

  @Log.NT
  public boolean hasCube() {
    return hardware.hasCube();
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
