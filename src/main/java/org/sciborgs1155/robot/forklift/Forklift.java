package org.sciborgs1155.robot.forklift;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Forklift extends SubsystemBase implements AutoCloseable, Logged {
  public static Forklift create() {
    return Robot.isReal() ? new Forklift(new RealForklift()) : none();
  }

  public static Forklift none() {
    return new Forklift(new NoForklift());
  }

  private final ForkliftIO hardware;

  public Forklift(ForkliftIO hardware) {
    this.hardware = hardware;
  }

  public Command deploy() {
    return runOnce(() -> hardware.set(true)).andThen(Commands.idle(this));
  }

  public Command retract() {
    return runOnce(() -> hardware.set(false)).andThen(Commands.idle(this));
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
