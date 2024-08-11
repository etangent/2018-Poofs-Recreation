package org.sciborgs1155.robot.forklift;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Forklift extends SubsystemBase {
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
    return run(() -> hardware.set(true)).andThen(Commands.idle(this));
  }

  public Command retract() {
    return run(() -> hardware.set(false)).andThen(Commands.idle(this));
  }
}
