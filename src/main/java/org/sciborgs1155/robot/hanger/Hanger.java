package org.sciborgs1155.robot.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Hanger extends SubsystemBase implements AutoCloseable {
  public static Hanger create() {
    return Robot.isReal() ? new Hanger(new RealHanger()) : none();
  }

  public static Hanger none() {
    return new Hanger(new NoHanger());
  }

  private final HangerIO hardware;

  public Hanger(HangerIO hardware) {
    this.hardware = hardware;
  }

  public Command deploy() {
    return runOnce(() -> hardware.set(true)).andThen(Commands.idle(this));
  }

  public Command stow() {
    return run(() -> hardware.set(false)).andThen(Commands.idle(this));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
