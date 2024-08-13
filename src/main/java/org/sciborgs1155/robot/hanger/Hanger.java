package org.sciborgs1155.robot.hanger;

import edu.wpi.first.wpilibj2.command.Command;
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
    return run(() -> hardware.set(true));
  }

  public Command stow() {
    return run(() -> hardware.set(false));
  }

  @Override
  public void close() throws Exception {
      hardware.close();
  }
}
