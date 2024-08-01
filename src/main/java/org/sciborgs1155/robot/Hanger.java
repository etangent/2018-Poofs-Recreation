package org.sciborgs1155.robot;

import static org.sciborgs1155.robot.Ports.Hanger.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hanger extends SubsystemBase {
  private final Solenoid deployer = new Solenoid(PneumaticsModuleType.CTREPCM, DEPLOYER);

  public Command deploy() {
    return run(() -> deployer.set(true));
  }

  public Command stow() {
    return run(() -> deployer.set(false));
  }
}
