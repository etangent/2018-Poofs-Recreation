package org.sciborgs1155.robot.forklift;

import static org.sciborgs1155.robot.Ports.Forklift.DEPLOYER;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RealForklift implements ForkliftIO {
  private final Solenoid deployer = new Solenoid(PneumaticsModuleType.CTREPCM, DEPLOYER);

  @Override
  public void set(boolean extended) {
    deployer.set(extended);
  }

  @Override
  public void close() throws Exception {
      deployer.close();
  }
}
