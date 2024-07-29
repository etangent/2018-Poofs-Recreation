package org.sciborgs1155.robot.wrist;

public class NoWrist implements WristIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public boolean atLimitSwitch() {
    return false;
  }

  @Override
  public void resetEncoder() {}
}
