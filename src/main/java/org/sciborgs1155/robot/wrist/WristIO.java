package org.sciborgs1155.robot.wrist;

public interface WristIO {
  public void setVoltage(double voltage);

  public double getPosition();

  public double getVelocity();

  /**
   * @return whether the REV magnetic limit switch detects a field (wrist is at 0 degrees)
   */
  public boolean atLimitSwitch();

  public void resetEncoder();
}
