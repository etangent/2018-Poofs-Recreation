package org.sciborgs1155.robot.wrist;

public interface WristIO extends AutoCloseable {
  public void setVoltage(double voltage);

  /**
   * @return position in radians
   */
  public double getPosition();

  /**
   * @return radians per second
   */
  public double getVelocity();

  /**
   * @return whether the REV magnetic limit switch detects a field (wrist is at stowed position)
   */
  public boolean atLimitSwitch();

  public void zeroEncoders();
}
