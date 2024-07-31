package org.sciborgs1155.robot.elevator;

public interface ElevatorIO {
  public void setVoltage(double voltage);

  /**
   * @return position in meters (from bottom of travel)
   */
  public double getPosition();

  /**
   * @return meters per second
   */
  public double getVelocity();

  /**
   * @param high If true, shifts into high gear; and shifts into low gear if false
   */
  public void shiftGear(boolean high);

  /**
   * @return whether we are at stowed position or not (min height)
   */
  public boolean atLimitSwitch();

  public void zeroEncoders();
}
