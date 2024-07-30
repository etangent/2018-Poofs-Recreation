package org.sciborgs1155.robot.elevator;

public interface ElevatorIO {
  public void setVoltage(double voltage);

  public double getPosition();

  public double getVelocity();

  /**
   * @param low If true, shifts into high gear; and shifts into low gear if false
   */
  public void shiftGear(boolean high);
}
