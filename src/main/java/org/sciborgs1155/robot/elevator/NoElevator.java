package org.sciborgs1155.robot.elevator;

public class NoElevator implements ElevatorIO {
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
  public void shiftGear(boolean high) {}

  @Override
  public boolean atLimitSwitch() {
      return false;
  }

  @Override
  public void zeroEncoders() {}
}
