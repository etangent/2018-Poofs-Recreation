package org.sciborgs1155.robot.intake;

public class NoIntake implements IntakeIO {
  @Override
  public void setRollerPower(double power) {}

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void setClamp(ClampState clamp) {}

  @Override
  public double rollerCurrent() {
    return 0;
  }

  @Override
  public boolean hasCube() {
    return false;
  }
}
