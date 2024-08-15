package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoDrive implements DriveIO {
  private final Rotation2d none = new Rotation2d();

  @Override
  public void setVoltages(double leftVoltage, double rightVoltage) {}

  @Override
  public double getLeftPosition() {
    return 0;
  }

  @Override
  public double getRightPosition() {
    return 0;
  }

  @Override
  public double getLeftVelocity() {
    return 0;
  }

  @Override
  public double getRightVelocity() {
    return 0;
  }

  @Override
  public Rotation2d getHeading() {
    return none;
  }

  @Override
  public void shiftGears(boolean high) {}

  @Override
  public void close() throws Exception {}
}
