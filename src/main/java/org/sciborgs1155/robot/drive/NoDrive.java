package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class NoDrive implements DriveIO {
  private final Rotation2d noRot = new Rotation2d();
  private final Measure<Voltage> noVolts = Volts.of(0);

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
  public Measure<Voltage> getLeftVoltage() {
    return noVolts;
  }

  @Override
  public Measure<Voltage> getRightVoltage() {
    return noVolts;
  }

  @Override
  public Rotation2d getHeading() {
    return noRot;
  }

  @Override
  public void close() throws Exception {}
}
