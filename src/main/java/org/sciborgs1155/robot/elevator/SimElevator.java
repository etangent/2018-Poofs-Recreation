package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Elevator;

public class SimElevator implements ElevatorIO {
  private final ElevatorSim elevator;

  public SimElevator() {
    elevator =
        new ElevatorSim(
            DCMotor.getVex775Pro(4),
            Elevator.HIGH_GEARING,
            Elevator.CARRIAGE_MASS.in(Kilograms),
            Elevator.DRUM_RADIUS.in(Meters),
            0,
            MAX_HEIGHT.in(Meters),
            true,
            0);
  }

  @Override
  public void setVoltage(double voltage) {
    elevator.setInputVoltage(voltage);
    elevator.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return elevator.getPositionMeters();
  }

  @Override
  public double getVelocity() {
    return elevator.getVelocityMetersPerSecond();
  }

  // unnecessary
  @Override
  public void shiftGear(boolean high) {}

  @Override
  public boolean atLimitSwitch() {
    return false;
  }

  // unnecessary
  @Override
  public void zeroEncoders() {}
}
