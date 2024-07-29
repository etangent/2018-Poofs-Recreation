package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.wrist.WristConstants.Pivot;

public class SimWrist implements WristIO {
  private final DCMotorSim pivot;

  public SimWrist() {
    pivot =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Pivot.kV, Pivot.kA),
            DCMotor.getVex775Pro(1),
            Pivot.GEARING);
  }

  @Override
  public void setVoltage(double voltage) {
    pivot.setInputVoltage(voltage);
    pivot.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return pivot.getAngularPositionRad();
  }

  @Override
  public double getVelocity() {
    return pivot.getAngularVelocityRadPerSec();
  }

  @Override
  public boolean atLimitSwitch() {
    return false;
  }

  @Override
  public void resetEncoder() {}
}
