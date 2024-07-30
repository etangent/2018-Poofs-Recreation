package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer implements Sendable {
  private final Mechanism2d mech;
  private final MechanismLigament2d elevator;

  public ElevatorVisualizer(Color8Bit color) {
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("Chassis", 0, 0);
    elevator =
        chassis.append(new MechanismLigament2d("elevator", MIN_HEIGHT.in(Meters), 90, 5, color));
  }

  public void setLength(double length) {
    elevator.setLength(length);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
