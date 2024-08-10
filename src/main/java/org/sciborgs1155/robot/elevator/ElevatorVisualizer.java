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
    MechanismRoot2d chassis = mech.getRoot("Chassis", 10, 10);
    elevator =
        chassis.append(new MechanismLigament2d("elevator", 10 * HEIGHT_FROM_GROUND.in(Meters), 90, 5, color));
  }

  public void setLength(double length) {
    elevator.setLength((length + HEIGHT_FROM_GROUND.in(Meters)) * 10);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
