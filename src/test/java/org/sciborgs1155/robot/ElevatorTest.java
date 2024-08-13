package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.reset;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.elevator.Elevator;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

public class ElevatorTest {
    private Elevator elevator;
    private final double DELTA = 0.001;

    @BeforeEach
    void setup() {
        setupHAL();
        elevator = Elevator.create();
    }

    @AfterEach
    void destroy() throws Exception {
        reset(elevator);
    }

    @ParameterizedTest
    @ValueSource(doubles = {2, 1, 0})
    void reachesPosition(double height) {
        run(elevator.goTo(() -> height));
        fastForward();

        assertEquals(height, elevator.measurement(), DELTA);
    }
}
