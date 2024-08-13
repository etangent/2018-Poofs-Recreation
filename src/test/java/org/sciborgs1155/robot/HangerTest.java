package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.TestingUtil.reset;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.hanger.Hanger;

public class HangerTest {
    @Test
    void initialize() throws Exception {
        Hanger.create().close();
        reset();
    }
}
