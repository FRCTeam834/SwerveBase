/**
 * @author Christian Piper
 * @since 10/21/21
 */
package frc.robot.enums;

import java.util.HashMap;
import java.util.Map;

public enum JOYSTICK_OUTPUT_TYPES {
    LINEAR(0),
    ZEROED_LINEAR(1),
    ZEROED_QUAD_LINEAR(2);

    private int value;
    private static Map map = new HashMap<>();

    private JOYSTICK_OUTPUT_TYPES(int value) {
        this.value = value;
    }

    static {
        for (JOYSTICK_OUTPUT_TYPES outputType : JOYSTICK_OUTPUT_TYPES.values()) {
            map.put(outputType.value, outputType);
        }
    }

    public static JOYSTICK_OUTPUT_TYPES lookupInt(int outputType) {
        return (JOYSTICK_OUTPUT_TYPES) map.get(outputType);
    }

    public int getInt() {
        return value;
    }
}
