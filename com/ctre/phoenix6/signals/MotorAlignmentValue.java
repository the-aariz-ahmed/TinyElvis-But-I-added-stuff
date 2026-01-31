/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.signals;

import java.util.HashMap;

/**
 * The relationship between two motors in a mechanism. Depending on hardware
 * setup, one motor may be inverted relative to the other motor.
 */
public enum MotorAlignmentValue
{
    /**
     * The two motor directions are aligned. Positive output on both motors moves
     * the mechanism forward/backward.
     */
    Aligned(0),
    /**
     * The two motor directions are opposed. To move forward/backward, one motor
     * needs positive output, and the other needs negative output.
     */
    Opposed(1),;

    public final int value;

    MotorAlignmentValue(int initValue)
    {
        this.value = initValue;
    }

    private static HashMap<Integer, MotorAlignmentValue> _map = null;
    static
    {
        _map = new HashMap<Integer, MotorAlignmentValue>();
        for (MotorAlignmentValue type : MotorAlignmentValue.values())
        {
            _map.put(type.value, type);
        }
    }

    /**
     * Gets MotorAlignmentValue from specified value
     * @param value Value of MotorAlignmentValue
     * @return MotorAlignmentValue of specified value
     */
    public static MotorAlignmentValue valueOf(int value)
    {
        MotorAlignmentValue retval = _map.get(value);
        if (retval != null) return retval;
        return MotorAlignmentValue.values()[0];
    }
}
