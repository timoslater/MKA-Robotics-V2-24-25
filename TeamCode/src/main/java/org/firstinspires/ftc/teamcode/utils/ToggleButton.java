package org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;

public class ToggleButton {
    private boolean lastButtonState = false;

    public boolean getState(Boolean[] buttonStates) {
        boolean currentButtonState = !Arrays.asList(buttonStates).contains(false);

        if(currentButtonState && !lastButtonState)
        {
            lastButtonState = true;

            return true;
        }
        else if (!currentButtonState)
        {
            lastButtonState = false;
        }
        return false;

    }
}
