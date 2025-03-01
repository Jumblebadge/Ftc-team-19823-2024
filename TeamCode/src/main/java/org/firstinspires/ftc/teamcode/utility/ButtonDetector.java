package org.firstinspires.ftc.teamcode.utility;

public class ButtonDetector {

    private boolean current = false;
    private boolean last = false;

    public ButtonDetector() {}

    public ButtonDetector(boolean initial) {
        current = initial;
        last = initial;
    }

    public boolean toggle(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
        return current;
    }

    public void toggleVoid(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
    }

    public boolean risingEdge(boolean currentState) {
        boolean state = currentState && !last;
        last = currentState;
        return state;
    }

    public void toFalse() {
        current = false;
    }

    public void toTrue()  { current = true;  }

    public boolean isTrue() { return current; }
}

