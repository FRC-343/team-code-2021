package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Debouncer {
    public static final double kDebouncePeriod = 0.1; // secs

    private final Timer m_timer = new Timer();

    private boolean m_lastVal = false;
    private boolean m_debouncing = false;

    public boolean isReady(boolean val) {
        boolean ready = true;

        if (val != m_lastVal) {
            m_timer.reset();
            m_timer.start();
            m_debouncing = true;

            ready = false;
        } else {
            if (m_debouncing) {
                double time = m_timer.get();
                if (time >= kDebouncePeriod) {
                    m_timer.stop();
                    m_debouncing = false;
                    ready = true;
                }
                else {
                    ready = false;
                }
            } else {
                ready = true;
            }
        }

        if (ready) {
            m_lastVal = val;
        }

        return ready;
    }
}