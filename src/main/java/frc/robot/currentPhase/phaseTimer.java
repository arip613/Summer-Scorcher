package frc.robot.currentPhase;

import org.wpilib.system.Timer;

public class phaseTimer {

	public enum Phase {
		TRANSITION_SHIFT,
		SHIFT_1,
		SHIFT_2,
		SHIFT_3,
		SHIFT_4,
		END_GAME
	}

	private static final double TRANSITION_SHIFT_END = 10.0;   // 0 - 10
	private static final double SHIFT1_END = 35.0;             // 10 - 35
	private static final double SHIFT2_END = 60.0;             // 35 - 60
	private static final double SHIFT3_END = 85.0;             // 60 - 85
	private static final double SHIFT4_END = 110.0;            // 85 - 110
	private static final double ENDGAME_DURATION = 30.0;       // "last 30 seconds"

	private double teleopStartSec = -1.0;

	public void markTeleopStart() {
		teleopStartSec = Timer.getTimestamp();
	}

	public double getElapsedSec() {
		if (teleopStartSec < 0) return 0.0;
		return Timer.getTimestamp() - teleopStartSec;
	}

	public Phase getCurrentPhase() {
		final double t = getElapsedSec();
		if (t < TRANSITION_SHIFT_END) return Phase.TRANSITION_SHIFT;
		if (t < SHIFT1_END) return Phase.SHIFT_1;
		if (t < SHIFT2_END) return Phase.SHIFT_2;
		if (t < SHIFT3_END) return Phase.SHIFT_3;
		if (t < SHIFT4_END) return Phase.SHIFT_4;
		return Phase.END_GAME;
	}

	public double getCurrentPhaseStartSec() {
		switch (getCurrentPhase()) {
			case TRANSITION_SHIFT:
				return 0.0;
			case SHIFT_1:
				return TRANSITION_SHIFT_END;
			case SHIFT_2:
				return SHIFT1_END;
			case SHIFT_3:
				return SHIFT2_END;
			case SHIFT_4:
				return SHIFT3_END;
			case END_GAME:
			default:
				return SHIFT4_END;
		}
	}

	public double getCurrentPhaseDurationSec() {
		switch (getCurrentPhase()) {
			case TRANSITION_SHIFT:
				return TRANSITION_SHIFT_END - 0.0; 
			case SHIFT_1:
				return SHIFT1_END - TRANSITION_SHIFT_END; 
			case SHIFT_2:
				return SHIFT2_END - SHIFT1_END;
			case SHIFT_3:
				return SHIFT3_END - SHIFT2_END; 
			case SHIFT_4:
				return SHIFT4_END - SHIFT3_END; 
			case END_GAME:
			default:
				return ENDGAME_DURATION; 
		}
	}

	public double getSecondsIntoCurrentPhase() {
		return Math.max(0.0, getElapsedSec() - getCurrentPhaseStartSec());
	}

	public double getSecondsRemainingInCurrentPhase() {
		return Math.max(0.0, getCurrentPhaseDurationSec() - getSecondsIntoCurrentPhase());
	}

}
