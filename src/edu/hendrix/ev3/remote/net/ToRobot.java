package edu.hendrix.ev3.remote.net;

import edu.hendrix.ev3.remote.Move;

public enum ToRobot {
	NULL_MESSAGE {
	},
	PICTURE {
	},
	QUIT {
	}, 
	STOP {
		@Override
		public Move getMove() {return Move.STOP;}
		@Override
		public boolean isMoveCmd() {return true;}
	},
	FORWARD {
		@Override
		public Move getMove() {return Move.FORWARD;}
		@Override
		public long getDurationMilliSec() {return 1000;}
		@Override
		public boolean isMoveCmd() {return true;}
	}, 
	BACKWARD {
		@Override
		public Move getMove() {return Move.BACKWARD;}
		@Override
		public boolean isMoveCmd() {return true;}
	}, 
	LEFT {
		@Override
		public Move getMove() {return Move.LEFT;}
		@Override
		public boolean isMoveCmd() {return true;}
	}, 
	RIGHT {
		@Override
		public Move getMove() {return Move.RIGHT;}
		@Override
		public boolean isMoveCmd() {return true;}
	}, 
	TRAIN {
	},
	ASK_PENDING {
	},
	GO_AUTO {
		@Override
		public boolean changesMode() {return true;}
	}, 
	NO_AUTO {
		@Override
		public boolean changesMode() {return true;}
	};
	
	public Move getMove() {return Move.NONE;}
	
	public boolean changesMode() {return false;}
	
	public boolean isMoveCmd() {return false;}
	
	public byte code() {
		return (byte)ordinal();
	}
	
	public static ToRobot from(byte code) {
		return ToRobot.values()[code];
	}
	
	public long getDurationMilliSec() {return 200;}
}
