package edu.hendrix.ev3.remote;

public enum Move {
	FORWARD, LEFT, RIGHT, SPIN_LEFT, SPIN_RIGHT, BACKWARD, STOP, NONE;
	
	public String uiName() {
		StringBuilder result = new StringBuilder();
		String str = toString();
		result.append(str.charAt(0));
		String rest = str.substring(1).replace('_', ' ').toLowerCase();
		result.append(rest);
		return result.toString();
	}
}
