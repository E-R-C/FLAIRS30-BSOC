package edu.hendrix.ev3.ai.supervised;

import edu.hendrix.ev3.remote.Move;

public class TestResult {
	private Move label, result;
	private int bestRankedAlternative;
	
	public TestResult(Move label, Move result, int bestRankedAlternative) {
		this.label = label;
		this.result = result;
		this.bestRankedAlternative = bestRankedAlternative;
	}
	
	public Move getTestLabel() {return label;}
	public Move getTestResult() {return result;}
	public int getBestAlternative() {return bestRankedAlternative;}
	
	public boolean matches(Move label, Move result) {
		return this.label == label && this.result == result;
	}
}
