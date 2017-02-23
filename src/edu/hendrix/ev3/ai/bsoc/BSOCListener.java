package edu.hendrix.ev3.ai.bsoc;

public interface BSOCListener {
	public void addingNode(int node);
	public void replacingNode(int target, int replacement);
}
