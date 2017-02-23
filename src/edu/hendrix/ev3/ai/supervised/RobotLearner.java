package edu.hendrix.ev3.ai.supervised;

import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;

public interface RobotLearner {
	public void train(AdaptedYUYVImage img, Move current);
	public Move bestMatchFor(AdaptedYUYVImage img);
	public boolean isTrained();
}
