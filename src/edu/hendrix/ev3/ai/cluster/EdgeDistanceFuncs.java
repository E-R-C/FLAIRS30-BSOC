package edu.hendrix.ev3.ai.cluster;

import edu.hendrix.ev3.ai.cluster.yuv.YUYVDistanceFuncs;

public class EdgeDistanceFuncs {
	public static long euclidean(EdgeImage img1, EdgeImage img2) {
		long ssd = 0;
		for (int x = 0; x < img1.getWidth(); ++x) {
			for (int y = 0; y < img1.getHeight(); ++y) {
				long diff = img1.get(x, y) - img2.get(x, y);
				ssd += YUYVDistanceFuncs.square(diff);
			}
		}
		return ssd;
	}
}
