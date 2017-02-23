package edu.hendrix.ev3.ai.cluster.yuv;

import java.util.ArrayList;

import edu.hendrix.ev3.ai.bsoc.BoundedSelfOrgCluster;
import edu.hendrix.ev3.ai.cluster.DistanceFunc;
import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.Util;

public class ImageBSOC extends BoundedSelfOrgCluster<AdaptedYUYVImage> {
	private int shrinkFactor;
	
	public ImageBSOC(int maxNumNodes, int shrinkFactor) {
		this(maxNumNodes, shrinkFactor, YUYVDistanceFuncs::euclideanAllChannels);
	}
	
	public ImageBSOC(int maxNumNodes, int shrinkFactor, DistanceFunc<AdaptedYUYVImage> dist) {
		super(maxNumNodes, dist);
		this.shrinkFactor = shrinkFactor;
	}
	
	@Override
	public ImageBSOC deepCopy() {
		ImageBSOC result = new ImageBSOC(this.size(), this.shrinkFactor, this.getDistanceFunc());
		deepCopyHelp(result);
		return result;
	}
	
	@Override
	public String toString() {
		return super.toString() + "{" + shrinkFactor + "}";
	}
	
	public static ImageBSOC fromString(String src) {
		ArrayList<String> parts = Util.debrace(src);
		int shrinkFactor = Integer.parseInt(parts.remove(parts.size() - 1));
		ImageBSOC result = new ImageBSOC(1, shrinkFactor);
		result.fromStringHelp(parts, AdaptedYUYVImage::fromString);
		return result;
	}
	
	@Override
	public int train(AdaptedYUYVImage example) {
		return super.train(example.shrunken(shrinkFactor));
	}
	
	@Override
	public long distanceToClosestMatch(AdaptedYUYVImage example) {
		return super.distanceToClosestMatch(example.shrunken(shrinkFactor));
	}
	
	@Override
	public ArrayList<Duple<Integer,Long>> getNodeRanking(AdaptedYUYVImage example) {
		return super.getNodeRanking(example.shrunken(shrinkFactor));
	}
}
