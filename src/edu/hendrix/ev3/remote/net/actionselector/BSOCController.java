package edu.hendrix.ev3.remote.net.actionselector;

import java.util.ArrayList;

import edu.hendrix.ev3.ai.bsoc.BSOCListener;
import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.ai.cluster.yuv.ImageBSOC;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.EnumHistogram;
import edu.hendrix.ev3.util.FixedSizeArray;
import edu.hendrix.ev3.util.Logger;
import edu.hendrix.ev3.util.Util;

public class BSOCController implements BSOCListener {
	private ImageBSOC bsoc;
	private FixedSizeArray<EnumHistogram<Move>> moves;
	private int clustNum, shrinkNum;
	
	public BSOCController(int size, int shrinkFactor) {
		clustNum = size;
		shrinkNum = shrinkFactor;
		bsoc = new ImageBSOC(size, shrinkFactor);
		bsoc.addListener(this);
		moves = FixedSizeArray.makeImmutableType(size + 1);
		resetMoves(moves);
	}
	
	private static void resetMoves(FixedSizeArray<EnumHistogram<Move>> moves) {
		for (int i = 0; i < moves.capacity(); i++) {
			moves.put(i, new EnumHistogram<>(Move.class));
		}
	}
	
	private BSOCController(ImageBSOC bsoc, FixedSizeArray<EnumHistogram<Move>> moves) {
		this.bsoc = bsoc;
		this.moves = moves;
	}
	
	public static BSOCController fromString(String input) {
		ArrayList<String> parts1 = Util.debrace(input);
		ImageBSOC bsoc = ImageBSOC.fromString(parts1.get(0));
		FixedSizeArray<EnumHistogram<Move>> moves = FixedSizeArray.makeImmutableType(bsoc.size()+1);
		resetMoves(moves);
		if (parts1.size() > 1) {
			for (String moveSpec: Util.debrace(parts1.get(1))) {
				ArrayList<String> parts2 = Util.debrace(moveSpec);
				int i = Integer.parseInt(parts2.get(0));
				EnumHistogram<Move> m = EnumHistogram.fromString(Move.class, parts2.get(1));
				moves.put(i, m);
			}
		}
		return new BSOCController(bsoc, moves);
	}
	
	public String toString() {
		StringBuilder result = new StringBuilder();
		result.append('{');
		result.append(bsoc.toString());
		result.append("}{");
		for (int i = 0; i < moves.capacity(); i++) {
			if (moves.containsKey(i)) {
				result.append("{{");
				result.append(i);
				result.append("}{");
				result.append(moves.get(i));
				result.append("}}");
			}
		}
		result.append("}");
		return result.toString();
	}
	
	public int train(AdaptedYUYVImage example) {
		return bsoc.train(example);
	}
	
	public AdaptedYUYVImage getIdealInputFor(int node) {
		return bsoc.getIdealInputFor(node);
	}
	
	public Move getMoveFor(int node) {
		return moves.get(node).getHighestCounted();
	}
	
	public boolean nodeExists(int i) {
		return bsoc.nodeExists(i);
	}
	
	public int getNumMergesFor(int node) {
		return bsoc.getNumMergesFor(node);
	}
	
	public int getTotalSourceInputs() {
		return bsoc.getTotalSourceInputs();
	}
	
	public ArrayList<Integer> getClusterIds() {
		return bsoc.getClusterIds();
	}
	
	public int size() {return bsoc.size();}
	
	public void assignMoveFor(int node, Move m) {
		if (nodeExists(node)) {
			moves.get(node).bump(m);
		}
	}
	
	public Move pickMoveFor(AdaptedYUYVImage input) {
		for (Duple<Integer,Long> candidate: bsoc.getNodeRanking(input)) {
			Move result = getMoveFor(candidate.getFirst());
			if (result != Move.NONE) {
				return result;
			}
 		}
		Logger.EV3Log.log("Nothing found");
		return Move.NONE;
	}
	public int getShrinkNum(){return shrinkNum;}
	public int getClustNum(){return clustNum;}

	@Override
	public void addingNode(int node) {}

	@Override
	public void replacingNode(int target, int replacement) {
		moves.get(replacement).addAllFrom(moves.get(target));
		moves.put(target, new EnumHistogram<>(Move.class));
	}
}
