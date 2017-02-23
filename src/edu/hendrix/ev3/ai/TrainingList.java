package edu.hendrix.ev3.ai;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.Map.Entry;

import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.EnumHistogram;

public class TrainingList implements Iterable<Duple<Move,AdaptedYUYVImage>> {

	private ArrayList<Duple<Move,AdaptedYUYVImage>> examples = new ArrayList<>();
	private EnumHistogram<Move> countsFor = new EnumHistogram<>(Move.class);
	
	public TrainingList() {}
	
	public TrainingList(String src) {
		for (String sample: src.split("\\|")) {
			String[] parts = sample.split(";");
			Move m = Move.valueOf(parts[0]);
			AdaptedYUYVImage img = AdaptedYUYVImage.fromString(parts[1]);
			add(m, img);
		}
	}
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Duple<Move, AdaptedYUYVImage> mImg: this) {
			sb.append(mImg.getFirst().toString() + ";" + mImg.getSecond().toString() + "|");
		}
		return sb.toString();
	}

	public int size() {return examples.size();}
	
	public Duple<Move, AdaptedYUYVImage> get(int i) {
		return examples.get(i);
	}

	public int getWidth() {
		return examples.get(0).getSecond().getWidth();
	}

	public int getHeight() {
		return examples.get(0).getSecond().getHeight();
	}

	public void add(Move m, AdaptedYUYVImage img) {
		add(new Duple<>(m,img));
	}
	
	public void add(Duple<Move,AdaptedYUYVImage> mImg) {
		examples.add(mImg);
		countsFor.bump(mImg.getFirst());
	}
	
	public void purgeTransitions(int transitionFrames) {
		ArrayList<Duple<Move,AdaptedYUYVImage>> purged = new ArrayList<>();
		for (int i = 0; i < examples.size(); i++) {
			int target = i + transitionFrames;
			if (target < examples.size() && examples.get(target).getFirst() == examples.get(i).getFirst()) {
				purged.add(examples.get(i));
			}
		}
		examples = purged;
	}

	public EnumSet<Move> getMovesPresent() {
		EnumSet<Move> result = EnumSet.noneOf(Move.class);
		for (Entry<Move, Integer> m: countsFor) {
			result.add(m.getKey());
		}
		return result;
	}
	
	public int getCountFor(Move m) {
		return countsFor.getCountFor(m);
	}

	@Override
	public Iterator<Duple<Move, AdaptedYUYVImage>> iterator() {
		return examples.iterator();
	}
}