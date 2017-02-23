package edu.hendrix.ev3.ai.cluster;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.Util;

public interface Clusterer<T> {
	public int train(T example);
	
	default public int getClosestMatchFor(T example) {
		return getClosestNodeDistanceFor(example).getFirst();
	}
	
	default public Duple<Integer,Long> getClosestNodeDistanceFor(T example) {
		Util.assertState(size() > 0, "No nodes exist");
		Duple<Integer,Long> result = null;
		for (int id: getClusterIds()) {
			long dist = getDistanceFunc().distance(example, getIdealInputFor(id));
			if (result == null || dist < result.getSecond()) {
				result = new Duple<>(id, dist);
			}
		}
		return result;
	}
	
	default public ArrayList<Duple<Integer,Long>> getNodeRanking(T example) {
		ArrayList<Duple<Integer, Long>> result = new ArrayList<>();
		for (int id: getClusterIds()) {
			result.add(new Duple<>(id, getDistanceFunc().distance(example, getIdealInputFor(id))));
		}
		Collections.sort(result, (o1, o2) -> o1.getSecond() < o2.getSecond() ? -1 : o1.getSecond() > o2.getSecond() ? 1 : 0);
		return result;
	}
	
	public DistanceFunc<T> getDistanceFunc();
	
	public T getIdealInputFor(int node);
	
	public int size();
	
	public Collection<Integer> getClusterIds();
}
