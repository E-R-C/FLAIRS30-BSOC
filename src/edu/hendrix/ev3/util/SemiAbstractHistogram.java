package edu.hendrix.ev3.util;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;

public class SemiAbstractHistogram<T, M extends Map<T,Integer>> implements Iterable<Entry<T,Integer>> {
	private M counts;
	
	protected SemiAbstractHistogram(M map) {
		counts = map;
	}
	
	public T getHighestCounted() {
		Entry<T,Integer> highest = null;
		for (Entry<T, Integer> entry: this) {
			if (highest == null || entry.getValue() > highest.getValue()) {
				highest = entry;
			}
		}
		return highest.getKey();
	}
	
	public void bump(T t) {
		setCountFor(t, getCountFor(t) + 1);
	}
	
	public int getNumKeys() {return counts.size();}
	
	public int getCountFor(T t) {
		return counts.containsKey(t) ? counts.get(t) : 0;
	}
	
	public double getPortionFor(T t) {
		return (double)getCountFor(t) / getNumKeys();
	}
	
	public void setCountFor(T t, int target) {
		counts.put(t, target);
	}
	
	protected <P extends Map<T,Double>> void getPortions(P portions) {
		int total = getTotalCount();
		for (Entry<T,Integer> entry: this) {
			portions.put(entry.getKey(), (double)entry.getValue() / (double)total);
		}
	}
	
	public int getTotalCount() {
		int total = 0;
		for (int value: counts.values()) {
			total += value;
		}
		return total;
	}

	@Override
	public Iterator<Entry<T,Integer>> iterator() {
		return counts.entrySet().iterator();
	}
	
	@Override
	public boolean equals(Object other) {
		if (other instanceof SemiAbstractHistogram<?,?>) {
			@SuppressWarnings("unchecked")
			SemiAbstractHistogram<T,M> that = (SemiAbstractHistogram<T,M>)other;
			return this.counts.equals(that.counts);
		} else {
			return false;
		}
	}
	
	@Override
	public int hashCode() {
		return toString().hashCode();
	}
	
	@Override
	public String toString() {
		return counts.toString();
	}
	
	public static LinkedHashMap<String,Integer> destringify(String histStr) {
		LinkedHashMap<String,Integer> result = new LinkedHashMap<String,Integer>();
		histStr = histStr.substring(1, histStr.length() - 1);
		for (String mapping: histStr.split(", ")) {
			String[] pair = mapping.split("=");
			if (pair.length == 2) {
				result.put(pair[0], Integer.parseInt(pair[1]));
			}
		}
		return result;
	}
	
	public void addAllFrom(SemiAbstractHistogram<T,M> src) {
		for (Entry<T, Integer> entry: src) {
			setCountFor(entry.getKey(), entry.getValue() + getCountFor(entry.getKey()));
		}
	}
}
