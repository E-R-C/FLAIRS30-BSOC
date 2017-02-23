package edu.hendrix.ev3.util;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.UnaryOperator;

public interface DeepCopyable<T extends DeepCopyable<T>> {
	public T deepCopy();
	
	public static <K,V extends DeepCopyable<V>> void copyFromInto(Map<K,V> src, Map<K,V> dest) {
		copyFromInto(src, dest, v -> v.deepCopy());
	}
	
	public static <K,V> void copyFromInto(Map<K,V> src, Map<K,V> dest, UnaryOperator<V> duplicator) {
		for (Entry<K, V> entry: src.entrySet()) {
			dest.put(entry.getKey(), duplicator.apply(entry.getValue()));
		}
	}
	
	public static <T extends DeepCopyable<T>> void copyFromInto(ArrayList<T> src, ArrayList<T> dest) {
		copyFromInto(src, dest, t -> t.deepCopy());
	}
	
	public static <T> void copyFromInto(ArrayList<T> src, ArrayList<T> dest, UnaryOperator<T> duplicator) {
		for (T t: src) {
			dest.add(duplicator.apply(t));
		}
	}
}
