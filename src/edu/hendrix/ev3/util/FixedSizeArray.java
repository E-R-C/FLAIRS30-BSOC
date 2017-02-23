package edu.hendrix.ev3.util;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.function.BiConsumer;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.UnaryOperator;

public class FixedSizeArray<T> implements DeepCopyable<FixedSizeArray<T>> {
	private T[] array;
	private BitSet availableIndices;
	private UnaryOperator<T> copier;
	
	public static <T extends DeepCopyable<T>> FixedSizeArray<T> make(int size) {
		return new FixedSizeArray<>(size, t -> t.deepCopy());
	}
	
	public static <T> FixedSizeArray<T> makeImmutableType(int size) {
		return new FixedSizeArray<>(size, i -> i);
	}
	
	@SuppressWarnings("unchecked")
	private FixedSizeArray(int size, UnaryOperator<T> deepCopier) {
		array = (T[])new Object[size];
		this.copier = deepCopier;
		availableIndices = new BitSet();
		availableIndices.set(0, size);
	}
	
	public static <T extends DeepCopyable<T>> FixedSizeArray<T> parse(String src, Function<String,T> extractor) {
		return FixedSizeArray.parse(src, extractor, t -> t.deepCopy());
	}
	
	public static <T> FixedSizeArray<T> parseImmutableType(String src, Function<String,T> extractor) {
		return FixedSizeArray.parse(src, extractor, i -> i);
	}
	
	private static <T> FixedSizeArray<T> parse(String src, Function<String,T> extractor, UnaryOperator<T> deepCopier) {
		ArrayList<String> top = Util.debrace(src);
		int size = Integer.parseInt(top.get(0));
		FixedSizeArray<T> result = new FixedSizeArray<>(size, deepCopier);
		for (String pair: Util.debrace(top.get(1))) {
			ArrayList<String> pairs = Util.debrace(pair);
			int i = Integer.parseInt(pairs.get(0));
			T v = extractor.apply(pairs.get(1));
			result.put(i, v);
		}
		return result;
	}
	
	@Override
	public String toString() {
		StringBuilder result = new StringBuilder();
		result.append("{");
		result.append(array.length);
		result.append("}{");
		doAll((i, v) -> {
			result.append("{{");
			result.append(i);
			result.append("}{");
			result.append(v);
			result.append("}}");
		});
		result.append("}");
		return result.toString();
	}
	
	@Override
	public boolean equals(Object other) {
		if (other instanceof FixedSizeArray<?>) {
			@SuppressWarnings("unchecked")
			FixedSizeArray<T> that = (FixedSizeArray<T>)other;
			if (this.size() == that.size() && this.capacity() == that.capacity()) {
				return forAll((i, v) -> that.containsKey(i) && that.get(i).equals(v));
			} else {
				return false;
			}
		} else {
			return false;
		}
	}
	
	@Override
	public int hashCode() {return toString().hashCode();}
	
	public int capacity() {
		return array.length;
	}
	
	public int size() {
		return capacity() - availableIndices.cardinality();
	}
	
	public void put(int i, T value) {
		array[i] = value;
		availableIndices.set(i, value == null);
	}
	
	public void add(T value) {
		if (isFull()) {
			throw new IllegalStateException("Array is full");
		}
		put(getLowestAvailable(), value);
	}
	
	public T remove(int i) {
		T value = array[i];
		array[i] = null;
		availableIndices.set(i);
		return value;
	}
	
	public T get(int i) {
		return array[i];
	}
	
	public void swap(int i, int j) {
		T temp = get(i);
		put(i, get(j));
		put(j, temp);
	}
	
	public boolean isAvailable(int i) {
		return availableIndices.get(i);
	}
	
	public boolean containsKey(int i) {
		return !isAvailable(i);
	}
	
	public boolean isFull() {
		return getLowestAvailable() == -1;
	}
	
	public int getLowestAvailable() {
		return availableIndices.nextSetBit(0);
	}
	
	public int nextAvailable(int i) {
		return next(i, result -> isAvailable(result));
	}
	
	public int getLowestInUse() {
		return availableIndices.nextClearBit(0);
	}
	
	public int nextInUse(int i) {
		return next(i, result -> containsKey(result));
	}
	
	public int getHighestInUse() {
		return availableIndices.previousClearBit(array.length - 1);
	}
	
	private int next(int i, IntFunction<Boolean> halt) {
		int result = i + 1;
		while (result < capacity() && !halt.apply(result)) {
			result += 1;
		}
		return result;		
	}

	@Override
	public FixedSizeArray<T> deepCopy() {
		FixedSizeArray<T> copy = new FixedSizeArray<>(size(), copier);
		for (int i = getLowestInUse(); i < array.length; i = nextInUse(i)) {
			copy.put(i, copier.apply(get(i)));
		}
		return copy;
	}
	
	public void doAll(BiConsumer<Integer,T> f) {
		for (int i = getLowestInUse(); i < array.length; i = nextInUse(i)) {
			f.accept(i, get(i));
		}
	}
	
	public boolean forAll(BiPredicate<Integer,T> p) {
		for (int i = getLowestInUse(); i < array.length; i = nextInUse(i)) {
			if (!p.test(i, get(i))) {
				return false;
			}
		}
		return true;
	}
	
	public ArrayList<Integer> indices() {
		ArrayList<Integer> indices = new ArrayList<>();
		doAll((i, v) -> indices.add(i));
		return indices;
	}
	
	public ArrayList<T> values() {
		ArrayList<T> values = new ArrayList<>();
		doAll((i, v) -> values.add(v));
		return values;
	}
}
