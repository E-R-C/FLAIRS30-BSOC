package edu.hendrix.ev3.ai.supervised;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;

import edu.hendrix.ev3.ai.TrainingList;
import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.storage.VideoStorage;
import edu.hendrix.ev3.util.Duple;

public class VideoEvalRobotLearner {
	public final static int PURGE = 0;
	
	private TrainingList examples, testing;
	private int numCorrect;
	private EnumSet<Move> labels;
	private EnumMap<Move,Integer> 
		numExamplesFor = new EnumMap<>(Move.class), 
		numCorrectFor = new EnumMap<>(Move.class);
	private ArrayList<TestResult> results;
	private RobotLearner learner;
	
	public VideoEvalRobotLearner(TrainingList trainingVideo, String robotLearnerType) throws FileNotFoundException, InstantiationException, IllegalAccessException, ClassNotFoundException {
		this(trainingVideo, (RobotLearner) Class.forName(robotLearnerType).newInstance());
	}
	
	public static TrainingList retrieveVideos(String id) throws FileNotFoundException {
		VideoStorage videos = VideoStorage.getPCStorage();
		TrainingList result = videos.open(id, EnumSet.allOf(Move.class));
		result.purgeTransitions(PURGE);
		return result;
	}
	
	public VideoEvalRobotLearner(TrainingList trainingVideo, RobotLearner learner) throws FileNotFoundException {
		examples = trainingVideo;
		labels = EnumSet.noneOf(Move.class);
		for (Duple<Move, AdaptedYUYVImage> ex: examples) {
			learner.train(ex.getSecond(), ex.getFirst());
			labels.add(ex.getFirst());
		}
		this.learner = learner;
	}
	
	public void test(TrainingList testingVideo) throws FileNotFoundException {
		for (Move label: labels) {
			numCorrectFor.put(label, 0);
			numExamplesFor.put(label, 0);
		}
		results = new ArrayList<>();
		testing = testingVideo;
		numCorrect = 0;
		for (Duple<Move, AdaptedYUYVImage> ex: testing) {
			Move learned = learner.bestMatchFor(ex.getSecond());
			if (learned == ex.getFirst()) {
				numCorrect += 1;
				numCorrectFor.put(ex.getFirst(), 1+numCorrectFor.get(ex.getFirst()));
			}
			numExamplesFor.put(ex.getFirst(), 1+numExamplesFor.get(ex.getFirst()));
		}
	}
	
	public int numCorrect() {return numCorrect;}
	public int numTests() {return testing.size();}
	
	public int numCorrectFor(Move m) {return numCorrectFor.get(m);}
	public int numExamplesFor(Move m) {return numExamplesFor.get(m);}
	
	public Set<Move> getMovesUsed() {return numExamplesFor.keySet();}
	
	public Duple<Integer,Integer> getRatioBestAlternativeFor(Move test, Move result) {
		int total = 0;
		int num = 0;
		for (TestResult info: results) {
			if (info.matches(test, result)) {
				total += info.getBestAlternative();
				num += 1;
			}
		}
		return new Duple<>(total, num);
	}
	
	public static String getFormattedStats(String title, int num, int denom) {
		int percent = (int)(100 * (float)num / denom);
		return String.format("%s: %d/%d (%d%%)", title, num, denom, percent);
	}
	
	public static TreeMap<String,TrainingList> retrieveVideosFrom(String[] strs) {
		TreeMap<String,TrainingList> result = new TreeMap<>();
		for (int i = 0; i < strs.length; i++) {
			try {
				Integer.parseInt(strs[i]);
				System.out.println("Loading video " + strs[i]);
				result.put(strs[i], retrieveVideos(strs[i]));
			} catch (NumberFormatException nfe) {
				// Intentionally left blank
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
		}
		return result;
	}
	
	public static void main(String[] args) throws FileNotFoundException, InstantiationException, IllegalAccessException, ClassNotFoundException {
		if (args.length < 2) {
			System.out.println("Usage: VideoEvalRobotLearner ids... RobotLearnerType...");
			System.exit(1);
		}

		TreeMap<String,TrainingList> ids = retrieveVideosFrom(args);

		for (int k = ids.size(); k < args.length; k++) {
			for (Entry<String, TrainingList> i: ids.entrySet()) {
				System.out.println("Training " + args[k] + " with " + i.getKey());
				long start = System.currentTimeMillis();
				VideoEvalRobotLearner erl = new VideoEvalRobotLearner(i.getValue(), args[k]);
				long duration = System.currentTimeMillis() - start;
				System.out.printf("Duration: %d ms\n", duration);
				for (Entry<String, TrainingList> j: ids.entrySet()) {
					System.out.printf("Training: %s; Testing: %s\n", i.getKey(), j.getKey());
					start = System.currentTimeMillis();
					erl.test(j.getValue());
					duration = System.currentTimeMillis() - start;
					System.out.println(getFormattedStats(args[k], erl.numCorrect(), erl.numTests()));
					System.out.printf("Duration: %d ms\n", duration);
					for (Move m: erl.getMovesUsed()) {
						System.out.println(getFormattedStats(m.toString(), erl.numCorrectFor(m), erl.numExamplesFor(m)));
					}
					for (Move test: Move.values()) {
						for (Move reply: Move.values()) {
							Duple<Integer,Integer> ratio = erl.getRatioBestAlternativeFor(test, reply);
							if (ratio.getSecond() > 0) {
								double mean = (double)ratio.getFirst() / ratio.getSecond();
								System.out.printf("%s -> %s: %4.2f (%d/%d)\n", test.toString(), reply.toString(), mean, ratio.getFirst(), ratio.getSecond());
							}
						}
					}
				}
				System.out.println();
			}
		}
	}
}
