package edu.hendrix.ev3.remote.net.actionselector;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.EnumMap;

import org.joda.time.LocalDateTime;
import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.remote.net.NetBot;
import edu.hendrix.ev3.remote.net.NetBotCommand;
import edu.hendrix.ev3.remote.net.RobotConstants;
import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.EnumHistogram;
import edu.hendrix.ev3.util.Logger;
import edu.hendrix.ev3.util.StampedStorage;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.video.Video;

public class ActionSelectorBot extends NetBot {
	
	public static void main(String[] args) throws IOException {
		new ActionSelectorBot().mainLoop();
	}
	
	private BSOCController ai;
	private LocalDateTime aiTimestamp;
	private byte lastTag;
	private Move lastMove;
	private Mode mode;
	private Video video;
	private byte[] frame;
	private EnumHistogram<Mode> modeCycles;
	private EnumMap<Mode,Long> modeTimes;
	private long currentModeStart;
	private EV3TouchSensor LEFT_TOUCH, RIGHT_TOUCH;
	private EV3UltrasonicSensor SONAR;
	private final float[] sensorData = new float[3];
	
	public ActionSelectorBot() throws IOException {
		super(ActionSelectorCommand.SIZE, ActionSelectorReply.SIZE);
	}

	@Override
	public void setup() {
		try {
			lastMove = Move.STOP;
			mode = Mode.WAITING;
			video = RobotConstants.setupEV3Video();
			frame = video.createFrame();
			LEFT_TOUCH = new EV3TouchSensor(SensorPort.S1);
			RIGHT_TOUCH = new EV3TouchSensor(SensorPort.S4);
			SONAR = new EV3UltrasonicSensor(SensorPort.S2);
			modeCycles = new EnumHistogram<>(Mode.class);
			modeTimes = new EnumMap<>(Mode.class);
			currentModeStart = System.currentTimeMillis();
		} catch (IOException ioe) {
			Logger.EV3Log.format("Could not set up video: %s", ioe.getMessage());
			throw new IllegalStateException(ioe.getMessage());
		}
	}

	@Override
	public void teardown() {
		try {
			changeMode(Mode.WAITING);
			for (Mode m: Mode.values()) {
				Logger.EV3Log.format("Mode: %s Cycles: %d ms: %d", m.toString(), modeCycles.getCountFor(m), modeTimes.containsKey(m) ? modeTimes.get(m) : 0);
				if (modeTimes.containsKey(m)) {
					double hz = modeCycles.getCountFor(m) / (modeTimes.get(m) / 1000.0);
					Logger.EV3Log.format("cycles/sec: %5.3f", hz);
				}
			}
			StampedStorage.save(ai, aiTimestamp);
		} catch (FileNotFoundException e) {
			Logger.EV3Log.log("Exception when saving: " + e.getMessage());
			throw new IllegalStateException(e.getMessage());
		}
	}

	@Override
	public void checkSensors() {
		try {
			LEFT_TOUCH.fetchSample(sensorData, 0);
			RIGHT_TOUCH.fetchSample(sensorData, 1);
			SONAR.fetchSample(sensorData, 2);
			video.grabFrame(frame);
		} catch (IOException ioe) {
			Logger.EV3Log.format("Exception when grabbing: %s", ioe.getMessage());
			throw new IllegalStateException(ioe.getMessage());
		}
	}
		
	public AdaptedYUYVImage wrapBytes() {
		return new AdaptedYUYVImage(frame, RobotConstants.WIDTH, RobotConstants.HEIGHT);
	}

	private void changeMode(Mode newMode) {
		long modeTime = System.currentTimeMillis() - currentModeStart;
		if (!modeTimes.containsKey(mode)) {
			modeTimes.put(mode, (long)0);
		}
		modeTimes.put(mode, modeTime + modeTimes.get(mode));
		mode = newMode;
		currentModeStart = System.currentTimeMillis();
	}

	@Override
	public NetBotCommand selectMoveAndReply(byte[] receivedMessage) {
		if (receivedMessage.length > 0) {
			decodeCmd(new ActionSelectorCommand(receivedMessage));
		}
		modeCycles.bump(mode);
		if (mode == Mode.LEARNING) {
			this.setLivefeed(false);
			if (lastMove != Move.STOP){
				this.setLivefeed(true);
				Logger.EV3Log.format("%s,%s,%f,%f,%f", "Learning",
						lastMove.uiName(),sensorData[0],
						sensorData[1],sensorData[2]);				
				if (ai != null) {
					int node = ai.train(wrapBytes());
					ai.assignMoveFor(node, getCurrentMove());
				}
			} else {
				Logger.EV3Log.log("Stopped");
			}
			
			return new NetBotCommand(lastMove);
		} else if (mode == Mode.PULSE){
			this.setLivefeed(false);
			byte[] pulse = createPulse();
			changeMode(Mode.LEARNING);
			return new NetBotCommand(lastMove, pulse, false);
		} else if (mode == Mode.RETRIEVING) {
			this.setLivefeed(false);
			Logger.EV3Log.format("retrieving"); 
			ActionSelectorReply reply = new ActionSelectorReply(lastTag);
			for (Duple<LocalDateTime, Integer> name: StampedStorage.getAvailableFor(BSOCController.class)) {
				reply.addName(name.getFirst(), name.getSecond());
			}
			changeMode(Mode.WAITING);
			return new NetBotCommand(Move.STOP, reply.toBytes(), false);
			
		} else if (mode == Mode.APPLYING) {
			this.setLivefeed(false);
			lastMove = ai.pickMoveFor(wrapBytes());
			Logger.EV3Log.format("%s,%s,%f,%f,%f", "Applying",
					lastMove.uiName(),sensorData[0],
					sensorData[1],sensorData[2]);
			print("APPLYING");
			return new NetBotCommand(lastMove);
			
			
		} else if (mode == Mode.QUIT) {
			this.setLivefeed(false);
			Logger.EV3Log.log("quitting");
			return NetBotCommand.makeQuit();
			
		} else {
			Logger.EV3Log.format("In mode %s and doing nothing", mode.toString());
			return new NetBotCommand(Move.STOP);
		}
	}
	private byte[] createPulse(){
		// the size of the buffer is the -1 + TIME + 1 Mode.Ordinal
		int size = StampedStorage.DATE_TIME_BYTES + 2;
		ByteBuffer buffer = ByteBuffer.allocate(size);
		buffer.put((byte)-1);
		StampedStorage.putInto(LocalDateTime.now(), buffer);
		buffer.put((byte)lastMove.ordinal());
		return buffer.array();		
	}
	private void decodeCmd(ActionSelectorCommand cmd) {
		lastMove = cmd.getMove();
		lastTag = cmd.getTag();
		Mode updated = cmd.getMode();
		if (updated == Mode.STARTING) {
			ai = new BSOCController(cmd.getNumClusters(), cmd.getShrinkFactor());
			aiTimestamp = cmd.getStamp();
			changeMode(Mode.PULSE);
			print("c:" + cmd.getNumClusters() + ";s:" + cmd.getShrinkFactor(),3);
			resetTimer();
			Logger.EV3Log.format("Starting: clusters: %d, shrink: %d", cmd.getNumClusters(), cmd.getShrinkFactor());
		} else {
			changeMode(updated);
			if (updated == Mode.APPLYING) {
				try {
					ai = StampedStorage.open(BSOCController.class, cmd.getStamp(), cmd.getSuffix(), BSOCController::fromString);
				} catch (FileNotFoundException e) {
					Logger.EV3Log.log("Can't open " + cmd.getStamp());
				}
			} else if (mode == Mode.LIVE_DEMO){
				print("Doin it!");
				mode = Mode.APPLYING;
			}
		}
	}
	
}
