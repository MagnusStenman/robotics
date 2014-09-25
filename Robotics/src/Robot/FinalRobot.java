package Robot;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import SuppliedFiles.DifferentialDriveRequest;
import SuppliedFiles.LaserEchoesResponse;
import SuppliedFiles.LocalizationResponse;
import SuppliedFiles.Position;
import SuppliedFiles.RobotCommunication;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

/**
 * FinalRobot class This class controls how the robot moves along a given path
 * of coordinates.
 * 
 * @author Magnus Stenman, dv12msn
 * @author Christer Jakobsson, dv12cjn
 *
 */
public class FinalRobot {

	private static final double SPEED_CONSTANT = -0.000123456790123;
	private static final int DIST_TO_GOAL = 1;
	private static final double MIN_DIST_TO_TARGET = 0.8;
	private static final double MAX_DIST_TO_NEXT_CP = 0.5;
	private static final double MAX_ANGLE_TO_NEXT_CP = 50;
	private int mapListIndex;
	private List<Map<String, Object>> mapList;
	private RobotCommunication robotComm;
	private Position goalPos;

	/**
	 * Constructor for FinalRobot.
	 * 
	 * @param host
	 *            communication IP-address or URL to MRDS
	 * @param port
	 *            communication port to MRDS
	 * @param filePath
	 *            path to the JSON-file
	 */
	public FinalRobot(String host, int port, String filePath) {
		robotComm = new RobotCommunication(host, port);

		try {
			mapList = readFile(filePath);
			LocalizationResponse goalLR = new LocalizationResponse();
			goalLR.setData(mapList.get(mapList.size() - 1));
			goalPos = new Position(goalLR.getPosition());
		} catch (JsonParseException e) {
			System.err.println(e.getMessage());
		} catch (JsonMappingException e) {
			System.err.println(e.getMessage());
		} catch (IOException e) {
			System.err.println(e.getMessage());
		}
	}

	/**
	 * Run method which in a do-while loop calculates the next move depending on
	 * the given path and executes it. If the while-loop completes the robot
	 * will stop as it has reached the goal.
	 * 
	 * @throws Exception
	 */
	public void run() throws Exception {
		mapListIndex = 0;
		do {
			LocalizationResponse robotLR = new LocalizationResponse();
			robotComm.getResponse(robotLR);
			LocalizationResponse nextLR = new LocalizationResponse();
			nextLR.setData(mapList.get(mapListIndex));

			Position nextCP = carrotPlanning(robotLR, nextLR);

			if (nextCP != null) {
				calculateAndMove(robotLR, nextCP);
			} else {
				mapListIndex = mapList.size() + 1;
			}

			mapListIndex++;
			if (hasReachedGoal(new Position(robotLR.getPosition()))) {
				mapListIndex = mapList.size() + 1;
			}
		} while (mapList.size() > mapListIndex);

		DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		ddr.setLinearSpeed(0);
		ddr.setAngularSpeed(0);
		robotComm.putRequest(ddr);
	}

	/**
	 * CarrotPlanning calculates where the next target point should be by using
	 * keepSearching method and returns that target position if it exist, else
	 * no new targets exists and it will return null.
	 * 
	 * @param robotLR
	 *            LocalizationResponse for the robot
	 * @param nextLR
	 *            LocalizationResponse for the next position on the path
	 * @return the new target position or null if none exists
	 */
	private Position carrotPlanning(LocalizationResponse robotLR,
			LocalizationResponse nextLR) {
		Position currentCP = new Position(nextLR.getPosition());
		LocalizationResponse tempLR = new LocalizationResponse();

		if (mapList.size() > mapListIndex + 1) {
			tempLR.setData(mapList.get(mapListIndex + 1));
			Position nextCP = new Position(tempLR.getPosition());

			while (keepSearching(robotLR, currentCP, nextCP)) {
				mapListIndex++;
				tempLR.setData(mapList.get(mapListIndex));
				nextCP = new Position(tempLR.getPosition());
			}
			return nextCP;
		}
		return null;
	}

	/**
	 * KeepSearching checks if the suggested next carrotpoint is in the set max
	 * distance and angle from the current carrotpoint and if so, returns true.
	 * 
	 * @param robotLR
	 *            the robots LocalizationResponse
	 * @param currentCP
	 *            the current carrotpoint
	 * @param nextCP
	 *            the suggested next carrotpoint
	 * @return true if robot should keep searching for a carrotpoint or false if
	 *         the suggested carrotpoint is invalid
	 */
	private boolean keepSearching(LocalizationResponse robotLR,
			Position currentCP, Position nextCP) {
		Position robotPos = new Position(robotLR.getPosition());

		double currentCPDistance = robotPos.getDistanceTo(currentCP);
		double nextCPDistance = robotPos.getDistanceTo(nextCP);

		double currentCPAngle = robotPos.getBearingTo(currentCP);
		double nextCPAngle = robotPos.getBearingTo(nextCP);

		if (mapList.size() <= mapListIndex + 1) {
			return false;
		}

		return (Math.abs(currentCPDistance - nextCPDistance) < MAX_DIST_TO_NEXT_CP)
				&& (Math.abs(calculateAngleDiff(currentCPAngle, nextCPAngle)) < MAX_ANGLE_TO_NEXT_CP);
	}

	/**
	 * ReadFile reads the file to a List of Map objects using Typefactory to
	 * construct the collection while parsing from JSON with ObjectMapper.
	 * 
	 * @param filePath
	 *            path to the JSON path object
	 * @return List<Map<String, Object>>
	 * @throws JsonParseException
	 * @throws JsonMappingException
	 * @throws IOException
	 */
	public List<Map<String, Object>> readFile(String filePath)
			throws JsonParseException, JsonMappingException, IOException {
		ObjectMapper mapper = new ObjectMapper();
		File file = new File(filePath);
		return mapper.readValue(file, TypeFactory.defaultInstance()
				.constructCollectionType(List.class, Map.class));
	}

	/**
	 * HasReachedGoal checks if the robot has reached the paths end-point by
	 * checking the distance between the robot and goal but also that atleast
	 * 80% of the path has passed as the start and goal can be at the same
	 * position.
	 * 
	 * @param robotPos
	 *            current position of the robot
	 * @return true if goal has been reached
	 */
	private boolean hasReachedGoal(Position robotPos) {
		return robotPos.getDistanceTo(goalPos) <= DIST_TO_GOAL
				&& (mapListIndex > mapList.size() * 0.8);
	}

	/**
	 * CalculateAndMove gets called for every new move the robot will make. It
	 * uses a while-loop to move closer to the carrotpoint until
	 * MIN_DIST_TO_TARGET is met. A quadratic equation is used for speed and
	 * depends on current angle between the robots heading and the target.
	 * Before the robot makes its move it calls collisionDetection.
	 * 
	 * @param robotLR
	 *            the robots LocalizationResponse
	 * @param nextCP
	 *            the next carrotpoint to aim for
	 * @throws Exception
	 */
	private void calculateAndMove(LocalizationResponse robotLR, Position nextCP)
			throws Exception {
		double targetDistance = 1;
		double speed = 0;
		double angle = 0;

		while (targetDistance > MIN_DIST_TO_TARGET) {
			robotComm.getResponse(robotLR);
			Position robotPos = new Position(robotLR.getPosition());
			double targetAngle = Math.toDegrees(robotPos.getBearingTo(nextCP));
			double robotHeading = robotLR.getHeadingAngle() * (180 / Math.PI);
			double angleDiff = calculateAngleDiff(robotHeading, targetAngle);
			targetDistance = robotPos.getDistanceTo(nextCP);
			targetAngle = positiveDegrees(targetAngle);
			DifferentialDriveRequest ddr = new DifferentialDriveRequest();

			if (Math.abs(angleDiff) > 90) {
				speed = 0;
			} else {
				speed = SPEED_CONSTANT * (angleDiff * angleDiff) + 1;
			}

			angle = degreesToRadians(angleDiff) * 2;

			ddr.setLinearSpeed(speed);
			ddr.setAngularSpeed(-angle);

			ddr = collisionDetection(ddr, angle);

			robotComm.putRequest(ddr);
		}
	}

	/**
	 * Converts angles to positives.
	 * 
	 * @param targetAngle
	 * @return positive version of input
	 */
	private double positiveDegrees(double targetAngle) {
		if (targetAngle < 0) {
			targetAngle += 360;
		}
		return targetAngle;
	}

	/**
	 * @param angle
	 *            the angle to check
	 * @return -1 if negative angle, otherwise returns 1
	 */
	private int getTurnHeading(double angle) {
		if (angle > 0) {
			return -1;
		} else {
			return 1;
		}
	}

	/**
	 * CollisionDetection gets called every time the robot plans to move and
	 * uses the laser to check for any obstacles 0.7m and +-15 degrees in that
	 * direction. If anything is in the way this method will alter the command
	 * to avoid this.
	 * 
	 * @param ddr
	 *            the current move command
	 * @param angle
	 *            current angularspeed
	 * @return
	 * @throws Exception
	 */
	private DifferentialDriveRequest collisionDetection(
			DifferentialDriveRequest ddr, double angle) throws Exception {
		LaserEchoesResponse ler = new LaserEchoesResponse();
		robotComm.getResponse(ler);
		double[] echoes = ler.getEchoes();
		
		for (int i = 120; i < 150; i++) {
			if (echoes[i] < 0.7) {
				ddr.setAngularSpeed(getTurnHeading(angle) * 1.4);
				ddr.setLinearSpeed(0.2);
			}
		}
		return ddr;
	}

	/**
	 * Converts degrees to radians.
	 * 
	 * @param angle
	 *            the angle to be converted
	 * @return the angle in radians
	 */
	private double degreesToRadians(double angle) {
		return angle * (Math.PI / 180);
	}

	/**
	 * Calculates the difference between two angles so that it can also be used
	 * to tell direction.
	 * 
	 * @param firstAngle
	 * @param secondAngle
	 * @return angle difference in the range -180 < angle < 180
	 */
	private double calculateAngleDiff(double firstAngle, double secondAngle) {
		double diffAngle = (firstAngle - secondAngle) + 180;
		diffAngle = (diffAngle / 360.0);
		diffAngle = ((diffAngle - Math.floor(diffAngle)) * 360.0) - 180;
		return diffAngle;
	}

	/**
	 * Main checks for filepath to the JSON-file containing the path to follow,
	 * otherwise the robot will run demonstration path
	 * (Path-around-the-table-and-back.json), then runs the robot.
	 * 
	 * @param args
	 *            must be a filepath to a JSONfile in the right format
	 */
	public static void main(String[] args) {
		FinalRobot robot;
		if (args.length > 0) {
			robot = new FinalRobot("http://127.0.0.1", 50000, args[0]);
		} else {
			robot = new FinalRobot("http://127.0.0.1", 50000,
					"Path-around-table-and-back.json");
		}

		try {
			robot.run();
		} catch (Exception e) {
			System.err.println("Something went wrong in robot: "
					+ e.getMessage());
			e.printStackTrace();
		}
	}
}
