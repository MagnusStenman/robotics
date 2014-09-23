package Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.text.SimpleDateFormat;
import java.util.List;
import java.util.Map;

import SuppliedFiles.DifferentialDriveRequest;
import SuppliedFiles.LaserEchoesResponse;
import SuppliedFiles.LocalizationResponse;
import SuppliedFiles.Position;
import SuppliedFiles.Request;
import SuppliedFiles.Response;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

public class FinalRobot {

	private static final double SPEED_CONSTANT = -0.000123456790123;
	private static final int DIST_TO_GOAL = 1;
	private static final double DIST_TO_TARGET_MIN = 0.8;
	private static final double DIST_TO_NEXT_CP = 0.5;
	private static final double ANGLE_TO_NEXT_CP = 50;
	private int mapListIndex = 0;
	private String host;
	private int port;
	private FinalRobot robot;
	private ObjectMapper mapper;
	private List<Map<String, Object>> mapList;

	public FinalRobot(String host, int port) {
		this.host = host;
		this.port = port;
		this.robot = this;

		mapper = new ObjectMapper();
	}

	public FinalRobot(String host, int port, String filePath) {
		this.host = host;
		this.port = port;
		this.robot = this;

		mapper = new ObjectMapper();
		try {
			mapList = readFile(filePath);
		} catch (JsonParseException e) {
			System.err.println(e.getMessage());
		} catch (JsonMappingException e) {
			System.err.println(e.getMessage());
		} catch (IOException e) {
			System.err.println(e.getMessage());
		}
	}

	public static void main(String[] args) {
		FinalRobot robot = new FinalRobot("http://127.0.0.1", 50000,
				"path-around-table-and-back.json");

		System.out.println("START");
		try {
			robot.run();
		} catch (Exception e) {
			System.err.println("Something went wrong: " + e.getMessage());
			e.printStackTrace();
		}
	}

	private Position carrotPlanning(LocalizationResponse robotLR,
			LocalizationResponse nextLR) {
		Position currentCP = new Position(nextLR.getPosition());

		LocalizationResponse tempLR = new LocalizationResponse();

		if (mapList.size() > mapListIndex + 1) {
			tempLR.setData(mapList.get(mapListIndex + 1));
			Position nextCP = new Position(tempLR.getPosition());

			while (isShortDistance(robotLR, currentCP, nextCP)) {
				mapListIndex++;
				tempLR.setData(mapList.get(mapListIndex));
				nextCP = new Position(tempLR.getPosition());
			}

			return nextCP;
		}
		return null;
	}

	private boolean isShortDistance(LocalizationResponse robotLR,
			Position currentCP, Position nextCP) {
		Position robotPos = new Position(robotLR.getPosition());

		double currentCPDistance = robotPos.getDistanceTo(currentCP);
		double nextCPDistance = robotPos.getDistanceTo(nextCP);

		double currentCPAngle = robotPos.getBearingTo(currentCP);
		double nextCPAngle = robotPos.getBearingTo(nextCP);

		if (mapList.size() < mapListIndex) {
			return false;
		}

		return (Math.abs(currentCPDistance - nextCPDistance) < DIST_TO_NEXT_CP)
				&& (Math.abs(calculateAngleDiff(currentCPAngle, nextCPAngle)) < ANGLE_TO_NEXT_CP);
	}

	public List<Map<String, Object>> readFile(String filePath)
			throws JsonParseException, JsonMappingException, IOException {
		File file = new File(filePath);
		return mapper.readValue(file, TypeFactory.defaultInstance()
				.constructCollectionType(List.class, Map.class));
	}

	public void run() throws Exception {
		long timeStart = System.currentTimeMillis();
		System.out.println("Number of path coordinates: " + mapList.size());

		mapListIndex = 0;
		do {
			LocalizationResponse robotLR = new LocalizationResponse();
			LocalizationResponse nextLR = new LocalizationResponse();
			robot.getResponse(robotLR);
			nextLR.setData(mapList.get(mapListIndex));

			Position nextCP = carrotPlanning(robotLR, nextLR);

			if (nextCP != null) {
				calculateAndMove(robotLR, nextCP);
			}

			mapListIndex++;
			if (hasReachedGoal(new Position(robotLR.getPosition()), nextCP)) {
				mapListIndex = mapList.size() + 1;
			}

		} while (mapList.size() > mapListIndex);

		DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		ddr.setLinearSpeed(0);
		ddr.setAngularSpeed(0);
		putRequest(ddr);

		System.out.println("TIME: "
				+ new SimpleDateFormat("mm:ss").format(System
						.currentTimeMillis() - timeStart));

	}

	private boolean hasReachedGoal(Position robotPos, Position next) {
		return robotPos.getDistanceTo(next) <= DIST_TO_GOAL
				&& (mapListIndex > mapList.size() * 0.8);
	}

	private void calculateAndMove(LocalizationResponse robotLR, Position nextCP)
			throws Exception {

		double targetDistance = 1; // init value
		double speed = 0;
		double angle = 0;

		while (targetDistance > DIST_TO_TARGET_MIN) {

			robot.getResponse(robotLR);
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

			putRequest(ddr);
			
			// System.out.println("Current SPEED: " + speed);
		}
	}

	private double positiveDegrees(double targetAngle) {
		if (targetAngle < 0) {
			targetAngle += 360;
		}
		return targetAngle;
	}

	private double turnHeading(double angle) {
		if (angle > 0) {
			return -1;
		} else {
			return 1;
		}
	}

	private DifferentialDriveRequest collisionDetection(
			DifferentialDriveRequest ddr, double angle) throws Exception {

		
//		LaserEchoesResponse ler = new LaserEchoesResponse();
//
//		boolean left = true, right = true, front = true;
//
//		while (left || right || front) {
//			robot.getResponse(ler);
//			double[] echoes = ler.getEchoes();
//			left = false;
//			right = false;
//			front = false;
//
//			double Llength = 0;
//			double Rlength = 0;
//			double Flength;
//
//			for (int i = 45; i < 225; i++) {
//
//				if (echoes[i] < 0.4) {
//					if (45 < i && i < 110) {
//						Rlength = echoes[i];
//						right = true;
//					}
//				}
//			}
//			for (int i = 45; i < 225; i++) {
//				if (echoes[i] < 0.4) {
//					if (160 < i && i < 225) {
//						Llength = echoes[i];
//						left = true;
//					}
//				}
//			}
//			for (int i = 45; i < 225; i++) {
//				if (echoes[i] < 0.5) {
//					if (110 < i && i < 160) {
//						Flength = echoes[i];
//						front = true;
//					}
//				}
//			}
//
//			if (left && right && !front) {
//				ddr.setLinearSpeed(1);
//				ddr.setAngularSpeed(0);
//				putRequest(ddr);
//				// Thread.sleep(100);
//			} else if (left && !right && front) {
//				ddr.setLinearSpeed(Llength);
//				ddr.setAngularSpeed(-(1 - Llength));
//				putRequest(ddr);
//				// Thread.sleep(300);
//				// ddr.setLinearSpeed(1);
//				// ddr.setAngularSpeed(0);
//				// putRequest(ddr);
//				// Thread.sleep(100);
//			} else if (!left && right && front) {
//				ddr.setLinearSpeed(Rlength);
//				ddr.setAngularSpeed(1 - Rlength);
//				putRequest(ddr);
//				// Thread.sleep(300);
//				// ddr.setLinearSpeed(1);
//				// ddr.setAngularSpeed(0);
//				// putRequest(ddr);
//				// Thread.sleep(100);
//			} else if (!left && !right && front) {
//				ddr.setLinearSpeed(0);
//				ddr.setAngularSpeed(1);
//				putRequest(ddr);
//				Thread.sleep(1000);
//			} else if (left && !right && !front) {
//				ddr.setLinearSpeed(Llength);
//				ddr.setAngularSpeed(-(1 - Llength));
//				putRequest(ddr);
//			} else if (!left && right && !front) {
//				ddr.setLinearSpeed(Rlength);
//				ddr.setAngularSpeed(1 - Rlength);
//				putRequest(ddr);
//			} else if (left && right && front) {
//				ddr.setLinearSpeed(0);
//				ddr.setAngularSpeed(1);
//				putRequest(ddr);
//				Thread.sleep(1000);
//			}
//
//			if (Llength > 0) {
//				System.out.println("LEFT: " + (-(1 - Llength)));
//				System.out.println("SPEED: " + Llength);
//				System.out.println();
//			}
//			if (Rlength > 0) {
//				System.out.println("RIGHT: " + (1 - Rlength));
//				System.out.println("SPEED: " + Rlength);
//				System.out.println();
//			}
//
//			// putRequest(ddr);
//
//			String debug = "";
//			if (left)
//				debug += " [LEFT] ";
//			if (right)
//				debug += " [RIGHT] ";
//			if (front)
//				debug += " [FRONT] ";
//
//			if (debug != "") {
//				// System.out.println(debug);
//			}
//		}
		
		LaserEchoesResponse ler = new LaserEchoesResponse();
		robot.getResponse(ler);
		double[] test = ler.getEchoes();
		for (int i = 120; i < 150; i++) {
			if (test[i] < 0.7) {
				System.out.println("HEJ");
				ddr.setAngularSpeed(turnHeading(angle) * 1.4);
				ddr.setLinearSpeed(0.2);
			}
		}

		
		
		return ddr;
	}

	private double degreesToRadians(double angle) {
		return angle * (Math.PI / 180);
	}

	private double calculateAngleDiff(double firstAngle, double secondAngle) {
		double diffAngle = (firstAngle - secondAngle) + 180;
		diffAngle = (diffAngle / 360.0);
		diffAngle = ((diffAngle - Math.floor(diffAngle)) * 360.0) - 180;
		return diffAngle;
	}

	/**
	 * Send a request to the robot.
	 * 
	 * @param r
	 *            request to send
	 * @return response code from the connection (the web server)
	 * @throws Exception
	 */
	public int putRequest(Request r) throws Exception {
		URL url = new URL(host + ":" + port + r.getPath());

		HttpURLConnection connection = (HttpURLConnection) url.openConnection();

		connection.setDoOutput(true);

		connection.setRequestMethod("POST");
		connection.setRequestProperty("Content-Type", "application/json");
		connection.setUseCaches(false);

		OutputStreamWriter out = new OutputStreamWriter(
				connection.getOutputStream());

		// construct a JSON string
		String json = mapper.writeValueAsString(r.getData());

		// write it to the web server
		out.write(json);
		out.close();

		// wait for response code
		int rc = connection.getResponseCode();

		return rc;
	}

	/**
	 * Get a response from the robot
	 * 
	 * @param r
	 *            response to fill in
	 * @return response same as parameter
	 * @throws Exception
	 */
	public Response getResponse(Response r) throws Exception {
		URL url = new URL(host + ":" + port + r.getPath());

		// open a connection to the web server and then get the resulting data
		URLConnection connection = url.openConnection();
		BufferedReader in = new BufferedReader(new InputStreamReader(
				connection.getInputStream()));

		// map it to a Java Map
		@SuppressWarnings("unchecked")
		Map<String, Object> data = mapper.readValue(in, Map.class);
		r.setData(data);

		in.close();

		return r;
	}
}
