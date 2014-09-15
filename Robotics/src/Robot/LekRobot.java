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
import SuppliedFiles.LocalizationResponse;
import SuppliedFiles.Position;
import SuppliedFiles.Request;
import SuppliedFiles.Response;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

public class LekRobot {

	private int mapListIndex = 0;
	private String host;
	private int port;
	private LekRobot robot;
	private ObjectMapper mapper;
	private List<Map<String, Object>> mapList;

	public LekRobot(String host, int port) {
		this.host = host;
		this.port = port;
		this.robot = this;

		mapper = new ObjectMapper();
	}

	public LekRobot(String host, int port, String filePath) {
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
		LekRobot robot = new LekRobot("http://127.0.0.1", 50000,
				"Path-around-table-and-back.json");

		System.out.println("START");
		try {
			robot.run();
		} catch (Exception e) {
			System.err.println("Something went wrong: " + e.getMessage());
		}
	}

	private Position carrotPlanning(LocalizationResponse robotLR,
			LocalizationResponse nextLR) {
		Position currentCP = new Position(nextLR.getPosition());

		LocalizationResponse tempLR = new LocalizationResponse();

		if (mapList.size() > mapListIndex + 1) {
			tempLR.setData(mapList.get(mapListIndex + 1));
			Position nextCP = new Position(tempLR.getPosition());

			while(isShortDistance(robotLR, currentCP, nextCP)) {
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
		
		if(mapListIndex >= mapList.size()) {
			return false;
		}  
		
		return (Math.abs(currentCPDistance - nextCPDistance) < 1)
				&& (Math.abs(calculateAngleDiff(currentCPAngle,
						nextCPAngle)) < 10);
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
			System.out.println("MAPCOUNT: " + mapListIndex);

		} while (mapList.size() > mapListIndex);

		DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		ddr.setLinearSpeed(0);
		ddr.setAngularSpeed(0);
		putRequest(ddr);

		long timeStop = System.currentTimeMillis();

		System.out.println("TIME IT TOOK: "
				+ new SimpleDateFormat("mm:ss").format(timeStop - timeStart));

	}

	private boolean hasReachedGoal(Position robotPos, Position next) {
		return robotPos.getDistanceTo(next) <= 1 && (mapListIndex > mapList.size() * 0.8);
	}

	private void calculateAndMove(LocalizationResponse robotLR, Position nextCP)
			throws Exception {

		double targetDistance = 1;

		double speed = 0, angle = 0;

		while (targetDistance > 0.5) {

			robot.getResponse(robotLR);
			Position robotPos = new Position(robotLR.getPosition());
			// TODO is it ok???
			double robotHeading = robotLR.getHeadingAngle() * (180 / Math.PI);

			targetDistance = robotPos.getDistanceTo(nextCP);
			
			double targetAngle = Math.toDegrees(robotPos.getBearingTo(nextCP));
			if (targetAngle < 0) {
				targetAngle += 360;
			}

			double angleDiff = calculateAngleDiff(robotHeading,
					targetAngle);

			DifferentialDriveRequest ddr = new DifferentialDriveRequest();

			if (Math.abs(angleDiff) > 2) {
				ddr.setLinearSpeed(0);
				speed = 0;
				ddr.setAngularSpeed(0.0);
				angle = 0;
				if (angleDiff < 0) {
					// RIGHT
					ddr.setAngularSpeed(0.6);
					angle = 0.6;
				} else {
					// LEFT
					ddr.setAngularSpeed(-0.6);
					angle = -0.6;
				}
				if (angleDiff > 40 || angleDiff < -40) {
					if (angleDiff > 0) {
						ddr.setAngularSpeed(-2);
						angle = -2;
					} else {
						ddr.setAngularSpeed(2);
						angle = 2;
					}
					ddr.setLinearSpeed(0.0);
					speed = 0.2;
				} else if (Math.abs(angleDiff) < 30) {
					ddr.setLinearSpeed(1.0);
					speed = 1;
				}
			} else {
				ddr.setAngularSpeed(0.0);
				angle = 0;
				ddr.setLinearSpeed(1.0);
				speed = 1;
			}
			putRequest(ddr);

			// System.out.println("robotPOS: " + robotPos.getX() + ", " +
			// robotPos.getY());
			// System.out.println("nextPOS: " + nextPos.getX() + ", " +
			// nextPos.getY());
			// System.out.println("target distance " + targetDistance);
			// System.out.println("target angle " + targetAngle);
			// System.out.println("robot angle " + robotHeading);
			// System.out.println("angle diff " + angleDiff);
		}
		// System.out.println("speed :" + speed + " and anglespeed: " + angle);
	}

	private double calculateAngleDiff(double firstAngle,
			double secondAngle) {
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
		Map<String, Object> data = mapper.readValue(in, Map.class);
		r.setData(data);

		in.close();

		return r;
	}
}
