package Robot;

import java.awt.peer.RobotPeer;
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

import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;

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
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (JsonMappingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args) {

		LekRobot robot = new LekRobot("http://127.0.0.1", 50000,
				"Path-around-table-and-back.json");
		try {
			System.out.println("START");
			robot.start();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public List<Map<String, Object>> readFile(String filePath)
			throws JsonParseException, JsonMappingException, IOException {
		File file = new File(filePath);
		return mapper.readValue(file, TypeFactory.defaultInstance()
				.constructCollectionType(List.class, Map.class));
	}

	public void start() throws Exception {

		/*
		 * Is path defined? if yes continue with algorithm
		 * 
		 * 1. check robot position 2. check length and angle towards next
		 * position 3. calculate angular and linear speed 4. run for calculated
		 * seconds
		 * 
		 * (loop) check #2 again to see if we are close to target - if close
		 * track next target position and run loop - if not -> continue loop (1
		 * -> 2 -> 3 -> 4)
		 */
		long timeStart = System.currentTimeMillis();
		System.out.println("Number of path coordinates: " + mapList.size());

		int i = 0;
		do {

			LocalizationResponse robotLR = new LocalizationResponse();
			LocalizationResponse nextLR = new LocalizationResponse();
			nextLR.setData(mapList.get(i));

			calculateAndMove(robotLR, nextLR);

			System.out.println("MAPCOUNT: " + i);
			i++;

		} while (mapList.size() > i);

		DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		ddr.setLinearSpeed(0);
		ddr.setAngularSpeed(0);
		putRequest(ddr);
		
		long timeStop = System.currentTimeMillis();
		
		System.out.println("TIME IT TOOK: "+ new SimpleDateFormat("mm:ss").format(timeStop-timeStart));
		// TESTCODE//
		/*
		 * DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		 * 
		 * LocalizationResponse lr = new LocalizationResponse();
		 * robot.getResponse(lr);
		 * 
		 * if (robot.getBearingAngle(lr) < 0) {
		 * 
		 * }
		 * 
		 * System.out.println(robot.getBearingAngle(lr));
		 * 
		 * ddr.setAngularSpeed(0.2); putRequest(ddr); try { Thread.sleep(10000);
		 * } catch (InterruptedException e) { // TODO Auto-generated catch block
		 * e.printStackTrace(); } ddr.setAngularSpeed(0); putRequest(ddr);
		 */
		// TESTCODE//

	}

	private void calculateAndMove(LocalizationResponse robotLR,
			LocalizationResponse nextLR) throws Exception {

		double targetDistance = 1;

		while (targetDistance > 0.5) {


			robot.getResponse(robotLR);
			Position robotPos = new Position(robotLR.getPosition());
			Position nextPos = new Position(nextLR.getPosition());
			double robotHeading = getBearingAngle(robotLR);
			targetDistance = robotPos.getDistanceTo(nextPos);
			double targetAngle = Math.toDegrees(robotPos.getBearingTo(nextPos));
			if (targetAngle < 0) {
				targetAngle += 360;
			}

			/*
			 * 1. Calculate angle and speed (depending on target distance &
			 * angle) 2. Putrequest 3. Sleep ms (30ms?) 4. goto 1
			 */

			// double angleDiff = robotHeading - targetAngle;
			// if (angleDiff > 180) {
			// angleDiff -= 360;
			// }

			double angleDiff = calculateDifferenceBetweenAngles(robotHeading,
					targetAngle);

			DifferentialDriveRequest ddr = new DifferentialDriveRequest();

			if (Math.abs(angleDiff) > 2) {
				ddr.setLinearSpeed(0.2);
				ddr.setAngularSpeed(0.0);
				if (angleDiff < 0) {
					// RIGHT
					ddr.setAngularSpeed(0.6);
				} else {
					// LEFT
					ddr.setAngularSpeed(-0.6);
				}
				if (angleDiff > 90 || angleDiff < -90) {
					if (angleDiff > 0) {
						ddr.setAngularSpeed(-1);
					} else {
						ddr.setAngularSpeed(1);
					}
					ddr.setLinearSpeed(0);
				} else if (Math.abs(angleDiff) < 30) {
					ddr.setLinearSpeed(0.8);
				}
				
				if(Math.abs(angleDiff) > 30 && targetDistance > 1) {
					System.out.println("INEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					ddr.setLinearSpeed(1);
				}
				putRequest(ddr);
			} else {
				ddr.setAngularSpeed(0.0);
				ddr.setLinearSpeed(1.0);
				putRequest(ddr);
			}

			// System.out.println("robotPOS: " + robotPos.getX() + ", " +
			// robotPos.getY());
			// System.out.println("nextPOS: " + nextPos.getX() + ", " +
			// nextPos.getY());
//			System.out.println("target distance " + targetDistance);
//			System.out.println("target angle " + targetAngle);
//			System.out.println("robot angle " + robotHeading);
//			System.out.println("angle diff " + angleDiff);
		}
		System.out.println("UTE!");
	}

	private double calculateDifferenceBetweenAngles(double firstAngle,
			double secondAngle) {
		double diffangle = (firstAngle - secondAngle) + 180;
		diffangle = (diffangle / 360.0);
		diffangle = ((diffangle - Math.floor(diffangle)) * 360.0) - 180;
		return diffangle;
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

	double getBearingAngle(LocalizationResponse lr) {
		double e[] = lr.getOrientation();

		double angle = 2 * Math.atan2(e[3], e[0]);
		double positiveAngle = angle * 180 / Math.PI;
		// if (positiveAngle < 0) {
		// positiveAngle += 360;
		// }
		return positiveAngle;
	}
}
