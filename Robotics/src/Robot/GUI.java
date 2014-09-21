package Robot;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.lang.ProcessBuilder.Redirect;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.text.Position;

import SuppliedFiles.*;

public class GUI extends JFrame {

	private JFrame frame;
	private JPanel panel;
	private List<Map<String, Object>> mapList;
	private CopyOnWriteArrayList<SuppliedFiles.Position> robotPath;
	
	public GUI(List<Map<String, Object>> mapList) {
		this.mapList = mapList;
		frame = new JFrame();
		panel = new JPanel2();
		robotPath = new CopyOnWriteArrayList<SuppliedFiles.Position>();

		SwingUtilities.invokeLater(new Runnable() {
			public void run() {

				frame.setSize(800, 800);
				frame.setBackground(Color.decode("#61876b"));
				frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

				panel = new JPanel2();
				
				frame.setContentPane(panel);
				frame.setVisible(true);
			}
		});
	}
	
	public void addStuff(SuppliedFiles.Position pos) {
		robotPath.add(pos);
		panel.repaint();
	}

	public class JPanel2 extends JPanel {
		private static final long serialVersionUID = 1L;

		public void paintComponent(Graphics g) {
			int scale = 80;
			int j = 1;
			for (int i = 0; i < mapList.size(); i++) {
				LocalizationResponse lri = new LocalizationResponse();
				lri.setData(mapList.get(i));
				SuppliedFiles.Position posi = new SuppliedFiles.Position(
						lri.getPosition());

				if (i < mapList.size()-1) {
					LocalizationResponse lrj = new LocalizationResponse();
					lrj.setData(mapList.get(j));
					SuppliedFiles.Position posj = new SuppliedFiles.Position(
							lrj.getPosition());
					g.drawLine((int) Math.round(posi.getX() * scale),
							(int) Math.round(posi.getY() * scale),
							(int) Math.round(posj.getX() * scale),
							(int) Math.round(posj.getY() * scale));
				} else {
					g.drawRect((int) Math.round(posi.getX()) * scale,
							(int) Math.round(posi.getY()) * scale, 5, 5);
				}
				j++;
			}
			
			if (robotPath.size() > 0) {
				for (SuppliedFiles.Position p : robotPath) {
					g.setColor(Color.RED);
					g.drawRect((int) Math.round(p.getX()*scale), (int) Math.round(p.getY()*scale), 2, 2);
				}
			}
		}
	}
}
