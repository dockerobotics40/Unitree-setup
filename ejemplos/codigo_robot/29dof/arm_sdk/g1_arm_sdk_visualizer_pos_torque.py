import csv
import time
from datetime import datetime
from collections import deque

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

class CSVVisualizer(QtWidgets.QMainWindow):
    def __init__(self, csv_file, max_samples=500, refresh_rate=50):
        super().__init__()
        self.setWindowTitle("Torque y Posición de Articulaciones")

        self.csv_file = csv_file
        self.max_samples = max_samples
        self.refresh_rate = refresh_rate

        # Leer encabezado
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            self.joint_labels = header[1:]
            self.num_joints = len(self.joint_labels) // 2

        # Preparar datos
        self.q_data = [deque(maxlen=max_samples) for _ in range(self.num_joints)]
        self.tau_data = [deque(maxlen=max_samples) for _ in range(self.num_joints)]
        self.time_data = deque(maxlen=max_samples)
        self.initial_time = None
        self.last_line = 1

        # Layout principal
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QHBoxLayout(central_widget)

        # Checkboxes para seleccionar articulaciones
        self.checkboxes = []
        checkbox_layout = QtWidgets.QVBoxLayout()
        for i in range(self.num_joints):
            cb = QtWidgets.QCheckBox(f"Joint {i}")
            cb.setChecked(True)
            cb.stateChanged.connect(self.update_visibility)
            self.checkboxes.append(cb)
            checkbox_layout.addWidget(cb)
        layout.addLayout(checkbox_layout)

        # Layout de gráficos
        graph_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(graph_layout)

        self.plot_widget_q = pg.PlotWidget(title="Posición (q)")
        self.plot_widget_tau = pg.PlotWidget(title="Torque Estimado (τ)")

        self.plot_widget_q.showGrid(x=True, y=True)
        self.plot_widget_tau.showGrid(x=True, y=True)

        self.plot_widget_tau.setXLink(self.plot_widget_q)  # Zoom sincronizado

        self.plot_widget_q.addLegend()
        self.plot_widget_tau.addLegend()

        self.curves_q = []
        self.curves_tau = []

        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'w']

        for i in range(self.num_joints):
            color = colors[i % len(colors)]
            curve_q = self.plot_widget_q.plot(pen=pg.mkPen(color, width=2), name=f"q{i}")
            curve_tau = self.plot_widget_tau.plot(pen=pg.mkPen(color, style=pg.QtCore.Qt.DashLine), name=f"τ{i}")
            self.curves_q.append(curve_q)
            self.curves_tau.append(curve_tau)

        graph_layout.addWidget(self.plot_widget_q)
        graph_layout.addWidget(self.plot_widget_tau)

        # Timer para actualización
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(refresh_rate)

    def update_visibility(self):
        for i, cb in enumerate(self.checkboxes):
            visible = cb.isChecked()
            self.curves_q[i].setVisible(visible)
            self.curves_tau[i].setVisible(visible)

    def update_plot(self):
        try:
            with open(self.csv_file, 'r') as f:
                reader = list(csv.reader(f))
                new_rows = reader[self.last_line:]
                if not new_rows:
                    return

                for row in new_rows:
                    if len(row) < 2:
                        continue

                    timestamp_str = row[0]
                    timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                    if self.initial_time is None:
                        self.initial_time = timestamp

                    elapsed = (timestamp - self.initial_time).total_seconds()
                    self.time_data.append(elapsed)

                    values = row[1:]
                    for j in range(self.num_joints):
                        q = float(values[j * 2])
                        tau = float(values[j * 2 + 1])
                        self.q_data[j].append(q)
                        self.tau_data[j].append(tau)

                self.last_line += len(new_rows)

                # Actualizar gráficas
                t = list(self.time_data)
                for j in range(self.num_joints):
                    self.curves_q[j].setData(t, list(self.q_data[j]))
                    self.curves_tau[j].setData(t, list(self.tau_data[j]))

        except Exception as e:
            print("Error actualizando gráfico:", e)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    csv_path = input("Ingrese la ruta del archivo .csv: ").strip()
    viewer = CSVVisualizer(csv_path)
    viewer.show()
    sys.exit(app.exec_())
