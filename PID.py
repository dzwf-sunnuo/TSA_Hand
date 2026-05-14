
import threading
import time
import serial
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

SERIAL_PORT = 'COM8'
BAUDRATE = 115200
FRAME_START = 0xFF
FRAME_END = 0xFE
SAMPLE_INTERVAL = 0.02  # 20 ms


class SerialReader(threading.Thread):
	def __init__(self, port, baudrate):
		super().__init__(daemon=True)
		self.ser = serial.Serial(port, baudrate, timeout=1)
		self.lock = threading.Lock()
		self.times = collections.deque(maxlen=1000)
		self.data1 = collections.deque(maxlen=1000)
		self.data2 = collections.deque(maxlen=1000)
		self.start_time = time.time()
		self._running = True

	def run(self):
		buf = ''
		while self._running:
			chunk = self.ser.read(self.ser.in_waiting or 1)
			if not chunk:
				continue
			buf += chunk.decode('ascii', errors='ignore')
			while '\n' in buf:
				line, buf = buf.split('\n', 1)
				line = line.strip().rstrip(',')
				if not line:
					continue

				parts = [p.strip() for p in line.split(',') if p.strip()]
				if len(parts) < 4:
					continue
				try:
					values = [int(p) for p in parts[:4]]
				except ValueError:
					continue

				if values[0] != FRAME_START or values[3] != FRAME_END:
					continue

				d1 = values[1]
				d2 = values[2]
				t = time.time() - self.start_time
				with self.lock:
					self.times.append(t)
					self.data1.append(d1)
					self.data2.append(d2)

	def stop(self):
		self._running = False
		try:
			self.ser.close()
		except Exception:
			pass


def main():
	reader = SerialReader(SERIAL_PORT, BAUDRATE)
	reader.start()

	plt.style.use('seaborn-v0_8-darkgrid')
	fig, ax = plt.subplots()
	line1, = ax.plot([], [], label='Data1', color='tab:blue')
	line2, = ax.plot([], [], label='Data2', color='tab:orange')
	ax.set_xlabel('Time (s)')
	ax.set_ylabel('Angle (deg)')
	ax.legend()

	def init():
		ax.set_xlim(0, 10)
		ax.set_ylim(0, 18000)
		return line1, line2

	def update(frame):
		with reader.lock:
			times = list(reader.times)
			d1 = list(reader.data1)
			d2 = list(reader.data2)
		if not times:
			return line1, line2
		ax.set_xlim(max(0, times[-1] - 10), times[-1] + 0.1)
		line1.set_data(times, d1)
		line2.set_data(times, d2)
		return line1, line2

	ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=True)

	try:
		plt.show()
	finally:
		reader.stop()


if __name__ == '__main__':
	main()
