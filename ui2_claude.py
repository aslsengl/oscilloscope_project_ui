import sys
import serial
import time
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtGui import QPalette, QColor

class SimpleOscilloscopeApp(QtWidgets.QMainWindow):
    def __init__(self, port='/dev/tty.usbserial-0001', baudrate=115200):
        super().__init__()
        self.setWindowTitle("Basit Osiloskop")
        self.resize(800, 600)
        
        # Veri buffer'ı - daha küçük ve yönetilmesi kolay
        self.buffer_size = 200
        self.data_buffer = np.zeros(self.buffer_size)
        
        # Kalibrasyon değerleri - bunları değiştirmeyin
        self.OFFSET = 525 # ADC offset değeri 
        self.GAIN = 3.3 / (820 - self.OFFSET)  # V/ADC birimi
        
        # Ölçüm değişkenleri
        self.v_min = 0
        self.v_max = 0
        self.v_pp = 0.0
        self.frequency = 0.0
        
        # Zaman tabanı değişkeni
        self.timebase_value = 50  # ms
        
        # Seri port açma
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"Port açıldı: {port}")
            time.sleep(0.2)
            self.ser.flushInput()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Bağlantı Hatası", f"Port açılamadı: {e}")
            sys.exit(1)
        
        # Ana widget ve layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        
        # Üst kontrol paneli
        control_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(control_layout)
        
        # Timebase ayarı
        timebase_layout = QtWidgets.QHBoxLayout()
        timebase_layout.addWidget(QtWidgets.QLabel("Timebase:"))
        self.timebase_combo = QtWidgets.QComboBox()
        self.timebase_combo.addItems(["10 ms", "20 ms", "50 ms", "100 ms", "200 ms"])
        self.timebase_combo.setCurrentIndex(2)  # 50ms default
        self.timebase_combo.currentIndexChanged.connect(self.update_timebase)
        timebase_layout.addWidget(self.timebase_combo)
        control_layout.addLayout(timebase_layout)
        
        # Volt/Div ayarı
        voltdiv_layout = QtWidgets.QHBoxLayout()
        voltdiv_layout.addWidget(QtWidgets.QLabel("Volt/Div:"))
        self.volt_div_spin = QtWidgets.QDoubleSpinBox()
        self.volt_div_spin.setRange(0.5, 5.0)
        self.volt_div_spin.setSingleStep(0.5)
        self.volt_div_spin.setValue(1.0)
        self.volt_div_spin.setSuffix(" V")
        self.volt_div_spin.valueChanged.connect(self.update_y_range)
        voltdiv_layout.addWidget(self.volt_div_spin)
        control_layout.addLayout(voltdiv_layout)
        
        # Durdur/Başlat butonu
        self.run_button = QtWidgets.QPushButton("Durdur")
        self.run_button.setCheckable(True)
        self.run_button.setChecked(True)
        self.run_button.clicked.connect(self.toggle_acquisition)
        control_layout.addWidget(self.run_button)
        
        # Temizle butonu
        self.clear_button = QtWidgets.QPushButton("Temizle")
        self.clear_button.clicked.connect(self.clear_data)
        control_layout.addWidget(self.clear_button)
        
        # Ölçüm bilgileri için panel
        measurements_layout = QtWidgets.QHBoxLayout()
        self.vmax_label = QtWidgets.QLabel("Vmax: 0.00 V")
        self.vmin_label = QtWidgets.QLabel("Vmin: 0.00 V")
        self.vpp_label = QtWidgets.QLabel("Vpp: 0.00 V")
        self.freq_label = QtWidgets.QLabel("Frekans: 0.00 Hz")
        
        # Tüm etiketler için font ve stil ayarları
        for label in [self.vmax_label, self.vmin_label, self.vpp_label, self.freq_label]:
            font = label.font()
            font.setBold(True)
            label.setFont(font)
            label.setStyleSheet("color: #00BFFF; background-color: #333333; padding: 5px;")
            measurements_layout.addWidget(label)
        
        main_layout.addLayout(measurements_layout)
        
        # Grafik alanı
        self.graph = pg.PlotWidget()
        self.graph.setLabel('left', 'Gerilim (V)')
        self.graph.setLabel('bottom', 'Zaman (ms)')
        self.graph.setYRange(0, 3.3)  # MSP430 ADC aralığı
        self.graph.showGrid(x=True, y=True)
        self.graph.setBackground('#1A1A1A')
        main_layout.addWidget(self.graph)
        
        # Grafik verisi
        self.x = np.arange(self.buffer_size) * (self.timebase_value / 20)
        self.curve = self.graph.plot(self.x, self.data_buffer, pen=pg.mkPen('g', width=2))
        
        # Min ve Max gösterge çizgileri
        self.min_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('b', width=1, style=QtCore.Qt.DotLine))
        self.max_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DotLine))
        self.graph.addItem(self.min_line)
        self.graph.addItem(self.max_line)
        
        # Status bar
        self.statusBar().showMessage(f'Bağlantı: {port}, {baudrate} baud')
        
        # Timer başlat
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(20)  # 20ms ile güncelle (50Hz)
        
        # Acquisition durumu
        self.acquisition_active = True
        
        # Örnekleme için sayaç ve zaman
        self.sample_counter = 0
        self.last_update_time = time.time()
        self.last_freq_calc_time = time.time()
        self.last_value = 0
        self.zero_crossings = []
        self.crossing_times = []
        
    def adc_to_voltage(self, adc_value):
        """ADC değerini gerilime dönüştür (kalibrasyon ile)"""
        voltage = (adc_value - self.OFFSET) * self.GAIN
        return max(0, min(voltage, 5.5))  # 0V ile 5.5V arasında sınırla
    
    def update_timebase(self):
        """Zaman tabanını güncelle"""
        selected = self.timebase_combo.currentText()
        self.timebase_value = int(selected.split()[0])
        self.x = np.arange(self.buffer_size) * (self.timebase_value / 20)
        self.curve.setData(self.x, self.data_buffer)
        self.statusBar().showMessage(f'Timebase: {self.timebase_value} ms')
    
    def update_y_range(self):
        """Y eksen aralığını güncelle"""
        selected_range = self.volt_div_spin.value()
        self.graph.setYRange(0, selected_range * 4)  # 4 bölme gösterimi
    
    def toggle_acquisition(self):
        """Veri toplama durumunu değiştir"""
        self.acquisition_active = self.run_button.isChecked()
        if self.acquisition_active:
            self.run_button.setText("Durdur")
            self.timer.start()
            self.statusBar().showMessage('Veri toplama başladı')
        else:
            self.run_button.setText("Başlat")
            self.timer.stop()
            self.statusBar().showMessage('Veri toplama durduruldu')
    
    def clear_data(self):
        """Veri tamponunu temizle"""
        self.data_buffer = np.zeros(self.buffer_size)
        self.curve.setData(self.x, self.data_buffer)
        self.v_min = 0.0
        self.v_max = 0.0
        self.v_pp = 0.0
        self.frequency = 0.0
        self.update_measurements()
    
    def calculate_frequency(self, voltage):
        """Frekans hesapla (sıfır geçişleri yöntemi)"""
        now = time.time()
        trigger_level = (self.v_max + self.v_min) / 2 # Trigger seviyesi, Vmax ve Vmin ortalaması
        
        # Trigger seviyesinden geçiş tespiti (yükselen kenar)
        if self.last_value < trigger_level and voltage >= trigger_level:
            self.zero_crossings.append(now)
            
            # Son 5 sıfır geçişini sakla
            if len(self.zero_crossings) > 50:
                self.zero_crossings.pop(0)
            
            # En az 2 geçiş varsa frekans hesapla
            if len(self.zero_crossings) >= 10:
                # Ortalama periyot hesabı
                periods = [self.zero_crossings[i] - self.zero_crossings[i-1] 
                          for i in range(1, len(self.zero_crossings))]
                avg_period = sum(periods) / len(periods)
                
                # Periyottan frekans hesabı
                if avg_period > 0:
                    self.frequency = 1.0 / avg_period
                    
                    if self.v_pp < 0.1:  # 100mV altı genlikte frekansı göstermeme örneği
                        self.frequency = 0.0
                        return
                    
        # Son değeri kaydet
        self.last_value = voltage
    
    def update_measurements(self):
        """Ölçüm değerlerini güncelle"""
        # Etiketleri güncelle
        self.vmax_label.setText(f"Vmax: {self.v_max:.2f} V")
        self.vmin_label.setText(f"Vmin: {self.v_min:.2f} V")
        self.vpp_label.setText(f"Vpp: {self.v_pp:.2f} V")
        self.freq_label.setText(f"Frekans: {self.frequency:.2f} Hz")
        
        # Min/Max çizgilerini güncelle
        self.min_line.setValue(self.v_min)
        self.max_line.setValue(self.v_max)
    
    def update_data(self):
        if not self.acquisition_active:
            return

        try:
            # Seri porttan gelen veriyi satır satır biriktirmek için
            if not hasattr(self, 'line_buffer'):
                self.line_buffer = ""

            while self.ser.in_waiting > 0:
                byte = self.ser.read(1).decode('ascii', errors='ignore')
                if byte == '\n':
                    line = self.line_buffer.strip()
                    self.line_buffer = ""
                    if line.isdigit():
                        adc_val = int(line)
                        voltage = self.adc_to_voltage(adc_val)

                        self.data_buffer = np.roll(self.data_buffer, -1)
                        self.data_buffer[-1] = voltage

                        print(f"ADC: {adc_val}, Gerilim: {voltage:.2f} V")

                        self.v_min = np.min(self.data_buffer)
                        self.v_max = np.max(self.data_buffer)
                        self.v_pp = self.v_max - self.v_min

                        self.calculate_frequency(voltage)
                else:
                    self.line_buffer += byte

            self.curve.setData(self.x, self.data_buffer)
            self.update_measurements()

        except Exception as e:
            self.statusBar().showMessage(f"Hata: {e}")

    
    def closeEvent(self, event):
        """Uygulama kapatılırken seri portu kapat"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    
    # Koyu tema ayarları
    app.setStyle('Fusion')
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Base, QColor(35, 35, 35))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.BrightText, QColor(0, 128, 255))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(dark_palette)
    
    # Seri port adını belirtin
    # Windows için: 'COM3', 'COM4' vb.
    # Linux için: '/dev/ttyUSB0', '/dev/ttyACM0' vb.
    # Mac için: '/dev/tty.usbserial-0001' vb.
    PORT = '/dev/tty.usbserial-1230'  # Sisteminize göre değiştirin
    
    # Uygulamayı başlat
    osc = SimpleOscilloscopeApp(port=PORT, baudrate=115200)
    osc.show()
    sys.exit(app.exec_())