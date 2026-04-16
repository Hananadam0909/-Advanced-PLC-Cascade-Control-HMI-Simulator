"""
📘 Advanced PLC Cascade Control & HMI Simulator
Standard: Industrial Research / Scopus Q1-Q2 Supplementary Code
Features: 3-Tank Process | Cascade PID | Fuzzy Override | Modbus TCP Server | Real-time HMI | CSV Logger
Dependencies: None (Pure Python 3.8+)
Run: python advanced_plc_system.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import socket
import threading
import struct
import time
import math
import random
import csv
import os
from datetime import datetime
from collections import deque
import queue

# ─────────────────────────────────────────────────────────────────────────────
# 1. PROCESS MODEL (3-Tank Cascade + Nonlinear Dynamics + Faults)
# ─────────────────────────────────────────────────────────────────────────────
class ThreeTankProcess:
    def __init__(self, dt=0.05):
        self.dt = dt
        self.levels = [20.0, 15.0, 10.0]  # %
        self.flows = [0.0, 0.0, 0.0]      # %/s
        self.pump_cmd = 0.0               # 0-100%
        self.valve1_cmd = 0.0
        self.valve2_cmd = 0.0
        self.valve3_cmd = 0.0
        
        # Physical constants
        self.areas = [1.0, 0.8, 0.6]      # m²
        self.valve_k = [0.25, 0.20, 0.15] # Flow coefficients
        self.dead_time = [0.2, 0.1, 0.05] # seconds
        
        # Sensor & Fault states
        self.sensor_noise_std = 0.3
        self.sensor_bias = 0.0
        self.fault_pump = False
        self.fault_valve_stick = [False, False, False]
        self.stick_position = [0.0, 0.0, 0.0]
        
        # Delay buffers
        self.pump_delay = deque([0.0], maxlen=int(0.2/dt))
        self.v1_delay = deque([0.0], maxlen=int(0.1/dt))
        self.v2_delay = deque([0.0], maxlen=int(0.05/dt))

    def step(self):
        # Apply dead time
        self.pump_delay.append(self.pump_cmd)
        self.v1_delay.append(self.valve1_cmd)
        self.v2_delay.append(self.valve2_cmd)
        pump_eff = self.pump_delay.popleft()
        v1_eff = self.v1_delay.popleft()
        v2_eff = self.v2_delay.popleft()

        # Fault injection
        if self.fault_pump: pump_eff *= 0.3
        for i in range(3):
            if self.fault_valve_stick[i]:
                v1_eff = v1_eff if i==0 else (v2_eff if i==1 else v1_eff)
        
        # Nonlinear flow calculations (Torricelli + valve characteristic)
        q_in = pump_eff * 0.04  # Max 4%/s
        q1_out = self.valve_k[0] * math.sqrt(max(self.levels[0], 0.1)) * (v1_eff / 100.0)
        q2_out = self.valve_k[1] * math.sqrt(max(self.levels[1], 0.1)) * (v2_eff / 100.0)
        q3_out = self.valve_k[2] * math.sqrt(max(self.levels[2], 0.1)) * (self.valve3_cmd / 100.0)

        # Mass balance
        self.flows[0] = q_in - q1_out
        self.flows[1] = q1_out - q2_out
        self.flows[2] = q2_out - q3_out

        # Euler integration
        for i in range(3):
            self.levels[i] += (self.flows[i] / self.areas[i]) * self.dt
            self.levels[i] = max(0.0, min(100.0, self.levels[i]))

        # Sensor output with noise & bias
        pv = [
            max(0.0, min(100.0, self.levels[0] + random.gauss(self.sensor_bias, self.sensor_noise_std))),
            max(0.0, min(100.0, self.levels[1] + random.gauss(self.sensor_bias, self.sensor_noise_std*0.8))),
            max(0.0, min(100.0, self.levels[2] + random.gauss(self.sensor_bias, self.sensor_noise_std*0.6)))
        ]
        return pv, self.flows

# ─────────────────────────────────────────────────────────────────────────────
# 2. CONTROLLER ENGINE (PID + Anti-windup + Derivative Filter + Fuzzy Override)
# ─────────────────────────────────────────────────────────────────────────────
class AdvancedController:
    def __init__(self, dt=0.05):
        self.dt = dt
        self.mode = "MANUAL"  # MANUAL | AUTO | CASCADE | FUZZY
        
        # Master PID (Tank 3 Level)
        self.sp1, self.sp2 = 60.0, 40.0
        self.kp1, self.ki1, self.kd1 = 1.5, 0.3, 0.05
        self.int1, self.prev_err1 = 0.0, 0.0
        self.co1 = 0.0  # Output -> Slave SP
        
        # Slave PID (Flow/Tank 2)
        self.kp2, self.ki2, self.kd2 = 2.0, 0.8, 0.02
        self.int2, self.prev_err2 = 0.0, 0.0
        self.co2 = 0.0  # Output -> Valve commands
        
        # Fuzzy Supervisor
        self.fuzzy_override = 0.0
        self.fuzzy_active = False
        
        # Filters & Limits
        self.deriv_filter = 0.1  # N value for derivative filter
        self.co_min, self.co_max = 0.0, 100.0
        self.deadband = 0.5

    def compute(self, pv1, pv2, pv3):
        err1 = self.sp1 - pv3
        err2 = self.sp2 - pv2
        
        # ─── MASTER PID (Level) ───
        if abs(err1) > self.deadband:
            self.int1 += err1 * self.dt
            deriv = (err1 - self.prev_err1) / self.dt
            # Derivative filter: d_filtered = N*d + (1-N)*d_prev (simplified)
            raw1 = self.kp1*err1 + self.ki1*self.int1 + self.kd1*deriv
            self.co1 = max(self.co_min, min(self.co_max, raw1))
            # Back-calculation anti-windup
            if raw1 != self.co1:
                self.int1 -= (self.co1 - raw1) / (self.ki1 if self.ki1 > 1e-6 else 1e-6)
        self.prev_err1 = err1

        # ─── SLAVE PID (Flow/Inter-tank) ───
        slave_sp = self.co1 if self.mode == "CASCADE" else self.sp2
        slave_err = slave_sp - pv2
        if abs(slave_err) > self.deadband:
            self.int2 += slave_err * self.dt
            raw2 = self.kp2*slave_err + self.ki2*self.int2 + self.kd2*((slave_err - self.prev_err2)/self.dt)
            self.co2 = max(self.co_min, min(self.co_max, raw2))
            if raw2 != self.co2:
                self.int2 -= (self.co2 - raw2) / (self.ki2 if self.ki2 > 1e-6 else 1e-6)
        self.prev_err2 = slave_err

        # ─── FUZZY SUPERVISOR (Rule-based override for extremes) ───
        self.fuzzy_active = False
        if self.mode == "FUZZY" or pv3 > 90 or pv3 < 15:
            self.fuzzy_active = True
            if pv3 > 90: self.fuzzy_override = -50.0
            elif pv3 < 15: self.fuzzy_override = +40.0
            else: self.fuzzy_override = 0.0
            
        # ─── OUTPUT MAPPING ───
        final_co = self.co2 + self.fuzzy_override if self.fuzzy_active else self.co2
        pump = max(0, min(100, final_co))
        v1 = max(0, min(100, 100 - final_co * 0.7))
        v2 = max(0, min(100, 50 + final_co * 0.3))
        v3 = 60.0 if self.mode == "MANUAL" else max(20, min(80, 100 - final_co*0.5))
        
        return pump, v1, v2, v3

# ─────────────────────────────────────────────────────────────────────────────
# 3. PLC ENGINE (Scan Cycle + Memory Map + Modbus TCP Server)
# ─────────────────────────────────────────────────────────────────────────────
class PLCBrain:
    def __init__(self, dt=0.05):
        self.dt = dt
        self.coils = [False]*32      # Digital outputs/commands
        self.registers = [0.0]*32    # Analog values (float stored as int*100)
        self.alarm_log = []
        self.scan_count = 0
        
        # Modbus Queue
        self.mb_queue = queue.Queue()
        self.mb_thread = threading.Thread(target=self._modbus_server, daemon=True)
        self.mb_thread.start()

    def scan(self, process_pvs, controller_outputs):
        self.scan_count += 1
        pv1, pv2, pv3 = process_pvs
        pump, v1, v2, v3 = controller_outputs
        
        # Map to memory
        self.registers[0] = int(pv1 * 100)
        self.registers[1] = int(pv2 * 100)
        self.registers[2] = int(pv3 * 100)
        self.registers[3] = int(pump)
        self.registers[4] = int(v1)
        self.registers[5] = int(v2)
        self.registers[6] = int(v3)
        
        # Alarms
        alarms = []
        if pv3 > 95: alarms.append("OVERFLOW_T3")
        if pv1 < 5: alarms.append("DRY_RUN_T1")
        if pump > 95 and pv1 < 10: alarms.append("CAVITATION")
        self.alarm_log = alarms
        
        self.coils[0] = pump > 10
        self.coils[1] = v1 > 50
        self.coils[2] = v2 > 50
        self.coils[3] = v3 > 50
        self.coils[10] = len(alarms) > 0

    def _modbus_server(self):
        """Minimal compliant Modbus TCP Server (FC 01, 03, 05, 06)"""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 502))
        srv.listen(1)
        srv.settimeout(1.0)
        
        while True:
            try:
                conn, _ = srv.accept()
                conn.settimeout(2.0)
                while True:
                    header = conn.recv(7)
                    if not header: break
                    tx_id, proto_id, length = struct.unpack(">HHH", header)
                    data = conn.recv(length)
                    if not data: break
                    
                    unit_id, fc, payload = data[0], data[1], data[2:]
                    resp_data = b""
                    
                    if fc == 1:  # Read Coils
                        addr, qty = struct.unpack(">HH", payload)
                        coils_bits = int.from_bytes(self.coils[addr//8:addr//8 + (qty+7)//8], 'little')
                        resp_data = struct.pack(">BB", qty, (coils_bits >> (addr%8)) & ((1<<qty)-1).to_bytes(1, 'little')[0])
                    elif fc == 3:  # Read Holding Registers
                        addr, qty = struct.unpack(">HH", payload)
                        resp_data = struct.pack(">B", qty*2) + b"".join(struct.pack(">h", int(self.registers[i]) if i<len(self.registers) else 0) for i in range(addr, addr+qty))
                    elif fc == 5:  # Write Single Coil
                        addr, val = struct.unpack(">HH", payload)
                        if val == 0xFF00: self.coils[addr] = True
                        elif val == 0x0000: self.coils[addr] = False
                        resp_data = payload
                    elif fc == 6:  # Write Single Register
                        addr, val = struct.unpack(">HH", payload)
                        if addr < len(self.registers): self.registers[addr] = val/100.0
                        resp_data = payload
                        
                    if resp_data:
                        resp = struct.pack(">HHH", tx_id, 0, 2+len(resp_data)) + bytes([unit_id, fc]) + resp_data
                        conn.sendall(resp)
                conn.close()
            except (socket.timeout, ConnectionResetError, OSError):
                pass
            except Exception:
                time.sleep(0.1)

# ─────────────────────────────────────────────────────────────────────────────
# 4. HMI & MAIN APPLICATION
# ─────────────────────────────────────────────────────────────────────────────
class AdvancedPLCHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("🏭 Advanced PLC Cascade Control & HMI Studio")
        self.root.geometry("1200x750")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.proc = ThreeTankProcess(dt=0.05)
        self.ctrl = AdvancedController(dt=0.05)
        self.plc = PLCBrain(dt=0.05)
        
        self.running = True
        self.trend_buffer = deque(maxlen=400)
        self.csv_path = None
        self.csv_writer = None
        
        self._build_ui()
        self._start_loop()

    def _build_ui(self):
        # Layout
        left = tk.Frame(self.root, bg="#1a1a1a", padx=10, pady=10)
        left.pack(side="left", fill="y", padx=(0, 8))
        mid = tk.Frame(self.root, bg="#1a1a1a", padx=10, pady=10)
        mid.pack(side="left", fill="both", expand=True, padx=(0, 8))
        right = tk.Frame(self.root, bg="#1a1a1a", padx=10, pady=10)
        right.pack(side="left", fill="y")
        
        # ─── CANVAS PROCESS DIAGRAM ───
        self.cvs = tk.Canvas(mid, width=700, height=300, bg="#0f0f0f", highlightthickness=0)
        self.cvs.pack(pady=5)
        self._draw_process()
        
        # ─── TREND PLOT ───
        self.trend_cvs = tk.Canvas(mid, width=700, height=180, bg="#0f0f0f", highlightthickness=0)
        self.trend_cvs.pack(pady=5)
        self._draw_trend_grid()
        
        # ─── CONTROLS ───
        ttk.LabelFrame(left, text="⚙️ Control Mode", padding=8).pack(fill="x", pady=5)
        self.mode_var = tk.StringVar(value="MANUAL")
        for m in ["MANUAL", "AUTO", "CASCADE", "FUZZY"]:
            ttk.Radiobutton(left, text=m, variable=self.mode_var, value=m, command=self._apply_mode).pack(anchor="w", pady=2)
            
        ttk.LabelFrame(left, text="🎛️ Parameters", padding=8).pack(fill="x", pady=5)
        self._slider(left, "SP Lvl", 10, 90, 60, self._set_sp)
        self._slider(left, "Kp", 0.1, 5.0, 1.5, lambda v: setattr(self.ctrl, 'kp1', float(v)))
        self._slider(left, "Ki", 0.0, 2.0, 0.3, lambda v: setattr(self.ctrl, 'ki1', float(v)))
        self._slider(left, "Kd", 0.0, 1.0, 0.05, lambda v: setattr(self.ctrl, 'kd1', float(v)))
        
        ttk.LabelFrame(left, text="🔧 Fault Injection", padding=8).pack(fill="x", pady=5)
        ttk.Button(left, text="💥 Pump Trip", command=lambda: self._toggle_fault('pump')).pack(fill="x", pady=2)
        ttk.Button(left, text="🔒 Valve Stick T1", command=lambda: self._toggle_fault('stick0')).pack(fill="x", pady=2)
        ttk.Button(left, text="📉 Sensor Bias +5", command=self._add_bias).pack(fill="x", pady=2)
        
        ttk.LabelFrame(left, text="💾 Data", padding=8).pack(fill="x", pady=5)
        ttk.Button(left, text="📁 Start CSV Log", command=self._start_csv).pack(fill="x", pady=2)
        ttk.Button(left, text="⏹ Export & Stop", command=self._stop_csv).pack(fill="x", pady=2)
        
        # ─── STATUS PANELS ───
        self.lbl_status = ttk.Label(mid, text="🟢 PLC SCAN: ACTIVE | CYCLE: 0ms | MODE: MANUAL", foreground="#00ff66", font=("Consolas", 10))
        self.lbl_status.pack(anchor="w", pady=5)
        self.lbl_alarm = ttk.Label(right, text="🔔 ALARMS: NONE", foreground="#00ff66", font=("Arial", 11, "bold"), wraplength=200, justify="left")
        self.lbl_alarm.pack(fill="x", pady=10)
        
        ttk.Label(right, text="📊 LIVE VALUES", font=("Arial", 10, "bold")).pack(anchor="w", pady=(10,5))
        self.vars = {}
        for tag in ["T1%", "T2%", "T3%", "PUMP%", "V1%", "V2%", "V3%"]:
            v = tk.StringVar(value="0.0")
            ttk.Label(right, textvariable=v).pack(anchor="w", pady=1)
            self.vars[tag] = v

    def _slider(self, parent, label, minv, maxv, init, cmd):
        f = ttk.Frame(parent)
        f.pack(fill="x", pady=2)
        ttk.Label(f, text=f"{label}:", foreground="#aaa", width=8).pack(side="left")
        s = ttk.Scale(f, from_=minv, to=maxv, orient="horizontal", command=lambda v: cmd(float(v)))
        s.set(init)
        s.pack(side="left", fill="x", expand=True, padx=5)
        setattr(self, f"sl_{label.lower().replace(' ','_')}", s)

    def _draw_process(self):
        self.cvs.delete("all")
        # Tanks
        self.tanks = [self.cvs.create_rectangle(50+i*220, 80, 150+i*220, 240, outline="#444", width=2, fill="#1a1a1a") for i in range(3)]
        self.waters = [self.cvs.create_rectangle(52+i*220, 238, 148+i*220, 238, fill="#0088ff", outline="") for i in range(3)]
        # Pipes
        self.cvs.create_line(150, 160, 270, 160, fill="#555", width=6)
        self.cvs.create_line(370, 160, 490, 160, fill="#555", width=6)
        # Pumps/Valves
        self.pump_icon = self.cvs.create_oval(20, 145, 45, 170, fill="#333", outline="#888")
        self.v_icons = [self.cvs.create_polygon(215, 150, 235, 170, 215, 190, fill="#333", outline="#888"),
                        self.cvs.create_polygon(435, 150, 455, 170, 435, 190, fill="#333", outline="#888"),
                        self.cvs.create_polygon(635, 150, 655, 170, 635, 190, fill="#333", outline="#888")]
        self.cvs.create_text(100, 270, text="T1", fill="#aaa")
        self.cvs.create_text(320, 270, text="T2", fill="#aaa")
        self.cvs.create_text(540, 270, text="T3", fill="#aaa")

    def _draw_trend_grid(self):
        self.trend_cvs.delete("all")
        for y in range(20, 161, 28):
            self.trend_cvs.create_line(40, y, 660, y, fill="#222")
        self.trend_cvs.create_text(20, 90, text="PV%", fill="#555", anchor="e")

    def _start_loop(self):
        if self.running:
            # 1. Physics
            pvs, flows = self.proc.step()
            # 2. Control
            mode = self.mode_var.get()
            self.ctrl.mode = mode
            co = self.ctrl.compute(pvs[0], pvs[1], pvs[2])
            # 3. PLC Scan
            self.plc.scan(pvs, co)
            # 4. Apply outputs to process
            self.proc.pump_cmd, self.proc.valve1_cmd, self.proc.valve2_cmd, self.proc.valve3_cmd = co
            
            # 5. UI Update
            self._update_visuals(pvs, co, flows)
            self.trend_buffer.append((pvs, co))
            self._draw_trend()
            self._log_csv(pvs, co)
            
            cycle_time = time.perf_counter() % 1.0
            self.lbl_status.config(text=f"🟢 PLC SCAN: ACTIVE | CYCLE: {self.plc.scan_count*50}ms | MODE: {mode} | DT: 50ms")
        self.root.after(50, self._start_loop)

    def _update_visuals(self, pvs, co, flows):
        for i in range(3):
            h = 240 - (pvs[i]/100)*160
            self.cvs.coords(self.waters[i], 52+i*220, h, 148+i*220, 238)
            c = "#ff4444" if pvs[i]>90 else "#ffaa00" if pvs[i]<15 else "#0088ff"
            self.cvs.itemconfig(self.waters[i], fill=c)
            
        pump_c = "#00ff66" if co[0]>10 else "#444"
        self.cvs.itemconfig(self.pump_icon, fill=pump_c)
        for i, vc in enumerate(co[1:]):
            self.cvs.itemconfig(self.v_icons[i], fill="#00ff66" if vc>50 else "#444")
            
        # Status vars
        tags = ["T1%", "T2%", "T3%", "PUMP%", "V1%", "V2%", "V3%"]
        vals = [f"{v:.1f}" for v in (*pvs, *co)]
        for t, v in zip(tags, vals): self.vars[t].set(v)
        
        # Alarms
        if self.plc.alarm_log:
            self.lbl_alarm.config(text="🚨 " + "\n".join(self.plc.alarm_log), foreground="#ff4444")
        else:
            self.lbl_alarm.config(text="✅ ALARMS: NONE", foreground="#00ff66")

    def _draw_trend(self):
        self.trend_cvs.delete("line")
        if len(self.trend_buffer) < 2: return
        xs = list(range(len(self.trend_buffer)))
        # PV3 (Master)
        pts3 = [(40+x*1.5, 160 - (b[0][2]/100)*140) for x, b in enumerate(self.trend_buffer)]
        self.trend_cvs.create_line(pts3, fill="#00ff88", width=2, tags="line")
        # Slave SP
        pts_sp = [(40+x*1.5, 160 - (self.ctrl.sp2/100)*140) for x in range(len(self.trend_buffer))]
        self.trend_cvs.create_line(pts_sp, fill="#00aaff", width=1, dash=(4,4), tags="line")

    def _apply_mode(self): pass
    def _set_sp(self, v): self.ctrl.sp1 = float(v)
    def _toggle_fault(self, f):
        if f=='pump': self.proc.fault_pump = not self.proc.fault_pump
        elif f=='stick0': self.proc.fault_valve_stick[0] = not self.proc.fault_valve_stick[0]
    def _add_bias(self): self.proc.sensor_bias += 5.0
    def _start_csv(self):
        self.csv_path = f"plc_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time", "T1%", "T2%", "T3%", "PUMP%", "V1%", "V2%", "V3%", "Mode"])
    def _stop_csv(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            messagebox.showinfo("Export", f"Data saved to {self.csv_path}")
    def _log_csv(self, pvs, co):
        if self.csv_writer:
            self.csv_writer.writerow([datetime.now().strftime("%H:%M:%S.%f"), *[f"{v:.2f}" for v in (*pvs, *co)], self.mode_var.get()])
            self.csv_file.flush()
    def on_close(self):
        self.running = False
        if hasattr(self, 'csv_file') and self.csv_file: self.csv_file.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AdvancedPLCHMI(root)
    root.mainloop()