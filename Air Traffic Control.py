import tkinter as tk
from tkinter import messagebox
import math
import uuid
import time
from typing import List, Tuple

# --- 1. AIRCRAFT CLASS DEFINITION ---
class Aircraft:
    """Defines the state and linear motion mechanics for an aircraft."""
    
    HORIZONTAL_MIN = 5.0  # Nautical Miles (NM)
    VERTICAL_MIN = 1000   # Feet (ft)
    NM_TO_FT = 6076.12

    def __init__(self, callsign: str, x: float, y: float, z: float, speed_knots: float, heading_deg: float):
        self.id = uuid.uuid4()
        self.callsign = callsign
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed_knots  # Knots (NM/hr)
        self.heading = heading_deg % 360
        self.status = "CRUISING"
        self.conflict_alert = False
        self.gui_object = None

    def update_position(self, delta_t_hrs: float):
        heading_rad = math.radians(self.heading)
        distance = self.speed * delta_t_hrs
        self.x += distance * math.sin(heading_rad)
        self.y += distance * math.cos(heading_rad)

    def get_projected_position(self, delta_t_hrs: float) -> Tuple[float, float, float]:
        heading_rad = math.radians(self.heading)
        distance = self.speed * delta_t_hrs
        proj_x = self.x + distance * math.sin(heading_rad)
        proj_y = self.y + distance * math.cos(heading_rad)
        return proj_x, proj_y, self.z

    def __str__(self):
        return (f"Aircraft {self.callsign} | Pos: ({self.x:.1f}NM, {self.y:.1f}NM, {self.z:.0f}ft) | "
                f"Speed: {self.speed:.0f}kts | Heading: {self.heading:.0f}° | Alert: {self.conflict_alert}")

# --- 2. ATC CONTROLLER CLASS ---
class ATC_Controller:
    """Manages all aircraft and implements conflict detection."""
    def __init__(self, lookahead_time_min: int = 10):
        self.aircraft_list: List[Aircraft] = []
        self.lookahead_time_min = lookahead_time_min
        self.lookahead_time_hrs = lookahead_time_min / 60.0

    def add_aircraft(self, aircraft: Aircraft):
        self.aircraft_list.append(aircraft)

    def check_for_conflicts(self) -> List[Tuple]:
        conflicts = []
        n = len(self.aircraft_list)
        for i in range(n):
            for j in range(i + 1, n):
                a = self.aircraft_list[i]
                b = self.aircraft_list[j]
                vertical_sep = abs(a.z - b.z)
                if vertical_sep >= Aircraft.VERTICAL_MIN:
                    continue
                vax = a.speed * math.sin(math.radians(a.heading))
                vay = a.speed * math.cos(math.radians(a.heading))
                vbx = b.speed * math.sin(math.radians(b.heading))
                vby = b.speed * math.cos(math.radians(b.heading))
                vrx = vbx - vax
                vry = vby - vay
                rx = b.x - a.x
                ry = b.y - a.y
                v_rel_sq = vrx**2 + vry**2
                if v_rel_sq < 0.001:
                    dist_current = math.sqrt(rx**2 + ry**2)
                    if dist_current < Aircraft.HORIZONTAL_MIN:
                        conflicts.append((a.callsign, b.callsign, "Immediate", dist_current, vertical_sep))
                        a.conflict_alert = b.conflict_alert = True
                    continue
                tcpa = - (rx * vrx + ry * vry) / v_rel_sq
                if 0 <= tcpa <= self.lookahead_time_hrs:
                    min_sep_x = rx + vrx * tcpa
                    min_sep_y = ry + vry * tcpa
                    min_sep_dist = math.sqrt(min_sep_x**2 + min_sep_y**2)
                    if min_sep_dist < Aircraft.HORIZONTAL_MIN:
                        time_to_conflict = tcpa * 60
                        conflicts.append((a.callsign, b.callsign, f"In {time_to_conflict:.1f} min", min_sep_dist, vertical_sep))
                        a.conflict_alert = True
                        b.conflict_alert = True
        return conflicts

# --- 3. TKINTER GUI APPLICATION ---
class ATCSimulatorGUI(tk.Tk):
    def __init__(self, controller: ATC_Controller, sim_speed_multiplier: float = 1.0):
        super().__init__()
        self.title("Air Traffic Control Simulator")
        self.geometry("1200x800")
        self.controller = controller
        self.running = False
        self.current_time_seconds = 0
        self.sim_speed_multiplier = sim_speed_multiplier
        self.animation_delay_ms = 100
        self.create_widgets()
        self.update_idletasks()
        self.setup_canvas_scaling()
        self.draw_aircraft()
        self.update_info_panel()

    def create_widgets(self):
        self.toolbar_frame = tk.Frame(self, bd=2, relief="raised")
        self.toolbar_frame.pack(side="top", fill="x", padx=5, pady=5)

        self.main_frame = tk.Frame(self)
        self.main_frame.pack(side="top", fill="both", expand=True)

        self.canvas_frame = tk.Frame(self.main_frame, bd=2, relief="sunken")
        self.canvas_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        self.info_panel_frame = tk.Frame(self.main_frame, width=300, bd=2, relief="groove")
        self.info_panel_frame.pack(side="right", fill="y", padx=5, pady=5)
        self.info_panel_frame.pack_propagate(False)

        self.start_button = tk.Button(self.toolbar_frame, text="Start Simulation", command=self.start_sim, bg="green", fg="white")
        self.start_button.pack(side="left", padx=5, pady=2)

        self.stop_button = tk.Button(self.toolbar_frame, text="Stop Simulation", command=self.stop_sim, bg="red", fg="white", state="disabled")
        self.stop_button.pack(side="left", padx=5, pady=2)

        # 🔹 Restart Button
        self.restart_button = tk.Button(self.toolbar_frame, text="Restart Simulation", command=self.restart_sim, bg="orange", fg="white")
        self.restart_button.pack(side="left", padx=5, pady=2)

        self.add_plane_button = tk.Button(self.toolbar_frame, text="Add New Plane", command=self.open_add_plane_dialog)
        self.add_plane_button.pack(side="left", padx=5, pady=2)

        tk.Label(self.toolbar_frame, text="Sim Time:").pack(side="left", padx=(20, 0))
        self.time_label = tk.Label(self.toolbar_frame, text="00:00:00", font=("Arial", 12, "bold"))
        self.time_label.pack(side="left", padx=5)

        self.canvas = tk.Canvas(self.canvas_frame, bg="darkblue", borderwidth=0, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        tk.Label(self.info_panel_frame, text="Aircraft Information", font=("Arial", 14, "bold")).pack(pady=10)
        self.aircraft_info_text = tk.Text(self.info_panel_frame, wrap="word", height=15, width=35, bg="#f0f0f0", bd=0)
        self.aircraft_info_text.pack(padx=5, pady=5, fill="both", expand=True)
        self.aircraft_info_text.config(state="disabled")

        tk.Label(self.info_panel_frame, text="Conflict Alerts", font=("Arial", 14, "bold")).pack(pady=10)
        self.conflict_info_text = tk.Text(self.info_panel_frame, wrap="word", height=10, width=35, bg="#fff0f0", bd=0)
        self.conflict_info_text.pack(padx=5, pady=5, fill="both", expand=True)
        self.conflict_info_text.config(state="disabled")

    def setup_canvas_scaling(self):
        self.airspace_x_min = -100.0
        self.airspace_x_max = 100.0
        self.airspace_y_min = -100.0
        self.airspace_y_max = 100.0
        self.canvas_width = self.canvas.winfo_width()
        self.canvas_height = self.canvas.winfo_height()
        airspace_range_x = self.airspace_x_max - self.airspace_x_min
        airspace_range_y = self.airspace_y_max - self.airspace_y_min
        self.scale_factor = min(self.canvas_width / airspace_range_x, self.canvas_height / airspace_range_y)
        scaled_airspace_width = airspace_range_x * self.scale_factor
        scaled_airspace_height = airspace_range_y * self.scale_factor
        self.offset_x = (self.canvas_width - scaled_airspace_width) / 2
        self.offset_y = (self.canvas_height - scaled_airspace_height) / 2

    def world_to_canvas(self, x_world: float, y_world: float) -> Tuple[float, float]:
        x_scaled = (x_world - self.airspace_x_min) * self.scale_factor
        y_scaled = (y_world - self.airspace_y_min) * self.scale_factor
        canvas_x = x_scaled + self.offset_x
        canvas_y = self.canvas_height - y_scaled - self.offset_y
        return canvas_x, canvas_y

    def on_canvas_resize(self, event):
        self.setup_canvas_scaling()
        self.draw_aircraft()

    def draw_aircraft(self):
        self.canvas.delete("all")
        for ac in self.controller.aircraft_list:
            canvas_x, canvas_y = self.world_to_canvas(ac.x, ac.y)
            color = "lightblue" if not ac.conflict_alert else "red"
            self.canvas.create_oval(canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5, fill=color, outline="white")
            self.canvas.create_text(canvas_x, canvas_y - 10, text=ac.callsign, fill="white")

    def update_info_panel(self, conflicts: List[Tuple] = None):
        self.aircraft_info_text.config(state="normal")
        self.aircraft_info_text.delete("1.0", tk.END)
        for ac in self.controller.aircraft_list:
            self.aircraft_info_text.insert(tk.END, str(ac) + "\n")
        self.aircraft_info_text.config(state="disabled")
        self.conflict_info_text.config(state="normal")
        self.conflict_info_text.delete("1.0", tk.END)
        if conflicts:
            for ca, cb, c_time, h_dist, v_dist in conflicts:
                self.conflict_info_text.insert(tk.END, f"🛑 {ca} vs {cb} — {c_time}\nH: {h_dist:.2f} NM | V: {v_dist:.0f} ft\n\n", "conflict")
            self.conflict_info_text.tag_config("conflict", foreground="red")
        else:
            self.conflict_info_text.insert(tk.END, "No active conflicts.\n", "safe")
            self.conflict_info_text.tag_config("safe", foreground="green")
        self.conflict_info_text.config(state="disabled")

    def update_time_label(self):
        h = int(self.current_time_seconds // 3600)
        m = int((self.current_time_seconds % 3600) // 60)
        s = int(self.current_time_seconds % 60)
        self.time_label.config(text=f"{h:02d}:{m:02d}:{s:02d}")

    def sim_step(self):
        if not self.running:
            return
        TIME_STEP_SECONDS = 5
        TIME_STEP_HOURS = TIME_STEP_SECONDS / 3600.0
        for ac in self.controller.aircraft_list:
            ac.conflict_alert = False
        conflicts = self.controller.check_for_conflicts()
        for ac in self.controller.aircraft_list:
            ac.update_position(TIME_STEP_HOURS)
        self.update_info_panel(conflicts)
        self.draw_aircraft()
        self.current_time_seconds += TIME_STEP_SECONDS
        self.update_time_label()
        self.after(int(self.animation_delay_ms / self.sim_speed_multiplier), self.sim_step)

    def start_sim(self):
        if not self.running:
            self.running = True
            self.start_button.config(state="disabled", bg="gray")
            self.stop_button.config(state="normal", bg="red")
            self.restart_button.config(state="disabled", bg="gray")
            self.sim_step()

    def stop_sim(self):
        if self.running:
            self.running = False
            self.start_button.config(state="normal", bg="green")
            self.stop_button.config(state="disabled", bg="gray")
            self.restart_button.config(state="normal", bg="orange")

    def restart_sim(self):
        """Restart the simulation with default aircraft setup."""
        self.stop_sim()
        self.controller.aircraft_list.clear()
        self.current_time_seconds = 0
        plane_a = Aircraft("AAL123", -20.0, 0.0, 30000, 450, 90)
        plane_b = Aircraft("UAL456", 20.0, 0.0, 30000, 450, 270)
        plane_c = Aircraft("SWA789", -10.0, -20.0, 35000, 500, 45)
        plane_d = Aircraft("DAL001", 10.0, -20.0, 34000, 500, 135)
        self.controller.add_aircraft(plane_a)
        self.controller.add_aircraft(plane_b)
        self.controller.add_aircraft(plane_c)
        self.controller.add_aircraft(plane_d)
        self.time_label.config(text="00:00:00")
        self.update_info_panel()
        self.draw_aircraft()
        messagebox.showinfo("Restart", "Simulation restarted successfully!")

    def open_add_plane_dialog(self):
        dialog = AddPlaneDialog(self, self.controller)
        self.wait_window(dialog)
        self.draw_aircraft()
        self.update_info_panel()

# --- 4. Add Plane Dialog ---
class AddPlaneDialog(tk.Toplevel):
    def __init__(self, parent, controller: ATC_Controller):
        super().__init__(parent)
        self.parent = parent
        self.controller = controller
        self.title("Add New Aircraft")
        self.geometry("300x350")
        self.transient(parent)
        self.grab_set()
        self.create_widgets()

    def create_widgets(self):
        labels = ["Callsign:", "Start X (NM):", "Start Y (NM):", "Altitude (ft):", "Speed (kts):", "Heading (deg):"]
        self.entries = {}
        defaults = {"Callsign:": "NEW123", "Start X (NM):": "0", "Start Y (NM):": "0", "Altitude (ft):": "33000", "Speed (kts):": "450", "Heading (deg):": "0"}
        for i, text in enumerate(labels):
            tk.Label(self, text=text).grid(row=i, column=0, padx=5, pady=2, sticky="w")
            entry = tk.Entry(self)
            entry.insert(0, defaults.get(text, ""))
            entry.grid(row=i, column=1, padx=5, pady=2, sticky="ew")
            self.entries[text] = entry
        tk.Button(self, text="Add Aircraft", command=self.add_aircraft).grid(row=len(labels), column=0, columnspan=2, pady=10)
        tk.Button(self, text="Cancel", command=self.destroy).grid(row=len(labels)+1, column=0, columnspan=2, pady=5)

    def add_aircraft(self):
        try:
            callsign = self.entries["Callsign:"].get().upper()
            x = float(self.entries["Start X (NM):"].get())
            y = float(self.entries["Start Y (NM):"].get())
            z = float(self.entries["Altitude (ft):"].get())
            speed = float(self.entries["Speed (kts):"].get())
            heading = float(self.entries["Heading (deg):"].get())
            if any(ac.callsign == callsign for ac in self.controller.aircraft_list):
                messagebox.showerror("Input Error", f"Callsign {callsign} already exists.")
                return
            new_plane = Aircraft(callsign, x, y, z, speed, heading)
            self.controller.add_aircraft(new_plane)
            messagebox.showinfo("Success", f"Aircraft {callsign} added successfully!")
            self.destroy()
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers.")

# --- 5. MAIN EXECUTION ---
if __name__ == "__main__":
    atc_controller = ATC_Controller(lookahead_time_min=5)
    plane_a = Aircraft("AAL123", -20.0, 0.0, 30000, 450, 90)
    plane_b = Aircraft("UAL456", 20.0, 0.0, 30000, 450, 270)
    plane_c = Aircraft("SWA789", -10.0, -20.0, 35000, 500, 45)
    plane_d = Aircraft("DAL001", 10.0, -20.0, 34000, 500, 135)
    atc_controller.add_aircraft(plane_a)
    atc_controller.add_aircraft(plane_b)
    atc_controller.add_aircraft(plane_c)
    atc_controller.add_aircraft(plane_d)
    app = ATCSimulatorGUI(atc_controller, sim_speed_multiplier=2.0)
    app.mainloop()