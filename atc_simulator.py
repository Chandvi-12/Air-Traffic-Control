import tkinter as tk
from tkinter import messagebox
import math
import uuid
import time
from typing import List, Tuple, Dict, Optional
import heapq # For Priority Queue in A*
from PIL import Image, ImageTk # Import PIL for image handling (install: pip install Pillow)

# --- 1. AIRCRAFT CLASS DEFINITION (Unchanged) ---
class Aircraft:
    """Defines the state and linear motion mechanics for an aircraft."""
    
    HORIZONTAL_MIN = 5.0    # Nautical Miles (NM)
    VERTICAL_MIN = 1000     # Feet (ft)
    NM_TO_FT = 6076.12

    def __init__(self, callsign: str, x: float, y: float, z: float, speed_knots: float, heading_deg: float):
        self.id = uuid.uuid4()
        self.callsign = callsign
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed_knots    # Knots (NM/hr)
        self.heading = heading_deg % 360
        self.status = "CRUISING"
        self.conflict_alert = False
        self.gui_object = None
        self.image_object_id = None # To store canvas image ID

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
    
    def apply_maneuver(self, maneuver: 'Maneuver'):
        self.heading = (self.heading + maneuver.heading_change) % 360
        self.z = max(5000, self.z + maneuver.altitude_change) 
        self.speed = max(200, self.speed + maneuver.speed_change) 

    def __str__(self):
        return (f"Aircraft {self.callsign} | Pos: ({self.x:.1f}NM, {self.y:.1f}NM, {self.z:.0f}ft) | "
                f"Speed: {self.speed:.0f}kts | Heading: {self.heading:.0f}° | Alert: {self.conflict_alert}")

# --- AI SEARCH STRUCTURES (Unchanged) ---
class Maneuver:
    def __init__(self, callsign: str, heading_change: float = 0, altitude_change: float = 0, speed_change: float = 0):
        self.callsign = callsign
        self.heading_change = heading_change
        self.altitude_change = altitude_change
        self.speed_change = speed_change

    def cost(self) -> float:
        return abs(self.heading_change) * 0.1 + abs(self.altitude_change / 1000) * 0.5 + abs(self.speed_change / 50) * 0.2
    
    def __repr__(self):
        return f"Maneuver(H:{self.heading_change:+.0f}°, Z:{self.altitude_change:+.0f}ft, S:{self.speed_change:+.0f}kts)"


class ConflictState:
    def __init__(self, aircraft_states: Dict[str, Tuple[float, float, float, float, float]], g_cost: float, parent: Optional['ConflictState'] = None, maneuver: Optional[Maneuver] = None):
        self.aircraft_states = aircraft_states
        self.g_cost = g_cost
        self.h_cost = 0.0
        self.f_cost = 0.0
        self.parent = parent
        self.maneuver = maneuver
        self._hash = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __hash__(self):
        if self._hash is None:
            state_tuple = tuple(
                (round(x, 0), round(y, 0), round(z, 0), round(s, 0), round(h, 1))
                for x, y, z, s, h in self.aircraft_states.values()
            )
            self._hash = hash(state_tuple)
        return self._hash
    
    def __eq__(self, other):
        return self.aircraft_states == other.aircraft_states

    def get_aircraft_data(self, callsign: str) -> Tuple[float, float, float, float, float]:
        return self.aircraft_states.get(callsign, (0, 0, 0, 0, 0))


class ConflictResolver:
    def __init__(self, controller: 'ATC_Controller', conflict_pair: Tuple[Aircraft, Aircraft], lookahead_hrs: float):
        self.controller = controller
        self.A, self.B = conflict_pair
        self.lookahead_hrs = lookahead_hrs
        self.MANEUVER_OPTIONS = [
            Maneuver(self.A.callsign, heading_change=d) for d in [-10, 10, -5, 5] 
        ] + [
            Maneuver(self.B.callsign, heading_change=d) for d in [-10, 10, -5, 5]
        ] + [
            Maneuver(self.A.callsign, altitude_change=d) for d in [-1000, 1000]
        ] + [
            Maneuver(self.B.callsign, altitude_change=d) for d in [-1000, 1000]
        ]

    def _calculate_cpa(self, state: ConflictState) -> Tuple[float, float, float]:
        ax, ay, az, aspd, ahead = state.get_aircraft_data(self.A.callsign)
        bx, by, bz, bspd, bhead = state.get_aircraft_data(self.B.callsign)

        vertical_sep = abs(az - bz)
        if vertical_sep >= Aircraft.VERTICAL_MIN:
            return float('inf'), vertical_sep, 0.0

        vax = aspd * math.sin(math.radians(ahead))
        vay = aspd * math.cos(math.radians(ahead))
        vbx = bspd * math.sin(math.radians(bhead))
        vby = bspd * math.cos(math.radians(bhead))
        
        vrx = vbx - vax
        vry = vby - vay
        rx = bx - ax
        ry = by - ay
        v_rel_sq = vrx**2 + vry**2

        if v_rel_sq < 0.001: 
            dist_current = math.sqrt(rx**2 + ry**2)
            tcpa = self.lookahead_hrs / 2 
            return dist_current, vertical_sep, tcpa
            
        tcpa = - (rx * vrx + ry * vry) / v_rel_sq
        
        if tcpa < 0:
             min_sep_dist = math.sqrt(rx**2 + ry**2)
        else:
            min_sep_x = rx + vrx * tcpa
            min_sep_y = ry + vry * tcpa
            min_sep_dist = math.sqrt(min_sep_x**2 + min_sep_y**2)
        
        if tcpa > self.lookahead_hrs:
            end_sep_x = rx + vrx * self.lookahead_hrs
            end_sep_y = ry + vry * self.lookahead_hrs
            end_sep_dist = math.sqrt(end_sep_x**2 + end_sep_y**2)
            min_sep_dist = min(min_sep_dist, end_sep_dist)
            
        return min_sep_dist, vertical_sep, max(0.0, tcpa)

    def _heuristic(self, state: ConflictState) -> float:
        h_dist, h_v_sep, _ = self._calculate_cpa(state)
        
        H_MIN = Aircraft.HORIZONTAL_MIN
        V_MIN = Aircraft.VERTICAL_MIN

        if h_dist >= H_MIN and h_v_sep >= V_MIN:
            return 0.0

        horizontal_penalty = max(0, H_MIN - h_dist) / H_MIN
        vertical_penalty = max(0, V_MIN - h_v_sep) / V_MIN

        return (horizontal_penalty + vertical_penalty) * 5.0 

    def a_star_search(self) -> Optional[List[Maneuver]]:
        initial_aircraft_data = {
            self.A.callsign: (self.A.x, self.A.y, self.A.z, self.A.speed, self.A.heading),
            self.B.callsign: (self.B.x, self.B.y, self.B.z, self.B.speed, self.B.heading),
        }
        initial_state = ConflictState(initial_aircraft_data, g_cost=0.0)
        initial_state.h_cost = self._heuristic(initial_state)
        initial_state.f_cost = initial_state.g_cost + initial_state.h_cost

        open_list = [initial_state]
        g_scores: Dict[int, float] = {hash(initial_state): 0.0}
        
        max_search_nodes = 1000 
        nodes_explored = 0

        while open_list and nodes_explored < max_search_nodes:
            nodes_explored += 1
            current_state = heapq.heappop(open_list)
            
            if current_state.h_cost == 0.0:
                path = []
                while current_state.parent is not None:
                    path.append(current_state.maneuver)
                    current_state = current_state.parent
                return path[::-1]

            for maneuver in self.MANEUVER_OPTIONS:
                new_aircraft_states = current_state.aircraft_states.copy()
                callsign = maneuver.callsign
                
                x, y, z, speed, heading = new_aircraft_states[callsign]
                temp_ac = Aircraft(callsign, x, y, z, speed, heading)
                
                temp_ac.apply_maneuver(maneuver)
                
                new_aircraft_states[callsign] = (temp_ac.x, temp_ac.y, temp_ac.z, temp_ac.speed, temp_ac.heading)

                successor = ConflictState(
                    aircraft_states=new_aircraft_states,
                    g_cost=current_state.g_cost + maneuver.cost(),
                    parent=current_state,
                    maneuver=maneuver
                )
                successor.h_cost = self._heuristic(successor)
                successor.f_cost = successor.g_cost + successor.h_cost
                
                state_hash = hash(successor)
                if state_hash in g_scores and successor.g_cost >= g_scores[state_hash]:
                    continue

                g_scores[state_hash] = successor.g_cost
                heapq.heappush(open_list, successor)

        return None

# ----------------------------------------------------
# --- 2. ATC CONTROLLER CLASS (CORRECTION APPLIED) ---
# ----------------------------------------------------
class ATC_Controller:
    def __init__(self, lookahead_time_min: int = 10):
        self.aircraft_list: List[Aircraft] = []
        
        # 🟢 FIX: Store argument in self.variable
        self.lookahead_time_min = lookahead_time_min
        
        # 🟢 FIX: Use self.variable for calculation to avoid NameError
        self.lookahead_time_hrs = self.lookahead_time_min / 60.0 
        
        self.resolved_maneuvers: Dict[str, List[Maneuver]] = {} 

    def add_aircraft(self, aircraft: Aircraft):
        self.aircraft_list.append(aircraft)

    def check_for_conflicts(self) -> List[Tuple]:
        conflicts = []
        
        for ac in self.aircraft_list:
             ac.conflict_alert = False
        self.resolved_maneuvers.clear() 

        n = len(self.aircraft_list)
        for i in range(n):
            for j in range(i + 1, n):
                a = self.aircraft_list[i]
                b = self.aircraft_list[j]
                
                if a.callsign in self.resolved_maneuvers or b.callsign in self.resolved_maneuvers:
                    continue

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
                        a.conflict_alert = b.conflict_alert = True
                        conflicts.append((a.callsign, b.callsign, "Immediate", dist_current, vertical_sep, "NO_RES_ATTEMPTED"))
                    continue
                
                tcpa = - (rx * vrx + ry * vry) / v_rel_sq
                
                if 0 <= tcpa <= self.lookahead_time_hrs:
                    min_sep_x = rx + vrx * tcpa
                    min_sep_y = ry + vry * tcpa
                    min_sep_dist = math.sqrt(min_sep_x**2 + min_sep_y**2)
                    
                    if min_sep_dist < Aircraft.HORIZONTAL_MIN:
                        resolver = ConflictResolver(self, (a, b), self.lookahead_time_hrs)
                        resolution_path = resolver.a_star_search()
                        
                        resolution_status = "❌ FAILED"
                        if resolution_path:
                            for maneuver in resolution_path:
                                self.resolved_maneuvers.setdefault(maneuver.callsign, []).append(maneuver)
                            resolution_status = f"✅ RESOLVED ({len(resolution_path)} steps)"
                        else:
                            a.conflict_alert = b.conflict_alert = True

                        time_to_conflict = tcpa * 60
                        conflicts.append((a.callsign, b.callsign, f"In {time_to_conflict:.1f} min", min_sep_dist, vertical_sep, resolution_status))
                        
        return conflicts

    def apply_resolved_maneuvers(self):
        for ac in self.aircraft_list:
            if ac.callsign in self.resolved_maneuvers:
                maneuver_list = self.resolved_maneuvers[ac.callsign]
                if maneuver_list:
                    maneuver = maneuver_list.pop(0)
                    ac.apply_maneuver(maneuver)
                    
                    if not maneuver_list:
                        del self.resolved_maneuvers[ac.callsign]

# ----------------------------------------------------------
# --- 3. TKINTER GUI APPLICATION (File Names Corrected) ---
# ----------------------------------------------------------
class ATCSimulatorGUI(tk.Tk):
    def __init__(self, controller: ATC_Controller, sim_speed_multiplier: float = 1.0):
        super().__init__()
        self.title("Air Traffic Control Simulator (A* Resolution) - Image Backgrounds")
        self.geometry("1200x800")
        self.controller = controller
        self.running = False
        self.current_time_seconds = 0
        self.sim_speed_multiplier = sim_speed_multiplier
        self.animation_delay_ms = 100
        
        # 🌟 Image references
        self.sky_image_tk = None
        self.aircraft_image_tk = None
        self.aircraft_image_original = None # Keep reference to original PIL Image for rotation
        self.rotated_aircraft_images = {} # Cache for rotated images

        self.create_widgets()
        self.update_idletasks()
        
        # Load images after canvas is created
        self._load_images() 
        
        self.on_canvas_resize(None) # Call after images loaded
        
        self.draw_aircraft()
        self.update_info_panel()

    def _load_images(self):
        try:
            # 🟢 CORRECTION: Use your file name 'sky_bg.png'
            self.sky_image_original = Image.open("sky_bg.png") 
            
            # 🟢 CORRECTION: Use your file name 'plane.png'
            self.aircraft_image_original = Image.open("plane.png").resize((20, 20), Image.LANCZOS)
            
            self.aircraft_image_tk = ImageTk.PhotoImage(self.aircraft_image_original)

        except FileNotFoundError as e:
            # Updated error message to match your file names
            messagebox.showerror("Image Error", f"Image file not found: {e}. Make sure 'sky_bg.png' and 'plane.png' are in the same directory.")
            self.quit() 
        except Exception as e:
            messagebox.showerror("Image Error", f"Could not load images: {e}. Is Pillow installed? (pip install Pillow)")
            self.quit()

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
        
        if self.canvas_width < 10 or self.canvas_height < 10:
            self.scale_factor = 1.0
            self.offset_x = 0
            self.offset_y = 0
            return
            
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
        # 🌟 Scale and display background image to fill canvas
        if self.sky_image_original:
            resized_sky = self.sky_image_original.resize((self.canvas_width, self.canvas_height), Image.LANCZOS)
            self.sky_image_tk = ImageTk.PhotoImage(resized_sky)
            # Create or update background image on canvas
            if not hasattr(self, '_background_image_id'):
                self._background_image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.sky_image_tk)
            else:
                self.canvas.itemconfig(self._background_image_id, image=self.sky_image_tk)
            self.canvas.tag_lower(self._background_image_id) # Ensure background is behind everything
            
        self.draw_aircraft()

    def _get_rotated_aircraft_image(self, heading: float) -> ImageTk.PhotoImage:
        """Rotates the aircraft image to match its heading and caches it."""
        if not self.aircraft_image_original:
            return None

        # Round heading to nearest 5 degrees to limit cache size
        rounded_heading = int(round(heading / 5.0) * 5.0) % 360
        if rounded_heading not in self.rotated_aircraft_images:
            # Rotate PIL Image. heading=0 means North (up), so rotate by -heading for correct display.
            rotated_pil_image = self.aircraft_image_original.rotate(-rounded_heading, expand=False, resample=Image.BICUBIC)
            self.rotated_aircraft_images[rounded_heading] = ImageTk.PhotoImage(rotated_pil_image)
        return self.rotated_aircraft_images[rounded_heading]

    def draw_aircraft(self):
        # Clear only aircraft and text, leave background image
        self.canvas.delete("aircraft_tag") 
        self.canvas.delete("text_tag")
        
        for ac in self.controller.aircraft_list:
            canvas_x, canvas_y = self.world_to_canvas(ac.x, ac.y)
            
            # 🌟 Use image for aircraft
            rotated_image = self._get_rotated_aircraft_image(ac.heading)
            if rotated_image:
                # Store the image ID directly in the aircraft object if needed for future updates
                ac.image_object_id = self.canvas.create_image(
                    canvas_x, canvas_y, 
                    image=rotated_image, 
                    tags=("aircraft_tag", f"aircraft_{ac.callsign}") # Add tags for easy deletion/identification
                )
                
                # Draw conflict alert outline around the image if needed
                if ac.conflict_alert:
                    # Draw a red circle around the aircraft image
                    self.canvas.create_oval(canvas_x - 15, canvas_y - 15, canvas_x + 15, canvas_y + 15, 
                                             outline="red", width=2, tags="aircraft_tag")
            else:
                 # Fallback to drawing a circle if image failed to load or rotate
                color = "lightblue" if not ac.conflict_alert else "red"
                self.canvas.create_oval(canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5, 
                                         fill=color, outline="white", tags="aircraft_tag")

            self.canvas.create_text(canvas_x, canvas_y - 15, text=ac.callsign, fill="white", tags="text_tag")
    
    def update_info_panel(self, conflicts: List[Tuple] = None):
        self.aircraft_info_text.config(state="normal")
        self.aircraft_info_text.delete("1.0", tk.END)
        for ac in self.controller.aircraft_list:
            self.aircraft_info_text.insert(tk.END, str(ac) + "\n")
        self.aircraft_info_text.config(state="disabled")
        
        self.conflict_info_text.config(state="normal")
        self.conflict_info_text.delete("1.0", tk.END)
        
        if conflicts:
            for ca, cb, c_time, h_dist, v_dist, res_status in conflicts:
                color = "red" if "FAILED" in res_status else "orange" if "RESOLVED" in res_status else "red"
                self.conflict_info_text.insert(tk.END, f"🛑 {ca} vs {cb} — {c_time}\n", "header")
                self.conflict_info_text.insert(tk.END, f"H: {h_dist:.2f} NM | V: {v_dist:.0f} ft\n")
                self.conflict_info_text.insert(tk.END, f"AI Status: {res_status}\n\n", color)

            self.conflict_info_text.tag_config("header", foreground="red", font=("Arial", 10, "bold"))
            self.conflict_info_text.tag_config("red", foreground="red")
            self.conflict_info_text.tag_config("orange", foreground="orange")

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
        
        conflicts = self.controller.check_for_conflicts()
        
        self.controller.apply_resolved_maneuvers()
        
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
        self.stop_sim()
        self.controller.aircraft_list.clear()
        self.current_time_seconds = 0
        self.rotated_aircraft_images.clear() # Clear image cache on restart
        
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
        messagebox.showinfo("Restart", "Simulation restarted successfully! AI Conflict Resolution is active.")

    def open_add_plane_dialog(self):
        dialog = AddPlaneDialog(self, self.controller)
        self.wait_window(dialog)
        self.draw_aircraft()
        self.update_info_panel()
        
# --- 4. Add Plane Dialog (Unchanged) ---
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
    
    # Initial setup for a potential conflict (A and B flying directly towards each other at the same altitude)
    plane_a = Aircraft("AAL123", -20.0, 0.0, 30000, 450, 90)  # West to East
    plane_b = Aircraft("UAL456", 20.0, 0.0, 30000, 450, 270)  # East to West
    
    # Additional planes for complexity
    plane_c = Aircraft("SWA789", -10.0, -20.0, 35000, 500, 45)
    plane_d = Aircraft("DAL001", 10.0, -20.0, 34000, 500, 135)
    
    atc_controller.add_aircraft(plane_a)
    atc_controller.add_aircraft(plane_b)
    atc_controller.add_aircraft(plane_c)
    atc_controller.add_aircraft(plane_d)
    
    app = ATCSimulatorGUI(atc_controller, sim_speed_multiplier=2.0)
    app.mainloop()