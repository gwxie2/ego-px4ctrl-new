#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VINS Drift Monitor for Phase 4 Dual-UAV System

This script monitors and compares VINS odometry against ground truth from Gazebo,
providing real-time visualization and CSV logging of drift metrics.

Features:
- Subscribes to VINS imu_propagate topics for both UAVs
- Subscribes to Gazebo model_states for ground truth
- Real-time CSV logging to /tmp/vins_drift_monitor/
- Interactive matplotlib visualization with 2x2 layout
- Periodic static image saving

Usage:
    rosrun clean_uav_core vins_drift_monitor.py
"""

import rospy
import os
import csv
import threading
from datetime import datetime
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Set backend before importing pyplot
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class VINSDriftMonitor:
    """Monitor VINS odometry drift compared to ground truth."""

    def __init__(self):
        rospy.init_node("vins_drift_monitor", anonymous=False)

        # Parameters
        self.output_dir = rospy.get_param("~output_dir", "/tmp/vins_drift_monitor")
        self.csv_save_interval = rospy.get_param("~csv_save_interval", 1.0)  # seconds
        self.plot_save_interval = rospy.get_param("~plot_save_interval", 5.0)  # seconds
        self.max_history_len = rospy.get_param("~max_history_len", 10000)  # max points for plotting (use large number to keep all data)

        # Model names in Gazebo
        self.uav0_model = rospy.get_param("~uav0_model", "iris_0")
        self.uav1_model = rospy.get_param("~uav1_model", "iris_1")

        # Create output directory
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo("[vins_drift_monitor] Created output directory: %s", self.output_dir)

        # Initialize timestamp
        self.start_time = rospy.Time.now()
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Data storage (using lists to keep ALL data)
        self.data = {
            "uav0": {
                "time": [],
                "vins_x": [], "vins_y": [],
                "truth_x": [], "truth_y": [],
                "drift_xy": [], "drift_x": [], "drift_y": [],
            },
            "uav1": {
                "time": [],
                "vins_x": [], "vins_y": [],
                "truth_x": [], "truth_y": [],
                "drift_xy": [], "drift_x": [], "drift_y": [],
            }
        }

        # Latest data holders (thread-safe)
        self.lock = threading.Lock()
        self.latest = {
            "uav0": {"vins": None, "truth": None, "vins_received": False, "truth_received": False},
            "uav1": {"vins": None, "truth": None, "vins_received": False, "truth_received": False}
        }

        # Gazebo model indices
        self.model_indices = {self.uav0_model: None, self.uav1_model: None}

        # Initial positions for coordinate alignment
        # VINS starts at (0,0), but Gazebo has different starting positions
        # We need to record initial true position to offset VINS coordinates
        self.initial_positions = {
            "uav0": {"truth": None, "initialized": False},
            "uav1": {"truth": None, "initialized": False}
        }

        # Initial axis bounds (for maintaining starting view)
        self.initial_bounds = {
            "uav0": {"x_min": None, "x_max": None, "y_min": None, "y_max": None},
            "uav1": {"x_min": None, "x_max": None, "y_min": None, "y_max": None}
        }

        # Track if initial bounds have been set
        self.bounds_initialized = {"uav0": False, "uav1": False}

        # CSV file paths
        self.csv_files = {
            "uav0": os.path.join(self.output_dir, f"uav0_vins_drift_{self.timestamp}.csv"),
            "uav1": os.path.join(self.output_dir, f"uav1_vins_drift_{self.timestamp}.csv"),
        }

        # Initialize CSV files with headers
        self._init_csv_files()

        # Subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback, queue_size=1)
        rospy.Subscriber("/iris_0/vins_estimator/imu_propagate", Odometry, self.uav0_vins_callback, queue_size=10)
        rospy.Subscriber("/iris_1/vins_estimator/imu_propagate", Odometry, self.uav1_vins_callback, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(self.csv_save_interval), self.csv_save_callback)
        rospy.Timer(rospy.Duration(self.plot_save_interval), self.trigger_plot_update)

        # Flag for plot update request
        self.plot_update_pending = False

        rospy.loginfo("[vins_drift_monitor] Initialized")
        rospy.loginfo("[vins_drift_monitor] Output directory: %s", self.output_dir)
        rospy.loginfo("[vins_drift_monitor] Monitoring UAV0: %s", self.uav0_model)
        rospy.loginfo("[vins_drift_monitor] Monitoring UAV1: %s", self.uav1_model)

    def _init_csv_files(self):
        """Initialize CSV files with headers."""
        headers = [
            "timestamp_ros",
            "timestamp_wall",
            "vins_x", "vins_y", "vins_z",
            "truth_x", "truth_y", "truth_z",
            "drift_x", "drift_y", "drift_xy",
            "vins_qx", "vins_qy", "vins_qz", "vins_qw",
            "truth_qx", "truth_qy", "truth_qz", "truth_qw"
        ]

        for uav in ["uav0", "uav1"]:
            with open(self.csv_files[uav], "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(headers)
            rospy.loginfo("[vins_drift_monitor] Created CSV: %s", self.csv_files[uav])

    def _setup_gui(self):
        """Setup Tkinter GUI with matplotlib integration."""
        # Create main window
        self.root = tk.Tk()
        self.root.title("VINS Drift Monitor - Real-time")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Create figure with 2x2 layout
        self.fig = Figure(figsize=(14, 10))
        self.fig.suptitle("VINS Drift Monitor", fontsize=16, fontweight="bold")

        # Create subplots: 2x2 grid
        # Top row: trajectory maps
        self.ax_map_0 = self.fig.add_subplot(2, 2, 1)
        self.ax_map_1 = self.fig.add_subplot(2, 2, 2)
        # Bottom row: time-drift plots
        self.ax_time_0 = self.fig.add_subplot(2, 2, 3)
        self.ax_time_1 = self.fig.add_subplot(2, 2, 4)

        self._setup_map_plot(self.ax_map_0, "UAV0 (iris_0)")
        self._setup_map_plot(self.ax_map_1, "UAV1 (iris_1)")
        self._setup_time_plot(self.ax_time_0, "UAV0 (iris_0)")
        self._setup_time_plot(self.ax_time_1, "UAV1 (iris_1)")

        plt.tight_layout(rect=[0, 0, 1, 0.96])

        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Control frame
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # Status label
        self.status_label = ttk.Label(control_frame, text="Initializing...")
        self.status_label.pack(side=tk.LEFT, padx=5)

        # Save button
        ttk.Button(control_frame, text="Save Plot Now", command=self.save_plot).pack(side=tk.RIGHT, padx=5)

        # Store plot elements
        self.plot_elements = {
            "uav0": {
                "truth_line": None,
                "vins_line": None,
                "truth_point": None,
                "vins_point": None,
                "drift_line": None,
                "drift_text": None,
                "time_drift_line": None,
                "time_text": None,
            },
            "uav1": {
                "truth_line": None,
                "vins_line": None,
                "truth_point": None,
                "vins_point": None,
                "drift_line": None,
                "drift_text": None,
                "time_drift_line": None,
                "time_text": None,
            }
        }

        self._init_plot_elements()

    def _setup_map_plot(self, ax, title):
        """Setup trajectory map subplot."""
        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")

    def _setup_time_plot(self, ax, title):
        """Setup time-drift subplot."""
        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Drift (m)")
        ax.grid(True, alpha=0.3)

    def _init_plot_elements(self):
        """Initialize plot elements for both UAVs."""
        map_axes = {"uav0": self.ax_map_0, "uav1": self.ax_map_1}
        time_axes = {"uav0": self.ax_time_0, "uav1": self.ax_time_1}

        for uav in ["uav0", "uav1"]:
            ax_map = map_axes[uav]
            ax_time = time_axes[uav]

            # Map plot elements
            self.plot_elements[uav]["truth_line"] = ax_map.plot([], [], "g-", linewidth=2, label="Ground Truth", alpha=0.7)[0]
            self.plot_elements[uav]["vins_line"] = ax_map.plot([], [], "r--", linewidth=2, label="VINS Odometry", alpha=0.7)[0]
            self.plot_elements[uav]["truth_point"] = ax_map.plot([], [], "go", markersize=8, label="Truth (current)")[0]
            self.plot_elements[uav]["vins_point"] = ax_map.plot([], [], "r^", markersize=10, label="VINS (current)")[0]
            self.plot_elements[uav]["drift_line"] = ax_map.plot([], [], "b:", linewidth=1.5, label="Drift Vector")[0]
            self.plot_elements[uav]["drift_text"] = ax_map.text(0.02, 0.98, "", transform=ax_map.transAxes,
                                                                verticalalignment="top",
                                                                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
                                                                fontsize=9)
            ax_map.legend(loc="lower right", fontsize=8)

            # Time plot elements
            self.plot_elements[uav]["time_drift_line"] = ax_time.plot([], [], "b-", linewidth=2, label="Position Drift")[0]
            self.plot_elements[uav]["time_text"] = ax_time.text(0.02, 0.98, "", transform=ax_time.transAxes,
                                                               verticalalignment="top",
                                                               bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.5),
                                                               fontsize=9)
            ax_time.legend(loc="upper right", fontsize=8)

    def gazebo_callback(self, msg: ModelStates):
        """Handle Gazebo model states (ground truth)."""
        with self.lock:
            # Find model indices on first receive
            if None in self.model_indices.values():
                for model_name in [self.uav0_model, self.uav1_model]:
                    if model_name in msg.name:
                        self.model_indices[model_name] = msg.name.index(model_name)
                        rospy.loginfo("[vins_drift_monitor] Found model '%s' at index %d",
                                      model_name, self.model_indices[model_name])

            # Update ground truth for each UAV
            for uav, model_name in [("uav0", self.uav0_model), ("uav1", self.uav1_model)]:
                idx = self.model_indices.get(model_name)
                if idx is not None and idx < len(msg.pose):
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = "world"
                    odom.pose.pose = msg.pose[idx]
                    odom.twist.twist = msg.twist[idx] if idx < len(msg.twist) else odom.twist.twist

                    self.latest[uav]["truth"] = odom
                    self.latest[uav]["truth_received"] = True

    def uav0_vins_callback(self, msg: Odometry):
        """Handle UAV0 VINS odometry."""
        with self.lock:
            self.latest["uav0"]["vins"] = msg
            self.latest["uav0"]["vins_received"] = True

    def uav1_vins_callback(self, msg: Odometry):
        """Handle UAV1 VINS odometry."""
        with self.lock:
            self.latest["uav1"]["vins"] = msg
            self.latest["uav1"]["vins_received"] = True

    def _process_data(self):
        """Process latest data and update storage."""
        current_time = (rospy.Time.now() - self.start_time).to_sec()
        wall_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        csv_rows = {"uav0": None, "uav1": None}

        with self.lock:
            for uav in ["uav0", "uav1"]:
                vins = self.latest[uav]["vins"]
                truth = self.latest[uav]["truth"]

                if vins is None or truth is None:
                    continue

                # Extract original positions
                vins_x = vins.pose.pose.position.x
                vins_y = vins.pose.pose.position.y
                vins_z = vins.pose.pose.position.z

                truth_x = truth.pose.pose.position.x
                truth_y = truth.pose.pose.position.y
                truth_z = truth.pose.pose.position.z

                # Initialize: record initial true position
                init_pos = self.initial_positions[uav]
                if not init_pos["initialized"]:
                    init_pos["truth"] = np.array([truth_x, truth_y])
                    init_pos["initialized"] = True
                    rospy.loginfo("[vins_drift_monitor] %s initial truth position: (%.3f, %.3f)",
                                  uav, truth_x, truth_y)

                # Calculate offset: how much to shift VINS to align with truth coordinate system
                # VINS starts at (0,0), truth starts at init_pos["truth"]
                # So offset = truth - vins = init_pos["truth"] - (0,0) = init_pos["truth"]
                if init_pos["initialized"]:
                    offset = init_pos["truth"]
                    # Shift VINS to truth coordinate system
                    vins_display_x = vins_x + offset[0]
                    vins_display_y = vins_y + offset[1]
                else:
                    vins_display_x = vins_x
                    vins_display_y = vins_y

                # For display: use shifted VINS and original truth
                # For drift: calculate in the aligned coordinate system
                # Drift = shifted_VINS - truth
                drift_x = vins_display_x - truth_x
                drift_y = vins_display_y - truth_y
                drift_xy = np.sqrt(drift_x**2 + drift_y**2)

                # Optional: limit size to prevent memory issues
                if len(self.data[uav]["time"]) >= self.max_history_len:
                    self.data[uav]["time"].pop(0)
                    self.data[uav]["vins_x"].pop(0)
                    self.data[uav]["vins_y"].pop(0)
                    self.data[uav]["truth_x"].pop(0)
                    self.data[uav]["truth_y"].pop(0)
                    self.data[uav]["drift_x"].pop(0)
                    self.data[uav]["drift_y"].pop(0)
                    self.data[uav]["drift_xy"].pop(0)

                # Store data for display (shifted VINS, original truth)
                self.data[uav]["time"].append(current_time)
                self.data[uav]["vins_x"].append(vins_display_x)
                self.data[uav]["vins_y"].append(vins_display_y)
                self.data[uav]["truth_x"].append(truth_x)
                self.data[uav]["truth_y"].append(truth_y)
                self.data[uav]["drift_x"].append(drift_x)
                self.data[uav]["drift_y"].append(drift_y)
                self.data[uav]["drift_xy"].append(drift_xy)

                # Prepare CSV row (store original positions and calculated drift)
                vins_q = vins.pose.pose.orientation
                truth_q = truth.pose.pose.orientation

                csv_rows[uav] = [
                    current_time, wall_time,
                    vins_x, vins_y, vins_z,  # Original VINS coordinates
                    truth_x, truth_y, truth_z,  # Original truth coordinates
                    drift_x, drift_y, drift_xy,  # Calculated drift (aligned)
                    vins_q.x, vins_q.y, vins_q.z, vins_q.w,
                    truth_q.x, truth_q.y, truth_q.z, truth_q.w
                ]

        return csv_rows

    def csv_save_callback(self, event):
        """Periodically save data to CSV files."""
        csv_rows = self._process_data()

        for uav in ["uav0", "uav1"]:
            if csv_rows[uav] is not None:
                try:
                    with open(self.csv_files[uav], "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow(csv_rows[uav])
                except IOError as e:
                    rospy.logwarn_throttle(10, "[vins_drift_monitor] Failed to write CSV: %s", e)

    def trigger_plot_update(self, event):
        """Trigger plot update from timer callback (thread-safe)."""
        self.plot_update_pending = True

    def update_plots(self):
        """Update all plots (must be called from main thread)."""
        for uav, ax_map in [("uav0", self.ax_map_0), ("uav1", self.ax_map_1)]:
            ax_time = self.ax_time_0 if uav == "uav0" else self.ax_time_1
            data = self.data[uav]

            if len(data["time"]) == 0:
                continue

            # Get data lists (no conversion needed, using lists directly)
            truth_x = data["truth_x"]
            truth_y = data["truth_y"]
            vins_x = data["vins_x"]
            vins_y = data["vins_y"]
            drift_xy = data["drift_xy"]
            times = data["time"]

            if len(truth_x) == 0:
                continue

            # Update trajectory lines (map plot)
            self.plot_elements[uav]["truth_line"].set_data(truth_x, truth_y)
            self.plot_elements[uav]["vins_line"].set_data(vins_x, vins_y)

            # Update current position markers
            self.plot_elements[uav]["truth_point"].set_data([truth_x[-1]], [truth_y[-1]])
            self.plot_elements[uav]["vins_point"].set_data([vins_x[-1]], [vins_y[-1]])

            # Update drift vector (line from truth to vins)
            self.plot_elements[uav]["drift_line"].set_data(
                [truth_x[-1], vins_x[-1]],
                [truth_y[-1], vins_y[-1]]
            )

            # Update map statistics text
            if drift_xy:
                mean_drift = np.mean(drift_xy)
                max_drift = np.max(drift_xy)
                current_drift = drift_xy[-1]

                stats_text = (
                    f"Current: {current_drift:.3f} m\n"
                    f"Mean: {mean_drift:.3f} m\n"
                    f"Max: {max_drift:.3f} m\n"
                    f"Points: {len(drift_xy)}"
                )
                self.plot_elements[uav]["drift_text"].set_text(stats_text)

            # Dynamic axis scaling (maintain initial bounds, expand as needed)
            all_x = truth_x + vins_x
            all_y = truth_y + vins_y

            if all_x and all_y:
                data_x_min = min(all_x)
                data_x_max = max(all_x)
                data_y_min = min(all_y)
                data_y_max = max(all_y)

                bounds = self.initial_bounds[uav]

                # Initialize bounds on first data
                if not self.bounds_initialized[uav]:
                    bounds["x_min"] = data_x_min
                    bounds["x_max"] = data_x_max
                    bounds["y_min"] = data_y_min
                    bounds["y_max"] = data_y_max
                    self.bounds_initialized[uav] = True
                    rospy.loginfo("[vins_drift_monitor] %s initial bounds - X: [%.2f, %.2f], Y: [%.2f, %.2f]",
                                  uav, bounds["x_min"], bounds["x_max"], bounds["y_min"], bounds["y_max"])

                # Expand bounds if data exceeds current range
                bounds["x_min"] = min(bounds["x_min"], data_x_min)
                bounds["x_max"] = max(bounds["x_max"], data_x_max)
                bounds["y_min"] = min(bounds["y_min"], data_y_min)
                bounds["y_max"] = max(bounds["y_max"], data_y_max)

                # Use initial bounds as minimum range, but expand if needed
                initial_x_range = bounds["x_max"] - bounds["x_min"] if bounds["x_max"] != bounds["x_min"] else 2.0
                initial_y_range = bounds["y_max"] - bounds["y_min"] if bounds["y_max"] != bounds["y_min"] else 2.0

                # Calculate center of current data
                center_x = (data_x_min + data_x_max) / 2
                center_y = (data_y_min + data_y_max) / 2

                # Calculate half ranges (at least initial range/2 + margin)
                half_x = max(initial_x_range / 2, (data_x_max - data_x_min) / 2 * 1.1 + 0.5)
                half_y = max(initial_y_range / 2, (data_y_max - data_y_min) / 2 * 1.1 + 0.5)

                ax_map.set_xlim(center_x - half_x, center_x + half_x)
                ax_map.set_ylim(center_y - half_y, center_y + half_y)

            # Update time-drift plot
            self.plot_elements[uav]["time_drift_line"].set_data(times, drift_xy)

            if times and drift_xy:
                ax_time.set_xlim(min(times), max(times) + 1)
                ax_time.set_ylim(0, max(drift_xy) * 1.1 + 0.1)

                # Update time plot statistics
                time_text = (
                    f"Current: {drift_xy[-1]:.3f} m\n"
                    f"Mean: {np.mean(drift_xy):.3f} m\n"
                    f"Max: {np.max(drift_xy):.3f} m"
                )
                self.plot_elements[uav]["time_text"].set_text(time_text)

        # Update status label
        uav0_points = len(self.data["uav0"]["time"])
        uav1_points = len(self.data["uav1"]["time"])
        self.status_label.config(text=f"UAV0: {uav0_points} points | UAV1: {uav1_points} points")

        # Refresh canvas
        self.canvas.draw_idle()

    def save_plot(self):
        """Save plot to file."""
        plot_path = os.path.join(self.output_dir, f"vins_drift_plot_{self.timestamp}.png")
        try:
            self.fig.savefig(plot_path, dpi=150, bbox_inches="tight")
            rospy.loginfo("[vins_drift_monitor] Saved plot: %s", plot_path)
            return True
        except Exception as e:
            rospy.logwarn("[vins_drift_monitor] Failed to save plot: %s", e)
            return False

    def on_closing(self):
        """Handle window close event."""
        rospy.loginfo("[vins_drift_monitor] Closing...")
        rospy.signal_shutdown("User closed window")
        self.root.destroy()

    def run(self):
        """Main loop with Tkinter GUI."""
        rospy.loginfo("[vins_drift_monitor] Starting GUI...")

        # Setup GUI (must be in main thread)
        self._setup_gui()

        # Last save time tracking
        last_save_time = rospy.Time.now()

        try:
            while not rospy.is_shutdown():
                # Process data for CSV
                self._process_data()

                # Check if plot update is pending
                if self.plot_update_pending:
                    self.update_plots()
                    self.plot_update_pending = False

                    # Save plot periodically
                    if (rospy.Time.now() - last_save_time).to_sec() >= self.plot_save_interval:
                        self.save_plot()
                        last_save_time = rospy.Time.now()

                # Update Tkinter
                self.root.update_idletasks()
                self.root.update()

                # Small sleep to prevent CPU spinning
                rospy.sleep(0.01)

        except rospy.ROSInterruptException:
            pass
        except tk.TclError:
            # Window was closed
            rospy.loginfo("[vins_drift_monitor] Window closed")
        finally:
            rospy.loginfo("[vins_drift_monitor] Shutting down...")
            try:
                self.root.destroy()
            except:
                pass


if __name__ == "__main__":
    try:
        monitor = VINSDriftMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[vins_drift_monitor] Exception: %s", e)
        import traceback
        traceback.print_exc()
