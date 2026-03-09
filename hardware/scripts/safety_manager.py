#!/usr/bin/env python3
"""
Safety Manager for Drone Operations
Implements comprehensive safety monitoring and failsafe mechanisms.

Features:
- Geofencing
- Altitude limits
- Battery monitoring
- Communication loss detection
- Obstacle avoidance integration
- Emergency procedures

Author: Drone Project Team
Version: 2.0.0
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple, Dict
import threading

from rclpy.node import Node


class SafetyState(Enum):
    """Safety system states."""
    NOMINAL = auto()
    WARNING = auto()
    CRITICAL = auto()
    EMERGENCY = auto()


class SafetyEvent(Enum):
    """Types of safety events."""
    BATTERY_LOW = auto()
    BATTERY_CRITICAL = auto()
    GEOFENCE_BREACH = auto()
    ALTITUDE_EXCEED = auto()
    COMMUNICATION_LOSS = auto()
    OBSTACLE_PROXIMITY = auto()
    GPS_LOSS = auto()
    MANUAL_OVERRIDE = auto()
    HARDWARE_FAULT = auto()


@dataclass
class SafetyLimits:
    """Configurable safety limits."""
    # Geofence
    max_distance_from_home: float = 100.0  # meters
    
    # Altitude
    min_altitude: float = 0.3  # meters
    max_altitude: float = 30.0  # meters
    
    # Battery
    battery_low_threshold: float = 25.0  # percent
    battery_critical_threshold: float = 15.0  # percent
    battery_emergency_threshold: float = 8.0  # percent
    
    # Communication
    comm_loss_timeout: float = 3.0  # seconds
    
    # Obstacles
    obstacle_warning_distance: float = 2.0  # meters
    obstacle_critical_distance: float = 0.5  # meters
    
    # Speed limits
    max_horizontal_speed: float = 5.0  # m/s
    max_vertical_speed: float = 2.0  # m/s
    
    # GPS
    gps_loss_timeout: float = 5.0  # seconds


@dataclass
class SafetyEventRecord:
    """Record of a safety event."""
    event_type: SafetyEvent
    timestamp: float
    message: str
    severity: SafetyState
    acknowledged: bool = False


class SafetyManager:
    """
    Centralized safety monitoring and management.
    Can be used as standalone or integrated with ROS node.
    """
    
    def __init__(
        self,
        limits: Optional[SafetyLimits] = None,
        logger: Optional[Callable] = None
    ):
        self.limits = limits or SafetyLimits()
        self.logger = logger or print
        
        # State tracking
        self.state = SafetyState.NOMINAL
        self.events: List[SafetyEventRecord] = []
        self.max_events = 100
        
        # Callbacks for actions
        self.action_callbacks: Dict[SafetyEvent, Callable] = {}
        
        # Monitoring state
        self.last_comm_time = time.time()
        self.last_gps_time = time.time()
        self.home_position: Optional[Tuple[float, float, float]] = None
        
        # Event counts for debouncing
        self.event_counts: Dict[SafetyEvent, int] = {}
        self.event_debounce_window = 1.0  # seconds
        self.event_last_time: Dict[SafetyEvent, float] = {}
        
        # Thread safety
        self._lock = threading.Lock()
    
    def set_home(self, x: float, y: float, z: float):
        """Set home position for geofencing."""
        with self._lock:
            self.home_position = (x, y, z)
            self.logger(f"[Safety] Home position set: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def register_action(self, event: SafetyEvent, callback: Callable):
        """Register callback for safety event."""
        with self._lock:
            self.action_callbacks[event] = callback
    
    def check_battery(self, battery_percent: float) -> SafetyState:
        """Check battery level and return safety state."""
        if battery_percent <= self.limits.battery_emergency_threshold:
            self._trigger_event(
                SafetyEvent.BATTERY_CRITICAL,
                f"EMERGENCY: Battery at {battery_percent:.1f}%",
                SafetyState.EMERGENCY
            )
            return SafetyState.EMERGENCY
        
        if battery_percent <= self.limits.battery_critical_threshold:
            self._trigger_event(
                SafetyEvent.BATTERY_CRITICAL,
                f"CRITICAL: Battery at {battery_percent:.1f}%",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        if battery_percent <= self.limits.battery_low_threshold:
            self._trigger_event(
                SafetyEvent.BATTERY_LOW,
                f"WARNING: Battery low at {battery_percent:.1f}%",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_geofence(self, x: float, y: float) -> SafetyState:
        """Check if position is within geofence."""
        if self.home_position is None:
            return SafetyState.NOMINAL
        
        home_x, home_y, _ = self.home_position
        distance = math.hypot(x - home_x, y - home_y)
        
        if distance > self.limits.max_distance_from_home:
            self._trigger_event(
                SafetyEvent.GEOFENCE_BREACH,
                f"CRITICAL: Geofence breach at {distance:.1f}m from home",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        # Warning at 90% of limit
        if distance > 0.9 * self.limits.max_distance_from_home:
            self._trigger_event(
                SafetyEvent.GEOFENCE_BREACH,
                f"WARNING: Near geofence limit ({distance:.1f}m)",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_altitude(self, z: float) -> SafetyState:
        """Check if altitude is within limits."""
        if z > self.limits.max_altitude:
            self._trigger_event(
                SafetyEvent.ALTITUDE_EXCEED,
                f"CRITICAL: Altitude {z:.1f}m exceeds max {self.limits.max_altitude}m",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        if z < self.limits.min_altitude:
            # This might be OK during landing
            if z < 0.1:
                self._trigger_event(
                    SafetyEvent.ALTITUDE_EXCEED,
                    f"WARNING: Very low altitude {z:.2f}m",
                    SafetyState.WARNING
                )
                return SafetyState.WARNING
        
        # Warning at 90% of max
        if z > 0.9 * self.limits.max_altitude:
            self._trigger_event(
                SafetyEvent.ALTITUDE_EXCEED,
                f"WARNING: Near max altitude ({z:.1f}m)",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_communication(self, last_heartbeat: float) -> SafetyState:
        """Check communication status."""
        now = time.time()
        time_since_comm = now - last_heartbeat
        
        if time_since_comm > self.limits.comm_loss_timeout:
            self._trigger_event(
                SafetyEvent.COMMUNICATION_LOSS,
                f"CRITICAL: No communication for {time_since_comm:.1f}s",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        # Warning at 50% of timeout
        if time_since_comm > 0.5 * self.limits.comm_loss_timeout:
            self._trigger_event(
                SafetyEvent.COMMUNICATION_LOSS,
                f"WARNING: Communication lag ({time_since_comm:.1f}s)",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_obstacle(self, min_distance: float) -> SafetyState:
        """Check obstacle proximity."""
        if min_distance <= self.limits.obstacle_critical_distance:
            self._trigger_event(
                SafetyEvent.OBSTACLE_PROXIMITY,
                f"CRITICAL: Obstacle at {min_distance:.2f}m!",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        if min_distance <= self.limits.obstacle_warning_distance:
            self._trigger_event(
                SafetyEvent.OBSTACLE_PROXIMITY,
                f"WARNING: Obstacle proximity ({min_distance:.1f}m)",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_gps(self, last_gps_time: float, fix_type: int) -> SafetyState:
        """Check GPS status."""
        now = time.time()
        time_since_gps = now - last_gps_time
        
        if time_since_gps > self.limits.gps_loss_timeout:
            self._trigger_event(
                SafetyEvent.GPS_LOSS,
                f"CRITICAL: GPS lost for {time_since_gps:.1f}s",
                SafetyState.CRITICAL
            )
            return SafetyState.CRITICAL
        
        if fix_type < 3:  # Less than 3D fix
            self._trigger_event(
                SafetyEvent.GPS_LOSS,
                f"WARNING: Weak GPS fix (type={fix_type})",
                SafetyState.WARNING
            )
            return SafetyState.WARNING
        
        return SafetyState.NOMINAL
    
    def check_speed(self, vx: float, vy: float, vz: float) -> SafetyState:
        """Check if speed is within limits."""
        horizontal_speed = math.hypot(vx, vy)
        vertical_speed = abs(vz)
        
        state = SafetyState.NOMINAL
        
        if horizontal_speed > self.limits.max_horizontal_speed:
            self._trigger_event(
                SafetyEvent.HARDWARE_FAULT,
                f"WARNING: Horizontal speed {horizontal_speed:.1f}m/s exceeds limit",
                SafetyState.WARNING
            )
            state = SafetyState.WARNING
        
        if vertical_speed > self.limits.max_vertical_speed:
            self._trigger_event(
                SafetyEvent.HARDWARE_FAULT,
                f"WARNING: Vertical speed {vertical_speed:.1f}m/s exceeds limit",
                SafetyState.WARNING
            )
            state = SafetyState.WARNING
        
        return state
    
    def comprehensive_check(
        self,
        x: float, y: float, z: float,
        vx: float, vy: float, vz: float,
        battery_percent: float,
        min_obstacle_distance: float = float('inf'),
        gps_fix_type: int = 3,
        last_comm_time: Optional[float] = None,
        last_gps_time: Optional[float] = None
    ) -> Tuple[SafetyState, List[str]]:
        """
        Perform comprehensive safety check.
        Returns (worst_state, list_of_warnings).
        """
        states = []
        messages = []
        
        # Battery check
        state = self.check_battery(battery_percent)
        states.append(state)
        if state != SafetyState.NOMINAL:
            messages.append(f"Battery: {battery_percent:.1f}%")
        
        # Geofence check
        state = self.check_geofence(x, y)
        states.append(state)
        if state != SafetyState.NOMINAL:
            messages.append(f"Distance from home: {math.hypot(x, y):.1f}m")
        
        # Altitude check
        state = self.check_altitude(z)
        states.append(state)
        if state != SafetyState.NOMINAL:
            messages.append(f"Altitude: {z:.1f}m")
        
        # Speed check
        state = self.check_speed(vx, vy, vz)
        states.append(state)
        
        # Obstacle check
        if min_obstacle_distance < float('inf'):
            state = self.check_obstacle(min_obstacle_distance)
            states.append(state)
            if state != SafetyState.NOMINAL:
                messages.append(f"Obstacle at: {min_obstacle_distance:.1f}m")
        
        # Communication check
        if last_comm_time is not None:
            state = self.check_communication(last_comm_time)
            states.append(state)
        
        # GPS check
        if last_gps_time is not None:
            state = self.check_gps(last_gps_time, gps_fix_type)
            states.append(state)
        
        # Update overall state
        with self._lock:
            self.state = max(states, key=lambda s: s.value)
        
        return self.state, messages
    
    def update_comm_time(self):
        """Update last communication time."""
        with self._lock:
            self.last_comm_time = time.time()
    
    def update_gps_time(self):
        """Update last GPS time."""
        with self._lock:
            self.last_gps_time = time.time()
    
    def _trigger_event(self, event: SafetyEvent, message: str, severity: SafetyState):
        """Trigger a safety event."""
        now = time.time()
        
        with self._lock:
            # Debounce check
            if event in self.event_last_time:
                if (now - self.event_last_time[event]) < self.event_debounce_window:
                    return  # Skip duplicate event
            
            self.event_last_time[event] = now
            
            # Create event record
            record = SafetyEventRecord(
                event_type=event,
                timestamp=now,
                message=message,
                severity=severity
            )
            
            # Add to history
            self.events.append(record)
            if len(self.events) > self.max_events:
                self.events.pop(0)
            
            # Log
            log_level = {
                SafetyState.NOMINAL: "INFO",
                SafetyState.WARNING: "WARN",
                SafetyState.CRITICAL: "ERROR",
                SafetyState.EMERGENCY: "CRITICAL"
            }.get(severity, "INFO")
            
            self.logger(f"[Safety][{log_level}] {message}")
            
            # Execute callback if registered
            if event in self.action_callbacks:
                try:
                    callback = self.action_callbacks[event]
                    callback(record)
                except Exception as e:
                    self.logger(f"[Safety] Callback error: {e}")
    
    def acknowledge_event(self, event_type: SafetyEvent) -> bool:
        """Acknowledge a safety event."""
        with self._lock:
            for event in reversed(self.events):
                if event.event_type == event_type and not event.acknowledged:
                    event.acknowledged = True
                    return True
        return False
    
    def get_active_events(self) -> List[SafetyEventRecord]:
        """Get list of unacknowledged events."""
        with self._lock:
            return [e for e in self.events if not e.acknowledged][-10:]
    
    def get_state(self) -> SafetyState:
        """Get current safety state."""
        with self._lock:
            return self.state
    
    def get_event_history(self, count: int = 20) -> List[SafetyEventRecord]:
        """Get recent event history."""
        with self._lock:
            return self.events[-count:]
    
    def is_safe_to_fly(self) -> Tuple[bool, str]:
        """Check if conditions are safe for flight."""
        state = self.get_state()
        
        if state == SafetyState.EMERGENCY:
            return False, "Emergency state - immediate action required"
        
        if state == SafetyState.CRITICAL:
            return False, "Critical safety issue - resolve before flight"
        
        if self.home_position is None:
            return False, "Home position not set"
        
        return True, "Safe to fly"


class ROS2SafetyNode(Node):
    """ROS 2 node wrapper for SafetyManager."""
    
    def __init__(self):
        super().__init__("safety_manager")
        
        self.safety = SafetyManager(
            limits=SafetyLimits(),
            logger=self._log_callback
        )
        
        # Publishers for safety status
        # Subscribers for sensor data
        # Timer for periodic checks
        
        self.get_logger().info("Safety Manager Node initialized")
    
    def _log_callback(self, message: str):
        """Route safety messages to ROS logger."""
        if "CRITICAL" in message or "EMERGENCY" in message:
            self.get_logger().error(message)
        elif "WARNING" in message or "WARN" in message:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)


# Example usage
if __name__ == "__main__":
    print("Safety Manager Test")
    print("=" * 40)
    
    safety = SafetyManager()
    safety.set_home(0.0, 0.0, 0.0)
    
    # Register emergency callbacks
    def emergency_land(event):
        print(f"!!! EMERGENCY ACTION TRIGGERED: {event.message}")
        print("    -> Would command emergency landing")
    
    def rtl_action(event):
        print(f"!!! RTL ACTION TRIGGERED: {event.message}")
        print("    -> Would command return to launch")
    
    safety.register_action(SafetyEvent.BATTERY_CRITICAL, emergency_land)
    safety.register_action(SafetyEvent.GEOFENCE_BREACH, rtl_action)
    
    # Test scenarios
    scenarios = [
        {"battery_percent": 50.0, "x": 0, "y": 0, "z": 4.0, "vx": 0, "vy": 0, "vz": 0},
        {"battery_percent": 22.0, "x": 0, "y": 0, "z": 4.0, "vx": 0, "vy": 0, "vz": 0},  # Low battery
        {"battery_percent": 12.0, "x": 0, "y": 0, "z": 4.0, "vx": 0, "vy": 0, "vz": 0},  # Critical
        {"battery_percent": 80.0, "x": 95, "y": 0, "z": 4.0, "vx": 0, "vy": 0, "vz": 0},  # Geofence
        {"battery_percent": 80.0, "x": 0, "y": 0, "z": 28.0, "vx": 0, "vy": 0, "vz": 0},  # Altitude
    ]
    
    for i, scenario in enumerate(scenarios):
        print(f"\nScenario {i+1}:")
        state, messages = safety.comprehensive_check(**scenario)
        print(f"  State: {state.name}")
        if messages:
            print(f"  Messages: {messages}")
    
    print("\n" + "=" * 40)
    print("Safety check examples completed")
