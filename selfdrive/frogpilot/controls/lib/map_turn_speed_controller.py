# PFEIFER - MTSC - Modified by FrogAi for FrogPilot
import json
import math

from openpilot.common.params import Params

params_memory = Params("/dev/shm/params")

R = 6373000.0  # approximate radius of Earth in meters
TO_RADIANS = math.pi / 180

TARGET_ACCEL = -1.2  # m/s^2 should match up with the long planner
TARGET_JERK = -0.6   # m/s^3 should match up with the long planner
TARGET_OFFSET = 1.0  # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                     # reach the target velocity when innacuracies in the distance modeling logic would cause overshoot.
                     # The value is multiplied against the target velocity to determine the additional distance. This is
                     # done to keep the distance calculations consistent but results in the offset actually being less
                     # time than specified depending on how much of a speed diffrential there is between v_ego and the
                     # target velocity.

def calculate_accel(t, target_jerk, a_ego):
  return a_ego + target_jerk * t

def calculate_velocity(t, target_jerk, a_ego, v_ego):
  return v_ego + a_ego * t + target_jerk / 2 * (t ** 2)

def calculate_distance(t, target_jerk, a_ego, v_ego):
  return t * v_ego + a_ego / 2 * (t ** 2) + target_jerk / 6 * (t ** 3)

def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx-ax)/2)**2 + math.cos(ax) * math.cos(bx) * math.sin((by-ay)/2)**2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
  return R * c  # in meters

class MapTurnSpeedController:
  def __init__(self):
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.target_v = 0.0

  def target_speed(self, v_ego, a_ego) -> float:
    try:
      position = json.loads(params_memory.get("LastGPSPosition"))
      lat, lon = position["latitude"], position["longitude"]
      target_velocities = json.loads(params_memory.get("MapTargetVelocities"))
    except:
      return 0.0

    lat, lon = lat * TO_RADIANS, lon * TO_RADIANS
    distances = [distance_to_point(lat, lon, tv["latitude"] * TO_RADIANS, tv["longitude"] * TO_RADIANS) for tv in target_velocities]
    min_idx = min(range(len(distances)), key=distances.__getitem__)

    forward_points = target_velocities[min_idx:]
    forward_distances = distances[min_idx:]

    valid_velocities = []
    for i, target_velocity in enumerate(forward_points):
      if target_velocity["velocity"] > v_ego:
        continue

      d = forward_distances[i]
      a_diff = a_ego - TARGET_ACCEL
      accel_t = abs(a_diff / TARGET_JERK)
      min_accel_v = calculate_velocity(accel_t, TARGET_JERK, a_ego, v_ego)
      max_d = calculate_distance(accel_t, TARGET_JERK, a_ego, v_ego)

      if target_velocity["velocity"] > min_accel_v:
        a, b, c = 0.5 * TARGET_JERK, a_ego, v_ego - target_velocity["velocity"]
        t = max((-b + math.sqrt(b**2 - 4 * a * c)) / (2 * a), (-b - math.sqrt(b**2 - 4 * a * c)) / (2 * a))
        if t > 0:
          max_d = calculate_distance(t, TARGET_JERK, a_ego, v_ego)
      else:
        t = abs((min_accel_v - target_velocity["velocity"]) / TARGET_ACCEL)
        max_d += calculate_distance(t, 0, TARGET_ACCEL, min_accel_v)

      if d < max_d + target_velocity["velocity"] * TARGET_OFFSET:
        valid_velocities.append((float(target_velocity["velocity"]), target_velocity["latitude"], target_velocity["longitude"]))

    min_v = min((tv[0] for tv in valid_velocities), default=100.0)
    if self.target_v < min_v and self.target_lat != 0 and self.target_lon != 0:
      if any(tv[0] == self.target_v and tv[1] == self.target_lat and tv[2] == self.target_lon for tv in valid_velocities):
        return float(self.target_v)
      self.target_v, self.target_lat, self.target_lon = 0.0, 0.0, 0.0

    self.target_v = min_v
    for tv in valid_velocities:
      if tv[0] == min_v:
        self.target_lat, self.target_lon = tv[1], tv[2]
        break

    return min_v
