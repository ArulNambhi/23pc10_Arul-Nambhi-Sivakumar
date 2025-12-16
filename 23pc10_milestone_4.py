import math
import json
import random

# ---------------- Geometry helpers ----------------

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def compute_center(vertices):
    x = sum(v[0] for v in vertices[:4]) / 4.0
    y = sum(v[1] for v in vertices[:4]) / 4.0
    return [x, y]

def compute_angle(vertices):
    dx = vertices[1][0] - vertices[0][0]
    dy = vertices[1][1] - vertices[0][1]
    ang = math.degrees(math.atan2(dy, dx))
    if ang < 0:
        ang += 360
    return ang

def angle_difference(a1, a2):
    diff = abs(a2 - a1) % 360
    if diff > 180:
        diff = 360 - diff
    if diff > 90:
        diff = 180 - diff
    if diff > 45:
        diff = 90 - diff
    return diff

def compute_motion_time(distance, velocity, acceleration):
    # Symmetric trapezoidal profile; halt is modeled separately as direction-change events.
    if distance <= 0:
        return 0.0
    if velocity <= 0:
        return float('inf')
    if acceleration <= 0:
        return distance / velocity
    accel_dist = (velocity ** 2) / (2.0 * acceleration)
    if distance <= 2.0 * accel_dist:
        # Triangular profile (no cruise)
        return 2.0 * math.sqrt(distance / acceleration)
    else:
        # Trapezoidal: accel + cruise + decel
        return 2.0 * (velocity / acceleration) + (distance - 2.0 * accel_dist) / velocity

# ---------------- Forbidden zone intersection ----------------

def line_intersects_rect(p1, p2, bl, tr):
    # Liang-Barsky or simple segment-rect test via segment-edge intersection + endpoint inside
    (x1, y1), (x2, y2) = p1, p2
    rx1, ry1 = bl
    rx2, ry2 = tr

    def inside(px, py):
        return rx1 <= px <= rx2 and ry1 <= py <= ry2

    if inside(x1, y1) or inside(x2, y2):
        return True

    corners = [(rx1, ry1), (rx2, ry1), (rx2, ry2), (rx1, ry2)]
    edges = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]

    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    for e in edges:
        if intersect(p1, p2, e[0], e[1]):
            return True
    return False

# ---------------- Wafer boundary enforcement ----------------

def inside_circle(p, radius, center=(0.0, 0.0)):
    dx = p[0] - center[0]
    dy = p[1] - center[1]
    return (dx * dx + dy * dy) <= (radius * radius + 1e-9)

def path_inside_circle(points, radius, center=(0.0, 0.0)):
    # Require every waypoint lie within the wafer circle
    return all(inside_circle(pt, radius, center) for pt in points)

# ---------------- Detour construction around a rectangle ----------------

def detour_candidates(p_start, p_end, bl, tr, clearance=0.0):
    """
    Construct multiple candidate polyline detours that skirt the rectangle:
    - Above (top side), below (bottom side), left (left side), right (right side).
    Waypoints are placed just outside the rectangle edge with optional clearance.
    """
    rx1, ry1 = bl
    rx2, ry2 = tr
    # Expand rectangle by clearance outward for safe margin
    left_x = rx1 - clearance
    right_x = rx2 + clearance
    bottom_y = ry1 - clearance
    top_y = ry2 + clearance

    candidates = []

    # Go above: vertical up to y=top_y, horizontal traverse at y=top_y, then down
    mid_x = max(min((p_start[0] + p_end[0]) / 2.0, right_x), left_x)
    path_above = [p_start, (p_start[0], top_y), (mid_x, top_y), (p_end[0], top_y), p_end]
    candidates.append(path_above)

    # Go below
    path_below = [p_start, (p_start[0], bottom_y), (mid_x, bottom_y), (p_end[0], bottom_y), p_end]
    candidates.append(path_below)

    # Go left
    mid_y = max(min((p_start[1] + p_end[1]) / 2.0, top_y), bottom_y)
    path_left = [p_start, (left_x, p_start[1]), (left_x, mid_y), (left_x, p_end[1]), p_end]
    candidates.append(path_left)

    # Go right
    path_right = [p_start, (right_x, p_start[1]), (right_x, mid_y), (right_x, p_end[1]), p_end]
    candidates.append(path_right)

    # De-duplicate consecutive equal points
    cleaned = []
    for path in candidates:
        dedup = [path[0]]
        for q in path[1:]:
            if euclidean_distance(dedup[-1], q) > 1e-9:
                dedup.append(q)
        cleaned.append(dedup)
    return cleaned

def polyline_time(points, stage_vel, stage_acc, halt_penalty=True):
    """
    Compute travel time along a polyline with optional halt at direction changes.
    """
    total = 0.0
    n = len(points)
    if n < 2:
        return 0.0
    for i in range(n - 1):
        total += compute_motion_time(euclidean_distance(points[i], points[i + 1]), stage_vel, stage_acc)
    if halt_penalty and n >= 3:
        for i in range(1, n - 1):
            v1 = (points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
            v2 = (points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
            # Halt at any direction change (including minor)
            if abs(v1[0] - v2[0]) > 1e-12 or abs(v1[1] - v2[1]) > 1e-12:
                # A complete halt implies decel to 0 before the turn; we model it as an additive fixed event.
                # Using accel-only decel+accel time approximation: 2 * (v/acc) at max velocity isn't known segment-wise.
                # We use a conservative fixed halt event equal to (stage_vel / stage_acc) for decel + (stage_vel / stage_acc) for accel
                # but since segments might not reach Vmax, we approximate halt as the time to stop and restart: 2 * (stage_vel / stage_acc).
                total += 2.0 * (stage_vel / stage_acc)
    return total

def best_detour(p_start, p_end, rect_bl, rect_tr, wafer_radius, stage_vel, stage_acc, clearance=0.0):
    """
    If the straight segment intersects the rectangle, build detours and pick the minimal-time path
    (that stays inside the wafer circle). Returns (points, time). If no valid detour, returns (None, inf).
    """
    candidates = detour_candidates(p_start, p_end, rect_bl, rect_tr, clearance=clearance)
    best_points, best_time = None, float('inf')
    for pts in candidates:
        # Enforce wafer boundary for all waypoints
        if not path_inside_circle(pts, wafer_radius):
            continue
        t = polyline_time(pts, stage_vel, stage_acc, halt_penalty=True)
        if t < best_time:
            best_time = t
            best_points = pts
    return best_points, best_time

# ---------------- Camera rotation time ----------------

def rotation_time(a1, a2, cam_vel, cam_acc):
    ang = angle_difference(a1, a2)
    return compute_motion_time(ang, cam_vel, cam_acc)

# ---------------- TSP helpers ----------------

def route_cost(route, cost_matrix):
    total = 0.0
    for i in range(len(route) - 1):
        total += cost_matrix[route[i]][route[i + 1]]
    return total

def nearest_neighbor_route(num_nodes, cost_matrix):
    visited = [False] * (num_nodes + 1)
    route = [0]
    visited[0] = True
    current = 0
    for _ in range(num_nodes):
        best_next = None
        best_cost = float('inf')
        for j in range(1, num_nodes + 1):
            if not visited[j] and cost_matrix[current][j] < best_cost:
                best_cost = cost_matrix[current][j]
                best_next = j
        if best_next is not None:
            route.append(best_next)
            visited[best_next] = True
            current = best_next
    return route

def two_optimize(route, cost_matrix):
    n = len(route)
    improved = True
    while improved:
        improved = False
        best_delta = -1e-9
        best_i, best_j = -1, -1
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                delta = -cost_matrix[route[i - 1]][route[i]] + cost_matrix[route[i - 1]][route[j]]
                if j < n - 1:
                    delta += -cost_matrix[route[j]][route[j + 1]] + cost_matrix[route[i]][route[j + 1]]
                if delta < best_delta:
                    best_delta = delta
                    best_i, best_j = i, j
        if best_i != -1:
            route = route[:best_i] + route[best_i:best_j + 1][::-1] + route[best_j + 1:]
            improved = True
    return route

def solve_tsp(num_nodes, cost_matrix, restarts=10):
    best_route, best_cost = None, float('inf')
    route = nearest_neighbor_route(num_nodes, cost_matrix)
    route = two_optimize(route, cost_matrix)
    curr_cost = route_cost(route, cost_matrix)
    if curr_cost < best_cost:
        best_cost, best_route = curr_cost, route[:]
    for _ in range(restarts):
        nodes = list(range(1, num_nodes + 1))
        random.shuffle(nodes)
        route = [0] + nodes
        route = two_optimize(route, cost_matrix)
        curr_cost = route_cost(route, cost_matrix)
        if curr_cost < best_cost:
            best_cost, best_route = curr_cost, route[:]
    return best_route, best_cost

# -------------------------- Main --------------------------

if __name__ == "__main__":
    with open("Input_Milestone4_Testcase2.json") as f:
        data = json.load(f)

    initial_position = data["InitialPosition"]
    initial_angle = float(data["InitialAngle"])
    wafer_diameter = float(data["WaferDiameter"])
    wafer_radius = wafer_diameter / 2.0
    forbidden_zones = data.get("ForbiddenZones", [])

    stage_velocity = data.get("MaxStageVelocity", data.get("StageVelocity", 1.0))
    stage_acceleration = data.get("MaxStageAcceleration", data.get("StageAcceleration", float('inf')))
    camera_velocity = data.get("MaxCameraVelocity", data.get("CameraVelocity", 1.0))
    camera_acceleration = data.get("MaxCameraAcceleration", data.get("CameraAcceleration", float('inf')))

    die_centers = [initial_position]
    die_angles = [initial_angle]
    for die in data["Dies"]:
        die_centers.append(compute_center(die["Corners"]))
        die_angles.append(compute_angle(die["Corners"]))

    # Wafer boundary check for die centers up front
    die_centers = [c for c in die_centers if inside_circle(c, wafer_radius)]
    die_angles = die_angles[:len(die_centers)]
    num_dies = len(die_centers) - 1
    print(f"Number of dies: {num_dies}")

    # Build cost matrix and store intermediate polylines per edge for final output
    cost_matrix = [[0.0] * (num_dies + 1) for _ in range(num_dies + 1)]
    edge_points = [[None] * (num_dies + 1) for _ in range(num_dies + 1)]

    for i in range(num_dies + 1):
        for j in range(num_dies + 1):
            if i == j:
                cost_matrix[i][j] = 0.0
                edge_points[i][j] = [die_centers[i]]
                continue

            p_start = die_centers[i]
            p_end = die_centers[j]

            # If straight line intersects any forbidden rect, compute best detour;
            # otherwise straight line with no intermediate points.
            intersects_any = False
            best_path = [p_start, p_end]
            best_time = polyline_time(best_path, stage_velocity, stage_acceleration, halt_penalty=True)

            for zone in forbidden_zones:
                bl = tuple(zone["BottomLeft"])
                tr = tuple(zone["TopRight"])
                if line_intersects_rect(p_start, p_end, bl, tr):
                    intersects_any = True
                    pts, t = best_detour(
                        p_start, p_end, bl, tr,
                        wafer_radius=wafer_radius,
                        stage_vel=stage_velocity,
                        stage_acc=stage_acceleration,
                        clearance=0.0
                    )
                    if pts is None:
                        # No valid detour inside wafer—mark edge invalid
                        best_path = None
                        best_time = float('inf')
                        break
                    else:
                        # Note: if multiple zones overlap along the segment, you could chain detours.
                        # Here, we conservatively apply detour for the first intersecting zone that yields minimal time.
                        best_path = pts
                        best_time = t
                        # Continue checking other zones; if they also intersect the polyline,
                        # you could iteratively refine pts, but this keeps complexity manageable.

            if best_path is None:
                cost_matrix[i][j] = float('inf')
                edge_points[i][j] = None
            else:
                # Camera rotation time between dies
                rot_t = rotation_time(die_angles[i], die_angles[j], camera_velocity, camera_acceleration)
                # Stage time is best_time; overall leg time is max(stage, camera) to finish both
                leg_time = max(best_time, rot_t)
                cost_matrix[i][j] = leg_time
                edge_points[i][j] = best_path

    # Solve route
    best_route, base_time = solve_tsp(num_dies, cost_matrix, restarts=10)

    # Build full path with intermediate points
    full_points = []
    for k in range(len(best_route) - 1):
        i, j = best_route[k], best_route[k + 1]
        pts = edge_points[i][j]
        if pts is None:
            # Should not happen if base_time is finite, but guard anyway
            continue
        if k == 0:
            full_points.extend(pts)
        else:
            # Avoid duplicating the connecting waypoint
            full_points.extend(pts[1:])

    # Final time (base_time already accounts for per-leg max(stage, camera)).
    total_time = base_time

    output_result = {
        "TotalTime": total_time,
        "Path": full_points  # includes all intermediate points used to skirt forbidden zones
    }

    with open("TestCase_4_2.json", "w") as f:
        json.dump(output_result, f, indent=2)

    print(f"Total time: {total_time:.6f}")
    print(f"Path length (waypoints): {len(full_points)}")
