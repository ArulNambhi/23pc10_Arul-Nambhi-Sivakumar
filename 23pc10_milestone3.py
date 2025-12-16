import math
import json
import random

def euclidean_distance(point1, point2):
    return math.sqrt(((point1[0] - point2[0])**2) + ((point1[1] - point2[1])**2))

def compute_center(vertices):
    x, y = 0, 0
    for i in range(4):
        x += vertices[i][0]
        y += vertices[i][1]
    return [x / 4, y / 4]

def compute_angle(vertices):
    dx = vertices[1][0] - vertices[0][0]
    dy = vertices[1][1] - vertices[0][1]
    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0:
        angle += 360
    return angle

def angle_difference(angle1, angle2):
    diff = abs(angle2 - angle1) % 360
    if diff > 180:
        diff = 360 - diff
    if diff > 90:
        diff = 180 - diff
    if diff > 45:
        diff = 90 - diff
    return diff

def compute_motion_time(distance, velocity, acceleration):
    if distance <= 0:
        return 0.0
    if velocity <= 0:
        return float('inf')
    if acceleration <= 0:
        return distance / velocity
    accel_distance = (velocity ** 2) / (2 * acceleration)
    if distance <= 2 * accel_distance:
        return 2 * math.sqrt(distance / acceleration)
    else:
        return 2 * (velocity / acceleration) + (distance - 2 * accel_distance) / velocity

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
    best_route = None
    best_cost = float('inf')
    route = nearest_neighbor_route(num_nodes, cost_matrix)
    route = two_optimize(route, cost_matrix)
    curr_cost = route_cost(route, cost_matrix)
    if curr_cost < best_cost:
        best_cost = curr_cost
        best_route = route[:]
    for _ in range(restarts):
        nodes = list(range(1, num_nodes + 1))
        random.shuffle(nodes)
        route = [0] + nodes
        route = two_optimize(route, cost_matrix)
        curr_cost = route_cost(route, cost_matrix)
        if curr_cost < best_cost:
            best_cost = curr_cost
            best_route = route[:]
    return best_route, best_cost

# -------------------------- Main --------------------------
with open("Input_Milestone3_Testcase4.json") as f:
    input_data = json.load(f)

initial_position = input_data["InitialPosition"]
initial_angle = input_data["InitialAngle"]
stage_velocity = input_data.get("MaxStageVelocity", input_data.get("StageVelocity", 1))
stage_acceleration = input_data.get("MaxStageAcceleration", input_data.get("StageAcceleration", float('inf')))
camera_velocity = input_data.get("MaxCameraVelocity", input_data.get("CameraVelocity", 1))
camera_acceleration = input_data.get("MaxCameraAcceleration", input_data.get("CameraAcceleration", float('inf')))

die_centers = [initial_position]
die_angles = [float(initial_angle)]
for die in input_data["Dies"]:
    die_centers.append(compute_center(die["Corners"]))
    die_angles.append(compute_angle(die["Corners"]))

num_dies = len(die_centers) - 1
print(f"Number of dies: {num_dies}")

cost_matrix = [[0.0] * (num_dies + 1) for _ in range(num_dies + 1)]
for i in range(num_dies + 1):
    for j in range(num_dies + 1):
        if i != j:
            distance = euclidean_distance(die_centers[i], die_centers[j])
            translation_time = compute_motion_time(distance, stage_velocity, stage_acceleration)
            angle_diff_val = angle_difference(die_angles[i], die_angles[j])
            rotation_time = compute_motion_time(angle_diff_val, camera_velocity, camera_acceleration)
            cost_matrix[i][j] = max(translation_time, rotation_time)

best_route, total_time = solve_tsp(num_dies, cost_matrix, restarts=10)

print(f"Total time: {total_time}")
print(f"Path length: {len(best_route)}")

output_result = {
    "TotalTime": total_time,
    "Path": [die_centers[idx] for idx in best_route]
}

with open("TestCase_3_4.json", 'w') as f:
    json.dump(output_result, f, indent=2)
