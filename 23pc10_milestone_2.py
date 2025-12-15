import json
import math

def minimal_angle_diff(a, b):
    
    diff = abs(a - b) % 90
    return min(diff, abs(90 - diff))

def dfs(cost, vis, last, cnt, velocity, coords, current_camera_angle, dice_angles, rotational_velocity):
    n = len(cost)
    if cnt == n:
        return 0, [coords[last]], current_camera_angle
    minTime = float('inf')
    minPath = []
    minCameraAngle = current_camera_angle

    for city in range(1, n):
        if not vis[city]:
            vis[city] = True

            die_angle = dice_angles[city]
            diff = minimal_angle_diff(current_camera_angle, die_angle)

            # If difference is 0 or 90, no rotation needed
            if diff in (0, 90):
                rotation_time = 0
                new_camera_angle = current_camera_angle
            else:
                rotation_time = diff / rotational_velocity
                new_camera_angle = die_angle

            travel_time = cost[last][city] / velocity
            total_time, path, final_camera_angle = dfs(
                cost, vis, city, cnt + 1, velocity, coords,
                new_camera_angle, dice_angles, rotational_velocity
            )
            total_time += max(travel_time, rotation_time)

            if total_time < minTime:
                minTime = total_time
                minPath = [coords[last]] + path
                minCameraAngle = final_camera_angle

            vis[city] = False

    return minTime, minPath, minCameraAngle

def tsp(cost, sv, coords, current_camera_angle, dice_angles, rotational_velocity):
    n = len(cost)
    vis = [False] * n
    vis[0] = True
    total_time, path, final_angle = dfs(cost, vis, 0, 1, sv, coords,
                                        current_camera_angle, dice_angles, rotational_velocity)
    return total_time, path

# --- Main ---
with open("input_Milestone2_Testcase3.json", "r") as file:
    input_data = json.load(file)

dies = input_data["Dies"]
center_of_dies = [input_data["InitialPosition"]]
angle_of_dies = [input_data["InitialAngle"]]

for j in dies:
    coords_list = j["Corners"]
    center_x = sum(point[0] for point in coords_list) / 4
    center_y = sum(point[1] for point in coords_list) / 4
    center_of_dies.append([center_x, center_y])

    p1, p2 = coords_list[0], coords_list[1]
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    angle_deg = math.degrees(math.atan2(dy, dx))
    if angle_deg < 0:
        angle_deg += 360
    angle_of_dies.append(angle_deg)

print("Die angles:", angle_of_dies)
print("Centers of dies:", center_of_dies)

no_of_dies = len(center_of_dies)
adj_list = [[-1 for _ in range(no_of_dies)] for _ in range(no_of_dies)]

for i in range(no_of_dies):
    for j in range(no_of_dies):
        if i != j and adj_list[i][j] == -1:
            dx = center_of_dies[i][0] - center_of_dies[j][0]
            dy = center_of_dies[i][1] - center_of_dies[j][1]
            dist = math.hypot(dx, dy)
            adj_list[i][j] = dist
            adj_list[j][i] = dist

print("Adjacency list with distances:", adj_list)

optimum_time, path = tsp(adj_list, input_data["StageVelocity"], center_of_dies,
                         input_data["InitialAngle"], angle_of_dies, input_data["CameraVelocity"])

print("Optimal time:", optimum_time)
print("Path:", path)

output = {
    "TotalTime": optimum_time,
    "Path": path
}

with open("TestCase_2_3.json", "w") as outfile:
    json.dump(output, outfile, indent=4)
