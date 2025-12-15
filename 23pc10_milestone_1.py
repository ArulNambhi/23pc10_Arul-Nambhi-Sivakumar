import json

def dfs(cost, vis, last, cnt, velocity, coords):
    n = len(cost)
    if cnt == n:
        return 0, [coords[last]]
    minTime = float('inf')
    minPath = []

    for city in range(1, n):
        if not vis[city]:
            vis[city] = True
            travel_time = cost[last][city] / velocity
            total_time, path = dfs(cost, vis, city, cnt + 1, velocity, coords)
            total_time += travel_time
            if total_time < minTime:
                minTime = total_time
                minPath = [coords[last]] + path
            vis[city] = False

    return minTime, minPath

def tsp(cost, sv, coords):
    n = len(cost)
    vis = [False] * n
    vis[0] = True
    total_time, path = dfs(cost, vis, 0, 1, sv, coords)
    return total_time, path

with open("input_Milestone1_Testcase4.json", "r") as file:
    input_data = json.load(file)

dies = input_data["Dies"]
center_of_dies = [input_data["InitialPosition"]]

for j in dies:
    coords_list = j["Corners"]
    x_sum = sum(point[0] for point in coords_list)
    y_sum = sum(point[1] for point in coords_list)
    center_x = x_sum / 4
    center_y = y_sum / 4
    center_of_dies.append([center_x, center_y])

print("Centers of dies:", center_of_dies)

no_of_dies = len(center_of_dies)
adj_list = [[-1 for _ in range(no_of_dies)] for _ in range(no_of_dies)]

for i in range(no_of_dies):
    for j in range(no_of_dies):
        if i != j and adj_list[i][j] == -1:
            dx = center_of_dies[i][0] - center_of_dies[j][0]
            dy = center_of_dies[i][1] - center_of_dies[j][1]
            dist = (dx ** 2 + dy ** 2) ** 0.5
            adj_list[i][j] = dist
            adj_list[j][i] = dist

print("Adjacency list with distances:", adj_list)


optimun_time, path = tsp(adj_list, input_data["StageVelocity"], center_of_dies)


print("Optimal time:", optimun_time)
print("Path:", path)

output = {
    "TotalTime": optimun_time,
    "Path": path
}

with open("TestCase_1_4.json", "w") as outfile:
    json.dump(output, outfile, indent=4)