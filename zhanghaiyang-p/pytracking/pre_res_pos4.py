import json

result_path = 'res_pos4_net2_15.json'
with open(result_path, 'r') as f:
    view_results = json.load(f)

new_res = []

for i in range(0, len(view_results)-1):
    if i == 0:
        new_res.append(view_results[i])
    else:
        if (view_results[i][0] - view_results[i - 1][0]) < 3:
            pass
        else:
            new_res.append(view_results[i])


# print(new_res)

with open("test_res_pos4_net2_15.json", 'w') as f:
    json.dump(new_res, f)
