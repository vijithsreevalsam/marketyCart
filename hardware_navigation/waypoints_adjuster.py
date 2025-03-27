import yaml

# === CONFIGURATION ===
origin_x = -10.1
origin_y = -10.0

# Replace this with your actual file path
input_file = "original_waypoints.yaml"
output_file = "adjusted_waypoints.yaml"

# === LOAD & ADJUST WAYPOINTS ===
with open(input_file, 'r') as f:
    data = yaml.safe_load(f)

adjusted_data = {'waypoints': {}}

for section, waypoints in data.get('waypoints', {}).items():
    adjusted_data['waypoints'][section] = []
    for wp in waypoints:
        pos = wp['pose']['position']
        new_wp = {
            'pose': {
                'position': {
                    'x': pos['x'] - origin_x,
                    'y': pos['y'] - origin_y,
                    'z': pos.get('z', 0.0)
                },
                'orientation': wp['pose']['orientation']
            }
        }
        adjusted_data['waypoints'][section].append(new_wp)

# === PRINT RESULT ===
print("\nAdjusted Waypoints:\n")
print(yaml.dump(adjusted_data, sort_keys=False))

# === OPTIONAL: SAVE TO FILE ===
with open(output_file, 'w') as f:
    yaml.dump(adjusted_data, f, sort_keys=False)

print(f"\n✅ Adjusted waypoints saved to: {output_file}")
