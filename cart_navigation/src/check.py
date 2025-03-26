import yaml
import pprint

# Check if the YAML file is valid and well-structured
waypoint_file = "/home/amen/waypoints.yaml"

try:
    with open(waypoint_file, 'r') as file:
        data = yaml.safe_load(file)

    print("YAML file successfully loaded.")
    pprint.pprint(data)

    if not isinstance(data, dict):
        print("⚠️ The root element of the YAML file is not a dictionary.")
    elif 'waypoints' not in data:
        print("⚠️ The 'waypoints' key is missing in the YAML file.")
    else:
        print("✅ 'waypoints' section found:")
        for section, wps in data['waypoints'].items():
            if not isinstance(wps, list):
                print(f"⚠️ Section '{section}' does not contain a list.")
            elif not wps:
                print(f"⚠️ Section '{section}' is empty.")
            else:
                print(f"✅ Section '{section}' has {len(wps)} waypoint(s). Example:")
                pprint.pprint(wps[0])
except Exception as e:
    print(f"❌ Error reading or parsing the YAML file: {e}")
