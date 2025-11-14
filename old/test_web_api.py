#!/usr/bin/env python3
"""Test web interface API."""
import requests
import json

url = "http://localhost:8080/api/servo_loop_update"
payload = {
    "speeds": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
    "cartesian": False
}

print(f"Testing {url}...")
print(f"Payload: {json.dumps(payload)}")

try:
    response = requests.post(url, json=payload, timeout=5)
    print(f"Status: {response.status_code}")
    print(f"Response: {response.text}")
except Exception as e:
    print(f"Error: {e}")


