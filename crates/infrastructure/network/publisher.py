# publisher.py

import paho.mqtt.client as mqtt
import ssl
import json
import time
import os

# --- Configuration ---
BROKER_ADDRESS = "mqtt.jantaus.com"        # Server's public domain
PORT = 8883                                # TLS port
TOPIC = "device/device1A/firmware"
CA_CERT = "fullchain.pem"                  # Fixed path to certificate
username = "device1A"
password = "device1A"

def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker"""
    if rc == 0:
        print(f"Connected to MQTT broker with result code {rc}")
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")
        print("Connection codes: 0=Success, 1=Incorrect protocol, 2=Invalid client ID, 3=Server unavailable, 4=Bad username/password, 5=Not authorized")

def on_publish(client, userdata, mid):
    """Callback for when a message is published"""
    print(f"✅ Message published with message ID: {mid}")

def on_disconnect(client, userdata, rc):
    """Callback for when the client disconnects"""
    if rc != 0:
        print(f"Unexpected disconnection with result code {rc}")
    else:
        print("Disconnected from MQTT broker")

# Check if certificate file exists
if not os.path.exists(CA_CERT):
    print(f"Certificate file '{CA_CERT}' not found!")
    print(f"Current working directory: {os.getcwd()}")
    print("Please ensure the fullchain.pem file is in the current directory")
    exit(1)

print(f"Using certificate: {CA_CERT}")
print(f"Connecting to {BROKER_ADDRESS}:{PORT}")

# Create client
client = mqtt.Client(client_id="test-publisher")
client.username_pw_set(username, password)

# Set up callbacks
client.on_connect = on_connect
client.on_publish = on_publish
client.on_disconnect = on_disconnect

# Set TLS settings - try with system default certificates first
try:
    # Try with system default certificates
    client.tls_set(ca_certs=None, cert_reqs=ssl.CERT_REQUIRED)
    client.tls_insecure_set(False)
    print("TLS configured with system default certificates")
except Exception as e:
    print(f"Failed to configure TLS with system certificates: {e}")
    try:
        # Fallback to specific certificate file
        client.tls_set(ca_certs=CA_CERT, cert_reqs=ssl.CERT_REQUIRED)
        client.tls_insecure_set(False)
        print("TLS configured with specific certificate file")
    except Exception as e2:
        print(f"Failed to configure TLS with certificate file: {e2}")
        exit(1)

# Connect to broker
try:
    print("Connecting to MQTT broker...")
    client.connect(BROKER_ADDRESS, PORT, 60)
    print("Starting MQTT loop...")
    client.loop_start()
    
    # Wait a bit for connection to establish
    time.sleep(2)
    
    # Check connection status
    if not client.is_connected():
        print("Failed to connect to MQTT broker")
        client.loop_stop()
        exit(1)
    
    # customer data id, first_name, last_name, email, address, password_hash 
    customer_data = [7, "Mike", "Green", "mgreen@gmail.com", "1225 Mile street", "pass2"]

    # Convert to JSON string
    payload = json.dumps(customer_data)
    
    print(f"Publishing message to topic: {TOPIC}")
    print(f"Payload: {payload}")
    
    # Publish the actual payload instead of "Hello world"
    result = client.publish(TOPIC, payload, qos=1, retain=False)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print("Message sent successfully!")
    else:
        print(f"Failed to send message: {result.rc}")
    
    # Wait for publish callback
    time.sleep(2)
    
except Exception as e:
    print(f"Error during MQTT operations: {e}")
finally:
    # Clean up
    print("Disconnecting...")
    client.loop_stop()
    client.disconnect()
    print("Publisher script completed")
