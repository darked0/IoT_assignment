import tkinter as tk
from tkinter import scrolledtext, ttk
import paho.mqtt.client as mqtt

# --- MQTT CONFIG ---
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC_LOGS = "iot/assignment/edge/logs"
TOPIC_CMD = "iot/assignment/edge/commands"

# --- MQTT FUNCTIONS ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        status_label.config(text="Connected to Broker MQTT", fg="green")
        client.subscribe(TOPIC_LOGS)
    else:
        status_label.config(text=f"Connection Error ({rc})", fg="red")

def on_message(client, userdata, msg):
    # Insert new report
    text = msg.payload.decode('utf-8')
    log_area.insert(tk.END, text + "\n")
    
    # --- LOG ROTATION ---
    MAX_RIGHE = 1000
    rows = int(log_area.index('end-1c').split('.')[0])
    
    if rows > MAX_RIGHE:
        rows_to_delete = rows - MAX_RIGHE
        log_area.delete('1.0', f'{rows_to_delete + 1}.0')
    
    log_area.see(tk.END)

def send_parameter():
    # Command to ESP32
    try:
        val = float(param_entry.get())
        command = f"P:{val}"
        client.publish(TOPIC_CMD, command)
        log_area.insert(tk.END, f">> Command sent: {command}\n", "command")
        log_area.see(tk.END)
    except ValueError:
        log_area.insert(tk.END, ">> Error: Please enter a valid number\n", "errore")

# --- SETUP GRAPHIC INTERFACE (Tkinter) ---
root = tk.Tk()
root.title("IoT Assignment")
root.geometry("600x500")
root.configure(padx=10, pady=10)
root.option_add('*Font', 'Consolas 10')

header = tk.Label(root, text="IoT Edge Dashboard", font=("Helvetica", 14, "bold"))
header.pack(pady=5)

status_label = tk.Label(root, text="Disconnected", fg="red")
status_label.pack()

# Log Area
tk.Label(root, text="Hardware Logs:").pack(anchor="w")
log_area = scrolledtext.ScrolledText(root, width=70, height=20, bg="#1e1e1e", fg="#00ff00")
log_area.pack(pady=5)
log_area.tag_config("command", foreground="cyan")
log_area.tag_config("error", foreground="red")

# Controls (Params change)
control_frame = tk.Frame(root)
control_frame.pack(pady=10, fill="x")

tk.Label(control_frame, text="Anomaly Probability:").pack(side=tk.LEFT, padx=5)
param_entry = tk.Entry(control_frame, width=10)
param_entry.insert(0, "0.05")
param_entry.pack(side=tk.LEFT, padx=5)

send_btn = tk.Button(control_frame, text="Update ESP32", command=send_parameter, bg="#36d700", fg="black")
send_btn.pack(side=tk.LEFT, padx=5)

# --- CLIENT MQTT ---
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(BROKER, PORT, 60)
    client.loop_start() # Excute MQTT loop in background
except Exception as e:
    status_label.config(text=f"Error: {e}", fg="red")

# Start GUI
root.mainloop()