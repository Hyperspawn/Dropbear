import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import threading
import time
import sys
import atexit
import logging

# Initialize Flask and Flask-SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secure_secret_key_here'  # Replace with a secure secret key
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Configure logging to DEBUG to capture all events
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Define the joints for each leg
LEFT_JOINTS = ["outer_calf", "inner_calf", "knee", "hip_pitch", "hip_yaw", "hip_roll"]
RIGHT_JOINTS = ["outer_calf", "inner_calf", "knee", "hip_pitch", "hip_yaw", "hip_roll"]

# EMA Smoothing factor for encoder values
EMA_ALPHA = 0.1  # Adjust between 0 (no smoothing) and 1 (no smoothing)

# Torque range (adjust based on your application's requirements)
TORQUE_MIN = -400
TORQUE_MAX = 400

# Global Variables for Devices and Chirality Assignments
left_device = None
right_device = None
device_lock = threading.Lock()  # To synchronize access to devices
left_chirality = None
right_chirality = None

# HTML Template with Bootstrap and Socket.IO for real-time updates
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robotic Leg Control Interface</title>
    <link
      href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css"
      rel="stylesheet"
    >
    <style>
        .slider-container {
            margin: 20px 0;
        }
        .leg-section {
            border: 1px solid #ccc;
            padding: 20px;
            border-radius: 5px;
            background-color: #f8f9fa;
        }
        .slider-label {
            margin-bottom: 5px;
            font-weight: bold;
        }
        .encoder-values {
            margin-top: 10px;
            font-size: 0.9em;
            color: #555;
        }
        .chirality-label {
            font-size: 1.1em;
            margin-bottom: 15px;
            color: #007bff;
        }
        .impedance-control {
            margin-top: 10px;
        }
        .joint-constraints {
            margin-top: 10px;
        }
        .command-section {
            margin-top: 40px;
        }
        #command_response {
            white-space: pre-wrap;
            background-color: #f1f1f1;
            padding: 10px;
            border-radius: 5px;
            max-height: 300px;
            overflow-y: auto;
        }
        .pid-section {
            margin-top: 15px;
            padding: 10px;
            background-color: #e9ecef;
            border-radius: 5px;
        }
        .assignment-section {
            margin: 20px 0;
            padding: 20px;
            border: 2px dashed #6c757d;
            border-radius: 5px;
            background-color: #f8f9fa;
        }
    </style>
</head>
<body>
<div class="container">
    <h1 class="mt-4">Robotic Leg Control Interface</h1>

    <!-- Serial Port and Chirality Assignment Section -->
    <div class="assignment-section">
        <h3>Assign Serial Ports and Chirality to Legs</h3>
        <form id="assignmentForm">
            <div class="mb-3">
                <label for="left_port" class="form-label">Left Leg Serial Port:</label>
                <select class="form-select" id="left_port" required>
                    <option value="" disabled selected>Select Left Leg Port</option>
                </select>
            </div>
            <div class="mb-3">
                <label for="left_chirality_select" class="form-label">Left Leg Chirality:</label>
                <select class="form-select" id="left_chirality_select" required>
                    <option value="" disabled selected>Select Chirality</option>
                    <option value="left">Left</option>
                    <option value="right">Right</option>
                </select>
            </div>
            <div class="mb-3">
                <label for="right_port" class="form-label">Right Leg Serial Port:</label>
                <select class="form-select" id="right_port" required>
                    <option value="" disabled selected>Select Right Leg Port</option>
                </select>
            </div>
            <div class="mb-3">
                <label for="right_chirality_select" class="form-label">Right Leg Chirality:</label>
                <select class="form-select" id="right_chirality_select" required>
                    <option value="" disabled selected>Select Chirality</option>
                    <option value="left">Left</option>
                    <option value="right">Right</option>
                </select>
            </div>
            <button type="submit" class="btn btn-primary">Assign Ports and Chirality</button>
        </form>
    </div>

    <!-- Control Interface (Hidden Until Ports are Assigned) -->
    <div id="controlInterface" style="display: none;">
        <div class="row mt-4">
            <!-- Left Leg Section -->
            <div class="col-md-6 leg-section">
                <h3>Left Leg</h3>
                <div class="chirality-label" id="left_chirality">Chirality: --</div>
                {% for joint in left_joints %}
                <div class="slider-container">
                    <label class="slider-label" for="left_{{ joint }}">{{ joint.replace('_', ' ').title() }}</label>
                    <input type="range" class="form-range joint-slider" id="left_{{ joint }}" name="{{ joint }}" min="{{ torque_min }}" max="{{ torque_max }}" value="0" data-joint="{{ joint }}" data-leg="left">
                    <span id="left_{{ joint }}_value">0</span> Torque Units
                    <div class="encoder-values" id="left_{{ joint }}_encoder">Encoder: --</div>
                    
                    <!-- PID Control Section -->
                    <div class="pid-section">
                        <h5>PID Control</h5>
                        <div class="mb-3">
                            <label for="left_{{ joint }}_setpoint" class="form-label">Setpoint (Units):</label>
                            <input type="number" class="form-control setpoint" id="left_{{ joint }}_setpoint" data-joint="{{ joint }}" data-leg="left" value="0">
                        </div>
                        <div class="mb-3">
                            <label for="left_{{ joint }}_pid_p" class="form-label">Proportional (P):</label>
                            <input type="number" step="0.1" class="form-control pid-p" id="left_{{ joint }}_pid_p" data-joint="{{ joint }}" data-leg="left" value="1.0">
                        </div>
                        <div class="mb-3">
                            <label for="left_{{ joint }}_pid_i" class="form-label">Integral (I):</label>
                            <input type="number" step="0.1" class="form-control pid-i" id="left_{{ joint }}_pid_i" data-joint="{{ joint }}" data-leg="left" value="0.0">
                        </div>
                        <div class="mb-3">
                            <label for="left_{{ joint }}_pid_d" class="form-label">Derivative (D):</label>
                            <input type="number" step="0.1" class="form-control pid-d" id="left_{{ joint }}_pid_d" data-joint="{{ joint }}" data-leg="left" value="0.0">
                        </div>
                        <div class="form-check form-switch">
                            <input class="form-check-input invert-switch" type="checkbox" id="left_{{ joint }}_invert" data-joint="{{ joint }}" data-leg="left">
                            <label class="form-check-label" for="left_{{ joint }}_invert">Invert Torque Direction</label>
                        </div>
                    </div>
                    
                    <!-- Joint Constraints Section -->
                    <div class="joint-constraints">
                        <button class="btn btn-secondary btn-sm set-constraints" data-joint="{{ joint }}" data-leg="left">Set Constraints</button>
                    </div>
                </div>
                {% endfor %}
            </div>
            <!-- Right Leg Section -->
            <div class="col-md-6 leg-section">
                <h3>Right Leg</h3>
                <div class="chirality-label" id="right_chirality">Chirality: --</div>
                {% for joint in right_joints %}
                <div class="slider-container">
                    <label class="slider-label" for="right_{{ joint }}">{{ joint.replace('_', ' ').title() }}</label>
                    <input type="range" class="form-range joint-slider" id="right_{{ joint }}" name="{{ joint }}" min="{{ torque_min }}" max="{{ torque_max }}" value="0" data-joint="{{ joint }}" data-leg="right">
                    <span id="right_{{ joint }}_value">0</span> Torque Units
                    <div class="encoder-values" id="right_{{ joint }}_encoder">Encoder: --</div>
                    
                    <!-- PID Control Section -->
                    <div class="pid-section">
                        <h5>PID Control</h5>
                        <div class="mb-3">
                            <label for="right_{{ joint }}_setpoint" class="form-label">Setpoint (Units):</label>
                            <input type="number" class="form-control setpoint" id="right_{{ joint }}_setpoint" data-joint="{{ joint }}" data-leg="right" value="0">
                        </div>
                        <div class="mb-3">
                            <label for="right_{{ joint }}_pid_p" class="form-label">Proportional (P):</label>
                            <input type="number" step="0.1" class="form-control pid-p" id="right_{{ joint }}_pid_p" data-joint="{{ joint }}" data-leg="right" value="1.0">
                        </div>
                        <div class="mb-3">
                            <label for="right_{{ joint }}_pid_i" class="form-label">Integral (I):</label>
                            <input type="number" step="0.1" class="form-control pid-i" id="right_{{ joint }}_pid_i" data-joint="{{ joint }}" data-leg="right" value="0.0">
                        </div>
                        <div class="mb-3">
                            <label for="right_{{ joint }}_pid_d" class="form-label">Derivative (D):</label>
                            <input type="number" step="0.1" class="form-control pid-d" id="right_{{ joint }}_pid_d" data-joint="{{ joint }}" data-leg="right" value="0.0">
                        </div>
                        <div class="form-check form-switch">
                            <input class="form-check-input invert-switch" type="checkbox" id="right_{{ joint }}_invert" data-joint="{{ joint }}" data-leg="right">
                            <label class="form-check-label" for="right_{{ joint }}_invert">Invert Torque Direction</label>
                        </div>
                    </div>
                    
                    <!-- Joint Constraints Section -->
                    <div class="joint-constraints">
                        <button class="btn btn-secondary btn-sm set-constraints" data-joint="{{ joint }}" data-leg="right">Set Constraints</button>
                    </div>
                </div>
                {% endfor %}
            </div>
        </div>
        <div class="row mt-4">
            <div class="col text-center">
                <!-- Play Controls -->
                <div class="btn-group" role="group" aria-label="Play Controls">
                    <button id="play_left_button" class="btn btn-success btn-lg">Play Left</button>
                    <button id="play_right_button" class="btn btn-success btn-lg">Play Right</button>
                    <button id="stop_button" class="btn btn-danger btn-lg">Stop All</button>
                    <button id="refresh_button" class="btn btn-warning btn-lg">Refresh</button>
                </div>
            </div>
        </div>
        
        <!-- Command Input Section -->
        <div class="row command-section">
            <div class="col">
                <h3>Send Command</h3>
                <form id="commandForm">
                    <div class="input-group mb-3">
                        <input type="text" class="form-control" placeholder="Enter command" id="command_input" required>
                        <button class="btn btn-primary" type="submit">Send</button>
                    </div>
                </form>
                <h5>Response:</h5>
                <div id="command_response"></div>
            </div>
        </div>
        
        <!-- Modal for Setting Joint Constraints -->
        <div class="modal fade" id="constraintsModal" tabindex="-1" aria-labelledby="constraintsModalLabel" aria-hidden="true">
          <div class="modal-dialog">
            <form id="constraintsForm">
              <div class="modal-content">
                <div class="modal-header">
                  <h5 class="modal-title">Set Joint Constraints</h5>
                  <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
                </div>
                <div class="modal-body">
                    <input type="hidden" id="modal_leg">
                    <input type="hidden" id="modal_joint">
                    <div class="mb-3">
                        <label for="min_angle" class="form-label">Minimum Torque (Units):</label>
                        <input type="number" class="form-control" id="min_angle" required>
                    </div>
                    <div class="mb-3">
                        <label for="max_angle" class="form-label">Maximum Torque (Units):</label>
                        <input type="number" class="form-control" id="max_angle" required>
                    </div>
                </div>
                <div class="modal-footer">
                  <button type="submit" class="btn btn-primary">Save Constraints</button>
                  <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                </div>
              </div>
            </form>
          </div>
        </div>
    </div>

    <!-- Include Socket.IO -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.4/socket.io.min.js"></script>
    <!-- Include Bootstrap JS -->
    <script
      src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"
    ></script>
    <script>
        document.addEventListener('DOMContentLoaded', function() {
            const socket = io();

            // Function to fetch available serial ports
            function fetchPorts() {
                fetch('/get_ports')
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        populatePortDropdown('left_port', data.ports);
                        populatePortDropdown('right_port', data.ports);
                    } else {
                        alert('Error fetching serial ports: ' + data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            }

            // Function to populate port dropdowns
            function populatePortDropdown(elementId, ports) {
                const dropdown = document.getElementById(elementId);
                // Clear existing options except the first
                dropdown.innerHTML = `<option value="" disabled selected>Select ${elementId === 'left_port' ? 'Left' : 'Right'} Leg Port</option>`;
                ports.forEach(port => {
                    const option = document.createElement('option');
                    option.value = port;
                    option.text = port;
                    dropdown.appendChild(option);
                });
            }

            // Fetch ports on page load
            fetchPorts();

            // Handle Port and Chirality Assignment Form Submission
            document.getElementById('assignmentForm').addEventListener('submit', function(e) {
                e.preventDefault();
                const leftPort = document.getElementById('left_port').value;
                const leftChirality = document.getElementById('left_chirality_select').value;
                const rightPort = document.getElementById('right_port').value;
                const rightChirality = document.getElementById('right_chirality_select').value;

                if (leftPort === rightPort) {
                    alert('Left and Right ports must be different.');
                    return;
                }

                // Ensure chirality assignments are valid
                if (leftChirality === rightChirality) {
                    alert('Chirality assignments must be different for Left and Right legs.');
                    return;
                }

                fetch('/assign_ports', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        left_port: leftPort,
                        left_chirality: leftChirality,
                        right_port: rightPort,
                        right_chirality: rightChirality
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Ports and Chirality assigned successfully.');
                        document.getElementById('controlInterface').style.display = 'block';
                        document.querySelector('.assignment-section').style.display = 'none';
                    } else {
                        alert('Error assigning ports and chirality: ' + data.message);
                        // Refresh the port list in case of dynamic changes
                        fetchPorts();
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            // Update encoder values and serial responses upon receiving data
            socket.on('encoder_update', function(data) {
                const device_port = data.device;
                const encoders = data.encoders;
                const port_mapping = data.port_mapping;  // Updated mapping from backend

                const leg = port_mapping[device_port];
                if (!leg) {
                    console.warn(`Unknown device port: ${device_port}`);
                    return;
                }

                // Update chirality labels
                document.getElementById(`${leg}_chirality`).innerText = `Chirality: ${leg.charAt(0).toUpperCase() + leg.slice(1)}`;

                for (const joint in encoders) {
                    const encoderValue = encoders[joint];
                    if (encoderValue !== null && encoderValue !== undefined) {
                        document.getElementById(`${leg}_${joint}_encoder`).innerText = `Encoder: ${encoderValue.toFixed(1)}`;
                    }
                }
            });

            // Listen for general serial responses and display them
            socket.on('serial_response', function(data) {
                const responseDiv = document.getElementById('command_response');
                responseDiv.textContent += `\n< [${data.device}] ${data.data}\n`;
                responseDiv.scrollTop = responseDiv.scrollHeight;
                console.log(`Received from ${data.device}: ${data.data}`);
            });

            // Handle joint slider input (Torque Control)
            const sliders = document.querySelectorAll('.joint-slider');
            sliders.forEach(function(slider) {
                slider.addEventListener('input', function() {
                    const joint = this.getAttribute('data-joint');
                    const leg = this.getAttribute('data-leg');
                    const value = this.value;
                    document.getElementById(`${leg}_${joint}_value`).innerText = value;
                    // Send torque command in the format: torque <legSide> <appendage> <torqueValue>
                    const command = `torque ${leg} ${joint} ${parseInt(value)}`;
                    fetch('/send_command', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            command: command
                        })
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            console.log(`Sent command: ${command}`);
                            const responseDiv = document.getElementById('command_response');
                            responseDiv.textContent += `> ${command}\n`;
                            responseDiv.scrollTop = responseDiv.scrollHeight;
                        } else {
                            alert('Error sending torque command: ' + data.message);
                            console.error('Error sending torque command:', data.message);
                        }
                    })
                    .catch((error) => {
                        console.error('Error:', error);
                    });
                });
            });

            // Handle PID control inputs
            const pidInputs = document.querySelectorAll('.setpoint, .pid-p, .pid-i, .pid-d, .invert-switch');
            pidInputs.forEach(function(input) {
                input.addEventListener('change', function() {
                    const joint = this.getAttribute('data-joint');
                    const leg = this.getAttribute('data-leg');

                    // Gather PID parameters and setpoint
                    const setpoint = parseFloat(document.getElementById(`${leg}_${joint}_setpoint`).value);
                    const pid_p = parseFloat(document.getElementById(`${leg}_${joint}_pid_p`).value);
                    const pid_i = parseFloat(document.getElementById(`${leg}_${joint}_pid_i`).value);
                    const pid_d = parseFloat(document.getElementById(`${leg}_${joint}_pid_d`).value);
                    const invert = document.getElementById(`${leg}_${joint}_invert`).checked;

                    // Construct the impedance command as per serial command handler
                    const command = `impedance ${leg} ${joint} 1 ${parseInt(setpoint)} 0 ${pid_p} ${pid_i} ${pid_d} ${invert ? 1 : 0}`;

                    fetch('/send_command', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            command: command
                        })
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            console.log(`Sent command: ${command}`);
                            const responseDiv = document.getElementById('command_response');
                            responseDiv.textContent += `> ${command}\n`;
                            responseDiv.scrollTop = responseDiv.scrollHeight;
                        } else {
                            alert('Error sending PID command: ' + data.message);
                            console.error('Error sending PID command:', data.message);
                        }
                    })
                    .catch((error) => {
                        console.error('Error:', error);
                    });
                });
            });

            // Play buttons
            document.getElementById('play_left_button').addEventListener('click', function() {
                const command = 'play left';
                fetch('/send_command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        command: `play left`
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Play command sent to Left Leg.');
                        console.log(`Play command sent to Left Leg: ${command}`);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> ${command}\n`;
                        responseDiv.scrollTop = responseDiv.scrollHeight;
                    } else {
                        alert('Error: ' + data.message);
                        console.error('Error sending play command:', data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            document.getElementById('play_right_button').addEventListener('click', function() {
                const command = 'play right';
                fetch('/send_command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        command: `play right`
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Play command sent to Right Leg.');
                        console.log(`Play command sent to Right Leg: ${command}`);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> ${command}\n`;
                        responseDiv.scrollTop = responseDiv.scrollHeight;
                    } else {
                        alert('Error: ' + data.message);
                        console.error('Error sending play command:', data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            // Stop button
            document.getElementById('stop_button').addEventListener('click', function() {
                const command = 'stop';
                fetch('/send_command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        command: `stop`
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Stop command sent to all legs.');
                        console.log('Stop command sent to all legs:', data.responses);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> ${command}\n`;
                        responseDiv.scrollTop = responseDiv.scrollHeight;
                    } else {
                        alert('Error: ' + data.message);
                        console.error('Error sending stop command:', data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            // Refresh button
            document.getElementById('refresh_button').addEventListener('click', function() {
                if (!confirm('Are you sure you want to refresh and reassign devices? This will stop all current operations.')) {
                    return;
                }

                // Disable the refresh button to prevent multiple clicks
                this.disabled = true;
                this.innerText = 'Refreshing...';

                // Fetch new list of ports
                fetchPorts();

                // Optionally, you can reset UI elements or take other actions here

                // Re-enable the refresh button after a short delay
                setTimeout(() => {
                    const refreshButton = document.getElementById('refresh_button');
                    refreshButton.disabled = false;
                    refreshButton.innerText = 'Refresh';
                }, 3000);
            });

            // Set Constraints Button
            const setConstraintsButtons = document.querySelectorAll('.set-constraints');
            setConstraintsButtons.forEach(function(button) {
                button.addEventListener('click', function() {
                    const joint = this.getAttribute('data-joint');
                    const leg = this.getAttribute('data-leg');
                    // Populate modal with current constraints
                    fetch('/get_constraints', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            leg: leg,
                            joint: joint
                        })
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            document.getElementById('modal_leg').value = leg;
                            document.getElementById('modal_joint').value = joint;
                            document.getElementById('min_angle').value = data.constraints.min;
                            document.getElementById('max_angle').value = data.constraints.max;
                            // Show modal
                            var constraintsModal = new bootstrap.Modal(document.getElementById('constraintsModal'));
                            constraintsModal.show();
                        } else {
                            alert('Error fetching constraints: ' + data.message);
                            console.error('Error fetching constraints:', data.message);
                        }
                    })
                    .catch((error) => {
                        console.error('Error:', error);
                    });
                });
            });

            // Handle Constraints Form Submission
            document.getElementById('constraintsForm').addEventListener('submit', function(e) {
                e.preventDefault();
                const leg = document.getElementById('modal_leg').value;
                const joint = document.getElementById('modal_joint').value;
                const minAngle = parseInt(document.getElementById('min_angle').value);
                const maxAngle = parseInt(document.getElementById('max_angle').value);

                if (minAngle > maxAngle) {
                    alert('Minimum torque cannot be greater than maximum torque.');
                    return;
                }

                // Construct the constrain command as per serial command handler
                const command = `constrain ${joint} ${minAngle} ${maxAngle}`;

                fetch('/send_command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        command: command
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Joint constraints updated successfully.');
                        console.log(`Set constraints for ${leg} leg, ${joint}: min=${minAngle}, max=${maxAngle}`);
                        // Hide modal
                        var constraintsModal = bootstrap.Modal.getInstance(document.getElementById('constraintsModal'));
                        constraintsModal.hide();
                        // Display the command in the response div
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> ${command}\n`;
                        responseDiv.scrollTop = responseDiv.scrollHeight;
                    } else {
                        alert('Error setting constraints: ' + data.message);
                        console.error('Error setting constraints:', data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            // Command Input Form
            document.getElementById('commandForm').addEventListener('submit', function(e) {
                e.preventDefault();
                const command = document.getElementById('command_input').value.trim();
                if (command === "") {
                    alert('Please enter a command.');
                    return;
                }

                fetch('/send_command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        command: command
                    })
                })
                .then(response => response.json())
                .then(data => {
                    const responseDiv = document.getElementById('command_response');
                    if (data.success) {
                        responseDiv.textContent += `> ${command}\nLeft Response: ${data.responses.left}\nRight Response: ${data.responses.right}\n\n`;
                        console.log(`Sent command: ${command}`);
                    } else {
                        responseDiv.textContent += `> ${command}\nError: ${data.message}\n\n`;
                        console.error(`Error sending command '${command}':`, data.message);
                    }
                    // Scroll to bottom
                    responseDiv.scrollTop = responseDiv.scrollHeight;
                    // Clear input
                    document.getElementById('command_input').value = "";
                })
                .catch((error) => {
                    console.error('Error:', error);
                    const responseDiv = document.getElementById('command_response');
                    responseDiv.textContent += `> ${command}\nError: ${error}\n\n`;
                    responseDiv.scrollTop = responseDiv.scrollHeight;
                });
            });
        });
    </script>
</div>
</body>
</html>
"""

# USBDevice Class
class USBDevice:
    def __init__(self, port, chirality, baudrate=115200, timeout=1):
        self.port = port
        self.chirality = chirality  # 'left' or 'right'
        self.baudrate = baudrate
        self.timeout = timeout
        self.lock = threading.Lock()
        self.joint_encoder_values = {
            "outer_calf": None,
            "inner_calf": None,
            "knee": None,
            "hip_pitch": None,
            "hip_yaw": None,
            "hip_roll": None
        }
        self.joint_constraints = {
            "outer_calf": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "inner_calf": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "knee": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "hip_pitch": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "hip_yaw": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "hip_roll": {"min": TORQUE_MIN, "max": TORQUE_MAX}
        }
        self.joint_ema = {
            "outer_calf": None,
            "inner_calf": None,
            "knee": None,
            "hip_pitch": None,
            "hip_yaw": None,
            "hip_roll": None
        }
        self.encoder_lock = threading.Lock()
        self.running = True
        self.serial = None

        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for the serial connection to initialize
            logging.info(f"Connected to {port} with chirality {chirality}")
        except serial.SerialException as e:
            logging.error(f"Error connecting to {port}: {e}")
            self.serial = None

        if self.serial and self.serial.is_open:
            # Start a thread to read serial data
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()

    def send_command(self, command):
        with self.lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.write((command + '\n').encode())
                    self.serial.flush()
                    logging.debug(f"Sent to {self.port}: {command}")
                    return True
                except serial.SerialException as e:
                    logging.error(f"Serial communication error on {self.port}: {e}")
                    return False
            else:
                logging.warning(f"Serial port {self.port} is not open.")
                return False

    def read_serial_data(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                line = self.serial.readline().decode().strip()
                if not line:
                    continue
                values = line.split(',')

                # Define expected joints
                expected_joints = ["outer_calf", "inner_calf", "knee", "hip_pitch", "hip_yaw", "hip_roll"]
                
                if len(values) == len(expected_joints):
                    try:
                        encoder_values = [float(value) for value in values]
                    except ValueError as e:
                        logging.error(f"Error converting values to float from {self.port}: {line} - {e}")
                        continue

                    with self.encoder_lock:
                        for joint, value in zip(expected_joints, encoder_values):
                            if self.joint_ema[joint] is None:
                                self.joint_ema[joint] = value
                            else:
                                self.joint_ema[joint] = EMA_ALPHA * value + (1 - EMA_ALPHA) * self.joint_ema[joint]
                            self.joint_encoder_values[joint] = self.joint_ema[joint]

                    # Emit the updated encoder values to the web clients
                    logging.debug(f"Emitting encoder data for {self.port}: {self.get_all_encoders()}")
                    socketio.emit('encoder_update', {
                        'device': self.port,
                        'encoders': self.get_all_encoders(),
                        'port_mapping': get_port_mapping()
                    }, to='/')  # Changed from broadcast=True to to='/'
                else:
                    # Handle general serial responses if the data doesn't match encoder format
                    logging.info(f"Received from {self.port}: {line}")
                    socketio.emit('serial_response', {
                        'device': self.port,
                        'data': line
                    }, to='/')  # Changed from broadcast=True to to='/'
            except serial.SerialException as e:
                logging.error(f"Serial read error on {self.port}: {e}")
                break
            except UnicodeDecodeError:
                logging.error(f"Unicode decode error on {self.port}. Ignoring line.")
                continue

    def get_encoder_values(self, joint):
        with self.encoder_lock:
            return self.joint_encoder_values.get(joint, None)

    def get_all_encoders(self):
        with self.encoder_lock:
            return self.joint_encoder_values.copy()

    def get_constraints(self, joint):
        return self.joint_constraints.get(joint, {"min": TORQUE_MIN, "max": TORQUE_MAX})

    def set_constraints(self, joint, min_angle, max_angle):
        if joint in self.joint_constraints:
            self.joint_constraints[joint]['min'] = min_angle
            self.joint_constraints[joint]['max'] = max_angle
            # Send command to Arduino to set constraints
            command = f"constrain {joint} {int(min_angle)} {int(max_angle)}"
            success = self.send_command(command)
            if success:
                logging.info(f"Set constraints for {joint}: min={min_angle}, max={max_angle}")
                return True
            else:
                logging.error(f"Failed to set constraints for {joint}")
                return False
        else:
            logging.warning(f"Joint {joint} not recognized.")
            return False

    def set_pid(self, joint, setpoint, pid_p, pid_i, pid_d, invert):
        if joint in self.joint_constraints:
            # Invert logic: multiply torque values by -1 if invert is True
            invert_str = "1" if invert else "0"
            # Assuming the Arduino expects the impedance command with PID parameters and inversion
            command = f"impedance {self.chirality} {joint} 1 {int(setpoint)} 0 {pid_p} {pid_i} {pid_d} {invert_str}"
            success = self.send_command(command)
            if success:
                logging.info(f"Set PID for {joint}: setpoint={setpoint}, P={pid_p}, I={pid_i}, D={pid_d}, invert={invert}")
                return True
            else:
                logging.error(f"Failed to set PID for {joint}")
                return False
        else:
            logging.warning(f"Joint {joint} not recognized.")
            return False

    def calibrate_direction(self, joint):
        if joint in self.joint_constraints:
            command = f"calibrateDirection {joint}"
            success = self.send_command(command)
            if success:
                logging.info(f"Calibrated direction for {joint}")
                return True
            else:
                logging.error(f"Failed to calibrate direction for {joint}")
                return False
        else:
            logging.warning(f"Joint {joint} not recognized.")
            return False

    def close(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            logging.info(f"Closed connection to {self.port}")

# Function to get current port mapping
def get_port_mapping():
    mapping = {}
    with device_lock:
        if left_device:
            mapping[left_device.port] = left_device.chirality
        if right_device:
            mapping[right_device.port] = right_device.chirality
    return mapping

# Flask Routes

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, left_joints=LEFT_JOINTS, right_joints=RIGHT_JOINTS, torque_min=TORQUE_MIN, torque_max=TORQUE_MAX)

@app.route('/get_ports', methods=['GET'])
def get_ports():
    try:
        ports = list_serial_ports()
        logging.debug(f"Available serial ports: {ports}")
        return jsonify({"success": True, "ports": ports})
    except Exception as e:
        logging.error(f"Error listing serial ports: {e}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/assign_ports', methods=['POST'])
def assign_ports():
    global left_device, right_device, left_chirality, right_chirality
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    left_port = data.get('left_port')
    left_chirality = data.get('left_chirality')
    right_port = data.get('right_port')
    right_chirality = data.get('right_chirality')

    if not left_port or not right_port or not left_chirality or not right_chirality:
        return jsonify({"success": False, "message": "All fields must be provided."}), 400

    if left_port == right_port:
        return jsonify({"success": False, "message": "Left and Right ports must be different."}), 400

    if left_chirality == right_chirality:
        return jsonify({"success": False, "message": "Chirality assignments must be different for Left and Right legs."}), 400

    # Check if ports are available
    available_ports = list_serial_ports()
    if left_port not in available_ports:
        return jsonify({"success": False, "message": f"Left port {left_port} is not available."}), 400
    if right_port not in available_ports:
        return jsonify({"success": False, "message": f"Right port {right_port} is not available."}), 400

    # Initialize devices
    with device_lock:
        # Close existing devices if any
        if left_device:
            left_device.close()
            left_device = None
        if right_device:
            right_device.close()
            right_device = None

        # Initialize new devices
        left_device = USBDevice(left_port, left_chirality)
        right_device = USBDevice(right_port, right_chirality)

        # Check if both devices are successfully connected
        if not left_device.serial or not left_device.serial.is_open:
            logging.error(f"Failed to connect to left device on {left_port}.")
            left_device = None
        if not right_device.serial or not right_device.serial.is_open:
            logging.error(f"Failed to connect to right device on {right_port}.")
            right_device = None

        if not left_device or not right_device:
            logging.error("Error: Could not initialize both left and right devices.")
            # Close any opened serial connections
            if left_device:
                left_device.close()
            if right_device:
                right_device.close()
            return jsonify({"success": False, "message": "Failed to initialize both devices."}), 500

        logging.info(f"Left device: {left_device.port} with chirality {left_device.chirality}")
        logging.info(f"Right device: {right_device.port} with chirality {right_device.chirality}")

    return jsonify({"success": True, "message": "Devices initialized successfully."})

@app.route('/set_joint', methods=['POST'])
def set_joint():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    joint = data.get('joint')
    value = data.get('value')

    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    if joint not in LEFT_JOINTS and joint not in RIGHT_JOINTS:
        return jsonify({"success": False, "message": "Invalid joint specified."}), 400

    if not isinstance(value, int) or not (TORQUE_MIN <= value <= TORQUE_MAX):
        return jsonify({"success": False, "message": f"Invalid value for joint. Must be between {TORQUE_MIN} and {TORQUE_MAX}."}), 400

    # Determine which device to send the command to
    device = left_device if leg == 'left' else right_device

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    # Construct the torque command, e.g., "torque left knee 100"
    command = f"torque {leg} {joint} {value}"
    success = device.send_command(command)

    if success:
        logging.info(f"Sent torque command to {leg} leg: {command}")
        return jsonify({"success": True, "message": f"Command '{command}' sent successfully."})
    else:
        logging.error(f"Failed to send torque command to {leg} leg: {command}")
        return jsonify({"success": False, "message": "Failed to send command to device."}), 500

@app.route('/play', methods=['POST'])
def play():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    device = left_device if leg == 'left' else right_device

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    success = device.send_command("play")
    if success:
        logging.info(f"Sent 'play' command to {leg} leg.")
        return jsonify({"success": True, "message": f"Play command sent to {leg} leg."})
    else:
        logging.error(f"Failed to send 'play' command to {leg} leg.")
        return jsonify({"success": False, "message": f"Failed to send play command to {leg} leg."}), 500

@app.route('/stop', methods=['POST'])
def stop():
    global left_device, right_device
    responses = {}
    with device_lock:
        for leg, device in [('left', left_device), ('right', right_device)]:
            if device:
                success = device.send_command("stop")
                if success:
                    responses[leg] = "Stop command sent."
                    logging.info(f"Sent 'stop' command to {leg} leg.")
                else:
                    responses[leg] = "Failed to send stop command."
                    logging.error(f"Failed to send 'stop' command to {leg} leg.")
            else:
                responses[leg] = "Device not initialized."
                logging.warning(f"{leg.capitalize()} device not initialized.")
    return jsonify({"success": True, "responses": responses})

@app.route('/set_pid', methods=['POST'])
def set_pid():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    joint = data.get('joint')
    setpoint = data.get('setpoint')
    pid_p = data.get('pid_p')
    pid_i = data.get('pid_i')
    pid_d = data.get('pid_d')
    invert = data.get('invert')

    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    if joint not in LEFT_JOINTS and joint not in RIGHT_JOINTS:
        return jsonify({"success": False, "message": "Invalid joint specified."}), 400

    if not isinstance(setpoint, (int, float)):
        return jsonify({"success": False, "message": "Setpoint must be a number."}), 400

    if not all(isinstance(x, (int, float)) for x in [pid_p, pid_i, pid_d]):
        return jsonify({"success": False, "message": "PID parameters must be numbers."}), 400

    if not isinstance(invert, bool):
        return jsonify({"success": False, "message": "Invert must be a boolean."}), 400

    # Determine which device to send the command to
    device = left_device if leg == 'left' else right_device

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    # Construct the PID control command
    # Example: impedance left knee 1 180 0 1.0 0.0 0.0 0
    invert_str = "1" if invert else "0"
    command = f"impedance {leg} {joint} 1 {int(setpoint)} 0 {pid_p} {pid_i} {pid_d} {invert_str}"
    success = device.send_command(command)

    if success:
        logging.info(f"Sent PID command to {leg} leg: {command}")
        return jsonify({"success": True, "message": f"Command '{command}' sent successfully."})
    else:
        logging.error(f"Failed to send PID command to {leg} leg: {command}")
        return jsonify({"success": False, "message": "Failed to send PID command to device."}), 500

@app.route('/send_command', methods=['POST'])
def send_command():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No command provided."}), 400

    command = data.get('command')
    if not command or not isinstance(command, str):
        return jsonify({"success": False, "message": "Invalid command."}), 400

    responses = {}
    with device_lock:
        for leg, device in [('left', left_device), ('right', right_device)]:
            if device:
                success = device.send_command(command)
                if success:
                    responses[leg] = f"Command '{command}' sent successfully."
                    logging.info(f"Sent custom command to {leg} leg: {command}")
                else:
                    responses[leg] = f"Failed to send command '{command}'."
                    logging.error(f"Failed to send custom command to {leg} leg: {command}")
            else:
                responses[leg] = "Device not initialized."
                logging.warning(f"{leg.capitalize()} device not initialized.")

    return jsonify({"success": True, "responses": responses})

@app.route('/set_constraints', methods=['POST'])
def set_constraints():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    joint = data.get('joint')
    min_angle = data.get('min_angle')
    max_angle = data.get('max_angle')

    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    if joint not in LEFT_JOINTS and joint not in RIGHT_JOINTS:
        return jsonify({"success": False, "message": "Invalid joint specified."}), 400

    if not isinstance(min_angle, (int, float)) or not isinstance(max_angle, (int, float)):
        return jsonify({"success": False, "message": "Min and Max angles must be numbers."}), 400

    if not (TORQUE_MIN <= min_angle <= TORQUE_MAX and TORQUE_MIN <= max_angle <= TORQUE_MAX):
        return jsonify({"success": False, "message": f"Angles must be between {TORQUE_MIN} and {TORQUE_MAX}."}), 400

    # Determine which device to send the command to
    device = left_device if leg == 'left' else right_device

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    # Construct the constrain command
    command = f"constrain {joint} {int(min_angle)} {int(max_angle)}"
    success = device.send_command(command)

    if success:
        # Update the device's internal constraints
        device.joint_constraints[joint]['min'] = int(min_angle)
        device.joint_constraints[joint]['max'] = int(max_angle)
        logging.info(f"Sent constrain command to {leg} leg: {command}")
        return jsonify({"success": True, "message": f"Command '{command}' sent successfully."})
    else:
        logging.error(f"Failed to send constrain command to {leg} leg: {command}")
        return jsonify({"success": False, "message": "Failed to send constrain command to device."}), 500

@app.route('/get_constraints', methods=['POST'])
def get_constraints():
    global left_device, right_device
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    joint = data.get('joint')

    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    if joint not in LEFT_JOINTS and joint not in RIGHT_JOINTS:
        return jsonify({"success": False, "message": "Invalid joint specified."}), 400

    # Determine which device
    device = left_device if leg == 'left' else right_device

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    constraints = device.get_constraints(joint)

    return jsonify({"success": True, "constraints": constraints})

@app.route('/refresh', methods=['POST'])
def refresh():
    """
    Refreshes the device assignments by stopping all devices and re-initializing them.
    """
    global left_device, right_device, left_chirality, right_chirality
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    left_port = data.get('left_port')
    left_chirality = data.get('left_chirality')
    right_port = data.get('right_port')
    right_chirality = data.get('right_chirality')

    if not left_port or not right_port or not left_chirality or not right_chirality:
        return jsonify({"success": False, "message": "All fields must be provided."}), 400

    if left_port == right_port:
        return jsonify({"success": False, "message": "Left and Right ports must be different."}), 400

    if left_chirality == right_chirality:
        return jsonify({"success": False, "message": "Chirality assignments must be different for Left and Right legs."}), 400

    # Check if ports are available
    available_ports = list_serial_ports()
    if left_port not in available_ports:
        return jsonify({"success": False, "message": f"Left port {left_port} is not available."}), 400
    if right_port not in available_ports:
        return jsonify({"success": False, "message": f"Right port {right_port} is not available."}), 400

    # Initialize devices
    with device_lock:
        # Close existing devices if any
        if left_device:
            left_device.close()
            left_device = None
        if right_device:
            right_device.close()
            right_device = None

        # Initialize new devices
        left_device = USBDevice(left_port, left_chirality)
        right_device = USBDevice(right_port, right_chirality)

        # Check if both devices are successfully connected
        if not left_device.serial or not left_device.serial.is_open:
            logging.error(f"Failed to connect to left device on {left_port}.")
            left_device = None
        if not right_device.serial or not right_device.serial.is_open:
            logging.error(f"Failed to connect to right device on {right_port}.")
            right_device = None

        if not left_device or not right_device:
            logging.error("Error: Could not initialize both left and right devices.")
            # Close any opened serial connections
            if left_device:
                left_device.close()
            if right_device:
                right_device.close()
            return jsonify({"success": False, "message": "Failed to initialize both devices."}), 500

        logging.info(f"Left device: {left_device.port} with chirality {left_device.chirality}")
        logging.info(f"Right device: {right_device.port} with chirality {right_device.chirality}")

    return jsonify({"success": True, "message": "Devices have been refreshed and reassigned successfully."})

# Function to list available serial ports
def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    port_list = [port.device for port in ports]
    logging.debug(f"Available serial ports: {port_list}")
    return port_list

# Gracefully close serial connections on exit
def cleanup():
    global left_device, right_device
    with device_lock:
        if left_device:
            left_device.close()
        if right_device:
            right_device.close()

atexit.register(cleanup)

# Run the Flask app with SocketIO
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
