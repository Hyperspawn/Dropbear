import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template_string, request, jsonify
import serial
import serial.tools.list_ports
import threading
import time
import atexit
import logging
import eventlet.queue as queue

# Initialize Flask
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secure_secret_key_here'  # Replace with a secure secret key

# Configure logging to DEBUG to capture all events
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Define the joints for each leg
LEFT_JOINTS = ["outer_calf", "inner_calf", "knee", "hip_pitch", "hip_roll"]
RIGHT_JOINTS = ["outer_calf", "inner_calf", "knee", "hip_pitch", "hip_roll"]

# EMA Smoothing factor for encoder values
EMA_ALPHA = 0.1  # Adjust between 0 (no smoothing) and 1 (no smoothing)

# Torque range
TORQUE_MIN = -400
TORQUE_MAX = 400

# Global Variables for Devices and Port-to-Leg Mapping
port_to_leg = {}  # Mapping from port to leg ('left' or 'right')
device_lock = threading.Lock()  # To synchronize access to devices
devices = {}  # Mapping from leg ('left'/'right') to USBDevice instances

# HTML Template with Bootstrap for UI
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

    <!-- Serial Port Assignment Section -->
    <div class="assignment-section">
        <h3>Assign Serial Ports to Legs</h3>
        <form id="assignmentForm">
            <div class="mb-3">
                <label for="first_port" class="form-label">First Serial Port:</label>
                <select class="form-select" id="first_port" required>
                    <option value="" disabled selected>Select First Port</option>
                </select>
            </div>
            <div class="mb-3">
                <label for="second_port" class="form-label">Second Serial Port:</label>
                <select class="form-select" id="second_port" required>
                    <option value="" disabled selected>Select Second Port</option>
                </select>
            </div>
            <button type="submit" class="btn btn-primary">Initialize Legs</button>
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

    <!-- Include Bootstrap JS -->
    <script
      src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"
    ></script>
    <script>
        document.addEventListener('DOMContentLoaded', function() {
            // Function to fetch available serial ports
            function fetchPorts() {
                fetch('/get_ports')
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        populatePortDropdown('first_port', data.ports);
                        populatePortDropdown('second_port', data.ports);
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
                dropdown.innerHTML = `<option value="" disabled selected>Select ${elementId === 'first_port' ? 'First' : 'Second'} Port</option>`;
                ports.forEach(port => {
                    const option = document.createElement('option');
                    option.value = port;
                    option.text = port;
                    dropdown.appendChild(option);
                });
            }

            // Fetch ports on page load
            fetchPorts();

            // Handle Port Assignment Form Submission
            document.getElementById('assignmentForm').addEventListener('submit', function(e) {
                e.preventDefault();
                const firstPort = document.getElementById('first_port').value;
                const secondPort = document.getElementById('second_port').value;

                if (firstPort === secondPort) {
                    alert('Please select two different ports.');
                    return;
                }

                fetch('/assign_ports', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        ports: [firstPort, secondPort]
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Legs initialized successfully.');
                        document.getElementById('controlInterface').style.display = 'block';
                        document.querySelector('.assignment-section').style.display = 'none';
                        // Start polling for encoder data
                        startEncoderPolling();
                    } else {
                        alert('Error initializing legs: ' + data.message);
                        // Refresh the port list in case of dynamic changes
                        fetchPorts();
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            });

            // Function to fetch encoder data and update UI
            function fetchEncoders() {
                fetch('/get_encoders')
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        const encoders = data.encoders;
                        // Update Left Leg Encoder Values
                        if (encoders.left) {
                            document.getElementById('left_chirality').innerText = `Chirality: ${capitalize(encoders.left.chirality)}`;
                            for (const joint in encoders.left.encoders) {
                                const encoderValue = encoders.left.encoders[joint];
                                const encoderDivId = `left_${joint}_encoder`;
                                const encoderDiv = document.getElementById(encoderDivId);
                                if (encoderDiv) {
                                    encoderDiv.innerText = `Encoder: ${encoderValue !== null ? encoderValue.toFixed(1) : '--'}`;
                                }
                            }
                        }

                        // Update Right Leg Encoder Values
                        if (encoders.right) {
                            document.getElementById('right_chirality').innerText = `Chirality: ${capitalize(encoders.right.chirality)}`;
                            for (const joint in encoders.right.encoders) {
                                const encoderValue = encoders.right.encoders[joint];
                                const encoderDivId = `right_${joint}_encoder`;
                                const encoderDiv = document.getElementById(encoderDivId);
                                if (encoderDiv) {
                                    encoderDiv.innerText = `Encoder: ${encoderValue !== null ? encoderValue.toFixed(1) : '--'}`;
                                }
                            }
                        }
                    } else {
                        console.error('Error fetching encoders:', data.message);
                    }
                })
                .catch((error) => {
                    console.error('Error:', error);
                });
            }

            // Function to capitalize first letter
            function capitalize(string) {
                return string.charAt(0).toUpperCase() + string.slice(1);
            }

            // Function to start polling for encoder data
            function startEncoderPolling() {
                // Immediately fetch once
                fetchEncoders();
                // Then fetch every second
                setInterval(fetchEncoders, 1000);
            }

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

                    // Send impedance command in the format: impedance <legSide> <appendage> <enable> <desiredPosition> <desiredVelocity>
                    // For simplicity, we'll assume enable=1 and desiredVelocity=0
                    const enable = 1;
                    const desiredVelocity = 0;
                    const impedanceCommand = `impedance ${leg} ${joint} ${enable} ${Math.round(setpoint)} ${desiredVelocity}`;
                    
                    // Send the impedance command
                    fetch('/send_command', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            command: impedanceCommand
                        })
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            console.log(`Sent command: ${impedanceCommand}`);
                            const responseDiv = document.getElementById('command_response');
                            responseDiv.textContent += `> ${impedanceCommand}\n`;
                            responseDiv.scrollTop = responseDiv.scrollHeight;
                        } else {
                            alert('Error sending impedance command: ' + data.message);
                            console.error('Error sending impedance command:', data.message);
                        }
                    })
                    .catch((error) => {
                        console.error('Error:', error);
                    });

                    // Additionally, send PID parameters
                    fetch('/set_pid', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            leg: leg,
                            joint: joint,
                            pid_p: pid_p,
                            pid_i: pid_i,
                            pid_d: pid_d,
                            invert: invert
                        })
                    })
                    .then(response => response.json())
                    .then(data => {
                        const responseDiv = document.getElementById('command_response');
                        if (data.success) {
                            const pidCommand = `pid ${joint} ${pid_p} ${pid_i} ${pid_d} ${invert ? 1 : 0}`;
                            console.log(`Sent PID command: ${pidCommand}`);
                            responseDiv.textContent += `> ${pidCommand}\n`;
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
                fetch('/play', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        leg: 'left'
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Play command sent to Left Leg.');
                        console.log(`Play command sent to Left Leg.`);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> play (Left Leg)\n`;
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
                fetch('/play', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        leg: 'right'
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Play command sent to Right Leg.');
                        console.log(`Play command sent to Right Leg.`);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> play (Right Leg)\n`;
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
                fetch('/stop', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({})
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Stop command sent to all legs.');
                        console.log('Stop command sent to all legs:', data.responses);
                        const responseDiv = document.getElementById('command_response');
                        responseDiv.textContent += `> stop\n`;
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

                // Show the assignment section again
                document.querySelector('.assignment-section').style.display = 'block';
                document.getElementById('controlInterface').style.display = 'none';

                // Clear previous responses
                document.getElementById('command_response').textContent = '';

                // Re-initialize devices by sending stop commands
                fetch('/stop', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({})
                })
                .then(() => {
                    // Fetch new list of ports
                    fetchPorts();
                })
                .finally(() => {
                    // Re-enable the refresh button after a short delay
                    setTimeout(() => {
                        const refreshButton = document.getElementById('refresh_button');
                        refreshButton.disabled = false;
                        refreshButton.innerText = 'Refresh';
                    }, 3000);
                });
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

                // Construct the constrain command as per Arduino command handler
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
                        responseDiv.textContent += `> ${command}\n`;
                        for (const [leg, resp] of Object.entries(data.responses)) {
                            responseDiv.textContent += `${leg.capitalize()} Response: ${resp}\n`;
                        }
                        responseDiv.textContent += `\n`;
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

            // Capitalize function added to String prototype for command responses
            String.prototype.capitalize = function() {
                return this.charAt(0).toUpperCase() + this.slice(1);
            };
        });
    </script>
</div>
</body>
</html>
"""

# USBDevice Class
class USBDevice:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.joint_encoder_values = {
            "outer_calf": None,
            "inner_calf": None,
            "knee": None,
            "hip_pitch": None,
            "hip_roll": None
        }
        self.joint_constraints = {
            "outer_calf": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "inner_calf": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "knee": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "hip_pitch": {"min": TORQUE_MIN, "max": TORQUE_MAX},
            "hip_roll": {"min": TORQUE_MIN, "max": TORQUE_MAX}
        }
        self.joint_ema = {
            "outer_calf": None,
            "inner_calf": None,
            "knee": None,
            "hip_pitch": None,
            "hip_roll": None
        }
        self.encoder_lock = threading.Lock()
        self.running = True
        self.serial = None

        # Command and response queues
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()

        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for the serial connection to initialize
            logging.info(f"Connected to {port}")
        except serial.SerialException as e:
            logging.error(f"Error connecting to {port}: {e}")
            self.serial = None

        if self.serial and self.serial.is_open:
            # Start a green thread to handle serial communication
            eventlet.spawn_n(self.read_serial_data)

    def send_command(self, command):
        if self.serial and self.serial.is_open:
            self.command_queue.put(command)
            return True
        else:
            logging.warning(f"Serial port {self.port} is not open.")
            return False

    def send_command_and_wait_response(self, command, expected_prefix=None, timeout=5):
        if not self.serial or not self.serial.is_open:
            logging.error(f"Serial port {self.port} is not open.")
            return None

        # Create an Event to wait for the response
        response_event = eventlet.event.Event()
        self.response_queue.put((expected_prefix, response_event))

        # Send the command
        self.command_queue.put(command)

        try:
            response = response_event.wait(timeout=timeout)
            return response
        except eventlet.timeout.Timeout:
            logging.error(f"Timeout waiting for response to command '{command}' on {self.port}.")
            return None

    def get_chirality(self):
        response = self.send_command_and_wait_response("chirality", expected_prefix="Current chirality:")
        if response:
            chirality = response.split(":")[1].strip().lower()
            if chirality in ['left', 'right', 'center']:
                return chirality
            else:
                logging.error(f"Invalid chirality response from {self.port}: {response}")
                return None
        else:
            return None

    def read_serial_data(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                # Check if there is a command to send
                try:
                    command = self.command_queue.get_nowait()
                    # Send the command followed by newline
                    self.serial.write((command + '\n').encode())
                    self.serial.flush()
                    logging.debug(f"Sent to {self.port}: {command}")
                except queue.Empty:
                    pass

                # Read incoming data
                if self.serial.in_waiting:
                    line = self.serial.readline().decode().strip()
                    if not line:
                        continue
                    logging.debug(f"Received from {self.port}: {line}")

                    # Check if it's a chirality response
                    if line.lower().startswith("current chirality:"):
                        try:
                            expected_prefix, event = self.response_queue.get_nowait()
                            response = line
                            event.send(response)
                            logging.debug(f"Chirality response from {self.port}: {response}")
                        except queue.Empty:
                            logging.warning(f"Received unexpected chirality response without waiting event: {line}")
                    elif ',' in line:
                        # Assume it's encoder data
                        values = line.split(',')
                        expected_joints = ["outer_calf", "inner_calf", "hip_pitch", "knee", "hip_roll"]  # Correct order
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
                            logging.debug(f"Updated encoder data for {self.port}: {self.get_all_encoders()}")
                        else:
                            logging.info(f"Unexpected encoder data format from {self.port}: {line}")
                    else:
                        # Handle other unsolicited messages
                        logging.info(f"Unrecognized message from {self.port}: {line}")

                # Yield control to allow other green threads to run
                eventlet.sleep(0.1)
            except Exception as e:
                logging.error(f"Error reading from {self.port}: {e}")
                break

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

    def set_impedance(self, leg, appendage, enable, desired_position, desired_velocity):
        # Ensure the command aligns with Arduino's expected format
        command = f"impedance {leg} {appendage} {enable} {int(desired_position)} {int(desired_velocity)}"
        success = self.send_command(command)
        return success

    def set_pid(self, leg, joint, pid_p, pid_i, pid_d, invert):
        if joint in self.joint_constraints:
            # Send PID parameters as per Arduino's expected command format
            invert_str = "1" if invert else "0"
            command = f"pid {joint} {pid_p} {pid_i} {pid_d} {invert_str}"
            success = self.send_command(command)
            if success:
                logging.info(f"Set PID for {joint}: P={pid_p}, I={pid_i}, D={pid_d}, invert={invert}")
                return True
            else:
                logging.error(f"Failed to set PID for {joint}")
                return False
        else:
            logging.warning(f"Joint {joint} not recognized.")
            return False

    def close(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            logging.info(f"Closed connection to {self.port}")

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
    global devices, port_to_leg
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    ports = data.get('ports')
    if not ports or not isinstance(ports, list) or len(ports) != 2:
        return jsonify({"success": False, "message": "Please provide exactly two ports."}), 400

    first_port, second_port = ports

    if first_port == second_port:
        return jsonify({"success": False, "message": "Both ports must be different."}), 400

    # Check if ports are available
    available_ports = list_serial_ports()
    if first_port not in available_ports:
        return jsonify({"success": False, "message": f"First port {first_port} is not available."}), 400
    if second_port not in available_ports:
        return jsonify({"success": False, "message": f"Second port {second_port} is not available."}), 400

    # Initialize devices
    with device_lock:
        # Close existing devices if any
        for leg, device in list(devices.items()):
            if device.port in ports:
                device.close()
                del devices[leg]
                del port_to_leg[device.port]

        # Initialize new devices
        device1 = USBDevice(first_port)
        device2 = USBDevice(second_port)

        if not device1.serial or not device1.serial.is_open:
            logging.error(f"Failed to connect to device on {first_port}.")
            device1 = None
        if not device2.serial or not device2.serial.is_open:
            logging.error(f"Failed to connect to device on {second_port}.")
            device2 = None

        if not device1 or not device2:
            logging.error("Error: Could not initialize both devices.")
            # Close any opened serial connections
            if device1:
                device1.close()
            if device2:
                device2.close()
            return jsonify({"success": False, "message": "Failed to initialize both devices."}), 500

        # Send stop commands twice with 2-second intervals
        device1.send_command("stop")
        device2.send_command("stop")
        time.sleep(2)
        device1.send_command("stop")
        device2.send_command("stop")
        time.sleep(2)

        # Get chirality for each device
        chirality1 = device1.get_chirality()
        chirality2 = device2.get_chirality()

        logging.debug(f"Chirality responses: Device1: {chirality1}, Device2: {chirality2}")

        if chirality1 is None or chirality2 is None:
            logging.error("Failed to retrieve chirality from one or both devices.")
            device1.close()
            device2.close()
            return jsonify({"success": False, "message": "Failed to retrieve chirality from devices."}), 500

        # Assign legs based on chirality
        if chirality1 == chirality2:
            logging.error("Both devices reported the same chirality.")
            device1.close()
            device2.close()
            return jsonify({"success": False, "message": "Both devices reported the same chirality. Please ensure one is left and the other is right."}), 500

        if chirality1 == 'left':
            devices['left'] = device1
            devices['right'] = device2
            port_to_leg[first_port] = 'left'
            port_to_leg[second_port] = 'right'
        elif chirality1 == 'right':
            devices['right'] = device1
            devices['left'] = device2
            port_to_leg[first_port] = 'right'
            port_to_leg[second_port] = 'left'
        elif chirality1 == 'center':
            # If a device is center, assign based on chirality2
            if chirality2 == 'left':
                devices['left'] = device2
                devices['right'] = device1
                port_to_leg[second_port] = 'left'
                port_to_leg[first_port] = 'right'
            elif chirality2 == 'right':
                devices['right'] = device2
                devices['left'] = device1
                port_to_leg[second_port] = 'right'
                port_to_leg[first_port] = 'left'
            else:
                logging.error(f"Invalid chirality response from devices: {chirality1}, {chirality2}")
                device1.close()
                device2.close()
                return jsonify({"success": False, "message": "Invalid chirality responses from devices."}), 500
        else:
            logging.error(f"Invalid chirality response from {first_port}: {chirality1}")
            device1.close()
            device2.close()
            return jsonify({"success": False, "message": f"Invalid chirality response from {first_port}: {chirality1}"}), 500

        logging.info(f"Assigned {first_port} as {chirality1} leg and {second_port} as {chirality2} leg.")

    return jsonify({"success": True, "message": "Devices have been initialized and assigned successfully."})

@app.route('/get_encoders', methods=['GET'])
def get_encoders():
    with device_lock:
        encoders = {}
        if 'left' in devices:
            encoders['left'] = {
                'chirality': 'left',
                'encoders': devices['left'].get_all_encoders()
            }
        else:
            encoders['left'] = None
        if 'right' in devices:
            encoders['right'] = {
                'chirality': 'right',
                'encoders': devices['right'].get_all_encoders()
            }
        else:
            encoders['right'] = None

    return jsonify({"success": True, "encoders": encoders})

@app.route('/set_joint', methods=['POST'])
def set_joint():
    global devices, port_to_leg
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
    device = devices.get(leg, None)

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
    global devices
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    device = devices.get(leg, None)

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
    global devices
    responses = {}
    with device_lock:
        for leg, device in devices.items():
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
    global devices
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No data provided."}), 400

    leg = data.get('leg')
    joint = data.get('joint')
    pid_p = data.get('pid_p')
    pid_i = data.get('pid_i')
    pid_d = data.get('pid_d')
    invert = data.get('invert')

    if leg not in ['left', 'right']:
        return jsonify({"success": False, "message": "Invalid leg specified."}), 400

    if joint not in LEFT_JOINTS and joint not in RIGHT_JOINTS:
        return jsonify({"success": False, "message": "Invalid joint specified."}), 400

    if not isinstance(pid_p, (int, float)) or not isinstance(pid_i, (int, float)) or not isinstance(pid_d, (int, float)):
        return jsonify({"success": False, "message": "PID parameters must be numbers."}), 400

    if not isinstance(invert, bool):
        return jsonify({"success": False, "message": "Invert must be a boolean."}), 400

    # Determine which device to send the command to
    device = devices.get(leg, None)

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    # Construct the PID control command
    # Assuming a hypothetical "pid" command: "pid <joint> <P> <I> <D> <invert>"
    command = f"pid {joint} {pid_p} {pid_i} {pid_d} {int(invert)}"
    success = device.send_command(command)

    if success:
        logging.info(f"Sent PID command to {leg} leg: {command}")
        return jsonify({"success": True, "message": f"Command '{command}' sent successfully."})
    else:
        logging.error(f"Failed to send PID command to {leg} leg: {command}")
        return jsonify({"success": False, "message": "Failed to send PID command to device."}), 500

@app.route('/send_command', methods=['POST'])
def send_command():
    global devices
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "No command provided."}), 400

    command = data.get('command')
    if not command or not isinstance(command, str):
        return jsonify({"success": False, "message": "Invalid command."}), 400

    responses = {}
    with device_lock:
        for leg, device in devices.items():
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
    global devices
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
    device = devices.get(leg, None)

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
    global devices
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
    device = devices.get(leg, None)

    if not device:
        return jsonify({"success": False, "message": f"{leg.capitalize()} device is not initialized."}), 400

    constraints = device.get_constraints(joint)

    return jsonify({"success": True, "constraints": constraints})

# Function to list available serial ports
def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    port_list = [port.device for port in ports]
    logging.debug(f"Available serial ports: {port_list}")
    return port_list

# Gracefully close serial connections on exit
def cleanup():
    global devices, port_to_leg
    with device_lock:
        for leg, device in devices.items():
            device.close()
    devices.clear()
    port_to_leg.clear()

atexit.register(cleanup)

# Run the Flask app
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
