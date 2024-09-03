<img src="https://github.com/Hyperspawn/Dropbear/blob/main/Media/Flows/dropbear.png" width="1024">
![fract](https://github.com/user-attachments/assets/9b977d62-3cf2-4aa8-bfd4-e3401927e4af)

Welcome to the official repository of the Dropbear Humanoid Robot! Developed by Hyperspawn & [Pointblank](https://www.pointblankllc.com/). Dropbear is an advanced humanoid robot designed to operate in varied environments, showcasing agility, precision, and intelligence.

## Overview
Dropbear Humanoid is a cutting-edge robot featuring advanced AI and superior hardware, designed for seamless human interaction, exploration, and task execution in extreme conditions. This project encapsulates our vision at Hyperspawn Robotics for the future of humanoid robots.

## Technical Specifications
- **Height:** 6 feet and 2.02 inches (1880 mm)
- **Weight:** 45 kg
- **Actuators:** Brushless Lightweight DC Servo Motor - Precise Planetary Rotation MCX500 Driver 
- **Sensors:** Vision, Audio, IMU, Pressure.

## Hardware Components
Detailed specifications, CAD models, and schematics of the hardware components can be found here:
- Actuator [MyActuator RMD-X8, X-10](https://www.myactuator.com/product-page/rmd-x8-pro)
- Sensors
  - Vision Sensors: Cameras: For visual perception, object recognition, and navigation.
  - IMU (Inertial Measurement Unit): Combining accelerometers and gyroscopes for orientation and balance.
  - Pressure Sensors: To detect the force exerted on the robot, aiding in gripping and interaction with objects.
  - Audio Sensors - Microphones: For voice recognition and environmental sound detection.
- Control Units
  - [Nvidia Jetson Orin](https://www.nvidia.com/en-in/autonomous-machines/embedded-systems/jetson-orin/)
  - Custom FPGAs
- Body Frame Material: 3D-printed ABS, Extruded Aluminium

## Software Components

### Autonomous mode
#### Vision Large Language Models (VLLMs)
Dropbear uses Large Language Models for it's ability to process and understand human language. Vision LLMs extend the capabilities of traditional LLMs by integrating visual data processing, enabling dropbear to not just "see" but understand and interpret visual information in a contextually relevant manner.
![image](https://github.com/Hyperspawn/Dropbear/assets/37779762/d34ad4ca-2385-4377-8852-23f5e13de1cf)

- **Natural Language Understanding**: Dropbear understands spoken or written instructions.
- **Object Recognition**: Dropbear can identify and categorize objects within it's visual field.
- **Navigation**: Dropbear can navigate complex environments by recognizing landmarks and obstacles.
- **Interaction**: Dropbear can engage in conversational AI, providing responses and acting on user commands.
- **Learning**: Continuously improves through interactions, adapting to new phrases and contexts.

##### Dropbear utilizes a pre-trained model (LLaVA-1.6 8B), finetuned for robotic applications, enhanced by continuous learning from interactions.

#### Utilization of Open X-Embodiment Data
Open X-Embodiment RT-2 shows that vision-language models (VLMs) can be transformed into powerful vision-language-action (VLA) models, which can directly control a robot by combining VLM pre-training with robotic data.
![image](https://github.com/Hyperspawn/Dropbear/assets/37779762/1c9407b2-da29-4758-a568-7aa9bf914ed4)

### Teleoperation mode
#### Dropbear can be used as a proxy avatar by controlling the robot using VR gear like a motion-tracking suit, etc. The robot precisely mimics your actions. While interacting with physical objects, VR gloves give you sensation feedback for an immersive teleportation-like experience.
![Dropbear Teleoperation](https://github.com/user-attachments/assets/7d74c8da-ae27-4cde-bb03-05f5a6bca405)


## Assembly Instructions
For a step-by-step guide on assembling the Dropbear Humanoid Robot, refer to the [Assembly Guide](URL).

## Usage
Instructions and guidelines for operating the Dropbear Humanoid Robot can be found in the [User Manual](URL).

## Contribution
Contributions are welcome! Please refer to the [Contribution Guide](URL) for details.

## License
Dropbear Humanoid Robot is licensed under the [MIT License](URL).

## Contact
For additional information and inquiries, please visit [Hyperspawn Robotics](http://www.hyperspawn-robotics.com) or contact us at contact@hyperspawn-robotics.com.

*Join us in advancing humanoid robotics!*
