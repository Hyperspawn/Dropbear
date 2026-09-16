<p align="center">
  <img src="https://raw.githubusercontent.com/Hyperspawn/Dropbear/main/Media/Flows/dropbear.png" alt="Dropbear humanoid robot" width="1024">
</p>

# Dropbear

### A humanoid you can build, understand, and make your own.

Dropbear is a full-size humanoid robotics project by [Hyperspawn](https://hyperspawn.org) and [Pointblank](https://www.pointblankllc.com/). Mechanical designs, printable parts, robot models, and control research give you places to start across the whole robot—from a single assembly on your workbench to locomotion experiments in simulation.

The aim is to make building and using a humanoid feel approachable: find the parts, see how they fit, try a movement, understand what happened, and keep improving it. The [platform](https://hyperspawn.org/platform) brings the build and robot tools into one place.

**[Open the platform →](https://hyperspawn.org/platform)** · [Try the simulator](https://hyperspawn.org/sim) · [Explore the hardware](https://github.com/Hyperspawn/dropbear_hardware) · [Join the community](https://hyperspawn.org/community)

## Start with something you can do today

| I want to… | Start here | First step |
|---|---|---|
| Explore Dropbear | [Browser simulator](https://hyperspawn.org/sim) | Open the scene and explore the robot before setting up a development environment. |
| Build my own | [Build platform](https://hyperspawn.org/platform) | Open the Workshop, choose an assembly, and review its parts and prints. |
| Print a part or assemble a limb | [Printable parts](https://github.com/Hyperspawn/dropbear_printables) | Choose a subassembly and open its prepared `.3mf` plates or individual STL files. |
| Work on walking | [Locomotion](https://github.com/Hyperspawn/dropbear-locomotion) | Follow the setup guide, load a published checkpoint, and use the interactive viewer. |
| Observe connected controllers | [dropbear_control](https://github.com/robit-man/dropbear_control) | Run the local dashboard, open `/?live=1`, and use the Devices view for receive-only leg state and firmware diagnostics. |
| Change the mechanical design | [Hardware](https://github.com/Hyperspawn/dropbear_hardware) | Open the full assembly or a component in CAD. |

You can begin with a head, an arm, or a simulated robot. Each is a useful project in its own right.

## Connected robot control and firmware

Use [`robit-man/dropbear_control`](https://github.com/robit-man/dropbear_control) for the browser digital twin, live USB leg observation, raw serial diagnostics, controller health, and guarded firmware compilation/upload. Its physical command channel remains locked behind staged acknowledgements; live observation starts as receive-only.

```bash
git clone https://github.com/robit-man/dropbear_control.git
cd dropbear_control
python3 web/serve.py 8000
```

Open <http://localhost:8000/?live=1&renderer=swiftshader>. Select **Connected ESP32 devices** to inspect both leg streams, compile a trusted firmware source, or review an upload before its physical safety interlock is released.

The controller sources live in [`Control System/Low Level Control`](https://github.com/Hyperspawn/Dropbear/tree/main/Control%20System/Low%20Level%20Control):

| Firmware | Use |
|---|---|
| [`firmware_full_libs_neck.ino`](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/firmware_full_libs_neck.ino) | Recommended universal Behemoth image. One build supports left leg, right leg, center IMU, or head/neck roles selected from persistent configuration. |
| [`esp32_devkitc_v4_hybrid.ino`](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/esp32_devkitc_v4_hybrid.ino) | Hybrid leg controller retained for existing PWM/CAN deployments. |
| [`esp32_devkit_v1_observation_safe.ino`](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/esp32_devkit_v1_observation_safe.ino) | Observation-only migration image. Motion commands stay disabled while five AS5600 and six motor-native CAN angles are reported. |
| [`esp32_devkit_v1.ino`](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/esp32_devkit_v1.ino) | Legacy/development leg controller. Keep it for compatibility and comparison; use Behemoth for new universal deployments. |

Read the [low-level firmware guide](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/README.md) and [Behemoth role/commissioning guide](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/BEHEMOTH.md) before selecting an image. The tracked [`partitions.csv`](https://github.com/Hyperspawn/Dropbear/blob/main/Control%20System/Low%20Level%20Control/partitions.csv) gives the application 2.5 MiB while retaining the Arduino default SPIFFS settings region at `0x290000`. The `dropbear_control` uploader reads the connected ESP32 partition table first and refuses a mismatched layout.

## From files to a robot on your workbench

The design is split into subassemblies so you can inspect, print, assemble, and revise one part at a time. The hardware repository includes full-robot STEP and Fusion archives, alongside separate arms, legs, head, pelvis, and torso designs. The print repository organizes parts and plates by assembly.

<p align="center">
  <a href="https://github.com/Hyperspawn/dropbear_hardware">
    <img src="https://media.githubusercontent.com/media/Hyperspawn/dropbear_hardware/main/Images/Right%20Arm.png" alt="CAD render of the Dropbear right arm assembly" width="280">
    <img src="https://media.githubusercontent.com/media/Hyperspawn/dropbear_hardware/main/Images/Right%20Leg.png" alt="CAD render of the Dropbear right leg assembly" width="280">
  </a>
</p>

*Right arm and right leg CAD renders from the hardware repository.*

1. **Choose your assembly.** Use the [Workshop](https://hyperspawn.org/platform/build) and [CAD files](https://github.com/Hyperspawn/dropbear_hardware) to understand the parts and how they connect.
2. **Plan the parts and prints.** Review the [bill of materials](https://github.com/Hyperspawn/dropbear_bom/blob/main/bom.md), then select the matching [print files and settings](https://github.com/Hyperspawn/dropbear_printables). You can print them yourself or use a print service.
3. **Assemble and document.** Follow the relevant [assembly documentation](https://github.com/Hyperspawn/dropbear_docs/tree/main/docs/03-assembly). Keep track of the design revision, substitutions, and anything the next builder should know.
4. **Bring up one subsystem at a time.** Review the [low-level control source](https://github.com/Hyperspawn/Dropbear/tree/main/Control%20System/Low%20Level%20Control) and [firmware project](https://github.com/Hyperspawn/dropbear_firmware) against your hardware. Establish wiring, joint directions, limits, and calibration before attempting coordinated motion.

Build files, documentation, and control software are evolving at different speeds. Match revisions across them before ordering parts or powering an assembly. Use the BOM to develop a budget for your chosen configuration and local suppliers.

## Walking you can inspect and reproduce

The [Dropbear locomotion project](https://github.com/Hyperspawn/dropbear-locomotion) publishes an Isaac Lab training environment, robot USD, policy checkpoints, evaluation reports, and an interactive viewer. You can load a policy, steer a simulated Dropbear, inspect its motion, and continue training.

[![Dropbear locomotion training shown in the interactive viewer](https://raw.githubusercontent.com/Hyperspawn/dropbear-locomotion/main/media/dropbear_train_live_domain_randomization.png)](https://github.com/Hyperspawn/dropbear-locomotion)

*Isaac Lab simulation, with a training environment mirrored into the live viewer.*

The published **v0.1.0 flat-ground policy completed 128 simulation trials with zero falls**: 32 each for forward, backward, left, and right commands. Each trial ran for 20 seconds on a plane, with pushes disabled and a commanded speed of 0.20 m/s. Backward and lateral speed tracking remain areas for improvement. [Read the evaluation results](https://github.com/Hyperspawn/dropbear-locomotion/blob/main/evaluation/cardinal_0.20_strict_128.json).

The **v0.2.0 terrain checkpoint is experimental**, adding terrain observations and low-obstacle training. These are simulation releases; physical walking and robustness require separate validation.

**[Run the locomotion quick start →](https://github.com/Hyperspawn/dropbear-locomotion#quick-start)**

## Find the part of the project you need

This repository is the project’s front door and contains the original hardware, software, and media. More focused work lives in the repositories below.

| Area | Repository | What you will find |
|---|---|---|
| Mechanical design | [dropbear_hardware](https://github.com/Hyperspawn/dropbear_hardware) | Full assembly CAD and mechanical subassemblies. |
| 3D printing | [dropbear_printables](https://github.com/Hyperspawn/dropbear_printables) | Prepared print plates, individual parts, and print guidance. |
| Parts and sourcing | [dropbear_bom](https://github.com/Hyperspawn/dropbear_bom) | Purchased components and assembly-level cost estimates. |
| Build documentation | [dropbear_docs](https://github.com/Hyperspawn/dropbear_docs) | Assembly guides and sections for electronics, calibration, testing, and operation. |
| Robot descriptions | [dropbear_urdf](https://github.com/Hyperspawn/dropbear_urdf) · [dropbear_mjcf](https://github.com/Hyperspawn/dropbear_mjcf) | Robot models and meshes for visualization and simulation. |
| Simulation workspace | [dropbear_sim](https://github.com/Hyperspawn/dropbear_sim) | Links and submodules for the simulation projects. |
| Locomotion research | [dropbear-locomotion](https://github.com/Hyperspawn/dropbear-locomotion) | Training, published policies, live viewing, and evaluations. |
| Embedded control | [dropbear_firmware](https://github.com/Hyperspawn/dropbear_firmware) · [Original low-level control](https://github.com/Hyperspawn/Dropbear/tree/main/Control%20System/Low%20Level%20Control) | ESP32/CAN development and the original controller implementation. |
| Motor tools | [myactuator-can](https://github.com/Hyperspawn/myactuator-can) | MyActuator driver and diagnostics work. |

[Browse all Hyperspawn repositories →](https://github.com/orgs/Hyperspawn/repositories)

## What we are working toward

We want a humanoid that people can understand well enough to repair, adapt, teach, and put to use. That means connecting the practical details—parts, wiring, calibration, and control—with simulation, teleoperation, and learned behavior.

The next steps include more complete build guidance, better locomotion across terrain, and bringing control and learning workflows onto physical hardware. Teleoperation and language-driven behavior are development directions; their integration into a dependable whole-robot system is ongoing.

The useful unit of progress is something another person can try: a better joint, a clearer assembly step, a reproducible policy, or a physical test with enough detail to repeat it.

## Build with us

Share a build photo, improve a part, reproduce an experiment, or fix the instruction that slowed you down. Small improvements travel a long way when the next person builds from the same files.

- **Building hardware?** Include the assembly revision, material, print settings, and photos of the fit or failure.
- **Working on software or policies?** Include the environment, configuration, and results; say whether the test ran in simulation or on hardware.
- **Improving documentation?** Show the missing step, the corrected part, or the explanation you wish you had.

Open an issue or pull request in the relevant repository. For general project questions, use [Dropbear issues](https://github.com/Hyperspawn/Dropbear/issues) or the [community](https://hyperspawn.org/community). The documentation project also has [contribution guidelines](https://github.com/Hyperspawn/dropbear_docs/blob/main/CONTRIBUTING.md).

## Credits and licensing

Dropbear is developed by **Hyperspawn and Pointblank**, with contributions from builders and researchers across the project. Locomotion training lineage and contributor credits are documented in the [locomotion repository](https://github.com/Hyperspawn/dropbear-locomotion#training-lineage-and-credits).

This repository uses the [Hyperspawn License](https://github.com/Hyperspawn/Dropbear/blob/main/LICENSE), which restricts use to non-commercial purposes and requires a separate license for commercial use. Related repositories and assets carry their own terms; check the license supplied with the files you use.

---

[Start a build](https://hyperspawn.org/platform) · [Try the simulator](https://hyperspawn.org/sim) · [Meet the community](https://hyperspawn.org/community)
