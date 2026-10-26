# Drone Interceptor Simulator

A Windows-oriented quadcopter simulation project combining a C++ physics/visualization simulator with a Python controller. The simulator uses SFML 3 and OpenGL for 3D visualization and sends telemetry to the Python controller over a local TCP connection (default port `8080`). The controller uses NumPy for target-state estimation, guidance, and motor commands.

> **Build status:** These instructions are derived from the supplied source files. A clean Windows build and end-to-end run have not been verified. No source code has been changed for this repository package.

## Repository contents

```text
.
├── CMakeLists.txt
├── main.cpp
├── physics.h
├── renderer.h
├── tcp_server.h
├── Drone Intercept Control Arch.py
├── requirements.txt
├── .gitignore
└── README.md
```

The `build/` folder is **generated locally** when you compile the simulator. It does not need to be committed to GitHub.

## Requirements (Windows x64)

- Windows 10/11, with a graphics driver supporting OpenGL 2.1.
- Visual Studio with **Desktop development with C++** (MSVC compiler and Windows SDK).
- CMake **3.20 or later**.
- **SFML 3** (Graphics, Window, System) available to CMake. The source uses SFML 3 APIs; SFML 2 is not a drop-in substitute.
- Python 3 and NumPy.
- OpenGL/GLU and Windows socket libraries (linked through the Windows SDK).

The renderer attempts to load `C:/Windows/Fonts/arial.ttf` for its on-screen text.

## 1. Download the source

Use **Code → Download ZIP** on GitHub and extract it, or clone the repository:

```powershell
git clone https://github.com/YOUR-USERNAME/YOUR-REPOSITORY.git
cd YOUR-REPOSITORY
```

Replace the placeholder URL and folder name with the actual repository. All six source files and `CMakeLists.txt` must remain in the **same root folder**.

## 2. Install SFML with vcpkg (one possible method)

If you do not already have SFML 3 configured, install [vcpkg](https://github.com/microsoft/vcpkg) in a separate directory. For example, from PowerShell:

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg.exe install sfml:x64-windows
```

The build below uses `C:/vcpkg` as an example. Change this path if vcpkg is installed elsewhere. Confirm that the installed SFML version is 3.x.

## 3. Build the C++ simulator

Open **x64 Native Tools Command Prompt for Visual Studio** (or a terminal with the Visual Studio build tools configured), navigate to the repository root, and run:

```powershell
cmake -S . -B build -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

The CMake project defines the target `DroneSimulator`. With the Visual Studio multi-configuration generator, the expected executable is:

```text
build/Release/DroneSimulator.exe
```

**This exact output path matters:** the Python script looks for `build/Release/DroneSimulator.exe` relative to its own location. If you use another CMake generator or build configuration, the executable may be written elsewhere and the unchanged Python script will not find it.

**SFML runtime DLLs:** If Windows reports missing SFML DLLs, ensure the SFML 3 runtime DLLs for your selected architecture/configuration are discoverable (for example, on `PATH` or alongside the executable). Do not mix Debug and Release DLLs.

## 4. Install the Python dependency

From the repository root:

```powershell
python -m pip install -r requirements.txt
```

A Python virtual environment is optional; if using one, activate it before installing NumPy and running the controller.

## 5. Run the complete simulation

From the **repository root**, run:

```powershell
python "Drone Intercept Control Arch.py"
```

The Python script automatically:

1. Starts `build/Release/DroneSimulator.exe` on port `8080`.
2. Waits briefly for the simulator to initialize.
3. Connects to `localhost:8080` over TCP.
4. Receives drone/target state and sends motor PWM commands.

Do **not** start a second copy of the simulator on the same port while using this launch method.

An alternate port can be supplied to the controller:

```powershell
python "Drone Intercept Control Arch.py" 8081
```

The Python script passes that port to the simulator it launches.

## Simulation details and controls

- **Physics:** nominal 120 Hz fixed-step update.
- **TCP telemetry/control:** nominal 50 Hz.
- **Motor ordering:** front-right (FR), front-left (FL), back-right (BR), back-left (BL).
- **Coordinate system:** EUS (X right/east, Y up, Z south/back).
- **Scenarios:** four stationary-target stages followed by three moving-target stages (linear, oscillating, circular).
- **R:** reset the simulation.
- **Esc:** close the visualization window.
- **Mouse drag:** orbit the camera; **mouse wheel:** zoom.
- **Ctrl+C:** interrupt the Python controller in its terminal.

## Troubleshooting

| Problem | What to check |
|---|---|
| `Could not find a package configuration file provided by SFML` | Install SFML 3 and ensure CMake uses the correct vcpkg toolchain/installation. |
| `FileNotFoundError` for `DroneSimulator.exe` | Build in **Release** using the Visual Studio generator; verify `build/Release/DroneSimulator.exe` exists. |
| Missing SFML DLL when launching | Make SFML 3 Release runtime DLLs available to the executable. |
| `ModuleNotFoundError: No module named 'numpy'` | Run `python -m pip install -r requirements.txt` using the same Python interpreter. |
| `ConnectionRefusedError` or port binding error | Confirm the simulator started successfully and port 8080 is free; check firewall/antivirus restrictions if applicable. |
| On-screen text missing | Check whether `C:/Windows/Fonts/arial.ttf` exists. |

## Scope and limitations

This is a research/demo simulation, not validated flight-control software. The setup described here is **Windows-focused** because the CMake file links Windows-specific libraries and the controller expects a `.exe` file. The source has not been modified to provide cross-platform builds or packaging.

## Sharing and licensing

Before publishing the repository publicly, confirm you have the rights to distribute the source code. No license is included by default; if you want others to reuse, modify, or redistribute it, add an appropriate license after confirming ownership.
