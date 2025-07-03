# Isaac Sim ROS2 Camera Control System with LLM-Driven Scenarios

A complete Isaac Sim + ROS2 integration for real-time camera control and intelligent scenario management.

## 🚀 Quick Start - Three Simple Steps

```bash
# 1. Set environment (replace with your Isaac Sim path)
export ISAAC_SIM_PATH=/home/user/.local/share/ov/pkg/isaac_sim-5.0.0
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws
colcon build
source install/setup.bash

# 2. Launch the LLM Camera Controller
ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=simulation

# 3. Control camera (in another terminal)
python3 camera_cli.py overhead
python3 camera_cli.py move --x 2.0
python3 camera_cli.py position --x 0 --y 0 --z 10
```

## 🎯 Key Features

- **🎮 Intuitive Camera Control**: Move, rotate, and position camera with simple commands.
- **🧠 LLM-Powered Camera**: Multiple LLM providers with **real Gemini 2.5 implementation** for robotics vision analysis.
- **🚀 ROS2 Native**: Controlled via standard ROS2 launch files and topics.
- **📊 Real-time Monitoring**: System status, scenario validation, and health checking.
- **🔄 Live Data Streaming**: RGB, depth, camera info via ROS2 topics.
- **✅ Proven Working**: Validated camera movement and scenario management.

## 🚀 Launch the System

### Prerequisites
- Isaac Sim 5.0+ installed
- ROS2 Jazzy
- Python 3.11+ (for Isaac Sim compatibility)

**Important Note**: There is a known Python version compatibility issue between Isaac Sim (Python 3.11) and ROS2 Jazzy (Python 3.12). While the ROS2 bridge extension loads in Isaac Sim, full ROS2 topic communication may be limited. The current system works around this by running the LLM camera controller as a separate ROS2 node outside of Isaac Sim.

### Environment Setup

1.  **Configure API Keys** (Choose one method):

    **Option A: Using .env file (Recommended)**:
    ```bash
    # Copy the example file
    cp .env.example .env
    
    # Edit .env file with your API keys
    nano .env
    
    # Add your keys:
    GEMINI_API_KEY=your_gemini_api_key_here
    OPENAI_API_KEY=your_openai_api_key_here
    ANTHROPIC_API_KEY=your_anthropic_api_key_here
    ```

    **Option B: Environment variables**:
    ```bash
    export GEMINI_API_KEY='your_gemini_api_key_here'
    export OPENAI_API_KEY='your_openai_api_key_here'
    ```

    **Option C: Launch parameters** (as shown below)

2.  **Run Environment Setup Helper**:
    ```bash
    python3 setup_environment.py
    ```

### Launch Instructions

1.  **Build and Source the Workspace**:
    ```bash
    cd /home/kimate/isaac_ws
    colcon build --packages-select isaac_test
    source install/setup.bash
    ```

2.  **Launch the LLM Camera Controller**:
    This is the primary method to run the system. API keys can be provided via .env file, environment variables, or launch parameters.

    *   **Simulation Mode (Default)**:
        ```bash
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=simulation
        ```

    *   **Google Gemini 2.5** (REAL IMPLEMENTATION):
        ```bash
        # Using .env file (recommended) - just add GEMINI_API_KEY to .env
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=gemini_2.5
        
        # Or with explicit API key parameter
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=gemini_2.5 gemini_api_key:='your-gemini-api-key'
        ```

    *   **OpenAI GPT-4.1**:
        ```bash
        # Using .env file or explicit parameter
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=openai_gpt4.1 openai_api_key:='your-openai-api-key'
        ```
        ```bash
        # Get your API key from: https://aistudio.google.com/app/apikey
        export GEMINI_API_KEY='your-gemini-api-key'
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=gemini_2.5 gemini_api_key:='your-gemini-api-key'
        ```

    *   **Anthropic Claude 4 Sonnet**:
        ```bash
        ros2 launch isaac_test llm_camera_controller_simple.launch.py llm_provider:=claude_4_sonnet anthropic_api_key:='your-claude-api-key'
        ```

## 🎮 Camera Control Commands

Use the `camera_cli.py` script in a separate terminal to control the camera in real-time.

*   **Apply a Preset View**:
    ```bash
    python3 camera_cli.py overhead
    python3 camera_cli.py side
    ```

*   **Move the Camera (Relative)**:
    ```bash
    python3 camera_cli.py move --x 1.5  # Move forward
    python3 camera_cli.py move --y -1.0 # Move left
    python3 camera_cli.py move --z 0.5  # Move up
    ```

*   **Rotate the Camera (Relative)**:
    ```bash
    python3 camera_cli.py rotate --pitch 15.0
    python3 camera_cli.py rotate --yaw -30.0
    ```

*   **Set Absolute Position**:
    ```bash
    python3 camera_cli.py position --x 0 --y 0 --z 10
    ```

*   **Reset Camera**:
    ```bash
    python3 camera_cli.py reset
    ```

## 📁 File Structure

**Core System:**
- `src/isaac_test/isaac_test/llm_camera_controller_simple.py` - The core LLM-powered camera controller node.
- `src/isaac_test/launch/llm_camera_controller_simple.launch.py` - The main ROS2 launch file.
- `camera_cli.py` - Command-line tool for camera control.
- `scenario_manager.py` - Handles loading of Isaac Sim scenarios.

**Documentation:**
- `README.md` - This overview guide.
- `CAMERA_CONTROL_README.md` - Detailed technical documentation.
- `AGENT_HANDOFF.md` - Handoff summary for developers.
- `FINAL_COMPLETION_REPORT.md` - Final project completion report.

## 🎉 Success Indicators

When everything is working correctly, you should see:
1. ✅ Isaac Sim window opens with the specified scenario.
2. ✅ The `llm_camera_controller` node starts successfully in your terminal.
3. ✅ Camera moves visibly in Isaac Sim when you use `camera_cli.py`.
4. ✅ ROS2 topics publish camera data continuously.

## 📞 Support

If you encounter issues:
1. Check Isaac Sim logs for errors.
2. Verify your ROS2 environment is properly sourced.
3. Ensure Python version compatibility (3.11+ for Isaac Sim).
4. Test with the `simulation` provider first before using real LLM providers.
5. **Note**: Due to Python version differences between Isaac Sim (3.11) and ROS2 Jazzy (3.12), some ROS2 topic communication may be limited. The system is designed to work around this limitation.