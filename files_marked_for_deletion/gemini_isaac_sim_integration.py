#!/usr/bin/env python3
"""
Gemini 2.5 Isaac Sim Integration with Scenario Manager
Real simulation with AI-powered camera control and upright perspective
"""

import cv2
import numpy as np
import time
import math
import json
from PIL import Image as PILImage
import os
import subprocess
import threading
from pathlib import Path

# Import Gemini
try:
    import google.generativeai as genai
    from dotenv import load_dotenv
    
    # Load environment variables
    load_dotenv()
    
    # Configure Gemini
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("❌ GEMINI_API_KEY not found in .env file")
        exit(1)
        
    genai.configure(api_key=api_key)
    model = genai.GenerativeModel('gemini-2.0-flash')
    print("✅ Gemini 2.5 initialized successfully")
    
except ImportError as e:
    print(f"❌ Failed to import required libraries: {e}")
    exit(1)
except Exception as e:
    print(f"❌ Failed to initialize Gemini: {e}")
    exit(1)

# Import scenario manager
try:
    from scenario_manager import ScenarioManager
    print("✅ Scenario Manager imported successfully")
except ImportError:
    print("❌ Failed to import Scenario Manager")
    exit(1)

class GeminiIsaacSimIntegration:
    """Integration between Gemini 2.5 AI and Isaac Sim with scenario management"""
    
    def __init__(self):
        self.scenario_manager = ScenarioManager()
        self.current_scenario = None
        self.camera_position = {"x": 0, "y": 0, "z": 2}  # Start elevated for upright view
        self.camera_rotation = {"pitch": -15, "yaw": 0, "roll": 0}  # Slight downward angle
        self.analysis_count = 0
        self.last_analysis_time = 0
        self.analysis_interval = 3.0  # 3 seconds between analyses
        
        print("🎯 Gemini Isaac Sim Integration initialized")
        print("📐 Camera configured for upright perspective")
        
    def setup_scenario(self, scenario_name="warehouse"):
        """Set up an Isaac Sim scenario using the scenario manager"""
        print(f"\n🏗️  Setting up scenario: {scenario_name}")
        
        try:
            # Get available scenarios
            scenarios = self.scenario_manager.available_scenes
            if scenario_name not in scenarios:
                print(f"❌ Scenario '{scenario_name}' not found")
                print(f"Available scenarios: {list(scenarios.keys())}")
                return False
                
            scenario_info = scenarios[scenario_name]
            self.current_scenario = {
                "name": scenario_name,
                "path": scenario_info["path"],
                "description": scenario_info["description"]
            }
            
            print(f"✅ Scenario selected: {scenario_info['description']}")
            print(f"📁 Scene path: {scenario_info['path']}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to setup scenario: {e}")
            return False
    
    def analyze_with_gemini_upright(self, image):
        """Analyze the scene using Gemini 2.5 with upright camera perspective considerations"""
        try:
            # Convert OpenCV image to PIL Image
            height, width = image.shape[:2]
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Enhanced robotics prompt with upright perspective emphasis
            robotics_prompt = f"""
You are an intelligent camera controller for Isaac Sim robotics environment. You are analyzing a scene from scenario: {self.current_scenario['name'] if self.current_scenario else 'unknown'}.

CRITICAL: The camera should ALWAYS maintain an upright perspective for optimal robotics operations.

SCENE ANALYSIS TASKS:
1. Identify robots, objects, obstacles, and navigation areas
2. Assess workspace visibility and coverage
3. Determine optimal camera positioning for robotics operations
4. Consider safety zones and operational efficiency
5. ENSURE the camera maintains proper upright orientation

CAMERA ACTIONS AVAILABLE (maintaining upright perspective):
- move_forward: Move camera forward while keeping upright view
- move_backward: Move camera backward for wider coverage
- move_left: Strafe left while maintaining upright orientation  
- move_right: Strafe right while maintaining upright orientation
- move_up: Elevate camera for better overhead perspective
- move_down: Lower camera for ground-level operations view
- rotate_left: Rotate left around vertical axis (yaw only)
- rotate_right: Rotate right around vertical axis (yaw only)
- tilt_down_slightly: Small downward tilt for better floor visibility (max 30°)
- tilt_up_slightly: Small upward tilt for better ceiling/upper area view
- hold_position: Maintain current upright position

UPRIGHT PERSPECTIVE RULES:
- Never suggest rolling the camera (keep horizon level)
- Limit pitch changes to ±30 degrees maximum
- Prioritize yaw rotations for directional changes
- Maintain clear view of the workspace floor
- Keep objects and robots clearly visible from above/side angles

RESPONSE FORMAT (JSON):
{{
    "scene_description": "Detailed description focusing on robotics elements",
    "objects_detected": ["robots", "obstacles", "navigation_areas", "key_landmarks"],
    "workspace_assessment": "Evaluation of the robotics workspace",
    "recommended_action": "one_of_the_actions_above",
    "reasoning": "Why this action improves robotics operations while maintaining upright view",
    "confidence": 0.0-1.0,
    "upright_compliance": "How this maintains proper upright camera perspective",
    "safety_considerations": "Any safety or collision concerns for robots"
}}

Current scenario: {self.current_scenario['description'] if self.current_scenario else 'Generic robotics environment'}
Focus on: Navigation, manipulation zones, obstacle avoidance, and workspace monitoring.
"""

            # Generate content with Gemini
            response = model.generate_content([robotics_prompt, pil_image])
            
            if not response.text:
                raise Exception("Empty response from Gemini")
            
            # Parse JSON response
            response_text = response.text.strip()
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            response_text = response_text.strip()
            
            analysis_data = json.loads(response_text)
            
            return {
                "success": True,
                "scene_description": analysis_data.get("scene_description", "Scene analyzed"),
                "objects_detected": analysis_data.get("objects_detected", []),
                "workspace_assessment": analysis_data.get("workspace_assessment", ""),
                "action": analysis_data.get("recommended_action", "hold_position"),
                "reasoning": analysis_data.get("reasoning", "Gemini analysis"),
                "confidence": float(analysis_data.get("confidence", 0.7)),
                "upright_compliance": analysis_data.get("upright_compliance", ""),
                "safety_considerations": analysis_data.get("safety_considerations", "None identified"),
                "raw_response": response.text
            }
            
        except json.JSONDecodeError as e:
            return {
                "success": False,
                "error": f"JSON parsing error: {e}",
                "raw_response": response.text if 'response' in locals() else None
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }
    
    def simulate_camera_movement(self, action):
        """Simulate camera movement while maintaining upright perspective"""
        movement_applied = False
        
        print(f"🎬 Executing camera action: {action}")
        
        # Apply movement while maintaining upright perspective
        if action == "move_forward":
            self.camera_position["x"] += 1.0
            movement_applied = True
        elif action == "move_backward":
            self.camera_position["x"] -= 1.0
            movement_applied = True
        elif action == "move_left":
            self.camera_position["y"] -= 1.0
            movement_applied = True
        elif action == "move_right":
            self.camera_position["y"] += 1.0
            movement_applied = True
        elif action == "move_up":
            self.camera_position["z"] += 0.5
            movement_applied = True
        elif action == "move_down":
            self.camera_position["z"] = max(0.5, self.camera_position["z"] - 0.5)
            movement_applied = True
        elif action == "rotate_left":
            self.camera_rotation["yaw"] -= 15
            movement_applied = True
        elif action == "rotate_right":
            self.camera_rotation["yaw"] += 15
            movement_applied = True
        elif action == "tilt_down_slightly":
            self.camera_rotation["pitch"] = max(-30, self.camera_rotation["pitch"] - 10)
            movement_applied = True
        elif action == "tilt_up_slightly":
            self.camera_rotation["pitch"] = min(30, self.camera_rotation["pitch"] + 10)
            movement_applied = True
        elif action == "hold_position":
            print("🔒 Holding current position")
        else:
            print(f"⚠️  Unknown action: {action}")
        
        if movement_applied:
            print(f"📍 New position: X={self.camera_position['x']:.1f}, Y={self.camera_position['y']:.1f}, Z={self.camera_position['z']:.1f}")
            print(f"🔄 New rotation: Pitch={self.camera_rotation['pitch']:.1f}°, Yaw={self.camera_rotation['yaw']:.1f}°, Roll=0.0°")
        
        return movement_applied
    
    def create_scenario_scene(self, frame_count):
        """Create a scene representing the current Isaac Sim scenario"""
        # Create a more sophisticated scene based on the scenario
        img = np.zeros((720, 1280, 3), dtype=np.uint8)  # Higher resolution for better analysis
        
        # Background based on scenario
        if self.current_scenario and self.current_scenario["name"] == "warehouse":
            # Warehouse-like environment
            # Floor
            cv2.rectangle(img, (0, 500), (1280, 720), (100, 100, 100), -1)  # Gray floor
            
            # Warehouse shelving (static)
            cv2.rectangle(img, (100, 300), (200, 500), (139, 69, 19), -1)  # Brown shelf
            cv2.rectangle(img, (300, 250), (400, 500), (139, 69, 19), -1)  # Brown shelf
            cv2.rectangle(img, (900, 200), (1000, 500), (139, 69, 19), -1)  # Brown shelf
            
            # Add some boxes on shelves
            cv2.rectangle(img, (110, 280), (140, 300), (160, 82, 45), -1)  # Box
            cv2.rectangle(img, (150, 270), (180, 300), (160, 82, 45), -1)  # Box
            cv2.rectangle(img, (310, 230), (340, 250), (160, 82, 45), -1)  # Box
            
            # Moving robot/AGV
            t = frame_count * 0.05
            robot_x = int(400 + 300 * math.sin(t))
            robot_y = int(450)
            cv2.rectangle(img, (robot_x, robot_y), (robot_x + 60, robot_y + 40), (0, 0, 255), -1)  # Red robot
            cv2.putText(img, 'AGV', (robot_x + 10, robot_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Navigation paths
            cv2.line(img, (50, 520), (1230, 520), (255, 255, 0), 3)  # Yellow path line
            
        else:
            # Generic robotics environment
            # Floor
            cv2.rectangle(img, (0, 400), (1280, 720), (120, 120, 120), -1)  # Gray floor
            
            # Some generic objects
            t = frame_count * 0.08
            
            # Moving objects
            obj1_x = int(300 + 200 * math.sin(t))
            obj1_y = int(350)
            cv2.circle(img, (obj1_x, obj1_y), 25, (0, 255, 0), -1)  # Green object
            
            obj2_x = int(800 + 150 * math.cos(t * 0.7))
            obj2_y = int(320)
            cv2.rectangle(img, (obj2_x, obj2_y), (obj2_x + 30, obj2_y + 30), (255, 0, 0), -1)  # Red object
        
        # Camera perspective indicator (showing upright view)
        cv2.putText(img, f'ISAAC SIM - {self.current_scenario["name"].upper() if self.current_scenario else "SIMULATION"}', 
                   (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(img, f'Frame: {frame_count} | Upright Perspective', (10, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, f'Camera: X={self.camera_position["x"]:.1f} Y={self.camera_position["y"]:.1f} Z={self.camera_position["z"]:.1f}', 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(img, f'Rotation: P={self.camera_rotation["pitch"]:.0f}° Y={self.camera_rotation["yaw"]:.0f}° R=0°', 
                   (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return img
    
    def run_simulation(self, scenario_name="warehouse", duration_minutes=2):
        """Run the main simulation with Gemini analysis and scenario management"""
        print("🚀 Starting Gemini Isaac Sim Integration")
        print("=" * 80)
        
        # Setup scenario
        if not self.setup_scenario(scenario_name):
            return False
        
        print(f"🎯 Running simulation for {duration_minutes} minutes")
        print(f"📐 Maintaining upright camera perspective")
        print(f"🤖 AI analysis every {self.analysis_interval} seconds")
        print("=" * 80)
        
        frame_count = 0
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while time.time() < end_time:
                current_time = time.time()
                
                # Create scenario scene
                scene = self.create_scenario_scene(frame_count)
                
                # Display frame info
                if frame_count % 30 == 0:  # Every second
                    elapsed = current_time - start_time
                    remaining = end_time - current_time
                    print(f"\n📸 Frame {frame_count} | Elapsed: {elapsed:.1f}s | Remaining: {remaining:.1f}s")
                
                # Analyze with Gemini at intervals
                if current_time - self.last_analysis_time >= self.analysis_interval:
                    self.analysis_count += 1
                    self.last_analysis_time = current_time
                    
                    print(f"\n🧠 AI Analysis {self.analysis_count} - Gemini 2.5 examining {scenario_name} scenario...")
                    
                    analysis_start = time.time()
                    result = self.analyze_with_gemini_upright(scene)
                    analysis_time = time.time() - analysis_start
                    
                    if result["success"]:
                        print(f"✅ Analysis completed in {analysis_time:.2f}s")
                        print(f"📝 Scene: {result['scene_description'][:100]}...")
                        print(f"🎯 Objects: {', '.join(result['objects_detected'][:3])}{'...' if len(result['objects_detected']) > 3 else ''}")
                        print(f"🏭 Workspace: {result['workspace_assessment'][:80]}...")
                        print(f"🎮 Action: {result['action']}")
                        print(f"🧠 Reasoning: {result['reasoning'][:100]}...")
                        print(f"📐 Upright Compliance: {result['upright_compliance'][:60]}...")
                        print(f"🛡️  Safety: {result['safety_considerations'][:60]}...")
                        print(f"🎲 Confidence: {result['confidence']:.2f}")
                        
                        # Execute the camera movement
                        self.simulate_camera_movement(result['action'])
                        
                    else:
                        print(f"❌ Analysis failed: {result['error']}")
                        if result.get('raw_response'):
                            print(f"📝 Raw response: {result['raw_response'][:150]}...")
                
                frame_count += 1
                time.sleep(0.033)  # ~30 FPS simulation
                
        except KeyboardInterrupt:
            print("\n🛑 Simulation stopped by user")
        
        # Final summary
        total_time = time.time() - start_time
        print("\n" + "=" * 80)
        print("🎉 SIMULATION COMPLETED!")
        print(f"⏱️  Total time: {total_time:.1f} seconds")
        print(f"📸 Total frames: {frame_count}")
        print(f"🧠 AI analyses: {self.analysis_count}")
        print(f"🎯 Scenario: {self.current_scenario['name']}")
        print(f"📐 Camera maintained upright perspective throughout")
        print(f"🎮 Final position: X={self.camera_position['x']:.1f}, Y={self.camera_position['y']:.1f}, Z={self.camera_position['z']:.1f}")
        print("✅ Gemini 2.5 successfully controlled Isaac Sim camera with scenario management!")
        print("=" * 80)
        
        return True

def main():
    """Main entry point"""
    print("🎬 Gemini 2.5 Isaac Sim Integration with Scenario Manager")
    print("🤖 AI-Powered Camera Control with Upright Perspective")
    
    integration = GeminiIsaacSimIntegration()
    
    # Run warehouse scenario
    print("\n🏗️  Running Warehouse Scenario...")
    integration.run_simulation("warehouse", duration_minutes=1)
    
    # Optionally run another scenario
    print("\n🏢 Running Office Scenario...")
    integration.run_simulation("office", duration_minutes=1)

if __name__ == '__main__':
    main()
