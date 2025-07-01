#!/usr/bin/env python3
"""
Isaac Sim Scenario Manager
Handles LLM-driven scenario setup for Isaac Sim environments
Enhanced with comprehensive logging and status tracking
"""

import json
import os
import re
import logging
import time
from typing import Dict, List, Optional, Tuple
from pathlib import Path
from datetime import datetime

class ScenarioManager:
    """Manages scenario creation and parsing for Isaac Sim environments"""
    
    def __init__(self, enable_logging=True, log_level=logging.INFO):
        # Set up logging
        self.enable_logging = enable_logging
        self.setup_logging(log_level)
        
        # Status tracking
        self.status = "initialized"
        self.last_error = None
        self.processing_steps = []
        self.start_time = time.time()
        
        self.log_info("🚀 Initializing Scenario Manager...")
        
        # Asset definitions
        self.available_scenes = {
            "warehouse": {
                "path": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
                "description": "Industrial warehouse environment with shelving and storage areas"
            },
            "simple_room": {
                "path": "/Isaac/Environments/Simple_Room/simple_room.usd", 
                "description": "Basic indoor room environment"
            },
            "office": {
                "path": "/Isaac/Environments/Office/office.usd",
                "description": "Modern office environment with furniture"
            },
            "grid_room": {
                "path": "/Isaac/Environments/Grid_Room/gridroom_curved.usd",
                "description": "Grid-based room for navigation testing"
            }
        }
        
        self.available_robots = {
            "carter": {
                "path": "/Isaac/Robots/Carter/carter_v1.usd",
                "description": "NVIDIA Carter autonomous mobile robot",
                "ros_enabled": True,
                "default_position": [0, 0, 0]
            },
            "carter_2": {
                "path": "/Isaac/Robots/Carter/carter_v2.usd", 
                "description": "NVIDIA Carter 2.0 autonomous mobile robot",
                "ros_enabled": True,
                "default_position": [0, 0, 0]
            },
            "franka": {
                "path": "/Isaac/Robots/Franka/franka.usd",
                "description": "Franka Emika Panda robotic arm",
                "ros_enabled": True,
                "default_position": [0, 0, 0]
            },
            "ur10": {
                "path": "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
                "description": "Universal Robots UR10 industrial arm",
                "ros_enabled": True,
                "default_position": [0, 0, 0]
            }
        }
        
        self.available_props = {
            "cones": {
                "path": "/Isaac/Props/Cones/traffic_cone.usd",
                "description": "Traffic cone for navigation obstacles"
            },
            "boxes": {
                "path": "/Isaac/Props/Blocks/basic_block.usd",
                "description": "Basic cubic blocks"
            },
            "barrels": {
                "path": "/Isaac/Props/Barrels/barrel.usd", 
                "description": "Industrial barrels"
            }
        }
        
        self.log_info(f"✅ Scenario Manager initialized successfully")
        self.log_info(f"   Available scenes: {len(self.available_scenes)}")
        self.log_info(f"   Available robots: {len(self.available_robots)}")
        self.log_info(f"   Available props: {len(self.available_props)}")
        self.status = "ready"
    
    def setup_logging(self, log_level=logging.INFO):
        """Set up logging configuration"""
        if self.enable_logging:
            # Create logs directory if it doesn't exist
            log_dir = Path("/tmp/isaac_logs")
            log_dir.mkdir(exist_ok=True)
            
            # Configure logging
            log_file = log_dir / f"scenario_manager_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            
            logging.basicConfig(
                level=log_level,
                format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                handlers=[
                    logging.FileHandler(log_file),
                    logging.StreamHandler()  # Also log to console
                ]
            )
            
            self.logger = logging.getLogger('ScenarioManager')
            self.logger.info(f"Logging initialized - Log file: {log_file}")
        else:
            self.logger = None
    
    def log_info(self, message):
        """Log info message"""
        print(message)  # Always print to console
        if self.logger:
            self.logger.info(message)
    
    def log_warning(self, message):
        """Log warning message"""
        print(f"⚠️  {message}")
        if self.logger:
            self.logger.warning(message)
    
    def log_error(self, message, exception=None):
        """Log error message"""
        print(f"❌ {message}")
        if self.logger:
            self.logger.error(message)
            if exception:
                self.logger.exception(exception)
        self.last_error = message
    
    def add_processing_step(self, step_name, status="in_progress", details=None):
        """Track processing steps"""
        step = {
            "name": step_name,
            "status": status,
            "timestamp": time.time(),
            "details": details or {}
        }
        self.processing_steps.append(step)
        
        if status == "completed":
            self.log_info(f"✅ Step completed: {step_name}")
        elif status == "failed":
            self.log_error(f"❌ Step failed: {step_name}")
        else:
            self.log_info(f"🔄 Step started: {step_name}")
    
    def get_status_report(self):
        """Generate comprehensive status report"""
        uptime = time.time() - self.start_time
        
        report = {
            "status": self.status,
            "uptime_seconds": uptime,
            "uptime_formatted": f"{uptime:.1f}s",
            "last_error": self.last_error,
            "processing_steps": len(self.processing_steps),
            "recent_steps": self.processing_steps[-5:] if self.processing_steps else [],
            "available_assets": {
                "scenes": len(self.available_scenes),
                "robots": len(self.available_robots),
                "props": len(self.available_props)
            }
        }
        
        return report
    
    def parse_scenario_request(self, user_request: str) -> Dict:
        """
        Parse natural language scenario request into structured data
        
        Args:
            user_request: Natural language description of desired scenario
            
        Returns:
            Dictionary containing parsed scenario components
        """
        self.add_processing_step("parse_request", "in_progress", {"request": user_request})
        
        try:
            request_lower = user_request.lower()
            scenario = {
                "scene": None,
                "robots": [],
                "props": [],
                "camera_position": None,
                "special_requirements": []
            }
            
            self.log_info(f"🔍 Parsing request: '{user_request}'")
            
            # Parse scene type
            scene_found = False
            for scene_name, scene_info in self.available_scenes.items():
                keywords = scene_info["description"].lower().split()
                if scene_name in request_lower or any(keyword in request_lower for keyword in keywords):
                    scenario["scene"] = scene_name
                    scene_found = True
                    self.log_info(f"🏗️  Scene detected: {scene_name}")
                    break
            
            # Default to simple_room if no scene specified
            if not scenario["scene"]:
                scenario["scene"] = "simple_room" 
                self.log_warning(f"No specific scene detected, defaulting to: simple_room")
            
            # Parse robots
            robots_found = 0
            for robot_name, robot_info in self.available_robots.items():
                # Check for robot name variations
                robot_patterns = [
                    robot_name,
                    robot_name.replace("_", " "),
                    robot_name.replace("_", "-"),
                    robot_name.replace("_", "")
                ]
                
                if any(pattern in request_lower for pattern in robot_patterns):
                    ros_enabled = ("ros" in request_lower or "ros2" in request_lower) and robot_info["ros_enabled"]
                    robot_config = {
                        "type": robot_name,
                        "name": f"{robot_name}_robot",
                        "position": robot_info["default_position"].copy(),
                        "ros_enabled": ros_enabled
                    }
                    scenario["robots"].append(robot_config)
                    robots_found += 1
                    
                    ros_status = "with ROS2" if ros_enabled else "without ROS2"
                    self.log_info(f"🤖 Robot detected: {robot_name} {ros_status}")
            
            if robots_found == 0:
                self.log_warning("No robots detected in request")
            
            # Parse special requirements
            requirements_found = []
            if "fully ros controlled" in request_lower or "ros controlled" in request_lower:
                scenario["special_requirements"].append("ros_control")
                requirements_found.append("ros_control")
                
            if "autonomous" in request_lower:
                scenario["special_requirements"].append("autonomous_navigation")
                requirements_found.append("autonomous_navigation")
                
            if "multiple" in request_lower:
                scenario["special_requirements"].append("multiple_entities")
                requirements_found.append("multiple_entities")
            
            if requirements_found:
                self.log_info(f"🎯 Special requirements: {', '.join(requirements_found)}")
                
            # Parse camera positioning requests
            camera_keywords = {
                "overhead": [0, 0, 10],
                "bird's eye": [0, 0, 10], 
                "side view": [5, 0, 2],
                "close up": [1, 1, 1],
                "far view": [10, 10, 5]
            }
            
            for keyword, position in camera_keywords.items():
                if keyword in request_lower:
                    scenario["camera_position"] = position
                    self.log_info(f"📷 Camera position: {keyword} -> {position}")
                    break
            
            self.add_processing_step("parse_request", "completed", {
                "scene": scenario["scene"],
                "robots_count": len(scenario["robots"]),
                "requirements_count": len(scenario["special_requirements"])
            })
            
            return scenario
            
        except Exception as e:
            self.log_error(f"Failed to parse scenario request", e)
            self.add_processing_step("parse_request", "failed", {"error": str(e)})
            raise
    
    def validate_scenario(self, scenario: Dict) -> Tuple[bool, List[str]]:
        """
        Validate that scenario components are available
        
        Args:
            scenario: Parsed scenario dictionary
            
        Returns:
            Tuple of (is_valid, list_of_warnings)
        """
        self.add_processing_step("validate_scenario", "in_progress")
        
        try:
            warnings = []
            
            self.log_info("🔍 Validating scenario components...")
            
            # Check scene availability
            if scenario["scene"] not in self.available_scenes:
                warning = f"Scene '{scenario['scene']}' not found, using 'simple_room'"
                warnings.append(warning)
                self.log_warning(warning)
                scenario["scene"] = "simple_room"
            else:
                self.log_info(f"✅ Scene validation passed: {scenario['scene']}")
            
            # Check robot availability
            valid_robots = []
            invalid_robots = []
            
            for robot in scenario["robots"]:
                if robot["type"] in self.available_robots:
                    valid_robots.append(robot)
                    self.log_info(f"✅ Robot validation passed: {robot['type']}")
                else:
                    invalid_robots.append(robot["type"])
                    warning = f"Robot '{robot['type']}' not found, skipping"
                    warnings.append(warning)
                    self.log_warning(warning)
            
            scenario["robots"] = valid_robots
            
            # Validation summary
            validation_summary = {
                "scene_valid": scenario["scene"] in self.available_scenes,
                "valid_robots": len(valid_robots),
                "invalid_robots": len(invalid_robots),
                "total_warnings": len(warnings)
            }
            
            is_valid = len(warnings) == 0
            
            if is_valid:
                self.log_info("✅ Scenario validation passed - no warnings")
            else:
                self.log_warning(f"Scenario validation completed with {len(warnings)} warning(s)")
            
            self.add_processing_step("validate_scenario", "completed", validation_summary)
            
            return is_valid, warnings
            
        except Exception as e:
            self.log_error(f"Failed to validate scenario", e)
            self.add_processing_step("validate_scenario", "failed", {"error": str(e)})
            raise
    
    def generate_scenario_config(self, user_request: str) -> Dict:
        """
        Generate complete scenario configuration from user request
        
        Args:
            user_request: Natural language scenario description
            
        Returns:
            Complete scenario configuration dictionary
        """
        self.add_processing_step("generate_config", "in_progress", {"request": user_request})
        
        try:
            self.log_info(f"🎯 Starting scenario generation for: '{user_request}'")
            self.status = "processing"
            
            # Parse the request
            scenario = self.parse_scenario_request(user_request)
            
            # Validate components
            is_valid, warnings = self.validate_scenario(scenario)
            
            # Print warnings if any
            for warning in warnings:
                self.log_warning(warning)
                
            # Create full configuration
            config = {
                "request": user_request,
                "parsed_scenario": scenario,
                "is_valid": is_valid,
                "warnings": warnings,
                "timestamp": time.time(),
                "processing_time": time.time() - self.start_time,
                "manager_status": self.get_status_report()
            }
            
            # Generate summary
            summary = {
                "scene": scenario['scene'],
                "robots": len(scenario['robots']),
                "special_requirements": len(scenario['special_requirements']),
                "camera_positioned": scenario.get('camera_position') is not None,
                "warnings": len(warnings),
                "is_valid": is_valid
            }
            
            self.log_info(f"✅ Scenario generation completed:")
            self.log_info(f"   📍 Scene: {summary['scene']}")
            self.log_info(f"   🤖 Robots: {summary['robots']} robot(s)")
            self.log_info(f"   🎯 Requirements: {summary['special_requirements']} requirement(s)")
            self.log_info(f"   📷 Camera positioned: {summary['camera_positioned']}")
            self.log_info(f"   ⚠️  Warnings: {summary['warnings']}")
            self.log_info(f"   ✅ Valid: {summary['is_valid']}")
            
            self.add_processing_step("generate_config", "completed", summary)
            self.status = "ready"
            
            return config
            
        except Exception as e:
            self.log_error(f"Failed to generate scenario configuration", e)
            self.add_processing_step("generate_config", "failed", {"error": str(e)})
            self.status = "error"
            raise
    
    def save_scenario_config(self, config: Dict, file_path: str = "/tmp/isaac_scenario_config.json"):
        """Save scenario configuration to file"""
        self.add_processing_step("save_config", "in_progress", {"file_path": file_path})
        
        try:
            self.log_info(f"💾 Saving scenario config to: {file_path}")
            
            # Ensure directory exists
            Path(file_path).parent.mkdir(parents=True, exist_ok=True)
            
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=2)
            
            # Verify file was written
            if Path(file_path).exists():
                file_size = Path(file_path).stat().st_size
                self.log_info(f"✅ Scenario config saved successfully ({file_size} bytes)")
                self.add_processing_step("save_config", "completed", {
                    "file_path": file_path,
                    "file_size": file_size
                })
            else:
                raise FileNotFoundError("File was not created")
                
        except Exception as e:
            self.log_error(f"Failed to save scenario config to {file_path}", e)
            self.add_processing_step("save_config", "failed", {"error": str(e)})
            raise
    
    def load_scenario_config(self, file_path: str = "/tmp/isaac_scenario_config.json") -> Optional[Dict]:
        """Load scenario configuration from file"""
        self.add_processing_step("load_config", "in_progress", {"file_path": file_path})
        
        try:
            if not os.path.exists(file_path):
                self.log_warning(f"Config file not found: {file_path}")
                self.add_processing_step("load_config", "completed", {"result": "not_found"})
                return None
            
            self.log_info(f"📂 Loading scenario config from: {file_path}")
            
            with open(file_path, 'r') as f:
                config = json.load(f)
            
            # Validate loaded config
            if not isinstance(config, dict) or 'parsed_scenario' not in config:
                raise ValueError("Invalid config file format")
            
            file_size = Path(file_path).stat().st_size
            self.log_info(f"✅ Scenario config loaded successfully ({file_size} bytes)")
            self.log_info(f"   Request: '{config.get('request', 'Unknown')}'")
            self.log_info(f"   Scene: {config['parsed_scenario'].get('scene', 'Unknown')}")
            
            self.add_processing_step("load_config", "completed", {
                "file_path": file_path,
                "file_size": file_size,
                "scene": config['parsed_scenario'].get('scene')
            })
            
            return config
            
        except Exception as e:
            self.log_error(f"Failed to load scenario config from {file_path}", e)
            self.add_processing_step("load_config", "failed", {"error": str(e)})
            return None

def main():
    """Enhanced demo/test function for scenario manager with comprehensive logging"""
    print("🚀 Isaac Sim Scenario Manager - Enhanced Testing")
    print("=" * 60)
    
    # Initialize with logging enabled
    manager = ScenarioManager(enable_logging=True)
    
    # Test scenarios
    test_requests = [
        "launch a warehouse scene with a carter 2.0 robot that is fully ros controlled",
        "create an office environment with a franka arm for manipulation tasks", 
        "simple room with multiple carter robots for swarm testing",
        "warehouse scene with overhead camera view and autonomous navigation",
        "invalid scene with nonexistent robot"  # Test error handling
    ]
    
    print(f"\n🧪 Running {len(test_requests)} test scenarios...")
    
    for i, request in enumerate(test_requests, 1):
        print(f"\n{'='*60}")
        print(f"🔬 Test {i}/{len(test_requests)}: '{request}'")
        print("-" * 60)
        
        try:
            # Generate config
            config = manager.generate_scenario_config(request)
            
            # Save config
            test_file = f"/tmp/test_scenario_{i}.json"
            manager.save_scenario_config(config, test_file)
            
            # Load it back to verify
            loaded_config = manager.load_scenario_config(test_file)
            
            if loaded_config:
                print(f"✅ Round-trip test passed")
            else:
                print(f"❌ Round-trip test failed")
                
            # Cleanup test file
            if Path(test_file).exists():
                Path(test_file).unlink()
                
        except Exception as e:
            print(f"❌ Test {i} failed: {e}")
            manager.log_error(f"Test {i} failed", e)
    
    # Print final status report
    print(f"\n{'='*60}")
    print("📊 Final Status Report")
    print("-" * 60)
    
    status_report = manager.get_status_report()
    print(f"Status: {status_report['status']}")
    print(f"Uptime: {status_report['uptime_formatted']}")
    print(f"Processing steps completed: {status_report['processing_steps']}")
    print(f"Last error: {status_report['last_error'] or 'None'}")
    print(f"Available assets: {status_report['available_assets']}")
    
    if status_report['recent_steps']:
        print(f"\nRecent processing steps:")
        for step in status_report['recent_steps']:
            status_icon = "✅" if step['status'] == 'completed' else "❌" if step['status'] == 'failed' else "🔄"
            print(f"  {status_icon} {step['name']} ({step['status']})")
    
    print(f"\n✅ Testing completed!")
    print(f"💡 Logs are available in /tmp/isaac_logs/")

if __name__ == "__main__":
    main()
