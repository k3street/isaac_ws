#!/usr/bin/env python3
"""
Comprehensive Scenario System Test
Tests all scenario management functionality
"""

import unittest
import json
import os
import time
from pathlib import Path
from scenario_manager import ScenarioManager

class TestScenarioSystem(unittest.TestCase):
    """Test suite for scenario management system"""
    
    def setUp(self):
        """Set up test environment"""
        self.scenario_manager = ScenarioManager()
        self.test_config_file = Path("/tmp/test_scenario_config.json")
        
        # Clean up any existing test files
        if self.test_config_file.exists():
            self.test_config_file.unlink()
    
    def tearDown(self):
        """Clean up after tests"""
        if self.test_config_file.exists():
            self.test_config_file.unlink()
    
    def test_scenario_manager_initialization(self):
        """Test scenario manager initializes correctly"""
        self.assertIsNotNone(self.scenario_manager)
        self.assertIsInstance(self.scenario_manager.available_scenes, dict)
        self.assertIsInstance(self.scenario_manager.available_robots, dict)
        self.assertGreater(len(self.scenario_manager.available_scenes), 0)
        self.assertGreater(len(self.scenario_manager.available_robots), 0)
    
    def test_simple_scenario_parsing(self):
        """Test parsing simple scenario requests"""
        request = "warehouse scene with carter robot"
        config = self.scenario_manager.generate_scenario_config(request)
        
        self.assertIsNotNone(config)
        self.assertIn('parsed_scenario', config)
        
        scenario = config['parsed_scenario']
        self.assertEqual(scenario['scene'], 'warehouse')
        self.assertEqual(len(scenario['robots']), 1)
        self.assertEqual(scenario['robots'][0]['type'], 'carter')
    
    def test_complex_scenario_parsing(self):
        """Test parsing complex scenario requests"""
        request = "launch a warehouse scene with a carter 2.0 robot that is fully ros controlled"
        config = self.scenario_manager.generate_scenario_config(request)
        
        scenario = config['parsed_scenario']
        self.assertEqual(scenario['scene'], 'warehouse')
        self.assertEqual(len(scenario['robots']), 1)
        self.assertEqual(scenario['robots'][0]['type'], 'carter_2')
        self.assertTrue(scenario['robots'][0]['ros_enabled'])
        self.assertIn('ros_control', scenario['special_requirements'])
    
    def test_multiple_robots_parsing(self):
        """Test parsing requests with multiple robots"""
        request = "office with franka arm and carter robot"
        config = self.scenario_manager.generate_scenario_config(request)
        
        scenario = config['parsed_scenario']
        self.assertEqual(scenario['scene'], 'office')
        self.assertEqual(len(scenario['robots']), 2)
        
        robot_types = [robot['type'] for robot in scenario['robots']]
        self.assertIn('franka', robot_types)
        self.assertIn('carter', robot_types)
    
    def test_camera_position_parsing(self):
        """Test parsing camera position requests"""
        request = "simple room with overhead camera view"
        config = self.scenario_manager.generate_scenario_config(request)
        
        scenario = config['parsed_scenario']
        self.assertIsNotNone(scenario['camera_position'])
        self.assertEqual(scenario['camera_position'], [0, 0, 10])  # overhead position
    
    def test_unknown_scene_fallback(self):
        """Test fallback for unknown scenes"""
        request = "mars colony with alien robots"  # Non-existent scene
        config = self.scenario_manager.generate_scenario_config(request)
        
        scenario = config['parsed_scenario']
        self.assertEqual(scenario['scene'], 'simple_room')  # Should fallback
    
    def test_scenario_validation(self):
        """Test scenario validation"""
        # Valid scenario
        valid_scenario = {
            'scene': 'warehouse',
            'robots': [{'type': 'carter', 'name': 'carter_robot', 'position': [0, 0, 0]}],
            'props': [],
            'camera_position': None,
            'special_requirements': []
        }
        
        is_valid, warnings = self.scenario_manager.validate_scenario(valid_scenario)
        self.assertTrue(is_valid)
        self.assertEqual(len(warnings), 0)
        
        # Invalid scenario
        invalid_scenario = {
            'scene': 'nonexistent_scene',
            'robots': [{'type': 'nonexistent_robot', 'name': 'fake_robot', 'position': [0, 0, 0]}],
            'props': [],
            'camera_position': None,
            'special_requirements': []
        }
        
        is_valid, warnings = self.scenario_manager.validate_scenario(invalid_scenario)
        self.assertFalse(is_valid)
        self.assertGreater(len(warnings), 0)
    
    def test_config_save_load(self):
        """Test saving and loading scenario configurations"""
        request = "warehouse with carter 2.0 robot"
        config = self.scenario_manager.generate_scenario_config(request)
        
        # Save config
        self.scenario_manager.save_scenario_config(config, str(self.test_config_file))
        self.assertTrue(self.test_config_file.exists())
        
        # Load config
        loaded_config = self.scenario_manager.load_scenario_config(str(self.test_config_file))
        self.assertIsNotNone(loaded_config)
        self.assertEqual(loaded_config['request'], request)
        self.assertEqual(loaded_config['parsed_scenario']['scene'], 'warehouse')
    
    def test_available_assets_completeness(self):
        """Test that all available assets have required fields"""
        # Test scenes
        for scene_name, scene_info in self.scenario_manager.available_scenes.items():
            self.assertIn('path', scene_info)
            self.assertIn('description', scene_info)
            self.assertIsInstance(scene_info['path'], str)
            self.assertIsInstance(scene_info['description'], str)
        
        # Test robots
        for robot_name, robot_info in self.scenario_manager.available_robots.items():
            self.assertIn('path', robot_info)
            self.assertIn('description', robot_info)
            self.assertIn('ros_enabled', robot_info)
            self.assertIn('default_position', robot_info)
            self.assertIsInstance(robot_info['ros_enabled'], bool)
            self.assertIsInstance(robot_info['default_position'], list)
            self.assertEqual(len(robot_info['default_position']), 3)

def run_integration_tests():
    """Run integration tests that require file system"""
    print("🧪 Running Integration Tests...")
    
    # Test file creation and cleanup
    test_files = [
        "/tmp/isaac_scenario_config.json",
        "/tmp/isaac_camera_commands.json"
    ]
    
    # Test scenario manager with file operations
    manager = ScenarioManager()
    
    # Test scenario generation and file saving
    request = "test scenario with carter robot"
    config = manager.generate_scenario_config(request)
    manager.save_scenario_config(config)
    
    # Verify file was created
    config_file = Path("/tmp/isaac_scenario_config.json")
    if config_file.exists():
        print("✅ Scenario config file creation: PASS")
        
        # Test loading
        loaded_config = manager.load_scenario_config()
        if loaded_config and loaded_config['request'] == request:
            print("✅ Scenario config file loading: PASS")
        else:
            print("❌ Scenario config file loading: FAIL")
        
        # Cleanup
        config_file.unlink()
    else:
        print("❌ Scenario config file creation: FAIL")
    
    print("✅ Integration tests completed")

def main():
    """Run all tests"""
    print("🚀 Isaac Sim Scenario System Test Suite")
    print("=" * 50)
    
    # Run unit tests
    print("\n📋 Running Unit Tests...")
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    # Run integration tests
    print("\n🔧 Running Integration Tests...")
    run_integration_tests()
    
    print("\n✅ All tests completed!")
    print("\n💡 Next steps:")
    print("   1. Test scenario CLI: python3 scenario_cli.py examples")
    print("   2. Start scenario service: python3 scenario_service_node.py")
    print("   3. Launch with scenarios: ./launch_camera_control_with_scenarios.sh")

if __name__ == '__main__':
    main()
