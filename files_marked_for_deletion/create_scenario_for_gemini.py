#!/usr/bin/env python3
"""
Create a specific scenario for Gemini camera control testing
Uses the ScenarioManager to create and save a scenario configuration
"""

import sys
import os
import json
from pathlib import Path

# Add the workspace to Python path
sys.path.append('/home/kimate/isaac_ws')

from scenario_manager import ScenarioManager

def create_gemini_test_scenario():
    """Create a scenario specifically for testing Gemini camera control"""
    
    print("🎯 Creating scenario for Gemini camera control testing...")
    
    # Initialize scenario manager
    manager = ScenarioManager(enable_logging=True)
    
    # Define a scenario request for testing camera control
    scenario_request = """
    I want to create a warehouse scene with a Carter robot positioned at coordinates (2, 1, 0).
    Add some traffic cones at positions (5, 2, 0), (-3, -1, 0), and (0, 4, 0) as obstacles.
    Include some boxes scattered around at (3, -2, 0), (-2, 3, 0), and (1, -1, 0).
    The robot should have cameras enabled for Gemini vision analysis.
    This scenario is for testing LLM-powered camera control and navigation assistance.
    """
    
    print(f"📝 Scenario request: {scenario_request}")
    
    # Generate scenario configuration
    try:
        config = manager.generate_scenario_config(scenario_request)
        
        print("\n📊 Generated scenario configuration:")
        print(json.dumps(config, indent=2))
        
        # Save the configuration
        scenario_file = "/tmp/gemini_camera_test_scenario.json"
        success = manager.save_scenario_config(config, scenario_file)
        
        if success:
            print(f"\n✅ Scenario saved to: {scenario_file}")
            
            # Verify file exists and is readable
            if os.path.exists(scenario_file):
                print(f"✅ File confirmed to exist at: {scenario_file}")
                with open(scenario_file, 'r') as f:
                    saved_config = json.load(f)
                print(f"✅ File is readable, contains {len(saved_config)} top-level keys")
            else:
                print(f"❌ File not found at expected location: {scenario_file}")
                
        else:
            print(f"❌ Failed to save scenario configuration")
            
        # Display status report
        status = manager.get_status_report()
        print(f"\n📈 Scenario Manager Status:")
        print(f"   Status: {status['status']}")
        print(f"   Processing steps: {status['processing_steps']}")
        print(f"   Uptime: {status['uptime_formatted']}")
        
        return config
        
    except Exception as e:
        print(f"❌ Error creating scenario: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def main():
    """Main function"""
    print("🚀 Starting Gemini scenario creation...")
    
    config = create_gemini_test_scenario()
    
    if config:
        print("\n🎉 Scenario creation completed successfully!")
        print("\nNext steps:")
        print("1. Launch Isaac Sim with this scenario")
        print("2. Start the Gemini camera controller ROS2 node")
        print("3. Test camera control via Gemini vision analysis")
    else:
        print("\n❌ Scenario creation failed!")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
