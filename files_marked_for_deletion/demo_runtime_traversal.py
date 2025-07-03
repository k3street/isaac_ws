#!/usr/bin/env python3
"""
Runtime Directory Traversal Demo
Demonstrates the new dynamic subdirectory discovery capabilities
"""

import subprocess
import time

def run_traversal_demo():
    """Run a comprehensive demo of the runtime traversal features"""
    
    print("🌐 Isaac Sim Runtime Directory Traversal Demo")
    print("=" * 55)
    print()
    
    demos = [
        {
            "name": "🤖 NVIDIA Robots Directory (Depth 2)",
            "command": [
                "python3", "asset_downloader.py", 
                "--runtime-traverse", 
                "--traverse-path", "/Isaac/Robots/NVIDIA/",
                "--max-depth", "2",
                "--verbose"
            ],
            "description": "Explore NVIDIA robot directory structure with depth limit of 2"
        },
        {
            "name": "🏭 Universal Robots Directory (Depth 1)", 
            "command": [
                "python3", "asset_downloader.py",
                "--runtime-traverse",
                "--traverse-path", "/Isaac/Robots/UniversalRobots/",
                "--max-depth", "1",
                "--verbose"
            ],
            "description": "Quick exploration of Universal Robots directory"
        },
        {
            "name": "🔍 All Robot Categories (Skip Environments)",
            "command": [
                "python3", "asset_downloader.py",
                "--runtime-traverse",
                "--skip-environments", 
                "--max-depth", "2",
                "--verbose"
            ],
            "description": "Traverse all Isaac Sim categories except environments"
        }
    ]
    
    for i, demo in enumerate(demos, 1):
        print(f"Demo {i}: {demo['name']}")
        print(f"Description: {demo['description']}")
        print(f"Command: {' '.join(demo['command'])}")
        print("-" * 50)
        
        try:
            start_time = time.time()
            result = subprocess.run(
                demo['command'], 
                cwd="/home/kimate/isaac_ws",
                capture_output=True,
                text=True,
                timeout=60  # 1 minute timeout per demo
            )
            
            elapsed = time.time() - start_time
            
            if result.returncode == 0:
                print("✅ Demo completed successfully!")
                print(f"⏱️  Execution time: {elapsed:.1f} seconds")
                
                # Show last few lines of output
                output_lines = result.stdout.strip().split('\n')
                if len(output_lines) > 10:
                    print("\n📋 Sample output (last 10 lines):")
                    for line in output_lines[-10:]:
                        print(f"  {line}")
                else:
                    print(f"\n📋 Output:\n{result.stdout}")
                    
            else:
                print(f"❌ Demo failed with return code: {result.returncode}")
                if result.stderr:
                    print(f"Error: {result.stderr}")
                    
        except subprocess.TimeoutExpired:
            print("⏰ Demo timed out after 60 seconds")
        except Exception as e:
            print(f"❌ Demo failed with error: {e}")
            
        print("\n" + "=" * 55 + "\n")
        
        # Small delay between demos
        time.sleep(1)
    
    print("🎯 All runtime traversal demos completed!")

def show_usage_examples():
    """Show practical usage examples for runtime traversal"""
    
    print("📖 Runtime Directory Traversal Usage Examples")
    print("=" * 50)
    print()
    
    examples = [
        {
            "scenario": "Explore a specific manufacturer's robots",
            "command": "python3 asset_downloader.py --runtime-traverse --traverse-path '/Isaac/Robots/Clearpath/' --max-depth 3",
            "use_case": "When you want to see all available robots from a specific manufacturer"
        },
        {
            "scenario": "Quick overview of all asset categories",
            "command": "python3 asset_downloader.py --runtime-traverse --max-depth 1 --skip-environments",
            "use_case": "Get a high-level view of what asset categories are available"
        },
        {
            "scenario": "Deep dive into props directory",
            "command": "python3 asset_downloader.py --runtime-traverse --traverse-path '/Isaac/Props/' --max-depth 4",
            "use_case": "Thoroughly explore all available props and their subcategories"
        },
        {
            "scenario": "Find all USD files in a directory tree",
            "command": "python3 asset_downloader.py --runtime-traverse --traverse-path '/Isaac/Robots/NVIDIA/' --verbose",
            "use_case": "Locate all available USD assets within a specific directory hierarchy"
        }
    ]
    
    for i, example in enumerate(examples, 1):
        print(f"Example {i}: {example['scenario']}")
        print(f"Command: {example['command']}")
        print(f"Use case: {example['use_case']}")
        print("-" * 40)
        print()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--demo":
        print("🚀 Running interactive demo...")
        run_traversal_demo()
    else:
        print("📚 Showing usage examples...")
        show_usage_examples()
        print("\nTo run interactive demos, use: python3 demo_runtime_traversal.py --demo")
