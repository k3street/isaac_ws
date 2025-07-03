#!/usr/bin/env python3
"""
Isaac Sim 5.0 Quick Asset Discovery
Provides fast, efficient asset discovery for production use
"""

import subprocess
import sys
import time
from pathlib import Path

def print_header():
    print("=" * 80)
    print("🚀 Isaac Sim 5.0 Quick Asset Discovery")
    print("   Fast, production-ready asset discovery with timeout protection")
    print("=" * 80)
    print()

def print_option(number, title, description, command, estimated_time):
    print(f"{number}. {title}")
    print(f"   📝 {description}")
    print(f"   ⏱️  Estimated time: {estimated_time}")
    print(f"   💻 Command: {command}")
    print()

def run_discovery(command_args, description):
    print(f"🔍 Running {description}...")
    print(f"💻 Command: python3 asset_downloader.py {' '.join(command_args)}")
    print("-" * 60)
    
    start_time = time.time()
    try:
        result = subprocess.run(
            ["python3", "asset_downloader.py"] + command_args,
            capture_output=False,  # Show output in real-time
            text=True
        )
        elapsed = time.time() - start_time
        
        print("-" * 60)
        if result.returncode == 0:
            print(f"✅ {description} completed successfully in {elapsed:.1f} seconds")
        else:
            print(f"❌ {description} failed with exit code {result.returncode}")
            
        return result.returncode == 0
        
    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n⏹️  Discovery interrupted after {elapsed:.1f} seconds")
        return False
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"❌ Error running {description}: {e}")
        print(f"   Duration before error: {elapsed:.1f} seconds")
        return False

def main():
    print_header()
    
    print("Select discovery mode:")
    print()
    print_option(
        "1", "🤖 Robot Discovery (Recommended)", 
        "Fast discovery of available robot assets using enhanced algorithm",
        "--robots-only --discovery-only --verbose",
        "~60 seconds"
    )
    print_option(
        "2", "🏗️  Non-Environment Discovery", 
        "Discover robots, props, and other assets (skips slow environments)",
        "--discovery-only --skip-environments --verbose", 
        "~5-10 minutes"
    )
    print_option(
        "3", "🌍 Full Discovery (Caution)", 
        "Complete discovery including environments (may take very long)",
        "--discovery-only --verbose",
        "~15+ minutes"
    )
    print_option(
        "4", "📥 Robot Discovery + Download", 
        "Enhanced robot discovery with automatic download",
        "--robots-only --verbose",
        "~2-5 minutes"
    )
    
    print("Enter your choice (1-4), or 'q' to quit: ", end="")
    choice = input().strip().lower()
    
    if choice == 'q':
        print("👋 Goodbye!")
        return
    
    # Command mappings
    commands = {
        "1": (["--robots-only", "--discovery-only", "--verbose"], "Enhanced Robot Discovery"),
        "2": (["--discovery-only", "--skip-environments", "--verbose"], "Non-Environment Asset Discovery"),
        "3": (["--discovery-only", "--verbose"], "Full Asset Discovery"),
        "4": (["--robots-only", "--verbose"], "Robot Discovery with Download")
    }
    
    if choice not in commands:
        print("❌ Invalid choice. Please select 1-4 or 'q' to quit.")
        return
    
    command_args, description = commands[choice]
    
    print()
    print(f"🎯 Starting {description}...")
    
    # Check if asset_downloader.py exists
    if not Path("asset_downloader.py").exists():
        print("❌ Error: asset_downloader.py not found in current directory")
        print("   Please run this script from the isaac_ws directory")
        return
    
    print()
    success = run_discovery(command_args, description)
    
    print()
    if success:
        print("🎉 Discovery completed! Check the output above for detailed results.")
        print()
        print("📁 Local assets are stored in: /home/kimate/isaac_assets/")
        print("📋 Asset catalog: /home/kimate/isaac_assets/asset_catalog.json")
        print("🎬 Scenario config: /home/kimate/isaac_assets/local_scenario_config.json")
    else:
        print("⚠️  Discovery was not completed successfully.")
        print("   You can try again or check the error messages above.")
    
    print()
    print("📖 For more details, see: ASSET_DOWNLOADER_REPORT.md")

if __name__ == "__main__":
    main()
