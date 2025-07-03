#!/usr/bin/env python3
"""
Test script for runtime directory traversal functionality
Demonstrates how to use the new runtime subdirectory discovery features
"""

import logging
from asset_downloader import IsaacAssetDownloader

def demo_runtime_traversal():
    """Demonstrate runtime directory traversal capabilities"""
    
    # Set up logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # Create downloader instance
    downloader = IsaacAssetDownloader(
        local_assets_path="/home/kimate/isaac_assets",
        force_update=False,
        discover_assets=True
    )
    
    print("🌐 Isaac Sim Runtime Directory Traversal Demo")
    print("=" * 50)
    
    # Test 1: Traverse NVIDIA robots directory
    print("\n🤖 Test 1: Traversing NVIDIA Robots Directory")
    print("-" * 40)
    
    nvidia_structure = downloader.discover_directory_structure_runtime(
        "/Isaac/Robots/NVIDIA/",
        max_depth=3,
        timeout_per_level=15
    )
    
    print("📁 NVIDIA Robots Directory Structure:")
    downloader.print_directory_tree(nvidia_structure)
    
    # Test 2: Quick directory check
    print("\n🔍 Test 2: Quick Directory Checks")
    print("-" * 30)
    
    test_dirs = [
        "/Isaac/Robots/NVIDIA/Carter/",
        "/Isaac/Robots/NVIDIA/Jetbot/",
        "/Isaac/Robots/Clearpath/Dingo/",
        "/Isaac/Robots/NonExistent/"
    ]
    
    for dir_path in test_dirs:
        exists = downloader.quick_directory_check(dir_path, timeout=5)
        status = "✅ EXISTS" if exists else "❌ NOT FOUND"
        print(f"  {status}: {dir_path}")
    
    # Test 3: Directory listing attempt
    print("\n📋 Test 3: HTTP Directory Listing")
    print("-" * 30)
    
    listing_paths = [
        "/Isaac/Robots/",
        "/Isaac/Robots/NVIDIA/",
        "/Isaac/Environments/"
    ]
    
    for path in listing_paths:
        print(f"\n  Attempting directory listing for: {path}")
        listing = downloader.get_directory_listing_http(path, timeout=10)
        
        if listing:
            print(f"    ✅ Found {len(listing)} items:")
            for item in listing[:5]:  # Show first 5 items
                print(f"      - {item}")
            if len(listing) > 5:
                print(f"      ... and {len(listing) - 5} more items")
        else:
            print(f"    ❌ No directory listing available")
    
    print("\n🎯 Runtime traversal demo complete!")

def test_specific_path_traversal():
    """Test traversal of a specific user-defined path"""
    
    downloader = IsaacAssetDownloader()
    
    # Let user specify a path to traverse
    test_paths = [
        "/Isaac/Robots/UniversalRobots/",
        "/Isaac/Robots/Clearpath/",
        "/Isaac/Props/"
    ]
    
    print("\n🔍 Testing specific path traversal...")
    
    for path in test_paths:
        print(f"\n📁 Traversing: {path}")
        try:
            structure = downloader.discover_directory_structure_runtime(
                path, 
                max_depth=2,
                timeout_per_level=10
            )
            
            if structure["subdirectories"] or structure["assets"]:
                downloader.print_directory_tree(structure)
            else:
                print("  📭 No subdirectories or assets found")
                
        except Exception as e:
            print(f"  ❌ Error traversing {path}: {e}")

if __name__ == "__main__":
    print("🚀 Starting Isaac Sim Runtime Directory Traversal Tests")
    
    try:
        demo_runtime_traversal()
        test_specific_path_traversal()
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
