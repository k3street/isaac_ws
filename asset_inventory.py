#!/usr/bin/env python3
"""
Comprehensive Asset Inventory Report
Shows all downloaded Isaac Sim assets organized by category
"""

import os
import json
from pathlib import Path

def generate_asset_inventory():
    """Generate a comprehensive inventory of all downloaded assets"""
    
    assets_path = Path("/home/kimate/isaac_assets")
    print("🎯 Isaac Sim 5.0 Asset Inventory Report")
    print("=" * 50)
    
    # Find all USD/USDA files
    usd_files = []
    for pattern in ["**/*.usd", "**/*.usda"]:
        usd_files.extend(assets_path.glob(pattern))
    
    print(f"📊 Total Assets Found: {len(usd_files)}")
    print("-" * 30)
    
    # Categorize assets
    categories = {
        "Robots": [],
        "Environments": [],
        "Props": [],
        "Materials": [],
        "Skies": [],
        "Other": []
    }
    
    for file_path in usd_files:
        relative_path = file_path.relative_to(assets_path)
        path_str = str(relative_path)
        
        if "/Robots/" in path_str:
            categories["Robots"].append(relative_path)
        elif "/Environments/" in path_str or "/Environment/" in path_str:
            categories["Environments"].append(relative_path)
        elif "/Props/" in path_str:
            categories["Props"].append(relative_path)
        elif "/Materials/" in path_str:
            categories["Materials"].append(relative_path)
        elif "/Skies/" in path_str:
            categories["Skies"].append(relative_path)
        else:
            categories["Other"].append(relative_path)
    
    # Print detailed breakdown
    for category, assets in categories.items():
        if assets:
            print(f"\n🔧 {category} ({len(assets)} assets):")
            for asset in sorted(assets):
                # Calculate file size
                full_path = assets_path / asset
                size_mb = full_path.stat().st_size / (1024 * 1024)
                print(f"  ✅ {asset} ({size_mb:.1f} MB)")
    
    # Show total disk usage
    total_size = 0
    for file_path in usd_files:
        total_size += file_path.stat().st_size
    
    print(f"\n📊 Storage Summary:")
    print(f"  Total disk usage: {total_size / (1024 * 1024):.1f} MB")
    print(f"  Average file size: {(total_size / len(usd_files)) / (1024 * 1024):.1f} MB")
    
    # Show robot manufacturers
    robot_manufacturers = set()
    for asset in categories["Robots"]:
        parts = str(asset).split("/")
        if len(parts) >= 3 and parts[1] == "Robots":
            robot_manufacturers.add(parts[2])
    
    if robot_manufacturers:
        print(f"\n🤖 Robot Manufacturers Available:")
        for manufacturer in sorted(robot_manufacturers):
            robot_count = len([r for r in categories["Robots"] if f"/Robots/{manufacturer}/" in str(r)])
            print(f"  📍 {manufacturer}: {robot_count} robots")
    
    # Load catalog for additional info
    catalog_file = assets_path / "asset_catalog.json"
    if catalog_file.exists():
        try:
            with open(catalog_file, 'r') as f:
                catalog = json.load(f)
            
            last_update = catalog.get('download_date', 'Unknown')
            stats = catalog.get('download_statistics', {})
            
            print(f"\n📋 Catalog Information:")
            print(f"  Last updated: {last_update}")
            print(f"  Download stats: {stats.get('downloaded', 0)} downloaded, {stats.get('found', 0)} found")
            
        except Exception as e:
            print(f"\n⚠️ Could not load catalog: {e}")
    
    print(f"\n✅ Asset inventory complete!")

if __name__ == "__main__":
    generate_asset_inventory()
