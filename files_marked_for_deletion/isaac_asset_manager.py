#!/usr/bin/env python3
"""
Isaac Sim 5.0 Comprehensive Asset Manager
Handles both official asset packs and individual asset discovery/download
Provides a unified interface for Isaac Sim asset management
"""

import os
import json
import time
import urllib.request
import urllib.error
from pathlib import Path
import logging
import hashlib
from datetime import datetime
import zipfile
import shutil
import argparse
import tempfile
import subprocess

class IsaacAssetManager:
    def __init__(self, local_assets_path="/home/kimate/isaac_assets", force_update=False):
        self.local_assets_path = Path(local_assets_path)
        self.force_update = force_update
        
        # Isaac Sim 5.0 configuration
        self.isaac_version = "5.0.0"
        self.asset_root_path = self.local_assets_path / "Assets" / "Isaac" / "5.0"
        
        # Create directory structure
        self.local_assets_path.mkdir(parents=True, exist_ok=True)
        self.asset_root_path.mkdir(parents=True, exist_ok=True)
        
        # Set up logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Asset management files
        self.catalog_file = self.local_assets_path / "comprehensive_asset_catalog.json"
        self.manual_packs_dir = self.local_assets_path / "manual_packs"
        self.manual_packs_dir.mkdir(exist_ok=True)
        
        # Known working asset URLs (from our previous testing)
        self.working_individual_assets = {
            "robots": {
                "carter_v1": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Carter/carter_v1.usd",
                "jetbot": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
                "leatherback": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd",
                "dingo": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Clearpath/Dingo/dingo.usd",
                "jackal": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Clearpath/Jackal/jackal.usd",
                "iw_hub": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
                "iw_hub_sensors": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Idealworks/iwhub/iw_hub_sensors.usd",
                "iw_hub_static": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd",
                "create_3": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/iRobot/Create3/create_3.usd",
                "ur10": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
                "ur5": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/UniversalRobots/ur5/ur5.usd",
                "ur3": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/UniversalRobots/ur3/ur3.usd"
            }
        }
    
    def check_manual_asset_packs(self):
        """Check for manually downloaded asset packs"""
        manual_packs = []
        
        # Look for common asset pack naming patterns
        patterns = [
            "isaac-sim-assets-*.zip",
            "isaac_sim_assets_*.zip", 
            "IsaacSim-Assets-*.zip",
            "assets-*.zip"
        ]
        
        for pattern in patterns:
            manual_packs.extend(self.manual_packs_dir.glob(pattern))
            manual_packs.extend(self.local_assets_path.glob(pattern))
            # Also check Downloads directory
            downloads_dir = Path.home() / "Downloads"
            if downloads_dir.exists():
                manual_packs.extend(downloads_dir.glob(pattern))
        
        if manual_packs:
            self.logger.info(f"📦 Found {len(manual_packs)} potential asset pack(s):")
            for pack in manual_packs:
                size_mb = pack.stat().st_size / (1024 * 1024)
                self.logger.info(f"    {pack.name} ({size_mb:.1f} MB) - {pack.parent}")
        
        return manual_packs
    
    def extract_manual_packs(self, pack_files):
        """Extract manually downloaded asset packs"""
        extracted_count = 0
        
        for pack_file in pack_files:
            try:
                self.logger.info(f"📦 Extracting {pack_file.name}...")
                
                with zipfile.ZipFile(pack_file, 'r') as zip_ref:
                    # Check archive contents first
                    file_list = zip_ref.namelist()
                    
                    # Look for Isaac assets structure
                    isaac_files = [f for f in file_list if 'Isaac' in f and f.endswith('.usd')]
                    if isaac_files:
                        self.logger.info(f"    Found {len(isaac_files)} Isaac USD files")
                        
                        # Extract to our asset root
                        zip_ref.extractall(self.local_assets_path)
                        extracted_count += 1
                        
                        self.logger.info(f"✅ Successfully extracted {pack_file.name}")
                    else:
                        self.logger.warning(f"⚠️ No Isaac USD files found in {pack_file.name}")
                        
            except zipfile.BadZipFile:
                self.logger.error(f"❌ Invalid zip file: {pack_file.name}")
            except Exception as e:
                self.logger.error(f"❌ Failed to extract {pack_file.name}: {e}")
        
        return extracted_count
    
    def download_individual_assets(self, asset_category="robots"):
        """Download individual assets using known working URLs"""
        if asset_category not in self.working_individual_assets:
            self.logger.warning(f"⚠️ No known assets for category: {asset_category}")
            return 0
        
        assets = self.working_individual_assets[asset_category]
        downloaded_count = 0
        
        self.logger.info(f"📥 Downloading {len(assets)} individual {asset_category}...")
        
        for asset_name, asset_url in assets.items():
            try:
                # Determine local path from URL
                url_path = asset_url.split("/Assets/Isaac/5.0/", 1)[1]
                local_path = self.asset_root_path / url_path
                
                # Check if already exists
                if local_path.exists() and not self.force_update:
                    self.logger.info(f"⏭️ Skipping {asset_name} (already exists)")
                    continue
                
                # Create directory structure
                local_path.parent.mkdir(parents=True, exist_ok=True)
                
                # Download
                self.logger.info(f"📥 Downloading {asset_name}...")
                
                response = urllib.request.urlopen(asset_url, timeout=30)
                
                with open(local_path, 'wb') as f:
                    shutil.copyfileobj(response, f)
                
                if local_path.exists():
                    size_kb = local_path.stat().st_size / 1024
                    self.logger.info(f"✅ Downloaded {asset_name} ({size_kb:.1f} KB)")
                    downloaded_count += 1
                else:
                    self.logger.error(f"❌ Failed to create {asset_name}")
                    
            except Exception as e:
                self.logger.error(f"❌ Failed to download {asset_name}: {e}")
        
        return downloaded_count
    
    def create_basic_asset_structure(self):
        """Create basic asset directory structure with downloaded assets"""
        
        # Create standard Isaac Sim directory structure
        directories = [
            "Isaac/Robots",
            "Isaac/Environments", 
            "Isaac/Props",
            "Isaac/Materials",
            "Isaac/Samples",
            "Isaac/People",
            "Isaac/Sensors",
            "Isaac/IsaacLab"
        ]
        
        for directory in directories:
            dir_path = self.asset_root_path / directory
            dir_path.mkdir(parents=True, exist_ok=True)
        
        self.logger.info("📁 Created basic asset directory structure")
    
    def verify_and_catalog_assets(self):
        """Verify and catalog all available assets"""
        
        asset_catalog = {
            "isaac_sim_version": self.isaac_version,
            "asset_root_path": str(self.asset_root_path),
            "scan_date": datetime.now().isoformat(),
            "assets": {}
        }
        
        # Scan for assets in each category
        categories = ["Robots", "Environments", "Props", "Materials", "Samples", "People", "Sensors"]
        
        for category in categories:
            category_path = self.asset_root_path / "Isaac" / category
            if category_path.exists():
                
                # Find all USD files
                usd_files = list(category_path.rglob("*.usd*"))
                
                asset_catalog["assets"][category] = {
                    "path": str(category_path),
                    "asset_count": len(usd_files),
                    "assets": []
                }
                
                for usd_file in usd_files:
                    relative_path = usd_file.relative_to(self.asset_root_path)
                    asset_info = {
                        "name": usd_file.stem,
                        "path": str(relative_path),
                        "full_path": str(usd_file),
                        "size": usd_file.stat().st_size,
                        "modified": datetime.fromtimestamp(usd_file.stat().st_mtime).isoformat()
                    }
                    asset_catalog["assets"][category]["assets"].append(asset_info)
                
                self.logger.info(f"📊 {category}: {len(usd_files)} assets found")
        
        # Save catalog
        with open(self.catalog_file, 'w') as f:
            json.dump(asset_catalog, f, indent=2)
        
        self.logger.info(f"💾 Asset catalog saved: {self.catalog_file}")
        return asset_catalog
    
    def generate_isaac_sim_config(self):
        """Generate Isaac Sim configuration files"""
        
        # Kit configuration
        kit_config = f"""[settings]
persistent.isaac.asset_root.default = "{self.asset_root_path}"

exts."isaacsim.asset.browser".folders = [
    "{self.asset_root_path}/Isaac/Robots",
    "{self.asset_root_path}/Isaac/People",
    "{self.asset_root_path}/Isaac/IsaacLab", 
    "{self.asset_root_path}/Isaac/Props",
    "{self.asset_root_path}/Isaac/Environments",
    "{self.asset_root_path}/Isaac/Materials",
    "{self.asset_root_path}/Isaac/Samples",
    "{self.asset_root_path}/Isaac/Sensors",
]"""
        
        kit_file = self.local_assets_path / "isaac_sim_local_assets.kit"
        with open(kit_file, 'w') as f:
            f.write(kit_config)
        
        # Launch script
        launch_script = f"""#!/bin/bash
# Isaac Sim 5.0 Local Assets Launch Script

ISAAC_SIM_PATH="/home/kimate/isaacsim"
ASSETS_ROOT="{self.asset_root_path}"

echo "🚀 Isaac Sim 5.0 with Local Assets"
echo "📁 Assets: $ASSETS_ROOT"

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "❌ Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Please update ISAAC_SIM_PATH variable"
    exit 1
fi

if [ ! -d "$ASSETS_ROOT/Isaac" ]; then
    echo "⚠️ Local assets not found at $ASSETS_ROOT"
    echo "Assets will be downloaded from remote server"
fi

cd "$ISAAC_SIM_PATH"
./isaac-sim.sh --/persistent/isaac/asset_root/default="$ASSETS_ROOT"
"""
        
        launch_file = self.local_assets_path / "launch_isaac_sim_local.sh"
        with open(launch_file, 'w') as f:
            f.write(launch_script)
        launch_file.chmod(0o755)
        
        # Python configuration for scenario manager
        scenario_config = {
            "available_scenes": {},
            "available_robots": {},
            "available_props": {}
        }
        
        # Add available robots to scenario config
        robots_dir = self.asset_root_path / "Isaac/Robots"
        if robots_dir.exists():
            for robot_file in robots_dir.rglob("*.usd"):
                robot_name = robot_file.stem
                scenario_config["available_robots"][robot_name] = {
                    "path": f"file://{robot_file}",
                    "description": f"Local {robot_name} robot",
                    "ros_enabled": True,
                    "default_position": [0, 0, 0]
                }
        
        # Add available environments
        envs_dir = self.asset_root_path / "Isaac/Environments"
        if envs_dir.exists():
            for env_file in envs_dir.rglob("*.usd"):
                env_name = env_file.stem
                scenario_config["available_scenes"][env_name] = {
                    "path": f"file://{env_file}",
                    "description": f"Local {env_name} environment"
                }
        
        scenario_file = self.local_assets_path / "local_scenario_config.json"
        with open(scenario_file, 'w') as f:
            json.dump(scenario_config, f, indent=2)
        
        self.logger.info("⚙️ Generated configuration files:")
        self.logger.info(f"    Kit config: {kit_file}")
        self.logger.info(f"    Launch script: {launch_file}")
        self.logger.info(f"    Scenario config: {scenario_file}")
        
        return kit_file, launch_file, scenario_file
    
    def print_usage_instructions(self):
        """Print detailed usage instructions"""
        
        self.logger.info("\n" + "="*80)
        self.logger.info("🎉 ISAAC SIM 5.0 ASSET SETUP COMPLETE!")
        self.logger.info("="*80)
        
        self.logger.info(f"\n📁 ASSET LOCATIONS:")
        self.logger.info(f"    Asset root: {self.asset_root_path}")
        self.logger.info(f"    Robots: {self.asset_root_path}/Isaac/Robots")
        self.logger.info(f"    Environments: {self.asset_root_path}/Isaac/Environments")
        self.logger.info(f"    Configuration: {self.local_assets_path}")
        
        self.logger.info(f"\n🚀 HOW TO USE LOCAL ASSETS:")
        
        self.logger.info(f"\n1. LAUNCH ISAAC SIM WITH LOCAL ASSETS:")
        launch_script = self.local_assets_path / "launch_isaac_sim_local.sh"
        self.logger.info(f"   {launch_script}")
        
        self.logger.info(f"\n2. OR MANUALLY SET ASSET ROOT:")
        self.logger.info(f"   cd /home/kimate/isaacsim")
        self.logger.info(f"   ./isaac-sim.sh --/persistent/isaac/asset_root/default=\"{self.asset_root_path}\"")
        
        self.logger.info(f"\n3. VERIFY IN ISAAC SIM:")
        self.logger.info(f"   - Open Isaac Sim Asset Browser")
        self.logger.info(f"   - Click the 'Gear' icon")
        self.logger.info(f"   - Select 'Check Default Assets Root Path'")
        self.logger.info(f"   - Should show: {self.asset_root_path}")
        
        self.logger.info(f"\n4. USE WITH SCENARIO MANAGER:")
        self.logger.info(f"   The scenario manager will automatically use:")
        self.logger.info(f"   {self.local_assets_path}/local_scenario_config.json")
        
        self.logger.info(f"\n📦 TO ADD MORE ASSETS:")
        self.logger.info(f"   1. Download official asset packs to: {self.manual_packs_dir}")
        self.logger.info(f"   2. Re-run this tool to extract them")
        self.logger.info(f"   3. Or use: python3 asset_downloader.py --robots-only")
        
        self.logger.info(f"\n🔧 TROUBLESHOOTING:")
        self.logger.info(f"   - If assets don't appear, check the asset root path setting")
        self.logger.info(f"   - Verify USD files exist in {self.asset_root_path}/Isaac/")
        self.logger.info(f"   - Check Isaac Sim logs for asset loading errors")
        
    def run(self, mode="comprehensive"):
        """Run the comprehensive asset management process"""
        
        self.logger.info("🚀 Isaac Sim 5.0 Comprehensive Asset Manager")
        self.logger.info(f"📁 Local assets path: {self.local_assets_path}")
        self.logger.info(f"🎯 Asset root: {self.asset_root_path}")
        self.logger.info(f"🔄 Mode: {mode}")
        
        total_assets_obtained = 0
        
        # Create basic directory structure
        self.create_basic_asset_structure()
        
        # Check for manual asset packs
        manual_packs = self.check_manual_asset_packs()
        if manual_packs:
            self.logger.info(f"\n📦 Processing {len(manual_packs)} manual asset pack(s)...")
            extracted = self.extract_manual_packs(manual_packs)
            total_assets_obtained += extracted
            self.logger.info(f"✅ Extracted {extracted} asset pack(s)")
        
        # Download individual assets if enabled
        if mode in ["comprehensive", "individual", "robots"]:
            self.logger.info(f"\n📥 Downloading individual robot assets...")
            downloaded = self.download_individual_assets("robots")
            total_assets_obtained += downloaded
            self.logger.info(f"✅ Downloaded {downloaded} individual assets")
        
        # Verify and catalog all assets
        self.logger.info(f"\n🔍 Scanning and cataloging assets...")
        catalog = self.verify_and_catalog_assets()
        
        # Generate configuration files
        self.logger.info(f"\n⚙️ Generating Isaac Sim configuration...")
        self.generate_isaac_sim_config()
        
        # Print usage instructions
        self.print_usage_instructions()
        
        return total_assets_obtained > 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim 5.0 Comprehensive Asset Manager")
    parser.add_argument("--assets-path", default="/home/kimate/isaac_assets",
                       help="Local path to store assets")
    parser.add_argument("--force-update", action="store_true",
                       help="Force re-download/re-extract assets")
    parser.add_argument("--mode", choices=["comprehensive", "manual-only", "individual", "robots"],
                       default="comprehensive", help="Asset management mode")
    parser.add_argument("--verbose", action="store_true",
                       help="Enable verbose logging")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    manager = IsaacAssetManager(
        local_assets_path=args.assets_path,
        force_update=args.force_update
    )
    
    success = manager.run(mode=args.mode)
    exit(0 if success else 1)
