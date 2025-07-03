#!/usr/bin/env python3
"""
Isaac Sim 5.0 Official Asset Pack Downloader
Downloads and extracts official Isaac Sim asset packs as recommended by NVIDIA
Based on: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#isaac-sim-setup-assets-content-pack
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

class IsaacAssetPackDownloader:
    def __init__(self, local_assets_path="/home/kimate/isaac_assets", force_update=False):
        self.local_assets_path = Path(local_assets_path)
        self.force_update = force_update
        
        # Isaac Sim 5.0 asset pack information
        self.isaac_version = "5.0.0"
        self.asset_root_path = self.local_assets_path / "Assets" / "Isaac" / "5.0"
        
        # Official asset pack URLs for Isaac Sim 5.0
        # Note: These URLs follow the pattern from the documentation but may need verification
        self.asset_pack_base_url = "https://github.com/NVIDIA-Omniverse/IsaacSim/releases/download/5.0.0"
        
        # Asset pack definitions (based on the 4.5.0 pattern, adapted for 5.0)
        self.asset_packs = [
            {
                "name": "isaac-sim-assets-1",
                "filename": f"isaac-sim-assets-1@{self.isaac_version}.zip",
                "url": f"{self.asset_pack_base_url}/isaac-sim-assets-1@{self.isaac_version}.zip",
                "description": "Core Isaac Sim assets pack 1"
            },
            {
                "name": "isaac-sim-assets-2", 
                "filename": f"isaac-sim-assets-2@{self.isaac_version}.zip",
                "url": f"{self.asset_pack_base_url}/isaac-sim-assets-2@{self.isaac_version}.zip",
                "description": "Core Isaac Sim assets pack 2"
            },
            {
                "name": "isaac-sim-assets-3",
                "filename": f"isaac-sim-assets-3@{self.isaac_version}.zip", 
                "url": f"{self.asset_pack_base_url}/isaac-sim-assets-3@{self.isaac_version}.zip",
                "description": "Core Isaac Sim assets pack 3"
            }
        ]
        
        # Alternative official content URLs (fallback)
        self.alternative_urls = [
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/isaac-sim-assets-1@5.0.0.zip",
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/isaac-sim-assets-2@5.0.0.zip", 
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/isaac-sim-assets-3@5.0.0.zip"
        ]
        
        # Create local assets directory structure
        self.local_assets_path.mkdir(parents=True, exist_ok=True)
        self.asset_root_path.mkdir(parents=True, exist_ok=True)
        
        # Set up logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Asset catalog for tracking
        self.catalog_file = self.local_assets_path / "asset_pack_catalog.json"
        
    def check_existing_installation(self):
        """Check if Isaac Sim assets are already installed"""
        expected_folders = ["Isaac/Robots", "Isaac/Environments", "Isaac/Props", "Isaac/Materials", "Isaac/Samples"]
        
        installed_folders = []
        for folder in expected_folders:
            folder_path = self.asset_root_path / folder
            if folder_path.exists() and any(folder_path.iterdir()):
                installed_folders.append(folder)
                
        if len(installed_folders) == len(expected_folders):
            self.logger.info("✅ Complete Isaac Sim asset installation detected")
            return True, installed_folders
        elif installed_folders:
            self.logger.warning(f"⚠️ Partial installation detected: {len(installed_folders)}/{len(expected_folders)} folders")
            return False, installed_folders
        else:
            self.logger.info("📦 No existing asset installation found")
            return False, []
    
    def find_working_asset_url(self, pack_info):
        """Find a working download URL for an asset pack"""
        urls_to_try = [
            pack_info["url"],
            # Try alternative S3 URLs
            f"https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/{pack_info['filename']}",
            f"https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/{pack_info['filename']}",
            # Try direct download from NVIDIA
            f"https://developer.nvidia.com/isaac-sim-assets/{pack_info['filename']}",
        ]
        
        for url in urls_to_try:
            try:
                self.logger.info(f"🔍 Testing URL: {url}")
                request = urllib.request.Request(url, method='HEAD')
                response = urllib.request.urlopen(request, timeout=10)
                
                if response.status == 200:
                    content_length = response.headers.get('Content-Length')
                    if content_length:
                        size_mb = int(content_length) / (1024 * 1024)
                        self.logger.info(f"✅ Working URL found: {url} (Size: {size_mb:.1f} MB)")
                        return url, int(content_length)
                    else:
                        self.logger.info(f"✅ Working URL found: {url} (Size: Unknown)")
                        return url, 0
                        
            except urllib.error.HTTPError as e:
                self.logger.debug(f"❌ HTTP {e.code} for {url}")
            except urllib.error.URLError as e:
                self.logger.debug(f"❌ URL Error for {url}: {e.reason}")
            except Exception as e:
                self.logger.debug(f"❌ Error testing {url}: {e}")
                
        return None, 0
    
    def download_asset_pack(self, pack_info, download_dir):
        """Download a single asset pack"""
        
        # Find working URL
        working_url, file_size = self.find_working_asset_url(pack_info)
        
        if not working_url:
            self.logger.error(f"❌ No working download URL found for {pack_info['name']}")
            self.logger.info("💡 Possible solutions:")
            self.logger.info("   1. Check your internet connection")
            self.logger.info("   2. The asset pack may not be publicly available yet for 5.0.0")
            self.logger.info("   3. Download manually from NVIDIA Developer portal")
            self.logger.info("   4. Use the official Omniverse Launcher if available")
            return False
            
        download_path = download_dir / pack_info["filename"]
        
        # Check if already downloaded
        if download_path.exists() and not self.force_update:
            if file_size > 0:
                local_size = download_path.stat().st_size
                if local_size == file_size:
                    self.logger.info(f"⏭️ {pack_info['name']} already downloaded (size match)")
                    return True
                else:
                    self.logger.info(f"🔄 Size mismatch, re-downloading {pack_info['name']}")
            else:
                self.logger.info(f"⏭️ {pack_info['name']} already downloaded")
                return True
        
        try:
            self.logger.info(f"📥 Downloading {pack_info['name']}...")
            self.logger.info(f"    URL: {working_url}")
            if file_size > 0:
                self.logger.info(f"    Size: {file_size / (1024*1024):.1f} MB")
            
            # Download with progress
            def progress_hook(block_num, block_size, total_size):
                if total_size > 0:
                    downloaded = block_num * block_size
                    percent = min(100, (downloaded / total_size) * 100)
                    if block_num % 100 == 0:  # Update every 100 blocks
                        mb_downloaded = downloaded / (1024 * 1024)
                        mb_total = total_size / (1024 * 1024)
                        self.logger.info(f"    Progress: {percent:.1f}% ({mb_downloaded:.1f}/{mb_total:.1f} MB)")
            
            urllib.request.urlretrieve(working_url, download_path, progress_hook)
            
            # Verify download
            if download_path.exists():
                actual_size = download_path.stat().st_size
                self.logger.info(f"✅ Downloaded {pack_info['name']}: {actual_size / (1024*1024):.1f} MB")
                return True
            else:
                self.logger.error(f"❌ Download failed for {pack_info['name']}: File not created")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Failed to download {pack_info['name']}: {e}")
            # Clean up partial download
            if download_path.exists():
                download_path.unlink()
            return False
    
    def extract_asset_pack(self, pack_file, extract_to):
        """Extract an asset pack to the target directory"""
        try:
            self.logger.info(f"📦 Extracting {pack_file.name}...")
            
            with zipfile.ZipFile(pack_file, 'r') as zip_ref:
                # Get list of files
                file_list = zip_ref.namelist()
                self.logger.info(f"    Found {len(file_list)} files in archive")
                
                # Extract all files
                zip_ref.extractall(extract_to)
                
                # Log some sample extracted files
                sample_files = file_list[:5]
                for file in sample_files:
                    self.logger.info(f"    Extracted: {file}")
                if len(file_list) > 5:
                    self.logger.info(f"    ... and {len(file_list) - 5} more files")
            
            self.logger.info(f"✅ Successfully extracted {pack_file.name}")
            return True
            
        except zipfile.BadZipFile:
            self.logger.error(f"❌ Invalid zip file: {pack_file.name}")
            return False
        except Exception as e:
            self.logger.error(f"❌ Failed to extract {pack_file.name}: {e}")
            return False
    
    def verify_asset_structure(self):
        """Verify that the extracted assets have the correct structure"""
        expected_structure = {
            "Isaac/Robots": "Robot assets",
            "Isaac/Environments": "Environment assets", 
            "Isaac/Props": "Prop assets",
            "Isaac/Materials": "Material assets",
            "Isaac/Samples": "Sample assets",
            "Isaac/People": "People assets (optional)",
            "Isaac/Sensors": "Sensor assets (optional)",
            "Isaac/IsaacLab": "Isaac Lab assets (optional)"
        }
        
        self.logger.info("🔍 Verifying asset directory structure...")
        
        verified_folders = []
        missing_folders = []
        
        for folder, description in expected_structure.items():
            folder_path = self.asset_root_path / folder
            if folder_path.exists():
                file_count = len(list(folder_path.rglob("*.usd*")))
                self.logger.info(f"    ✅ {folder}: {file_count} USD files found")
                verified_folders.append(folder)
            else:
                if "optional" in description:
                    self.logger.info(f"    ⚪ {folder}: Not found (optional)")
                else:
                    self.logger.warning(f"    ❌ {folder}: Missing!")
                    missing_folders.append(folder)
        
        if missing_folders:
            self.logger.warning(f"⚠️ Missing {len(missing_folders)} required asset folders")
            return False
        else:
            self.logger.info(f"✅ Asset structure verification complete: {len(verified_folders)} folders verified")
            return True
    
    def generate_isaac_sim_config(self):
        """Generate Isaac Sim configuration for using local assets"""
        
        # Generate the kit file configuration
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
        
        # Save kit configuration
        config_file = self.local_assets_path / "isaac_sim_local_assets.kit"
        with open(config_file, 'w') as f:
            f.write(kit_config)
        
        # Generate launch script
        launch_script = f"""#!/bin/bash
# Isaac Sim 5.0 Launch Script with Local Assets
# Generated by Isaac Asset Pack Downloader

ISAAC_SIM_PATH="/home/kimate/isaacsim"
ASSETS_ROOT="{self.asset_root_path}"

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "❌ Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Please update ISAAC_SIM_PATH in this script"
    exit 1
fi

if [ ! -d "$ASSETS_ROOT" ]; then
    echo "❌ Local assets not found at $ASSETS_ROOT"
    echo "Please run the asset downloader first"
    exit 1
fi

echo "🚀 Launching Isaac Sim with local assets..."
echo "📁 Assets path: $ASSETS_ROOT"

cd "$ISAAC_SIM_PATH"
./isaac-sim.sh --/persistent/isaac/asset_root/default="$ASSETS_ROOT"
"""
        
        launch_file = self.local_assets_path / "launch_isaac_sim_local.sh"
        with open(launch_file, 'w') as f:
            f.write(launch_script)
        launch_file.chmod(0o755)  # Make executable
        
        # Generate Python configuration
        python_config = {
            "isaac_sim_version": self.isaac_version,
            "local_assets_root": str(self.asset_root_path),
            "kit_config_file": str(config_file),
            "launch_script": str(launch_file),
            "installation_date": datetime.now().isoformat(),
            "asset_folders": [
                str(self.asset_root_path / "Isaac/Robots"),
                str(self.asset_root_path / "Isaac/Environments"), 
                str(self.asset_root_path / "Isaac/Props"),
                str(self.asset_root_path / "Isaac/Materials"),
                str(self.asset_root_path / "Isaac/Samples")
            ]
        }
        
        config_json_file = self.local_assets_path / "isaac_local_assets_config.json"
        with open(config_json_file, 'w') as f:
            json.dump(python_config, f, indent=2)
        
        self.logger.info("⚙️ Generated Isaac Sim configuration files:")
        self.logger.info(f"    Kit config: {config_file}")
        self.logger.info(f"    Launch script: {launch_file}")
        self.logger.info(f"    Python config: {config_json_file}")
        
        return config_file, launch_file, config_json_file
    
    def save_installation_catalog(self, download_stats):
        """Save installation catalog with metadata"""
        
        catalog_data = {
            "isaac_sim_version": self.isaac_version,
            "installation_type": "official_asset_packs",
            "local_assets_path": str(self.local_assets_path),
            "asset_root_path": str(self.asset_root_path),
            "installation_date": datetime.now().isoformat(),
            "force_update_used": self.force_update,
            "asset_packs": self.asset_packs,
            "download_statistics": download_stats
        }
        
        with open(self.catalog_file, 'w') as f:
            json.dump(catalog_data, f, indent=2)
        
        self.logger.info(f"💾 Installation catalog saved: {self.catalog_file}")
        return self.catalog_file
    
    def run(self):
        """Run the complete asset pack download and installation process"""
        
        self.logger.info("🚀 Isaac Sim 5.0 Official Asset Pack Downloader")
        self.logger.info(f"📁 Local assets path: {self.local_assets_path}")
        self.logger.info(f"🎯 Asset root path: {self.asset_root_path}")
        self.logger.info(f"🔄 Force update mode: {self.force_update}")
        
        # Check existing installation
        is_complete, existing_folders = self.check_existing_installation()
        if is_complete and not self.force_update:
            self.logger.info("✅ Complete asset installation already exists!")
            self.logger.info("💡 Use --force-update to reinstall")
            
            # Still generate config files
            self.generate_isaac_sim_config()
            return True
        
        # Create temporary download directory
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            
            download_stats = {
                "packs_attempted": len(self.asset_packs),
                "packs_downloaded": 0,
                "packs_extracted": 0,
                "total_download_size": 0,
                "errors": []
            }
            
            # Download all asset packs
            self.logger.info(f"\n📦 Downloading {len(self.asset_packs)} asset packs...")
            downloaded_packs = []
            
            for i, pack_info in enumerate(self.asset_packs, 1):
                self.logger.info(f"\n[{i}/{len(self.asset_packs)}] Processing {pack_info['name']}...")
                
                if self.download_asset_pack(pack_info, temp_path):
                    pack_file = temp_path / pack_info["filename"]
                    if pack_file.exists():
                        download_stats["packs_downloaded"] += 1
                        download_stats["total_download_size"] += pack_file.stat().st_size
                        downloaded_packs.append(pack_file)
                    else:
                        error_msg = f"Downloaded pack file not found: {pack_info['filename']}"
                        download_stats["errors"].append(error_msg)
                        self.logger.error(f"❌ {error_msg}")
                else:
                    error_msg = f"Failed to download: {pack_info['name']}"
                    download_stats["errors"].append(error_msg)
                    self.logger.error(f"❌ {error_msg}")
            
            # Extract downloaded packs
            if downloaded_packs:
                self.logger.info(f"\n📦 Extracting {len(downloaded_packs)} downloaded asset packs...")
                
                for i, pack_file in enumerate(downloaded_packs, 1):
                    self.logger.info(f"\n[{i}/{len(downloaded_packs)}] Extracting {pack_file.name}...")
                    
                    if self.extract_asset_pack(pack_file, self.local_assets_path):
                        download_stats["packs_extracted"] += 1
                    else:
                        error_msg = f"Failed to extract: {pack_file.name}"
                        download_stats["errors"].append(error_msg)
                        self.logger.error(f"❌ {error_msg}")
            else:
                self.logger.error("❌ No asset packs were successfully downloaded!")
                return False
        
        # Verify installation
        self.logger.info("\n🔍 Verifying asset installation...")
        if self.verify_asset_structure():
            self.logger.info("✅ Asset structure verification passed!")
        else:
            self.logger.warning("⚠️ Asset structure verification had issues")
        
        # Generate configuration files
        self.logger.info("\n⚙️ Generating Isaac Sim configuration...")
        kit_config, launch_script, python_config = self.generate_isaac_sim_config()
        
        # Save installation catalog
        catalog_file = self.save_installation_catalog(download_stats)
        
        # Print final summary
        total_size_mb = download_stats["total_download_size"] / (1024 * 1024)
        
        self.logger.info("\n🎉 Isaac Sim Asset Pack Installation Complete!")
        self.logger.info("=" * 60)
        self.logger.info(f"📊 INSTALLATION SUMMARY:")
        self.logger.info(f"    Packs downloaded: {download_stats['packs_downloaded']}/{download_stats['packs_attempted']}")
        self.logger.info(f"    Packs extracted: {download_stats['packs_extracted']}/{download_stats['packs_downloaded']}")
        self.logger.info(f"    Total size: {total_size_mb:.1f} MB")
        self.logger.info(f"    Errors: {len(download_stats['errors'])}")
        
        if download_stats['errors']:
            self.logger.info(f"⚠️ ERRORS ENCOUNTERED:")
            for error in download_stats['errors']:
                self.logger.info(f"    - {error}")
        
        self.logger.info(f"\n📁 ASSET LOCATIONS:")
        self.logger.info(f"    Root path: {self.asset_root_path}")
        self.logger.info(f"    Kit config: {kit_config}")
        self.logger.info(f"    Launch script: {launch_script}")
        
        self.logger.info(f"\n🚀 NEXT STEPS:")
        self.logger.info(f"    1. To launch Isaac Sim with local assets:")
        self.logger.info(f"       {launch_script}")
        self.logger.info(f"    2. Or manually add to your Isaac Sim kit file:")
        self.logger.info(f"       persistent.isaac.asset_root.default = \"{self.asset_root_path}\"")
        self.logger.info(f"    3. Verify assets in Isaac Sim Asset Browser")
        
        return download_stats['packs_extracted'] > 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim 5.0 Official Asset Pack Downloader")
    parser.add_argument("--assets-path", default="/home/kimate/isaac_assets",
                       help="Local path to store assets (default: /home/kimate/isaac_assets)")
    parser.add_argument("--force-update", action="store_true",
                       help="Force re-download even if assets exist")
    parser.add_argument("--verbose", action="store_true",
                       help="Enable verbose logging")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    downloader = IsaacAssetPackDownloader(
        local_assets_path=args.assets_path,
        force_update=args.force_update
    )
    
    success = downloader.run()
    exit(0 if success else 1)
