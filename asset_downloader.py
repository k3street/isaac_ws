#!/usr/bin/env python3
"""
Isaac Sim 5.0 Enhanced Asset Downloader
Downloads and manages local Isaac Sim assets for scenario manager
Enhanced with automatic asset discovery, duplicate checking, update detection, and comprehensive asset management
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
from email.utils import parsedate_to_datetime
import xml.etree.ElementTree as ET
import re
import threading
import socket
import argparse

class IsaacAssetDownloader:
    def __init__(self, local_assets_path="/home/kimate/isaac_assets", force_update=False, discover_assets=True):
        self.local_assets_path = Path(local_assets_path)
        self.remote_base_url = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
        self.force_update = force_update
        self.discover_assets = discover_assets
        
        # Create local assets directory structure
        self.local_assets_path.mkdir(parents=True, exist_ok=True)
        
        # Set up logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Load existing asset catalog if available
        self.catalog_file = self.local_assets_path / "asset_catalog.json"
        self.existing_catalog = self.load_existing_catalog()
        
        # Asset discovery patterns and categories
        self.asset_discovery_paths = [
            "/Isaac/Environments/",
            "/Isaac/Robots/", 
            "/Isaac/Props/",
            "/Isaac/People/",
            "/Isaac/Vehicles/",
            "/Isaac/Materials/",
            "/Isaac/Samples/",
            "/Isaac/Animals/",
            "/Isaac/Furniture/",
            "/Isaac/Tools/"
        ]
        
        # Asset categorization patterns based on Isaac Sim 5.0 official documentation
        self.asset_categories = {
            "environments": {
                "paths": ["/Isaac/Environments/", "/Isaac/Samples/Environments/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["environment", "scene", "room", "warehouse", "office", "hospital", "factory", "grid", "track", "digital_twin"],
                "known_assets": ["simple_room", "warehouse", "hospital", "office", "jetracer_track_solid", "small_warehouse_digital_twin"]
            },
            "robots": {
                "paths": ["/Isaac/Robots/", "/Isaac/Samples/Robots/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["robot", "arm", "mobile", "manipulator", "carter", "franka", "ur10", "limo", "evobot", "jetbot", "create"],
                "manufacturers": ["AgilexRobotics", "NVIDIA", "Clearpath", "Fraunhofer", "IsaacSim", "Idealworks", "iRobot"],
                "known_robots": [
                    "limo", "carter_v1", "nova_carter", "dingo", "jackal", "evobot", 
                    "forklift", "jetbot", "iw_hub", "create_3", "leatherback"
                ],
                "variant_suffixes": ["_sensors", "_static", "_base", "_solid", "_v1", "_v2"]
            },
            "props": {
                "paths": ["/Isaac/Props/", "/Isaac/Furniture/", "/Isaac/Tools/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["prop", "block", "object", "furniture", "tool", "blocks"]
            },
            "people": {
                "paths": ["/Isaac/People/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["person", "human", "character"]
            },
            "vehicles": {
                "paths": ["/Isaac/Vehicles/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["vehicle", "car", "truck", "drone"]
            },
            "animals": {
                "paths": ["/Isaac/Animals/"],
                "extensions": [".usd", ".usda"],
                "keywords": ["animal", "creature"]
            },
            "materials": {
                "paths": ["/Isaac/Materials/"],
                "extensions": [".mdl", ".usd", ".usda"],
                "keywords": ["material", "texture", "surface"]
            }
        }
        
        # Fallback hardcoded assets for initial discovery
        self.fallback_assets = {
            "environments": {
                "simple_room": ["/Isaac/Environments/Simple_Room/simple_room.usd"],
                "warehouse": ["/Isaac/Environments/Simple_Warehouse/warehouse.usd"],
                "office": ["/Isaac/Environments/Office/office.usd"],
                "hospital": ["/Isaac/Environments/Hospital/hospital.usd"]
            },
            "robots": {
                "carter": ["/Isaac/Robots/Carter/carter_v1.usd"],
                "franka": ["/Isaac/Robots/Franka/franka.usd"],
                "ur10": ["/Isaac/Robots/UniversalRobots/ur10/ur10.usd"]
            },
            "props": {
                "basic_block": ["/Isaac/Props/Blocks/basic_block.usd"]
            }
        }
        
    def load_existing_catalog(self):
        """Load existing asset catalog to check for duplicates and updates"""
        if self.catalog_file.exists():
            try:
                with open(self.catalog_file, 'r') as f:
                    catalog = json.load(f)
                self.logger.info(f"📋 Loaded existing catalog with {len(catalog.get('assets', {}))} asset categories")
                return catalog
            except Exception as e:
                self.logger.warning(f"⚠️ Failed to load existing catalog: {e}")
        return None
    
    def discover_remote_assets(self, download_on_discovery=True):
        """Discover all available assets by exploring the remote directory structure
        
        Args:
            download_on_discovery (bool): If True, automatically download assets as they are discovered
        """
        self.logger.info("🔍 Starting comprehensive asset discovery...")
        if download_on_discovery:
            self.logger.info("📥 Auto-download enabled - will download assets as they are discovered")
        
        discovered_assets = {}
        download_stats = {"found": 0, "downloaded": 0, "skipped": 0, "failed": 0}
        
        for category, config in self.asset_categories.items():
            discovered_assets[category] = {}
            self.logger.info(f"🔍 Discovering {category}...")
            
            for base_path in config["paths"]:
                self.logger.info(f"   Exploring {base_path}...")
                assets_found = self.explore_directory_structure(base_path, config["extensions"])
                
                for asset_path in assets_found:
                    asset_name = self.extract_asset_name(asset_path)
                    if asset_name:
                        if asset_name not in discovered_assets[category]:
                            discovered_assets[category][asset_name] = []
                        discovered_assets[category][asset_name].append(asset_path)
                        download_stats["found"] += 1
                        self.logger.info(f"      ✅ Found {category}: {asset_name} -> {asset_path}")
                        
                        # Auto-download if enabled
                        if download_on_discovery:
                            download_result = self.download_single_asset(asset_path, category, asset_name)
                            if download_result == "downloaded":
                                download_stats["downloaded"] += 1
                            elif download_result == "skipped":
                                download_stats["skipped"] += 1
                            else:
                                download_stats["failed"] += 1
        
        total_assets = sum(len(cat) for cat in discovered_assets.values())
        self.logger.info(f"🎯 Asset discovery complete! Found {total_assets} unique assets")
        
        if download_on_discovery:
            self.logger.info(f"📊 DOWNLOAD STATS: Found: {download_stats['found']}, "
                           f"Downloaded: {download_stats['downloaded']}, "
                           f"Skipped: {download_stats['skipped']}, "
                           f"Failed: {download_stats['failed']}")
        
        return discovered_assets
    
    def get_context_appropriate_subdirs(self, base_path):
        """Get context-appropriate subdirectories based on the base path
        
        Args:
            base_path (str): Base path to get subdirectories for
            
        Returns:
            list: List of appropriate subdirectories for the given base path
        """
        # Check if this is a robot manufacturer path
        if "/Robots/" in base_path:
            # Extract manufacturer name if this is a specific manufacturer path
            path_parts = [p for p in base_path.split('/') if p]  # Remove empty parts
            
            # Find the manufacturer in the path
            manufacturer = None
            if "Robots" in path_parts:
                robots_index = path_parts.index("Robots")
                if robots_index + 1 < len(path_parts):
                    manufacturer = path_parts[robots_index + 1]
            
            # If we found a manufacturer, use manufacturer-specific subdirs
            if manufacturer:
                self.logger.debug(f"Detected manufacturer path: {manufacturer} in {base_path}")
                return self.get_manufacturer_specific_subdirs(manufacturer)
            else:
                # General robot subdirectories for /Isaac/Robots/ base path
                robot_subdirs = [
                    # Base directory
                    "",
                    
                    # Common robot model subdirectories
                    "/carter_v1", "/carter_v2", "/carter_v3",
                    "/panda", "/franka_panda",
                    "/ur10", "/ur5", "/ur3", "/ur16e", "/ur20",
                    "/kuka_kr10", "/kuka_iiwa", "/kuka_kr16",
                    "/abb_irb120", "/abb_irb1600", "/abb_yumi",
                    "/fanuc_lr_mate", "/fanuc_m20ia",
                    "/kinova_gen3", "/kinova_jaco",
                    "/anymal_b", "/anymal_c", "/anymal_d",
                    "/a1", "/go1", "/b1", "/spot",
                    "/nova_carter", "/jetbot", "/turtlebot3",
                    "/fetch", "/pr2", "/hsrb",
                    
                    # Version directories
                    "/v1", "/v2", "/v3", "/latest",
                    "/gen1", "/gen2", "/gen3",
                    
                    # Configuration directories
                    "/default", "/standard", "/mobile", "/fixed",
                    "/warehouse", "/manipulation", "/navigation"
                ]
                self.logger.debug(f"Using general robot subdirs for {base_path}")
                return robot_subdirs
            
        elif "/Environments/" in base_path:
            # For environment paths, focus on environment-specific subdirectories
            env_subdirs = [
                # Base directory
                "",
                
                # Room types
                "/Simple_Room", "/Simple_Warehouse", "/Office", "/Hospital", "/Factory",
                "/Grid_Room", "/GridRoom", "/Curved_Gridroom", "/Full_Warehouse",
                "/Apartment", "/Classroom", "/Kitchen", "/Bathroom", "/Garage",
                "/Laboratory", "/Studio", "/Conference", "/Lobby",
                
                # Outdoor environments
                "/Outdoor", "/Street", "/Park", "/Construction", "/Parking",
                
                # Industrial environments
                "/Manufacturing", "/Assembly", "/Quality_Control", "/Storage",
                
                # Versions and variants
                "/v1", "/v2", "/default", "/basic", "/detailed"
            ]
            self.logger.debug(f"Using environment-specific subdirs for {base_path}")
            return env_subdirs
            
        elif "/Props/" in base_path:
            # For props, focus on prop categories
            prop_subdirs = [
                # Base directory
                "",
                
                # YCB objects (limited selection)
                "/YCB/003_cracker_box", "/YCB/004_sugar_box", "/YCB/005_tomato_soup_can",
                "/YCB/006_mustard_bottle", "/YCB/035_power_drill", "/YCB/036_wood_block",
                
                # Furniture categories
                "/Furniture/Tables", "/Furniture/Chairs", "/Furniture/Desks",
                "/Furniture/Shelves", "/Furniture/Cabinets",
                
                # Tools and equipment
                "/Tools", "/Electronics", "/Containers", "/Shapes",
                "/Appliances", "/Kitchen", "/Sports", "/Toys"
            ]
            self.logger.debug(f"Using prop-specific subdirs for {base_path}")
            return prop_subdirs
            
        else:
            # For unknown/general paths, use a more conservative approach
            general_subdirs = [
                # Base directory
                "",
                
                # Common versioning
                "/v1", "/v2", "/v3", "/latest", "/default",
                
                # Common subdirectory patterns found in Isaac Sim
                "/Models", "/Meshes", "/Materials", "/Textures",
                "/Animations", "/Configurations"
            ]
            self.logger.debug(f"Using general subdirs for {base_path}")
            return general_subdirs
    
    def get_manufacturer_specific_subdirs(self, manufacturer):
        """Get manufacturer-specific subdirectories for robot exploration
        
        Args:
            manufacturer (str): Robot manufacturer name
            
        Returns:
            list: List of subdirectories specific to this manufacturer
        """
        manufacturer_subdirs = {
            "AgilexRobotics": [
                "",
                "/Limo",
                "/Limo/limo_base",
                "/Limo/limo_sensors"
            ],
            "NVIDIA": [
                "",
                "/Carter", "/Carter/carter_v1", "/Carter/carter_v2", "/Carter/nova_carter",
                "/Jetbot", "/Jetbot/jetbot_base", "/Jetbot/jetbot_detailed",
                "/Leatherback", "/Leatherback/leatherback_base"
            ],
            "Clearpath": [
                "",
                "/Dingo", "/Dingo/dingo_base",
                "/Jackal", "/Jackal/jackal_base",
                "/Ridgeback", "/Ridgeback/ridgeback_base"
            ],
            "Fraunhofer": [
                "",
                "/EvoBot", "/EvoBot/evobot_base"
            ],
            "IsaacSim": [
                "",
                "/forklift", "/forklift_a", "/forklift_b",
                "/warehouse_robot", "/industrial_robot"
            ],
            "Idealworks": [
                "",
                "/iwhub", "/iwhub/iw_hub_base", "/iwhub/iw_hub_sensors", "/iwhub/iw_hub_static"
            ],
            "iRobot": [
                "",
                "/Create3", "/Create3/create_3_base"
            ],
            # Add common standalone robot manufacturers that might be under general paths
            "UniversalRobots": [
                "",
                "/ur10", "/ur5", "/ur3", "/ur16e", "/ur20"
            ],
            "Franka": [
                "",
                "/panda", "/franka_panda", "/emika_panda"
            ]
        }
        
        subdirs = manufacturer_subdirs.get(manufacturer, [
            # Default fallback for unknown manufacturers
            "",
            "/base", "/sensors", "/static", "/mobile",
            "/v1", "/v2", "/default"
        ])
        
        self.logger.debug(f"Using {len(subdirs)} manufacturer-specific subdirs for {manufacturer}")
        return subdirs
    
    def explore_directory_structure(self, base_path, valid_extensions, timeout_per_path=30):
        """Explore a directory structure to find assets with valid extensions
        
        Args:
            base_path (str): Base path to explore
            valid_extensions (list): List of valid file extensions
            timeout_per_path (int): Maximum time in seconds to spend exploring each path
        """
        import time
        from concurrent.futures import ThreadPoolExecutor, TimeoutError, as_completed
        
        found_assets = []
        start_time = time.time()
        
        self.logger.info(f"      🔍 Exploring {base_path} with {timeout_per_path}s timeout per path...")
        
        # Determine appropriate subdirectories based on the base path
        common_subdirs = self.get_context_appropriate_subdirs(base_path)
        
        # Use ThreadPoolExecutor for concurrent probing with timeout
        with ThreadPoolExecutor(max_workers=3) as executor:
            # Submit tasks for exploration
            future_to_path = {}
            for subdir in common_subdirs:
                full_path = base_path.rstrip('/') + subdir
                future = executor.submit(self.probe_directory_for_assets_with_timeout, full_path, valid_extensions, 3)  # 3s per probe (reduced)
                future_to_path[future] = full_path
            
            # Collect results with overall timeout protection
            completed_paths = 0
            timeout_paths = 0
            error_paths = 0
            
            try:
                for future in as_completed(future_to_path, timeout=timeout_per_path):
                    try:
                        assets = future.result(timeout=1)  # Quick result collection
                        found_assets.extend(assets)
                        completed_paths += 1
                        
                        if assets:
                            path = future_to_path[future]
                            self.logger.info(f"        ✅ Found {len(assets)} assets in {path}")
                            
                    except TimeoutError:
                        path = future_to_path[future]
                        self.logger.warning(f"        ⏰ Timeout exploring {path} - cataloging as undownloaded and moving on")
                        timeout_paths += 1
                    except Exception as e:
                        path = future_to_path[future]
                        self.logger.warning(f"        ❌ Error exploring {path}: {e}")
                        error_paths += 1
                    
                    # Check overall timeout
                    if time.time() - start_time > timeout_per_path:
                        self.logger.warning(f"        ⏰ Overall timeout reached for {base_path} after {timeout_per_path}s")
                        break
                        
            except TimeoutError:
                self.logger.warning(f"        ⏰ Overall exploration timeout for {base_path} - moving on")
        
        elapsed = time.time() - start_time
        total_paths = len(common_subdirs)
        
        self.logger.info(f"      📊 Explored {completed_paths}/{total_paths} paths in {elapsed:.1f}s")
        self.logger.info(f"      📊 Results: {len(found_assets)} assets found, {timeout_paths} timeouts, {error_paths} errors")
        
        return found_assets
    
    def probe_directory_for_assets_with_timeout(self, dir_path, valid_extensions, timeout=5):
        """Probe a specific directory for asset files with timeout protection
        
        Args:
            dir_path (str): Directory path to probe
            valid_extensions (list): List of valid file extensions
            timeout (int): Timeout in seconds for this probe
        """
        import threading
        import time
        
        # Use threading-based timeout that works across all threads
        result = []
        exception_holder = []
        
        def target():
            try:
                result.extend(self.probe_directory_for_assets(dir_path, valid_extensions))
            except Exception as e:
                exception_holder.append(e)
        
        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()
        thread.join(timeout)
        
        if thread.is_alive():
            # Thread is still running, it timed out
            self.logger.warning(f"Probe timed out for {dir_path} after {timeout}s - cataloging as undownloaded and moving on")
            return []
        
        if exception_holder:
            self.logger.warning(f"Probe error for {dir_path}: {exception_holder[0]}")
            return []
            
        return result
    
    def probe_directory_for_assets(self, dir_path, valid_extensions):
        """Probe a specific directory for asset files"""
        found_assets = []
        
        # Extract directory name for generating asset names
        dir_name = dir_path.split('/')[-1].lower() if dir_path.split('/')[-1] else dir_path.split('/')[-2].lower()
        
        # Expanded asset naming patterns based on Isaac Sim structure
        test_files = [
            # Standard naming conventions
            f"{dir_path}/{dir_name}.usd",
            f"{dir_path}/{dir_name}.usda",
            f"{dir_path}/main.usd",
            f"{dir_path}/scene.usd",
            f"{dir_path}/robot.usd",
            f"{dir_path}/prop.usd",
            f"{dir_path}/{dir_name}_main.usd",
            f"{dir_path}/{dir_name}_scene.usd",
            
            # Isaac Sim specific patterns
            f"{dir_path}/instanceable.usd",
            f"{dir_path}/standalone.usd",
            f"{dir_path}/assembled.usd",
            f"{dir_path}/final.usd",
            f"{dir_path}/complete.usd",
            
            # Robot specific patterns
            f"{dir_path}/carter.usd",
            f"{dir_path}/carter_v1.usd",
            f"{dir_path}/carter_v2.usd",
            f"{dir_path}/franka.usd",
            f"{dir_path}/panda.usd",
            f"{dir_path}/ur10.usd",
            f"{dir_path}/jetbot.usd",
            f"{dir_path}/quadruped.usd",
            f"{dir_path}/a1.usd",
            f"{dir_path}/anymal.usd",
            f"{dir_path}/go1.usd",
            
            # Environment specific patterns  
            f"{dir_path}/warehouse.usd",
            f"{dir_path}/simple_warehouse.usd",
            f"{dir_path}/simple_room.usd",
            f"{dir_path}/office.usd",
            f"{dir_path}/hospital.usd",
            f"{dir_path}/factory.usd",
            f"{dir_path}/gridroom.usd",
            f"{dir_path}/gridroom_curved.usd",
            
            # Prop specific patterns
            f"{dir_path}/basic_block.usd",
            f"{dir_path}/cube.usd",
            f"{dir_path}/sphere.usd",
            f"{dir_path}/cylinder.usd",
            f"{dir_path}/table.usd",
            f"{dir_path}/chair.usd",
            f"{dir_path}/desk.usd",
            
            # YCB object patterns
            f"{dir_path}/003_cracker_box.usd",
            f"{dir_path}/004_sugar_box.usd",
            f"{dir_path}/005_tomato_soup_can.usd",
            f"{dir_path}/006_mustard_bottle.usd",
            f"{dir_path}/007_tuna_fish_can.usd",
            f"{dir_path}/008_pudding_box.usd",
            f"{dir_path}/009_gelatin_box.usd",
            f"{dir_path}/010_potted_meat_can.usd",
            
            # Material patterns
            f"{dir_path}/material.mdl",
            f"{dir_path}/surface.mdl",
            f"{dir_path}/texture.mdl"
        ]
        
        for test_file in test_files:
            if self.check_remote_asset_exists(test_file):
                if any(test_file.endswith(ext) for ext in valid_extensions):
                    found_assets.append(test_file)
                    self.logger.debug(f"      Found asset: {test_file}")
        
        return found_assets
    
    def check_remote_asset_exists(self, asset_path):
        """Quick check if a remote asset exists"""
        try:
            full_url = self.remote_base_url + asset_path
            request = urllib.request.Request(full_url, method='HEAD')
            response = urllib.request.urlopen(request, timeout=5)
            return response.status == 200
        except:
            return False
    
    def extract_asset_name(self, asset_path):
        """Extract a meaningful asset name from the asset path"""
        # Remove file extension and path components
        name = asset_path.split('/')[-2] if asset_path.split('/')[-1] in ['main.usd', 'scene.usd', 'robot.usd'] else asset_path.split('/')[-1]
        name = name.replace('.usd', '').replace('.usda', '').replace('.mdl', '')
        
        # Convert to lowercase and replace special characters
        name = re.sub(r'[^a-zA-Z0-9_]', '_', name.lower())
        name = re.sub(r'_+', '_', name).strip('_')
        
        return name if name else None
        
    def check_remote_asset(self, asset_path):
        """Check if a remote asset exists and get its info including size and modification time"""
        full_url = self.remote_base_url + asset_path
        try:
            request = urllib.request.Request(full_url, method='HEAD')
            with urllib.request.urlopen(request) as response:
                size = int(response.headers.get('Content-Length', 0))
                last_modified = response.headers.get('Last-Modified')
                modified_time = None
                if last_modified:
                    try:
                        modified_time = parsedate_to_datetime(last_modified).timestamp()
                    except:
                        modified_time = None
                return True, size, modified_time
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return False, 0, None
            raise
        except Exception as e:
            self.logger.warning(f"Error checking {full_url}: {e}")
            return False, 0, None
            
    def needs_download(self, asset_path, local_path, remote_size, remote_modified_time):
        """Check if asset needs to be downloaded (doesn't exist, size mismatch, or forced update)"""
        if not local_path.exists():
            return True, "File doesn't exist locally"
            
        if self.force_update:
            return True, "Force update requested"
            
        # Check file size
        local_size = local_path.stat().st_size
        if local_size != remote_size and remote_size > 0:
            return True, f"Size mismatch: local={local_size}, remote={remote_size}"
            
        # Check modification time if available
        if remote_modified_time:
            local_modified_time = local_path.stat().st_mtime
            if remote_modified_time > local_modified_time:
                return True, f"Remote file is newer: {datetime.fromtimestamp(remote_modified_time)} > {datetime.fromtimestamp(local_modified_time)}"
                
        return False, "File is up to date"
        
    def calculate_file_hash(self, file_path):
        """Calculate SHA256 hash of a file for integrity checking"""
        hash_sha256 = hashlib.sha256()
        try:
            with open(file_path, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_sha256.update(chunk)
            return hash_sha256.hexdigest()
        except Exception as e:
            self.logger.warning(f"Failed to calculate hash for {file_path}: {e}")
            return None
    
    def download_asset(self, asset_path, local_path, expected_size=None):
        """Download an asset from remote to local path with integrity checking"""
        full_url = self.remote_base_url + asset_path
        
        try:
            self.logger.info(f"⬇️ Downloading {asset_path}...")
            local_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Download with progress indication for large files
            def progress_hook(block_num, block_size, total_size):
                if total_size > 1024 * 1024:  # Show progress for files > 1MB
                    downloaded = block_num * block_size
                    percent = min(100, (downloaded / total_size) * 100)
                    if block_num % 50 == 0:  # Log every 50 blocks
                        self.logger.info(f"    Progress: {percent:.1f}% ({downloaded}/{total_size} bytes)")
            
            urllib.request.urlretrieve(full_url, local_path, progress_hook)
            
            # Verify file size if expected
            if expected_size and expected_size > 0:
                actual_size = local_path.stat().st_size
                if actual_size != expected_size:
                    self.logger.warning(f"⚠️ Size mismatch for {asset_path}: expected {expected_size}, got {actual_size}")
                    
            self.logger.info(f"✅ Downloaded: {asset_path} -> {local_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to download {asset_path}: {e}")
            # Clean up partial download
            if local_path.exists():
                local_path.unlink()
            return False
    
    def download_single_asset(self, asset_path, category, asset_name):
        """Download a single asset immediately when discovered
        
        Returns:
            str: 'downloaded', 'skipped', or 'failed'
        """
        try:
            remote_url = f"{self.remote_base_url}{asset_path}"
            local_path = self.local_assets_path / asset_path.lstrip('/')
            
            # Check if already exists and up to date
            if local_path.exists() and not self.force_update:
                if self.is_asset_up_to_date(local_path, remote_url):
                    self.logger.info(f"      ⏭️ Skipping {asset_name} (up to date)")
                    return "skipped"
            
            # Create directory structure
            local_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Download the asset
            self.logger.info(f"      📥 Downloading {asset_name}...")
            response = urllib.request.urlopen(remote_url, timeout=30)
            
            # Get file size for progress tracking
            content_length = response.headers.get('Content-Length')
            file_size = 0
            if content_length:
                file_size = int(content_length)
                self.logger.info(f"         Size: {file_size / (1024*1024):.1f} MB")
            
            # Download with progress indication
            with open(local_path, 'wb') as f:
                downloaded = 0
                chunk_size = 8192
                while True:
                    chunk = response.read(chunk_size)
                    if not chunk:
                        break
                    f.write(chunk)
                    downloaded += len(chunk)
                    
                    # Show progress for large files
                    if file_size > 1024*1024:  # > 1MB
                        progress = (downloaded / file_size) * 100
                        if downloaded % (chunk_size * 100) == 0:  # Update every ~800KB
                            self.logger.info(f"         Progress: {progress:.1f}%")
            
            self.logger.info(f"      ✅ Downloaded {asset_name} to {local_path}")
            
            # Update catalog
            self.update_asset_catalog(asset_path, category, local_path)
            
            return "downloaded"
            
        except Exception as e:
            self.logger.warning(f"      ❌ Failed to download {asset_name}: {e}")
            return "failed"
    
    def update_asset_catalog(self, asset_path, category, local_path):
        """Update the local asset catalog with new asset information"""
        try:
            # Load existing catalog
            catalog = self.existing_catalog or {"assets": {}, "metadata": {}}
            
            if "assets" not in catalog:
                catalog["assets"] = {}
            if "metadata" not in catalog:
                catalog["metadata"] = {}
            
            # Add/update asset entry
            if category not in catalog["assets"]:
                catalog["assets"][category] = {}
            
            asset_name = self.extract_asset_name(asset_path)
            if asset_name:
                # Calculate file hash
                file_hash = self.calculate_file_hash(local_path)
                
                catalog["assets"][category][asset_name] = {
                    "path": asset_path,
                    "local_path": str(local_path),
                    "download_date": datetime.now().isoformat(),
                    "file_size": local_path.stat().st_size,
                    "sha256": file_hash
                }
                
                # Update metadata
                catalog["metadata"].update({
                    "last_updated": datetime.now().isoformat(),
                    "total_assets": sum(len(cat) for cat in catalog["assets"].values())
                })
                
                # Save catalog
                with open(self.catalog_file, 'w') as f:
                    json.dump(catalog, f, indent=2)
                
                # Update our internal catalog reference
                self.existing_catalog = catalog
                
        except Exception as e:
            self.logger.warning(f"Failed to update catalog for {asset_name}: {e}")
    
    def is_asset_up_to_date(self, local_path, remote_url):
        """Check if local asset is up to date compared to remote"""
        try:
            # Get remote file info
            request = urllib.request.Request(remote_url, method='HEAD')
            response = urllib.request.urlopen(request, timeout=10)
            
            remote_size = response.headers.get('Content-Length')
            local_size = local_path.stat().st_size
            
            if remote_size and int(remote_size) != local_size:
                return False
                
            # If sizes match, consider it up to date
            return True
            
        except Exception:
            # If we can't check, assume it's up to date to avoid unnecessary downloads
            return True
    
    def discover_and_download_assets(self):
        """Discover available assets and download them locally with comprehensive discovery"""
        # Start with discovered assets if discovery is enabled
        if self.discover_assets:
            self.logger.info("🚀 Starting comprehensive asset discovery...")
            self.asset_catalog = self.discover_remote_assets()
        else:
            self.logger.info("📋 Using predefined asset catalog...")
            self.asset_catalog = self.fallback_assets
        
        # Merge with fallback assets to ensure we have basic coverage
        for category, assets in self.fallback_assets.items():
            if category not in self.asset_catalog:
                self.asset_catalog[category] = {}
            for asset_name, paths in assets.items():
                if asset_name not in self.asset_catalog[category]:
                    self.asset_catalog[category][asset_name] = paths
        
        downloaded_assets = {category: {} for category in self.asset_catalog.keys()}
        
        download_stats = {
            "total_checked": 0,
            "found": 0,
            "downloaded": 0,
            "skipped_up_to_date": 0,
            "skipped_duplicate": 0,
            "failed": 0,
            "discovered_new": len([asset for category in self.asset_catalog.values() for asset in category.keys()])
        }
        
        for category, assets in self.asset_catalog.items():
            self.logger.info(f"\n🔍 Processing {category} ({len(assets)} assets discovered)...")
            
            for asset_name, possible_paths in assets.items():
                self.logger.info(f"  Checking {asset_name}...")
                download_stats["total_checked"] += 1
                
                for asset_path in possible_paths:
                    exists, size, modified_time = self.check_remote_asset(asset_path)
                    
                    if exists:
                        self.logger.info(f"    ✅ Found: {asset_path} ({size} bytes)")
                        download_stats["found"] += 1
                        
                        # Create local path
                        local_path = self.local_assets_path / asset_path.lstrip('/')
                        
                        # Check if download is needed
                        needs_dl, reason = self.needs_download(asset_path, local_path, size, modified_time)
                        
                        if needs_dl:
                            self.logger.info(f"    📥 Downloading: {reason}")
                            if self.download_asset(asset_path, local_path, size):
                                file_hash = self.calculate_file_hash(local_path)
                                downloaded_assets[category][asset_name] = {
                                    "path": asset_path,
                                    "local_path": str(local_path),
                                    "size": size,
                                    "modified_time": modified_time,
                                    "hash": file_hash,
                                    "downloaded": True,
                                    "category": category
                                }
                                download_stats["downloaded"] += 1
                            else:
                                download_stats["failed"] += 1
                        else:
                            self.logger.info(f"    ⏭️ Skipping: {reason}")
                            file_hash = self.calculate_file_hash(local_path)
                            downloaded_assets[category][asset_name] = {
                                "path": asset_path,
                                "local_path": str(local_path),
                                "size": local_path.stat().st_size,
                                "modified_time": local_path.stat().st_mtime,
                                "hash": file_hash,
                                "downloaded": False,
                                "category": category
                            }
                            download_stats["skipped_up_to_date"] += 1
                        break
                    else:
                        self.logger.debug(f"    ❌ Not found: {asset_path}")
                
                if asset_name not in downloaded_assets[category]:
                    self.logger.warning(f"    ⚠️  {asset_name} not found in any location")
        
        # Log comprehensive download statistics
        self.logger.info(f"\n📊 COMPREHENSIVE DOWNLOAD STATISTICS:")
        self.logger.info(f"  Assets discovered: {download_stats['discovered_new']}")
        self.logger.info(f"  Total assets checked: {download_stats['total_checked']}")
        self.logger.info(f"  Assets found: {download_stats['found']}")
        self.logger.info(f"  New downloads: {download_stats['downloaded']}")
        self.logger.info(f"  Skipped (up-to-date): {download_stats['skipped_up_to_date']}")
        self.logger.info(f"  Failed downloads: {download_stats['failed']}")
        
        return downloaded_assets, download_stats
    
    def save_asset_catalog(self, downloaded_assets, download_stats):
        """Save the discovered asset catalog to a JSON file with enhanced metadata"""
        
        catalog_data = {
            "isaac_sim_version": "5.0",
            "local_assets_path": str(self.local_assets_path),
            "remote_base_url": self.remote_base_url,
            "download_timestamp": time.time(),
            "download_date": datetime.now().isoformat(),
            "force_update_used": self.force_update,
            "download_statistics": download_stats,
            "assets": downloaded_assets
        }
        
        with open(self.catalog_file, 'w') as f:
            json.dump(catalog_data, f, indent=2)
        
        self.logger.info(f"💾 Enhanced asset catalog saved to: {self.catalog_file}")
        return self.catalog_file
    
    def generate_local_scenario_manager_config(self, downloaded_assets):
        """Generate an updated scenario manager configuration using local paths"""
        
        local_scenes = {}
        local_robots = {}
        local_props = {}
        
        # Convert to scenario manager format
        for env_name, env_info in downloaded_assets["environments"].items():
            if env_info:  # Only include if successfully downloaded
                local_scenes[env_name] = {
                    "path": f"file://{env_info['local_path']}",
                    "description": f"Local {env_name} environment"
                }
        
        for robot_name, robot_info in downloaded_assets["robots"].items():
            if robot_info:
                local_robots[robot_name] = {
                    "path": f"file://{robot_info['local_path']}", 
                    "description": f"Local {robot_name} robot",
                    "ros_enabled": True,
                    "default_position": [0, 0, 0]
                }
        
        for prop_name, prop_info in downloaded_assets["props"].items():
            if prop_info:
                local_props[prop_name] = {
                    "path": f"file://{prop_info['local_path']}",
                    "description": f"Local {prop_name} prop"
                }
        
        config = {
            "available_scenes": local_scenes,
            "available_robots": local_robots, 
            "available_props": local_props
        }
        
        # Save configuration
        config_file = self.local_assets_path / "local_scenario_config.json"
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        
        self.logger.info(f"🎬 Local scenario manager config saved to: {config_file}")
        return config_file
    
    def run(self, auto_download=True):
        """Run the complete asset discovery and download process
        
        Args:
            auto_download (bool): If True, download assets as they are discovered
        """
        self.logger.info("🚀 Starting Isaac Sim 5.0 Enhanced Asset Discovery and Download")
        self.logger.info(f"📁 Local assets path: {self.local_assets_path}")
        self.logger.info(f"🌐 Remote base URL: {self.remote_base_url}")
        self.logger.info(f"🔄 Force update mode: {self.force_update}")
        self.logger.info(f"🔍 Asset discovery mode: {'Enabled' if self.discover_assets else 'Disabled (using predefined catalog)'}")
        self.logger.info(f"📥 Auto-download mode: {'Enabled' if auto_download else 'Discovery only'}")
        
        if self.existing_catalog:
            last_download = self.existing_catalog.get('download_date', 'Unknown')
            self.logger.info(f"📋 Previous catalog found from: {last_download}")
        
        if auto_download:
            # Use the original download method
            downloaded_assets, download_stats = self.discover_and_download_assets()
        else:
            # Use discovery-only mode
            self.logger.info("🔍 Running asset discovery without downloading...")
            discovered_assets = self.discover_remote_assets(download_on_discovery=False)
            
            # Convert to the expected format
            downloaded_assets = {}
            download_stats = {"discovered_new": 0, "total_checked": 0, "found": 0, "downloaded": 0, "skipped_up_to_date": 0, "failed": 0}
            
            for category, assets in discovered_assets.items():
                downloaded_assets[category] = {}
                for asset_name, paths in assets.items():
                    downloaded_assets[category][asset_name] = {
                        "path": paths[0] if paths else "",
                        "local_path": "",
                        "size": 0,
                        "modified_time": 0,
                        "hash": "",
                        "downloaded": False,
                        "category": category
                    }
                    download_stats["found"] += 1
        
        # Save enhanced asset catalog
        catalog_file = self.save_asset_catalog(downloaded_assets, download_stats)
        
        # Generate scenario manager config (only if we have downloads)
        if auto_download:
            config_file = self.generate_local_scenario_manager_config(downloaded_assets)
        
        # Print comprehensive summary
        self.logger.info("\n📊 FINAL COMPREHENSIVE SUMMARY:")
        if self.discover_assets:
            self.logger.info(f"🔍 Discovery mode: Found {download_stats.get('discovered_new', download_stats.get('found', 0))} unique assets")
        
        total_local_assets = 0
        for category, assets in downloaded_assets.items():
            found_count = len([a for a in assets.values() if a])
            total_local_assets += found_count
            self.logger.info(f"  {category}: {found_count} assets available locally")
        
        self.logger.info(f"\n✅ Enhanced asset download complete!")
        self.logger.info(f"📁 Assets location: {self.local_assets_path}")
        self.logger.info(f"📋 Asset catalog: {catalog_file}")
        self.logger.info(f"🎬 Scenario config: {config_file}")
        self.logger.info(f"📊 Statistics: {download_stats['downloaded']} new, {download_stats['skipped_up_to_date']} up-to-date, {download_stats['failed']} failed")
        self.logger.info(f"🎯 Total local assets: {total_local_assets}")
        
        return downloaded_assets, catalog_file, config_file

    def discover_robot_assets_enhanced(self, download_on_discovery=True):
        """Enhanced robot discovery using official Isaac Sim 5.0 manufacturer structure
        
        Args:
            download_on_discovery (bool): If True, automatically download robots as they are discovered
        """
        self.logger.info("🤖 Starting enhanced robot asset discovery using official structure...")
        if download_on_discovery:
            self.logger.info("📥 Auto-download enabled for robots")
        else:
            self.logger.info("🔍 Discovery-only mode - no automatic downloads")
            
        discovered_robots = {}
        download_stats = {"found": 0, "downloaded": 0, "skipped": 0, "failed": 0}
        
        # Robot manufacturers from official documentation
        robot_manufacturers = [
            "AgilexRobotics", "NVIDIA", "Clearpath", "Fraunhofer", 
            "IsaacSim", "Idealworks", "iRobot"
        ]
        
        # Known robot models from documentation
        known_robot_paths = [
            # AgileX Limo
            "/Isaac/Robots/AgilexRobotics/Limo/limo.usd",
            
            # NVIDIA robots
            "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd",
            "/Isaac/Robots/NVIDIA/Carter/nova_carter.usd",
            "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
            "/Isaac/Robots/NVIDIA/Jetbot/jetbot_detailed.usd",
            "/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd",
            
            # Clearpath robots
            "/Isaac/Robots/Clearpath/Dingo/dingo.usd",
            "/Isaac/Robots/Clearpath/Jackal/jackal.usd",
            
            # Fraunhofer
            "/Isaac/Robots/Fraunhofer/EvoBot/evobot.usd",
            
            # Isaac Sim robots (forklifts, etc.)
            "/Isaac/Robots/IsaacSim/forklift.usd",
            "/Isaac/Robots/IsaacSim/forklift_a.usd",
            "/Isaac/Robots/IsaacSim/forklift_b.usd",
            
            # Idealworks
            "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
            "/Isaac/Robots/Idealworks/iwhub/iw_hub_sensors.usd",
            "/Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd",
            
            # iRobot
            "/Isaac/Robots/iRobot/Create3/create_3.usd",
            
            # Universal Robots (commonly found)
            "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            "/Isaac/Robots/UniversalRobots/ur5/ur5.usd",
            "/Isaac/Robots/UniversalRobots/ur3/ur3.usd",
            
            # Franka (commonly found)
            "/Isaac/Robots/Franka/franka.usd",
            "/Isaac/Robots/Franka/panda.usd"
        ]
        
        self.logger.info(f"📋 Testing {len(known_robot_paths)} known robot paths from official documentation...")
        
        # Test each known robot path
        total_known_paths = len(known_robot_paths)
        for i, robot_path in enumerate(known_robot_paths, 1):
            robot_name = self.extract_robot_name_from_path(robot_path)
            if robot_name:
                self.logger.info(f"   [{i}/{total_known_paths}] Testing known robot: {robot_name} -> {robot_path}")
                
                start_time = time.time()
                if self.test_asset_availability(robot_path):
                    elapsed = time.time() - start_time
                    discovered_robots[robot_name] = [robot_path]
                    download_stats["found"] += 1
                    self.logger.info(f"      ✅ Found robot: {robot_name} (tested in {elapsed:.1f}s)")
                    
                    # Auto-download if enabled
                    if download_on_discovery:
                        download_result = self.download_single_asset(robot_path, "robots", robot_name)
                        if download_result == "downloaded":
                            download_stats["downloaded"] += 1
                        elif download_result == "skipped":
                            download_stats["skipped"] += 1
                        else:
                            download_stats["failed"] += 1
                else:
                    elapsed = time.time() - start_time
                    self.logger.info(f"      ❌ Robot not available: {robot_name} (tested in {elapsed:.1f}s) - asset doesn't exist on remote server")
            else:
                self.logger.warning(f"   [{i}/{total_known_paths}] ⚠️ Could not extract robot name from path: {robot_path}")
        
        known_robots_found = len([r for r in discovered_robots if not r.endswith('_unprobed')])
        self.logger.info(f"📊 Known robot paths testing complete: {known_robots_found} robots found from {len(known_robot_paths)} paths")
        
        # Detailed breakdown of found vs not found robots
        if self.logger.isEnabledFor(logging.INFO):
            self.logger.info(f"📋 DETAILED ROBOT DISCOVERY BREAKDOWN:")
            found_robots = []
            missing_robots = []
            
            for robot_path in known_robot_paths:
                robot_name = self.extract_robot_name_from_path(robot_path)
                if robot_name and robot_name in discovered_robots:
                    found_robots.append(f"{robot_name} ({robot_path})")
                else:
                    missing_robots.append(f"{robot_name or 'unnamed'} ({robot_path})")
            
            self.logger.info(f"  ✅ FOUND ROBOTS ({len(found_robots)}):")
            for robot in found_robots:
                self.logger.info(f"    - {robot}")
                
            self.logger.info(f"  ❌ NOT AVAILABLE ROBOTS ({len(missing_robots)}):")
            for robot in missing_robots:
                self.logger.info(f"    - {robot} (not on remote server or network issue)")
        
        # Explore manufacturer directories for additional robots with timeout protection
        self.logger.info(f"🏭 Starting manufacturer directory exploration for additional robots...")
        manufacturer_count = len(robot_manufacturers)
        for i, manufacturer in enumerate(robot_manufacturers, 1):
            manufacturer_path = f"/Isaac/Robots/{manufacturer}/"
            self.logger.info(f"   🔍 [{i}/{manufacturer_count}] Exploring manufacturer: {manufacturer} (max 20s)")
            
            start_time = time.time()
            try:
                # Use existing directory exploration method with reduced timeout
                found_assets = self.explore_directory_structure(manufacturer_path, [".usd", ".usda"], timeout_per_path=20)  # 20s timeout per manufacturer
                
                new_robots_found = 0
                for asset_path in found_assets:
                    robot_name = self.extract_robot_name_from_path(asset_path)
                    if robot_name and robot_name not in discovered_robots:
                        discovered_robots[robot_name] = [asset_path]
                        download_stats["found"] += 1
                        new_robots_found += 1
                        self.logger.info(f"      ✅ Found new robot: {robot_name} -> {asset_path}")
                        
                        # Auto-download if enabled
                        if download_on_discovery:
                            download_result = self.download_single_asset(asset_path, "robots", robot_name)
                            if download_result == "downloaded":
                                download_stats["downloaded"] += 1
                            elif download_result == "skipped":
                                download_stats["skipped"] += 1
                            else:
                                download_stats["failed"] += 1
                                
                elapsed = time.time() - start_time
                self.logger.info(f"   ✅ [{i}/{manufacturer_count}] {manufacturer} completed in {elapsed:.1f}s: {new_robots_found} new robots, {len(found_assets)} total assets")
                                
            except Exception as e:
                elapsed = time.time() - start_time
                self.logger.warning(f"   ⚠️ [{i}/{manufacturer_count}] Error exploring {manufacturer} after {elapsed:.1f}s: {e} - moving to next")
                # Catalog this manufacturer as having undownloaded/unprobed assets
                if manufacturer not in discovered_robots:
                    discovered_robots[f"{manufacturer.lower()}_unprobed"] = [f"/Isaac/Robots/{manufacturer}/"]
        
        self.logger.info(f"🎯 Enhanced robot discovery complete! Found {len(discovered_robots)} robot variants")
        self.logger.info(f"📊 FINAL ROBOT DISCOVERY STATISTICS:")
        self.logger.info(f"    🔍 Known paths tested: {len(known_robot_paths)}")
        self.logger.info(f"    🏭 Manufacturers explored: {len(robot_manufacturers)}")
        self.logger.info(f"    🤖 Total robots found: {len(discovered_robots)}")
        
        if download_on_discovery:
            self.logger.info(f"� DOWNLOAD STATISTICS: Found: {download_stats['found']}, "
                           f"Downloaded: {download_stats['downloaded']}, "
                           f"Skipped: {download_stats['skipped']}, "
                           f"Failed: {download_stats['failed']}")
        
        return discovered_robots
    
    def extract_robot_name_from_path(self, robot_path):
        """Extract a meaningful robot name from its USD path"""
        if not robot_path:
            return None
            
        # Remove extension and extract filename
        base_name = robot_path.split('/')[-1].replace('.usd', '').replace('.usda', '')
        
        # Clean up common patterns
        base_name = base_name.replace('_', ' ').replace('-', ' ')
        
        # Handle special cases
        special_cases = {
            'carter v1': 'carter_v1',
            'nova carter': 'nova_carter',
            'iw hub': 'iw_hub',
            'create 3': 'create_3'
        }
        
        clean_name = special_cases.get(base_name.lower(), base_name.lower().replace(' ', '_'))
        return clean_name if clean_name else None
    
    def test_asset_availability(self, asset_path, timeout=5):
        """Test if an asset is available at the given remote path with timeout"""
        import socket
        import threading
        import time
        
        result = [False]  # Use list to allow modification in nested function
        exception_info = [None]
        status_code = [None]
        
        def test_url():
            try:
                full_url = f"{self.remote_base_url}{asset_path}"
                self.logger.debug(f"Testing URL: {full_url}")
                
                request = urllib.request.Request(full_url, method='HEAD')
                
                # Set socket timeout as backup
                socket.setdefaulttimeout(timeout)
                
                response = urllib.request.urlopen(request, timeout=timeout)
                status_code[0] = response.getcode()
                result[0] = status_code[0] == 200
                
            except urllib.error.HTTPError as e:
                status_code[0] = e.code
                if e.code == 404:
                    exception_info[0] = f"Asset not found (HTTP 404)"
                    result[0] = False
                elif e.code == 403:
                    exception_info[0] = f"Access forbidden (HTTP 403)"
                    result[0] = False
                else:
                    exception_info[0] = f"HTTP error {e.code}: {e.reason}"
                    result[0] = False
            except urllib.error.URLError as e:
                exception_info[0] = f"URL error: {e.reason}"
                result[0] = False
            except socket.timeout:
                exception_info[0] = f"Network timeout after {timeout}s"
                result[0] = False
            except Exception as e:
                exception_info[0] = f"General error: {type(e).__name__}: {e}"
                result[0] = False
            finally:
                socket.setdefaulttimeout(None)  # Reset default timeout
        
        # Use threading timeout as additional protection
        thread = threading.Thread(target=test_url)
        thread.daemon = True
        thread.start()
        thread.join(timeout + 1)  # Give 1 extra second
        
        if thread.is_alive():
            self.logger.debug(f"Asset availability test timed out for {asset_path} after {timeout}s")
            return False
            
        if exception_info[0]:
            self.logger.debug(f"Asset test failed for {asset_path}: {exception_info[0]}")
        elif status_code[0] and status_code[0] != 200:
            self.logger.debug(f"Asset test failed for {asset_path}: HTTP {status_code[0]}")
            
        return result[0]
    
    def discover_directory_structure_runtime(self, base_path, max_depth=3, timeout_per_level=30):
        """Dynamically discover directory structure at runtime with depth control
        
        Args:
            base_path (str): Starting path to explore
            max_depth (int): Maximum depth to traverse (default: 3)
            timeout_per_level (int): Timeout per directory level in seconds
            
        Returns:
            dict: Nested dictionary representing directory structure
        """
        from concurrent.futures import ThreadPoolExecutor, TimeoutError, as_completed
        import time
        
        self.logger.info(f"🔍 Starting runtime directory discovery for {base_path} (max depth: {max_depth})")
        
        discovered_structure = {
            "path": base_path,
            "depth": 0,
            "subdirectories": [],
            "assets": [],
            "exploration_time": 0
        }
        
        def explore_level(current_path, current_depth):
            """Recursively explore directory levels"""
            if current_depth >= max_depth:
                return []
                
            start_time = time.time()
            subdirs = []
            
            try:
                # Try to get directory listing via HTTP directory index (if available)
                directory_contents = self.get_directory_listing_http(current_path, timeout=10)
                
                if directory_contents:
                    self.logger.info(f"    📁 Found {len(directory_contents)} items in {current_path}")
                    
                    for item in directory_contents:
                        item_path = f"{current_path.rstrip('/')}/{item}"
                        
                        # Check if it's a directory (ends with /) or file
                        if item.endswith('/') or self.looks_like_directory(item):
                            subdirs.append({
                                "name": item.rstrip('/'),
                                "path": item_path,
                                "depth": current_depth + 1
                            })
                        elif any(item.endswith(ext) for ext in ['.usd', '.usda', '.mdl']):
                            # Found an asset file
                            discovered_structure["assets"].append(item_path)
                            
                else:
                    # Fallback to pattern-based probing
                    self.logger.info(f"    🔍 No directory listing available for {current_path}, using pattern-based discovery")
                    subdirs = self.probe_common_subdirectory_patterns(current_path, current_depth)
                    
            except Exception as e:
                self.logger.warning(f"    ⚠️ Error exploring {current_path}: {e}")
                
            elapsed = time.time() - start_time
            discovered_structure["exploration_time"] += elapsed
            
            return subdirs
        
        # Start recursive exploration
        def recursive_explore(path_info):
            """Recursively explore subdirectories"""
            subdirs = explore_level(path_info["path"], path_info["depth"])
            path_info["subdirectories"] = subdirs
            
            # Recursively explore subdirectories if within depth limit
            if path_info["depth"] < max_depth - 1:
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = []
                    for subdir in subdirs:
                        future = executor.submit(recursive_explore, subdir)
                        futures.append(future)
                    
                    # Wait for all subdirectory explorations to complete
                    for future in as_completed(futures, timeout=timeout_per_level):
                        try:
                            future.result(timeout=5)
                        except TimeoutError:
                            self.logger.warning(f"    ⏰ Timeout exploring subdirectory")
                        except Exception as e:
                            self.logger.warning(f"    ❌ Error in subdirectory exploration: {e}")
        
        # Start the exploration
        recursive_explore(discovered_structure)
        
        total_time = discovered_structure["exploration_time"]
        total_assets = len(discovered_structure["assets"])
        total_dirs = self.count_discovered_directories(discovered_structure)
        
        self.logger.info(f"🎯 Runtime directory discovery complete!")
        self.logger.info(f"    Time: {total_time:.1f}s")
        self.logger.info(f"    Directories: {total_dirs}")
        self.logger.info(f"    Assets: {total_assets}")
        
        return discovered_structure
    
    def get_directory_listing_http(self, dir_path, timeout=10):
        """Attempt to get directory listing via HTTP directory index
        
        Args:
            dir_path (str): Directory path to list
            timeout (int): Request timeout in seconds
            
        Returns:
            list: List of directory contents, or None if not available
        """
        try:
            full_url = f"{self.remote_base_url}{dir_path}"
            if not full_url.endswith('/'):
                full_url += '/'
                
            self.logger.debug(f"Attempting directory listing for: {full_url}")
            
            request = urllib.request.Request(full_url)
            response = urllib.request.urlopen(request, timeout=timeout)
            
            if response.status == 200:
                content = response.read().decode('utf-8')
                
                # Parse HTML directory listing (common Apache/nginx format)
                import re
                
                # Look for href links in directory listing
                href_pattern = r'href=["\']([^"\']+)["\']'
                matches = re.findall(href_pattern, content, re.IGNORECASE)
                
                # Filter out navigation links and extract actual directory/file names
                items = []
                for match in matches:
                    if match not in ['..', '.', '../']:
                        # Clean up the match
                        clean_match = match.strip('/')
                        if clean_match and not clean_match.startswith('http'):
                            items.append(clean_match)
                
                if items:
                    self.logger.debug(f"Found {len(items)} items via directory listing")
                    return items
                    
        except Exception as e:
            self.logger.debug(f"Directory listing failed for {dir_path}: {e}")
            
        return None
    
    def looks_like_directory(self, item_name):
        """Heuristic to determine if an item name looks like a directory
        
        Args:
            item_name (str): Item name to check
            
        Returns:
            bool: True if item looks like a directory
        """
        # Common directory patterns in Isaac Sim
        directory_indicators = [
            # No file extension
            '.' not in item_name,
            # Common directory names
            any(keyword in item_name.lower() for keyword in [
                'robot', 'environment', 'prop', 'material', 'scene',
                'carter', 'franka', 'ur10', 'jetbot', 'warehouse',
                'models', 'meshes', 'textures', 'animations'
            ]),
            # Version indicators
            any(pattern in item_name.lower() for pattern in [
                'v1', 'v2', 'v3', 'version', 'base', 'sensors'
            ])
        ]
        
        return any(directory_indicators)
    
    def probe_common_subdirectory_patterns(self, base_path, current_depth):
        """Probe for common subdirectory patterns when directory listing is not available
        
        Args:
            base_path (str): Base path to probe
            current_depth (int): Current exploration depth
            
        Returns:
            list: List of discovered subdirectories
        """
        common_patterns = [
            # Version patterns
            "v1", "v2", "v3", "latest", "current",
            # Configuration patterns  
            "base", "sensors", "static", "mobile", "default",
            # Model patterns
            "models", "meshes", "materials", "textures",
            # Robot-specific patterns
            "carter", "franka", "ur10", "ur5", "jetbot", "nova",
            # Environment patterns
            "simple", "warehouse", "office", "hospital", "factory"
        ]
        
        discovered_subdirs = []
        
        # Test each pattern
        for pattern in common_patterns:
            test_path = f"{base_path.rstrip('/')}/{pattern}"
            
            # Quick check if this subdirectory exists
            if self.quick_directory_check(test_path):
                discovered_subdirs.append({
                    "name": pattern,
                    "path": test_path,
                    "depth": current_depth + 1
                })
                self.logger.debug(f"    ✅ Found subdirectory: {pattern}")
                
        return discovered_subdirs
    
    def quick_directory_check(self, dir_path, timeout=3):
        """Quick check if a directory exists by testing for common files
        
        Args:
            dir_path (str): Directory path to check
            timeout (int): Timeout for check
            
        Returns:
            bool: True if directory appears to exist
        """
        # Test for common file patterns in the directory
        test_files = [
            f"{dir_path}/main.usd",
            f"{dir_path}/scene.usd", 
            f"{dir_path}/robot.usd",
            f"{dir_path}/{dir_path.split('/')[-1]}.usd"
        ]
        
        for test_file in test_files:
            try:
                full_url = f"{self.remote_base_url}{test_file}"
                request = urllib.request.Request(full_url, method='HEAD')
                response = urllib.request.urlopen(request, timeout=timeout)
                if response.status == 200:
                    return True
            except:
                continue
                
        return False
    
    def count_discovered_directories(self, structure):
        """Recursively count discovered directories in structure
        
        Args:
            structure (dict): Directory structure from discover_directory_structure_runtime
            
        Returns:
            int: Total count of directories
        """
        count = 1  # Count this directory
        
        for subdir in structure.get("subdirectories", []):
            count += self.count_discovered_directories(subdir)
            
        return count
    
    def print_directory_tree(self, structure, indent=0):
        """Print a visual tree representation of discovered directory structure
        
        Args:
            structure (dict): Directory structure from discover_directory_structure_runtime
            indent (int): Current indentation level
        """
        prefix = "  " * indent
        path_name = structure["path"].split('/')[-1] or structure["path"]
        
        print(f"{prefix}📁 {path_name}/ (depth: {structure['depth']})")
        
        # Print assets in this directory
        for asset in structure.get("assets", []):
            asset_name = asset.split('/')[-1]
            print(f"{prefix}  📄 {asset_name}")
        
        # Print subdirectories
        for subdir in structure.get("subdirectories", []):
            self.print_directory_tree(subdir, indent + 1)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Isaac Sim 5.0 Enhanced Asset Downloader with Auto-Discovery")
    parser.add_argument("--assets-path", default="/home/kimate/isaac_assets", 
                       help="Local path to store assets (default: /home/kimate/isaac_assets)")
    parser.add_argument("--force-update", action="store_true",
                       help="Force re-download of all assets even if they exist locally")
    parser.add_argument("--no-discovery", action="store_true",
                       help="Disable automatic asset discovery, use predefined catalog only")
    parser.add_argument("--verbose", action="store_true",
                       help="Enable verbose logging")
    parser.add_argument("--discovery-only", action="store_true",
                       help="Only run asset discovery without downloading")
    parser.add_argument("--no-auto-download", action="store_true",
                       help="Disable automatic downloading during discovery (discovery-only mode)")
    parser.add_argument("--robots-only", action="store_true",
                       help="Run enhanced robot discovery only")
    parser.add_argument("--skip-environments", action="store_true",
                       help="Skip environment discovery (useful since many environment paths are unresponsive)")
    parser.add_argument("--runtime-traverse", action="store_true",
                       help="Use runtime directory traversal to dynamically discover subdirectories")
    parser.add_argument("--max-depth", type=int, default=3,
                       help="Maximum depth for runtime directory traversal (default: 3)")
    parser.add_argument("--traverse-path", type=str,
                       help="Specific path to traverse (e.g., /Isaac/Robots/NVIDIA/)")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    downloader = IsaacAssetDownloader(
        local_assets_path=args.assets_path,
        force_update=args.force_update,
        discover_assets=not args.no_discovery
    )
    
    # Runtime directory traversal mode
    if args.runtime_traverse:
        downloader.logger.info("🌐 Running runtime directory traversal...")
        
        if args.traverse_path:
            # Traverse specific path
            downloader.logger.info(f"🎯 Traversing specific path: {args.traverse_path}")
            structure = downloader.discover_directory_structure_runtime(
                args.traverse_path, 
                max_depth=args.max_depth
            )
            
            downloader.logger.info("📁 Directory tree structure:")
            downloader.print_directory_tree(structure)
            
        else:
            # Traverse common Isaac Sim paths
            paths_to_traverse = [
                "/Isaac/Robots/",
                "/Isaac/Environments/", 
                "/Isaac/Props/"
            ]
            
            if args.skip_environments:
                paths_to_traverse = [p for p in paths_to_traverse if "Environments" not in p]
            
            for path in paths_to_traverse:
                downloader.logger.info(f"🔍 Traversing {path}...")
                structure = downloader.discover_directory_structure_runtime(
                    path, 
                    max_depth=args.max_depth
                )
                
                downloader.logger.info(f"📁 Directory tree for {path}:")
                downloader.print_directory_tree(structure)
                print("\n" + "="*60 + "\n")
    
    # Determine download behavior
    auto_download = not (args.discovery_only or args.no_auto_download)
    
    if args.robots_only and not args.runtime_traverse:
        downloader.logger.info("🤖 Running enhanced robot discovery only...")
        discovered_robots = downloader.discover_robot_assets_enhanced(download_on_discovery=auto_download)
        downloader.logger.info(f"🎯 Robot discovery complete! Found {len(discovered_robots)} robot variants")
        for robot_name, paths in discovered_robots.items():
            downloader.logger.info(f"  ✅ {robot_name}: {paths[0]}")
    
    elif (args.discovery_only or args.no_auto_download) and not args.runtime_traverse:
        downloader.logger.info("🔍 Running discovery-only mode...")
        
        # Apply environment skip if requested
        if args.skip_environments:
            downloader.logger.info("⏭️  Skipping environment discovery (--skip-environments flag)")
            # Temporarily modify asset categories to exclude environments
            original_categories = downloader.asset_categories.copy()
            downloader.asset_categories = {k: v for k, v in original_categories.items() if k != "environments"}
        
        discovered = downloader.discover_remote_assets(download_on_discovery=False)
        
        # Restore original categories if modified
        if args.skip_environments:
            downloader.asset_categories = original_categories
            
        downloader.logger.info(f"🎯 Discovery complete! Found assets in categories: {list(discovered.keys())}")
        for category, assets in discovered.items():
            downloader.logger.info(f"  {category}: {len(assets)} assets")
    
    elif not args.runtime_traverse:
        # Full download with auto-discovery and auto-download
        downloader.logger.info("🚀 Running full asset discovery and download...")
        
        # Apply environment skip if requested
        if args.skip_environments:
            downloader.logger.info("⏭️  Skipping environment discovery (--skip-environments flag)")
            # Temporarily modify asset categories to exclude environments
            original_categories = downloader.asset_categories.copy()
            downloader.asset_categories = {k: v for k, v in original_categories.items() if k != "environments"}
        
        downloader.run()
        
        # Restore original categories if modified
        if args.skip_environments:
            downloader.asset_categories = original_categories
