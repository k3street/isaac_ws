#!/usr/bin/env python3
"""
Isaac Sim Scenario Status Monitor
Real-time monitoring of scenario system status
"""

import json
import time
import os
from pathlib import Path
from datetime import datetime
from scenario_manager import ScenarioManager

class ScenarioStatusMonitor:
    """Monitor and display scenario system status"""
    
    def __init__(self):
        self.scenario_manager = ScenarioManager(enable_logging=False)  # Silent for monitoring
        self.config_file = Path("/tmp/isaac_scenario_config.json")
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        self.log_dir = Path("/tmp/isaac_logs")
        
    def get_system_status(self):
        """Get comprehensive system status"""
        status = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "scenario_manager": {
                "status": "active",
                "assets": {
                    "scenes": len(self.scenario_manager.available_scenes),
                    "robots": len(self.scenario_manager.available_robots),
                    "props": len(self.scenario_manager.available_props)
                }
            },
            "files": {
                "scenario_config": {
                    "exists": self.config_file.exists(),
                    "size": self.config_file.stat().st_size if self.config_file.exists() else 0,
                    "modified": datetime.fromtimestamp(self.config_file.stat().st_mtime).strftime("%H:%M:%S") if self.config_file.exists() else "N/A"
                },
                "camera_commands": {
                    "exists": self.command_file.exists(),
                    "size": self.command_file.stat().st_size if self.command_file.exists() else 0,
                    "modified": datetime.fromtimestamp(self.command_file.stat().st_mtime).strftime("%H:%M:%S") if self.command_file.exists() else "N/A"
                }
            },
            "logs": {
                "directory_exists": self.log_dir.exists(),
                "log_files": len(list(self.log_dir.glob("*.log"))) if self.log_dir.exists() else 0,
                "latest_log": self.get_latest_log_info()
            }
        }
        
        # Load current scenario if available
        if self.config_file.exists():
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    status["current_scenario"] = {
                        "request": config.get("request", "Unknown"),
                        "scene": config.get("parsed_scenario", {}).get("scene", "Unknown"),
                        "robots": len(config.get("parsed_scenario", {}).get("robots", [])),
                        "valid": config.get("is_valid", False),
                        "warnings": len(config.get("warnings", []))
                    }
            except Exception as e:
                status["current_scenario"] = {"error": str(e)}
        else:
            status["current_scenario"] = None
            
        return status
    
    def get_latest_log_info(self):
        """Get information about the latest log file"""
        if not self.log_dir.exists():
            return None
            
        log_files = list(self.log_dir.glob("*.log"))
        if not log_files:
            return None
            
        latest_log = max(log_files, key=lambda f: f.stat().st_mtime)
        return {
            "file": latest_log.name,
            "size": latest_log.stat().st_size,
            "modified": datetime.fromtimestamp(latest_log.stat().st_mtime).strftime("%H:%M:%S")
        }
    
    def display_status(self, status):
        """Display formatted status information"""
        print(f"\n🎯 Isaac Sim Scenario System Status - {status['timestamp']}")
        print("=" * 60)
        
        # Scenario Manager Status
        sm = status["scenario_manager"]
        print(f"📊 Scenario Manager: {sm['status']}")
        print(f"   Available assets: {sm['assets']['scenes']} scenes, {sm['assets']['robots']} robots, {sm['assets']['props']} props")
        
        # Current Scenario
        if status["current_scenario"]:
            if "error" in status["current_scenario"]:
                print(f"❌ Current Scenario: Error - {status['current_scenario']['error']}")
            else:
                cs = status["current_scenario"]
                validity = "✅ Valid" if cs["valid"] else f"⚠️  {cs['warnings']} warnings"
                print(f"🎬 Current Scenario: {cs['scene']} with {cs['robots']} robot(s) ({validity})")
                print(f"   Request: '{cs['request']}'")
        else:
            print(f"🎬 Current Scenario: None loaded")
        
        # File Status
        files = status["files"]
        print(f"\n📁 Files:")
        
        config_status = "✅" if files["scenario_config"]["exists"] else "❌"
        print(f"   {config_status} Scenario Config: {files['scenario_config']['size']} bytes (modified: {files['scenario_config']['modified']})")
        
        cmd_status = "✅" if files["camera_commands"]["exists"] else "❌"
        print(f"   {cmd_status} Camera Commands: {files['camera_commands']['size']} bytes (modified: {files['camera_commands']['modified']})")
        
        # Log Status
        logs = status["logs"]
        log_status = "✅" if logs["directory_exists"] else "❌"
        print(f"   {log_status} Logs: {logs['log_files']} log file(s)")
        
        if logs["latest_log"]:
            print(f"      Latest: {logs['latest_log']['file']} ({logs['latest_log']['size']} bytes, {logs['latest_log']['modified']})")
    
    def monitor_continuous(self, interval=5):
        """Continuously monitor and display status"""
        print("🔄 Starting continuous monitoring (Press Ctrl+C to stop)")
        
        try:
            while True:
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Get and display status
                status = self.get_system_status()
                self.display_status(status)
                
                print(f"\n💡 Refreshing every {interval} seconds...")
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped")
    
    def export_status_report(self, filename=None):
        """Export detailed status report to file"""
        if not filename:
            filename = f"/tmp/scenario_status_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        status = self.get_system_status()
        
        with open(filename, 'w') as f:
            json.dump(status, f, indent=2)
        
        print(f"📄 Status report exported to: {filename}")
        return filename

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Isaac Sim Scenario Status Monitor')
    parser.add_argument('--monitor', '-m', action='store_true', help='Continuous monitoring mode')
    parser.add_argument('--interval', '-i', type=int, default=5, help='Monitoring interval in seconds')
    parser.add_argument('--export', '-e', action='store_true', help='Export status report to file')
    
    args = parser.parse_args()
    
    monitor = ScenarioStatusMonitor()
    
    if args.export:
        monitor.export_status_report()
    elif args.monitor:
        monitor.monitor_continuous(args.interval)
    else:
        # Single status check
        status = monitor.get_system_status()
        monitor.display_status(status)
        
        print(f"\n💡 Usage:")
        print(f"   --monitor/-m: Continuous monitoring")
        print(f"   --export/-e: Export status report")
        print(f"   --interval/-i: Set monitoring interval")

if __name__ == '__main__':
    main()
