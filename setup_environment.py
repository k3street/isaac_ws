#!/usr/bin/env python3
"""
Environment setup script for Isaac Sim ROS2 Camera Controller
Helps users configure their .env file with API keys
"""

import os
import sys
import shutil

def print_header():
    print("🔧" + "="*60 + "🔧")
    print("🔑 Isaac Sim Camera Controller - Environment Setup")
    print("="*64)

def check_env_file():
    """Check if .env file exists"""
    env_path = ".env"
    example_path = ".env.example"
    
    if os.path.exists(env_path):
        print(f"✅ Found existing .env file: {env_path}")
        return env_path
    elif os.path.exists(example_path):
        print(f"📋 Copying {example_path} to {env_path}")
        shutil.copy(example_path, env_path)
        print(f"✅ Created .env file: {env_path}")
        return env_path
    else:
        print(f"❌ No .env or .env.example file found")
        return None

def display_env_template():
    """Display the .env template"""
    print("\n📝 Your .env file should contain:")
    print("-" * 40)
    template = """# LLM API Keys
OPENAI_API_KEY=your_openai_api_key_here
GEMINI_API_KEY=your_gemini_api_key_here
ANTHROPIC_API_KEY=your_anthropic_api_key_here

# Optional: Camera Controller Settings
ANALYSIS_INTERVAL=2.0
MOVEMENT_SCALE=0.5
ROTATION_SCALE=0.3
CONFIDENCE_THRESHOLD=0.5"""
    print(template)
    print("-" * 40)

def get_api_key_instructions():
    """Provide instructions for getting API keys"""
    print("\n🔑 How to get API keys:")
    print()
    print("1️⃣ Gemini API Key (Recommended for robotics):")
    print("   🌐 Visit: https://aistudio.google.com/app/apikey")
    print("   📝 Sign in with Google account")
    print("   🔧 Click 'Create API Key'")
    print("   📋 Copy and add to .env: GEMINI_API_KEY=your_key_here")
    print()
    print("2️⃣ OpenAI API Key:")
    print("   🌐 Visit: https://platform.openai.com/api-keys")
    print("   📝 Sign in to OpenAI account")
    print("   🔧 Click 'Create new secret key'")
    print("   📋 Copy and add to .env: OPENAI_API_KEY=your_key_here")
    print()
    print("3️⃣ Anthropic Claude API Key:")
    print("   🌐 Visit: https://console.anthropic.com/")
    print("   📝 Sign in to Anthropic account")
    print("   🔧 Go to API Keys section")
    print("   📋 Copy and add to .env: ANTHROPIC_API_KEY=your_key_here")

def check_current_env():
    """Check what's currently in the environment"""
    print("\n📊 Current Environment Status:")
    print("-" * 30)
    
    keys_to_check = [
        ('OPENAI_API_KEY', 'OpenAI GPT-4'),
        ('GEMINI_API_KEY', 'Google Gemini 2.5'),
        ('GOOGLE_API_KEY', 'Google AI (alternative)'),
        ('ANTHROPIC_API_KEY', 'Anthropic Claude')
    ]
    
    for key, name in keys_to_check:
        value = os.getenv(key)
        if value:
            masked_value = value[:8] + "..." + value[-4:] if len(value) > 12 else "***"
            print(f"   ✅ {name}: {masked_value}")
        else:
            print(f"   ❌ {name}: Not set")

def test_dependencies():
    """Test if required dependencies are available"""
    print("\n🔍 Checking Dependencies:")
    print("-" * 25)
    
    try:
        from dotenv import load_dotenv
        print("   ✅ python-dotenv: Available")
    except ImportError:
        print("   ❌ python-dotenv: Missing")
        print("      Install with: pip install python-dotenv")
    
    try:
        import google.generativeai
        print("   ✅ google-generativeai: Available")
    except ImportError:
        print("   ❌ google-generativeai: Missing")
        print("      Install with: pip install google-generativeai")

def provide_usage_examples():
    """Show usage examples"""
    print("\n🚀 Usage Examples:")
    print("-" * 20)
    print("1. Test your setup:")
    print("   python3 test_gemini_integration.py")
    print()
    print("2. Launch with Gemini 2.5:")
    print("   ros2 launch isaac_test llm_camera_controller_simple.launch.py \\")
    print("     llm_provider:=gemini_2.5")
    print()
    print("3. Launch with specific API key override:")
    print("   ros2 launch isaac_test llm_camera_controller_simple.launch.py \\")
    print("     llm_provider:=gemini_2.5 \\")
    print("     gemini_api_key:='your-key-here'")
    print()
    print("4. Use environment variables only:")
    print("   export GEMINI_API_KEY='your-key'")
    print("   ros2 launch isaac_test llm_camera_controller_simple.launch.py \\")
    print("     llm_provider:=gemini_2.5")

def main():
    print_header()
    
    # Check .env file
    env_file = check_env_file()
    
    # Load environment if possible
    try:
        from dotenv import load_dotenv
        if env_file:
            load_dotenv()
            print("🔄 Environment variables loaded")
    except ImportError:
        pass
    
    # Display template and instructions
    display_env_template()
    get_api_key_instructions()
    
    # Check current status
    check_current_env()
    test_dependencies()
    
    # Provide usage examples
    provide_usage_examples()
    
    print("\n" + "="*64)
    print("🎯 Next Steps:")
    print("1. Edit .env file with your API keys")
    print("2. Run: python3 test_gemini_integration.py")
    print("3. Launch the camera controller with your preferred LLM")
    print("="*64)

if __name__ == "__main__":
    main()
