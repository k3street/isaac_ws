#!/usr/bin/env python3
"""
Isaac Test Package - Simplified LLM Camera Controller Node
Supports multiple LLM providers via ROS2 parameters with real Gemini 2.5 implementation
Environment variables loaded from .env file for API keys
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import json
import time
import threading
import os
from enum import Enum
import base64
import io
from PIL import Image as PILImage

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
    DOTENV_AVAILABLE = True
except ImportError:
    DOTENV_AVAILABLE = False

# LLM API imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    
try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

class LLMProvider(Enum):
    """Supported LLM providers"""
    OPENAI_GPT41 = "openai_gpt4.1"
    GEMINI_25 = "gemini_2.5"
    CLAUDE_4_SONNET = "claude_4_sonnet"
    SIMULATION = "simulation"  # Computer vision simulation

class LLMCameraController(Node):
    """ROS2 node for intelligent camera control using LLM vision analysis"""
    
    def __init__(self):
        super().__init__('isaac_test_llm_controller')
        
        # Declare ROS2 parameters with environment variable defaults
        self.declare_parameter('analysis_interval', float(os.getenv('ANALYSIS_INTERVAL', '2.0')))
        self.declare_parameter('movement_scale', float(os.getenv('MOVEMENT_SCALE', '0.5')))
        self.declare_parameter('rotation_scale', float(os.getenv('ROTATION_SCALE', '0.3')))
        self.declare_parameter('confidence_threshold', float(os.getenv('CONFIDENCE_THRESHOLD', '0.5')))
        
        # LLM Configuration Parameters with environment variable defaults
        self.declare_parameter('llm_provider', 'simulation')
        self.declare_parameter('openai_api_key', os.getenv('OPENAI_API_KEY', ''))
        self.declare_parameter('gemini_api_key', os.getenv('GEMINI_API_KEY', '') or os.getenv('GOOGLE_API_KEY', ''))
        self.declare_parameter('anthropic_api_key', os.getenv('ANTHROPIC_API_KEY', ''))
        self.declare_parameter('llm_model_version', os.getenv('LLM_MODEL_VERSION', ''))
        self.declare_parameter('llm_temperature', float(os.getenv('LLM_TEMPERATURE', '0.1')))
        self.declare_parameter('llm_max_tokens', int(os.getenv('LLM_MAX_TOKENS', '300')))
        
        # Get parameters
        self.analysis_interval = self.get_parameter('analysis_interval').value
        self.movement_scale = self.get_parameter('movement_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # LLM Configuration
        llm_provider_str = self.get_parameter('llm_provider').value
        self.llm_provider = self._validate_llm_provider(llm_provider_str)
        self.openai_api_key = self.get_parameter('openai_api_key').value
        self.gemini_api_key = self.get_parameter('gemini_api_key').value
        self.anthropic_api_key = self.get_parameter('anthropic_api_key').value
        self.llm_model_version = self.get_parameter('llm_model_version').value
        self.llm_temperature = self.get_parameter('llm_temperature').value
        self.llm_max_tokens = self.get_parameter('llm_max_tokens').value
        
        # Initialize LLM client (stub)
        self.llm_client = None
        self._initialize_llm_client()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State variables
        self.current_image = None
        self.analysis_active = True
        self.last_analysis_time = 0
        self.analysis_queue = []
        
        # ROS2 Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/camera/cmd_vel', 10)
        self.analysis_pub = self.create_publisher(String, '/isaac_test/llm_analysis', 10)
        self.status_pub = self.create_publisher(String, '/isaac_test/controller_status', 10)
        self.active_pub = self.create_publisher(Bool, '/isaac_test/controller_active', 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb', self.image_callback, 10)
        
        # Analysis thread
        self.analysis_thread = threading.Thread(target=self._analysis_worker, daemon=True)
        self.analysis_thread.start()
        
        # Status timer (every 5 seconds)
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('🤖 Isaac Test LLM Controller initialized')
        if DOTENV_AVAILABLE:
            self.get_logger().info('📁 Environment variables loaded from .env file')
        else:
            self.get_logger().info('⚠️  python-dotenv not available, using system environment only')
        self.get_logger().info(f'🧠 LLM Provider: {self.llm_provider.value}')
        self.get_logger().info(f'📸 Subscribing: /camera/rgb')
        self.get_logger().info(f'🎮 Publishing: /camera/cmd_vel')
        self.get_logger().info(f'📝 Publishing: /isaac_test/llm_analysis')
        self.get_logger().info(f'⚙️  Analysis interval: {self.analysis_interval}s')
        
        if self.llm_provider != LLMProvider.SIMULATION:
            api_status = "✅ Ready" if self._check_api_key() else "❌ API key needed"
            self.get_logger().info(f'🔑 API Status: {api_status}')
    
    def _validate_llm_provider(self, provider_str):
        """Validate and return LLM provider enum"""
        provider_map = {
            'openai': LLMProvider.OPENAI_GPT41,
            'openai_gpt4.1': LLMProvider.OPENAI_GPT41,
            'openai_gpt41': LLMProvider.OPENAI_GPT41,
            'gpt4': LLMProvider.OPENAI_GPT41,
            'gpt41': LLMProvider.OPENAI_GPT41,
            
            'gemini': LLMProvider.GEMINI_25,
            'gemini_2.5': LLMProvider.GEMINI_25,
            'gemini25': LLMProvider.GEMINI_25,
            'google': LLMProvider.GEMINI_25,
            
            'claude': LLMProvider.CLAUDE_4_SONNET,
            'claude_4_sonnet': LLMProvider.CLAUDE_4_SONNET,
            'claude4': LLMProvider.CLAUDE_4_SONNET,
            'anthropic': LLMProvider.CLAUDE_4_SONNET,
            'sonnet': LLMProvider.CLAUDE_4_SONNET,
            
            'simulation': LLMProvider.SIMULATION,
            'sim': LLMProvider.SIMULATION,
            'cv': LLMProvider.SIMULATION
        }
        
        provider = provider_map.get(provider_str.lower(), LLMProvider.SIMULATION)
        self.get_logger().info(f'🤖 Using LLM Provider: {provider.value}')
        return provider
    
    def _check_api_key(self):
        """Check if API key is available for the selected provider"""
        if self.llm_provider == LLMProvider.OPENAI_GPT41:
            return bool(self.openai_api_key)
        elif self.llm_provider == LLMProvider.GEMINI_25:
            return bool(self.gemini_api_key)
        elif self.llm_provider == LLMProvider.CLAUDE_4_SONNET:
            return bool(self.anthropic_api_key)
        return True  # Simulation doesn't need API key
    
    def _initialize_llm_client(self):
        """Initialize LLM client with real implementations"""
        self.get_logger().info(f'🔍 DEBUG: GEMINI_AVAILABLE = {GEMINI_AVAILABLE}')
        self.get_logger().info(f'🔍 DEBUG: llm_provider = {self.llm_provider}')
        self.get_logger().info(f'🔍 DEBUG: gemini_api_key = {"*REDACTED*" if self.gemini_api_key else "None"}')
        try:
            if self.llm_provider == LLMProvider.GEMINI_25:
                # Try to import Gemini at runtime
                try:
                    import google.generativeai as genai
                    self.get_logger().info('🔍 DEBUG: Gemini import successful at runtime')
                except ImportError:
                    self.get_logger().error('🔍 DEBUG: Gemini import failed at runtime')
                    self.get_logger().info('🎯 Using computer vision simulation mode')
                    self.llm_client = "simulation"
                    return
                if not self.gemini_api_key:
                    self.get_logger().warn('� Gemini API key not provided, using STUB mode')
                    self.llm_client = "gemini_stub"
                    return
                    
                # Configure Gemini
                genai.configure(api_key=self.gemini_api_key)
                
                # Choose the right model
                model_name = self.llm_model_version or "gemini-2.5-pro-vision"
                if "flash" in model_name.lower():
                    model_name = "gemini-2.5-flash"
                elif "pro" in model_name.lower():
                    model_name = "gemini-2.5-pro"
                    
                self.llm_client = genai.GenerativeModel(model_name)
                self.get_logger().info(f'🔧 Gemini 2.5 client initialized with model: {model_name}')
                self.get_logger().info(f'🔍 DEBUG: llm_client type: {type(self.llm_client)}')
                
            elif self.llm_provider == LLMProvider.OPENAI_GPT41 and OPENAI_AVAILABLE:
                if not self.openai_api_key:
                    self.get_logger().warn('🔑 OpenAI API key not provided, using STUB mode')
                    self.llm_client = "openai_stub"
                    return
                    
                # Configure OpenAI
                openai.api_key = self.openai_api_key
                self.llm_client = openai
                self.get_logger().info('🔧 OpenAI GPT-4.1 client initialized')
                
            elif self.llm_provider == LLMProvider.CLAUDE_4_SONNET and ANTHROPIC_AVAILABLE:
                if not self.anthropic_api_key:
                    self.get_logger().warn('� Anthropic API key not provided, using STUB mode')
                    self.llm_client = "claude_stub"
                    return
                    
                # Configure Claude
                self.llm_client = anthropic.Anthropic(api_key=self.anthropic_api_key)
                self.get_logger().info('🔧 Anthropic Claude 4 Sonnet client initialized')
                
            else:
                self.get_logger().info('🎯 Using computer vision simulation mode')
                self.llm_client = "simulation"
                
        except Exception as e:
            self.get_logger().error(f'LLM initialization error: {e}')
            self.get_logger().warn('🔄 Falling back to stub mode')
            self.llm_provider = LLMProvider.SIMULATION
            self.llm_client = "simulation"
    
    def image_callback(self, msg):
        """Handle incoming camera images"""
        if not self.analysis_active:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Add to analysis queue
            timestamp = time.time()
            if len(self.analysis_queue) < 5:  # Limit queue size
                self.analysis_queue.append((cv_image.copy(), timestamp))
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def _analysis_worker(self):
        """Background thread for image analysis"""
        while rclpy.ok():
            try:
                if self.analysis_queue and time.time() - self.last_analysis_time > self.analysis_interval:
                    # Get latest image
                    cv_image, timestamp = self.analysis_queue.pop(0)
                    self.analysis_queue.clear()  # Clear old images
                    
                    # Analyze scene
                    analysis = self.analyze_scene(cv_image, timestamp)
                    
                    # Execute movement if confidence is high enough
                    if analysis['confidence'] > self.confidence_threshold:
                        self.execute_movement(analysis)
                    
                    # Publish analysis
                    self.publish_analysis(analysis)
                    self.last_analysis_time = timestamp
                    
                time.sleep(0.1)  # Small delay to prevent busy waiting
                
            except Exception as e:
                self.get_logger().error(f'Analysis worker error: {e}')
    
    def analyze_scene(self, image, timestamp):
        """Analyze scene using selected LLM provider or simulation"""
        try:
            if self.llm_provider == LLMProvider.SIMULATION:
                return self._analyze_scene_simulation(image, timestamp)
            elif self.llm_provider == LLMProvider.GEMINI_25:
                return self._analyze_scene_gemini(image, timestamp)
            elif self.llm_provider == LLMProvider.OPENAI_GPT41:
                return self._analyze_scene_openai(image, timestamp)
            elif self.llm_provider == LLMProvider.CLAUDE_4_SONNET:
                return self._analyze_scene_claude(image, timestamp)
            else:
                return self._analyze_scene_simulation(image, timestamp)
                
        except Exception as e:
            self.get_logger().error(f'Scene analysis error: {e}')
            return {
                "timestamp": timestamp,
                "action": "hold_position",
                "reasoning": f"Analysis error: {e}",
                "confidence": 0.0,
                "provider": self.llm_provider.value,
                "error": str(e)
            }
    
    def _analyze_scene_gemini(self, image, timestamp):
        """Analyze scene using Gemini 2.5 for robotics vision"""
        try:
            # Check if we have a real client or fallback to stub
            self.get_logger().info(f'🔍 DEBUG: llm_client type in analysis: {type(self.llm_client)}')
            self.get_logger().info(f'🔍 DEBUG: llm_client value: {self.llm_client}')
            if isinstance(self.llm_client, str) and "stub" in self.llm_client:
                return self._analyze_scene_llm_stub(image, timestamp)
            
            # Convert OpenCV image to PIL Image
            height, width = image.shape[:2]
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Resize image for API efficiency (keep aspect ratio)
            max_size = 1024
            if max(width, height) > max_size:
                ratio = max_size / max(width, height)
                new_width = int(width * ratio)
                new_height = int(height * ratio)
                pil_image = pil_image.resize((new_width, new_height), PILImage.Resampling.LANCZOS)
                self.get_logger().info(f'📏 Resized image from {width}x{height} to {new_width}x{new_height}')
            
            # Gemini 2.5 robotics prompt based on the blog examples
            robotics_prompt = f"""
You are an intelligent camera controller for an Isaac Sim robotics environment. Analyze this scene and provide camera movement recommendations.

SCENE ANALYSIS TASKS:
1. Identify key objects, robots, and areas of interest in the scene
2. Assess the current camera perspective and coverage
3. Determine if the camera should move for better observation
4. Consider robotics principles: object tracking, workspace visibility, safety zones

CAMERA ACTIONS AVAILABLE:
- move_closer: Move camera forward to get closer to objects
- move_back: Move camera backward for wider view  
- move_left: Move camera left
- move_right: Move camera right
- move_up: Move camera up for overhead perspective
- move_down: Move camera down for ground-level view
- rotate_left: Rotate camera left
- rotate_right: Rotate camera right
- tilt_up: Tilt camera up
- tilt_down: Tilt camera down
- hold_position: Stay in current position

RESPONSE FORMAT (JSON):
{{
    "scene_description": "Brief description of what you see in the robotics scene",
    "objects_detected": ["list", "of", "key", "objects"],
    "recommended_action": "one_of_the_actions_above",
    "reasoning": "Why you chose this action based on robotics principles",
    "confidence": 0.0-1.0,
    "priority_areas": ["areas", "that", "need", "better", "coverage"],
    "safety_considerations": "Any safety or collision concerns"
}}

Focus on practical robotics applications: manipulation zones, robot workspace visibility, object tracking, and operational efficiency.
"""

            # Generate content with Gemini 2.5
            response = self.llm_client.generate_content([robotics_prompt, pil_image])
            
            if not response.text:
                raise Exception("Empty response from Gemini")
            
            # Parse JSON response
            try:
                # Clean the response text (remove markdown formatting if present)
                response_text = response.text.strip()
                if response_text.startswith('```json'):
                    response_text = response_text[7:]
                if response_text.endswith('```'):
                    response_text = response_text[:-3]
                response_text = response_text.strip()
                
                analysis_data = json.loads(response_text)
                
                # Structure the response according to our expected format
                analysis = {
                    "timestamp": timestamp,
                    "provider": "gemini_2.5",
                    "model": self._get_model_name(),
                    "scene_description": analysis_data.get("scene_description", "Scene analyzed by Gemini 2.5"),
                    "objects_detected": analysis_data.get("objects_detected", []),
                    "action": analysis_data.get("recommended_action", "hold_position"),
                    "reasoning": analysis_data.get("reasoning", "Gemini 2.5 robotics analysis"),
                    "confidence": float(analysis_data.get("confidence", 0.7)),
                    "priority_areas": analysis_data.get("priority_areas", []),
                    "safety_considerations": analysis_data.get("safety_considerations", "None identified"),
                    "raw_response": response.text
                }
                
                self.get_logger().info(f'🧠 Gemini 2.5 Analysis: {analysis["action"]} - {analysis["reasoning"]}')
                self.get_logger().info(f'🎯 Objects detected: {", ".join(analysis["objects_detected"])}')
                
                return analysis
                
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse Gemini JSON response: {e}')
                self.get_logger().error(f'Raw response: {response.text}')
                
                # Fallback: extract action from text if JSON parsing fails
                action = "hold_position"
                if "move_closer" in response.text.lower():
                    action = "move_closer"
                elif "move_back" in response.text.lower():
                    action = "move_back"
                elif "move_up" in response.text.lower():
                    action = "move_up"
                elif "rotate_left" in response.text.lower():
                    action = "rotate_left"
                elif "rotate_right" in response.text.lower():
                    action = "rotate_right"
                
                return {
                    "timestamp": timestamp,
                    "provider": "gemini_2.5",
                    "model": self._get_model_name(),
                    "scene_description": "Gemini 2.5 analysis (text parsing fallback)",
                    "action": action,
                    "reasoning": f"Extracted from text response: {response.text[:100]}...",
                    "confidence": 0.6,
                    "raw_response": response.text,
                    "parse_error": str(e)
                }
                
        except Exception as e:
            self.get_logger().error(f'Gemini analysis error: {e}')
            # Fallback to stub mode
            return self._analyze_scene_llm_stub(image, timestamp)
    
    def _analyze_scene_openai(self, image, timestamp):
        """Analyze scene using OpenAI GPT-4.1 Vision"""
        try:
            # Check if we have a real client or fallback to stub
            if isinstance(self.llm_client, str) and "stub" in self.llm_client:
                return self._analyze_scene_llm_stub(image, timestamp)
            
            # Convert image to base64 for OpenAI API
            height, width = image.shape[:2]
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Resize for efficiency
            max_size = 1024
            if max(width, height) > max_size:
                ratio = max_size / max(width, height)
                new_size = (int(width * ratio), int(height * ratio))
                pil_image = pil_image.resize(new_size, PILImage.Resampling.LANCZOS)
            
            # Convert to base64
            buffer = io.BytesIO()
            pil_image.save(buffer, format='JPEG')
            image_b64 = base64.b64encode(buffer.getvalue()).decode()
            
            # OpenAI Vision API call (placeholder - would need actual implementation)
            analysis = {
                "timestamp": timestamp,
                "provider": "openai_gpt4.1",
                "model": self._get_model_name(),
                "scene_description": "OpenAI GPT-4.1 Vision analysis (STUB - implement with actual API)",
                "action": "move_closer",
                "reasoning": "OpenAI analysis suggests closer inspection",
                "confidence": 0.8
            }
            
            self.get_logger().info(f'🧠 OpenAI Analysis (STUB): {analysis["action"]} - {analysis["reasoning"]}')
            return analysis
            
        except Exception as e:
            self.get_logger().error(f'OpenAI analysis error: {e}')
            return self._analyze_scene_llm_stub(image, timestamp)
    
    def _analyze_scene_claude(self, image, timestamp):
        """Analyze scene using Anthropic Claude 4 Sonnet"""
        try:
            # Check if we have a real client or fallback to stub
            if isinstance(self.llm_client, str) and "stub" in self.llm_client:
                return self._analyze_scene_llm_stub(image, timestamp)
            
            # Claude vision implementation (placeholder - would need actual implementation)
            analysis = {
                "timestamp": timestamp,
                "provider": "claude_4_sonnet",
                "model": self._get_model_name(),
                "scene_description": "Claude 4 Sonnet analysis (STUB - implement with actual API)",
                "action": "move_up",
                "reasoning": "Claude analysis suggests elevated perspective",
                "confidence": 0.8
            }
            
            self.get_logger().info(f'🧠 Claude Analysis (STUB): {analysis["action"]} - {analysis["reasoning"]}')
            return analysis
            
        except Exception as e:
            self.get_logger().error(f'Claude analysis error: {e}')
            return self._analyze_scene_llm_stub(image, timestamp)

    def _analyze_scene_llm_stub(self, image, timestamp):
        """STUB: Analyze scene using LLM - placeholder implementation"""
        height, width = image.shape[:2]
        
        # Simulate LLM analysis with basic computer vision
        analysis = {
            "timestamp": timestamp,
            "provider": self.llm_provider.value,
            "model": self._get_model_name(),
            "scene_description": f"Scene analysis via {self.llm_provider.value} (STUB)",
            "confidence": 0.8,
            "action": "hold_position",
            "reasoning": "LLM analysis placeholder - maintaining current position"
        }
        
        # Add some variety based on provider with higher confidence to ensure movement
        if self.llm_provider == LLMProvider.OPENAI_GPT41:
            analysis.update({
                "action": "move_closer",
                "reasoning": "OpenAI GPT-4.1 suggests closer inspection (STUB)",
                "confidence": 0.9
            })
        elif self.llm_provider == LLMProvider.GEMINI_25:
            analysis.update({
                "action": "rotate_left",
                "reasoning": "Gemini 2.5 recommends left rotation for better view (STUB)",
                "confidence": 0.9
            })
        elif self.llm_provider == LLMProvider.CLAUDE_4_SONNET:
            analysis.update({
                "action": "move_up",
                "reasoning": "Claude 4 Sonnet suggests elevated perspective (STUB)",
                "confidence": 0.9
            })
        
        self.get_logger().info(f'🧠 {self.llm_provider.value} Analysis (STUB): {analysis["action"]} - {analysis["reasoning"]}')
        return analysis
    
    def _get_model_name(self):
        """Get the specific model name for the provider"""
        model_map = {
            LLMProvider.OPENAI_GPT41: self.llm_model_version or "gpt-4.1-vision-preview",
            LLMProvider.GEMINI_25: self.llm_model_version or "gemini-2.5-pro-vision",
            LLMProvider.CLAUDE_4_SONNET: self.llm_model_version or "claude-4-sonnet-20240620",
            LLMProvider.SIMULATION: "computer_vision_simulation"
        }
        return model_map.get(self.llm_provider, "unknown")
    
    def _analyze_scene_simulation(self, image, timestamp):
        """Computer vision simulation analysis (original implementation)"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Simple blob detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        analysis = {
            "timestamp": timestamp,
            "provider": "simulation",
            "model": "computer_vision",
            "scene_description": f"Found {len(contours)} objects in scene",
            "confidence": 0.7,
            "action": "hold_position",
            "reasoning": "Scene appears stable"
        }
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Determine action based on object position
                if cx < center_x - 50:
                    analysis.update({"action": "move_left", "reasoning": "Object is right of center"})
                elif cx > center_x + 50:
                    analysis.update({"action": "move_right", "reasoning": "Object is left of center"})
                elif cy < center_y - 50:
                    analysis.update({"action": "move_up", "reasoning": "Object is below center"})
                elif cy > center_y + 50:
                    analysis.update({"action": "move_down", "reasoning": "Object is above center"})
        
        return analysis
    
    def execute_movement(self, analysis):
        """Execute camera movement based on analysis"""
        twist = Twist()
        
        action = analysis.get('action', 'hold_position')
        confidence = analysis.get('confidence', 0.0)
        
        self.get_logger().info(f'🎯 Analysis result: action={action}, confidence={confidence:.2f}, threshold={self.confidence_threshold}')
        
        # Scale movement by confidence
        linear_scale = self.movement_scale * confidence
        angular_scale = self.rotation_scale * confidence
        
        self.get_logger().info(f'📏 Movement scales: linear={linear_scale:.3f}, angular={angular_scale:.3f}')
        
        # Map actions to movements
        if action == 'move_closer':
            twist.linear.x = float(linear_scale)
        elif action == 'move_back':
            twist.linear.x = float(-linear_scale)
        elif action == 'move_left':
            twist.linear.y = float(linear_scale)
        elif action == 'move_right':
            twist.linear.y = float(-linear_scale)
        elif action == 'move_up':
            twist.linear.z = float(linear_scale)
        elif action == 'move_down':
            twist.linear.z = float(-linear_scale)
        elif action == 'rotate_left':
            twist.angular.z = float(angular_scale)
        elif action == 'rotate_right':
            twist.angular.z = float(-angular_scale)
        elif action == 'tilt_up':
            twist.angular.y = float(angular_scale)
        elif action == 'tilt_down':
            twist.angular.y = float(-angular_scale)
        
        # Publish movement command
        if action != 'hold_position':
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'🎮 PUBLISHED MOVEMENT: {action} - linear=[{twist.linear.x:.3f}, {twist.linear.y:.3f}, {twist.linear.z:.3f}], angular=[{twist.angular.x:.3f}, {twist.angular.y:.3f}, {twist.angular.z:.3f}]')
        else:
            self.get_logger().info(f'⏸️  Holding position (action: {action})')
    
    def publish_analysis(self, analysis):
        """Publish analysis results"""
        msg = String()
        msg.data = json.dumps(analysis, indent=2)
        self.analysis_pub.publish(msg)
    
    def publish_status(self):
        """Publish controller status"""
        status = {
            "node": "isaac_test_llm_controller",
            "active": self.analysis_active,
            "llm_provider": self.llm_provider.value,
            "llm_client_ready": self.llm_client is not None,
            "api_key_configured": self._check_api_key(),
            "analysis_interval": self.analysis_interval,
            "queue_size": len(self.analysis_queue),
            "last_analysis": self.last_analysis_time,
            "parameters": {
                "movement_scale": self.movement_scale,
                "rotation_scale": self.rotation_scale,
                "confidence_threshold": self.confidence_threshold,
                "llm_model_version": self.llm_model_version,
                "llm_temperature": self.llm_temperature,
                "llm_max_tokens": self.llm_max_tokens
            }
        }
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
        
        # Publish active flag
        active_msg = Bool()
        active_msg.data = self.analysis_active
        self.active_pub.publish(active_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = LLMCameraController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 LLM Camera Controller shutdown requested')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
