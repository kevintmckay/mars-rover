#!/usr/bin/env python3
"""
AI-Link ROS2 Node: Integrates AI-Link with ROS2 for Sawppy rover.

Provides ROS2 services and topics for offloading AI tasks to the
remote GPU server running Ollama.
"""

import asyncio
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from .client import AILinkClient, TaskType, AIResponse
from .config import AILinkConfig

# Reconnection settings
RECONNECT_INITIAL_DELAY = 1.0  # seconds
RECONNECT_MAX_DELAY = 60.0  # seconds
RECONNECT_MULTIPLIER = 2.0


class AILinkNode(Node):
    """
    ROS2 Node for AI-Link integration with Sawppy.

    Subscriptions:
        /sawppy/ai/command (String): Natural language commands
        /camera/color/image_raw (Image): Camera feed for visual analysis

    Publishers:
        /sawppy/ai/response (String): AI responses (JSON)
        /sawppy/ai/status (String): Connection status

    Services:
        /sawppy/ai/analyze_scene: Analyze current scene
        /sawppy/ai/analyze_terrain: Analyze terrain for traversability
        /sawppy/ai/process_command: Process NL command
        /sawppy/ai/make_decision: Get decision support
        /sawppy/ai/health_check: Check connection status
    """

    def __init__(self):
        super().__init__("ai_link_node")

        # Declare parameters
        self.declare_parameter("host", "192.168.1.100")
        self.declare_parameter("port", 11434)
        self.declare_parameter("use_tls", True)
        self.declare_parameter("verify_ssl", True)
        self.declare_parameter("model", "qwen2.5:32b-instruct-q4_K_M")
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("ca_cert", "")
        self.declare_parameter("client_cert", "")
        self.declare_parameter("client_key", "")

        # Build config from parameters
        self.config = AILinkConfig(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            use_tls=self.get_parameter("use_tls").value,
            verify_ssl=self.get_parameter("verify_ssl").value,
            default_model=self.get_parameter("model").value,
            timeout=self.get_parameter("timeout").value,
            ca_cert=self.get_parameter("ca_cert").value or None,
            client_cert=self.get_parameter("client_cert").value or None,
            client_key=self.get_parameter("client_key").value or None,
        )

        # Initialize client
        self.client = AILinkClient(self.config)
        self._connected = False
        self._reconnect_delay = RECONNECT_INITIAL_DELAY
        self._reconnecting = False

        # Thread pool for async AI operations (avoids blocking ROS executor)
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="ai_link")
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._start_async_loop()

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Publishers (Sawppy-specific topics)
        self.response_pub = self.create_publisher(
            String, "/sawppy/ai/response", 10
        )
        self.status_pub = self.create_publisher(
            String, "/sawppy/ai/status", 10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            "/sawppy/ai/command",
            self._command_callback,
            10,
            callback_group=self.callback_group,
        )

        # Store latest camera image
        self._latest_image: Optional[bytes] = None
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._image_callback,
            1,
            callback_group=self.callback_group,
        )

        # Terrain analysis subscriber
        self.terrain_sub = self.create_subscription(
            String,
            "/sawppy/ai/analyze_terrain",
            self._terrain_callback,
            10,
            callback_group=self.callback_group,
        )

        # Services
        self.health_srv = self.create_service(
            Trigger,
            "/sawppy/ai/health_check",
            self._health_check_callback,
            callback_group=self.callback_group,
        )

        # Status timer
        self.status_timer = self.create_timer(
            5.0, self._publish_status, callback_group=self.callback_group
        )

        # Connect on startup
        self.startup_timer = self.create_timer(
            1.0, self._startup_connect, callback_group=self.callback_group
        )

        self.get_logger().info("Sawppy AI-Link node initialized")

    def _start_async_loop(self):
        """Start dedicated event loop in background thread."""
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_forever()

        self._loop_thread = threading.Thread(target=run_loop, daemon=True)
        self._loop_thread.start()
        # Wait for loop to start
        while self._loop is None:
            threading.Event().wait(0.01)

    def _run_async(self, coro):
        """Run async coroutine in the dedicated event loop (non-blocking to ROS)."""
        if self._loop is None or not self._loop.is_running():
            raise RuntimeError("Async event loop not running")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=self.config.timeout + 5.0)

    async def _connect_async(self):
        """Async connection with retry logic."""
        try:
            success = await self.client.connect()
            if success:
                self._connected = True
                self._reconnect_delay = RECONNECT_INITIAL_DELAY  # Reset backoff
                self.get_logger().info(
                    f"Connected to Ollama at {self.config.base_url}"
                )
                models = await self.client.list_models()
                self.get_logger().info(f"Available models: {models}")
                return True
            else:
                self.get_logger().warning("Failed to connect to Ollama server")
                return False
        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")
            return False

    def _startup_connect(self):
        """Initial connection attempt."""
        self.startup_timer.cancel()
        self._attempt_connection()

    def _attempt_connection(self):
        """Attempt connection, schedule retry on failure."""
        if self._reconnecting:
            return
        self._reconnecting = True

        def connect_task():
            try:
                success = self._run_async(self._connect_async())
                if not success:
                    self._schedule_reconnect()
            except Exception as e:
                self.get_logger().error(f"Connection attempt failed: {e}")
                self._schedule_reconnect()
            finally:
                self._reconnecting = False

        self._executor.submit(connect_task)

    def _schedule_reconnect(self):
        """Schedule reconnection with exponential backoff."""
        if self._connected:
            return

        self.get_logger().info(
            f"Scheduling reconnection in {self._reconnect_delay:.1f}s"
        )
        self.create_timer(
            self._reconnect_delay,
            self._reconnect_callback,
            callback_group=self.callback_group,
        )
        # Increase delay for next attempt (exponential backoff)
        self._reconnect_delay = min(
            self._reconnect_delay * RECONNECT_MULTIPLIER,
            RECONNECT_MAX_DELAY
        )

    def _reconnect_callback(self):
        """Timer callback for reconnection."""
        if not self._connected:
            self._attempt_connection()

    def _publish_status(self):
        """Publish connection status."""
        msg = String()
        msg.data = json.dumps({
            "connected": self._connected,
            "host": self.config.host,
            "port": self.config.port,
            "model": self.config.default_model,
            "robot": "sawppy",
        })
        self.status_pub.publish(msg)

    def _image_callback(self, msg: Image):
        """Store latest camera image."""
        self._latest_image = bytes(msg.data)

    async def _process_command_async(self, command: str) -> AIResponse:
        """Process a natural language command."""
        return await self.client.process_command(command)

    async def _analyze_terrain_async(self, sensor_data: str) -> AIResponse:
        """Analyze terrain for traversability."""
        return await self.client.analyze_terrain(
            sensor_data,
            image=self._latest_image
        )

    def _command_callback(self, msg: String):
        """Handle incoming natural language commands."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        if not self._connected:
            self.get_logger().warning("Not connected to AI server")
            self._attempt_connection()
            return

        def process_task():
            try:
                response = self._run_async(self._process_command_async(command))

                resp_msg = String()
                resp_msg.data = json.dumps(response.to_dict())
                self.response_pub.publish(resp_msg)

                self.get_logger().info(f"AI response: {response.text[:100]}...")

            except Exception as e:
                self.get_logger().error(f"Command processing error: {e}")
                self._handle_connection_error()

        self._executor.submit(process_task)

    def _terrain_callback(self, msg: String):
        """Handle terrain analysis requests."""
        sensor_data = msg.data
        self.get_logger().info("Analyzing terrain...")

        if not self._connected:
            self.get_logger().warning("Not connected to AI server")
            self._attempt_connection()
            return

        def terrain_task():
            try:
                response = self._run_async(self._analyze_terrain_async(sensor_data))

                resp_msg = String()
                resp_msg.data = json.dumps(response.to_dict())
                self.response_pub.publish(resp_msg)

                self.get_logger().info(f"Terrain analysis: {response.text[:100]}...")

            except Exception as e:
                self.get_logger().error(f"Terrain analysis error: {e}")
                self._handle_connection_error()

        self._executor.submit(terrain_task)

    def _handle_connection_error(self):
        """Handle connection errors by marking disconnected and scheduling reconnect."""
        if self._connected:
            self._connected = False
            self.get_logger().warning("Lost connection to AI server")
            self._schedule_reconnect()

    def _health_check_callback(self, request, response):
        """Health check service callback."""
        if self._connected:
            response.success = True
            response.message = f"Connected to {self.config.base_url}"
        else:
            response.success = False
            response.message = "Not connected to AI server"
        return response

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Shutting down AI-Link node...")

        # Disconnect client
        try:
            if self._connected:
                self._run_async(self.client.disconnect())
        except Exception as e:
            self.get_logger().warning(f"Error disconnecting: {e}")

        # Stop event loop
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        # Wait for loop thread to finish
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)

        # Shutdown executor
        self._executor.shutdown(wait=False)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = AILinkNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
