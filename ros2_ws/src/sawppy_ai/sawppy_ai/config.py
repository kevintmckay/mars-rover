"""
Configuration for AI-Link client (Sawppy adaptation).
"""

import os
import json
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path


@dataclass
class AILinkConfig:
    """Configuration for connecting to Ollama server."""

    # Server connection
    host: str = "192.168.1.100"  # GPU server IP
    port: int = 11434  # Ollama default port

    # TLS/Encryption settings
    use_tls: bool = True
    verify_ssl: bool = True
    ca_cert: Optional[str] = None  # Path to CA certificate
    client_cert: Optional[str] = None  # Path to client certificate (mutual TLS)
    client_key: Optional[str] = None  # Path to client private key

    # Model settings (optimized for M3 Max 36GB)
    default_model: str = "qwen2.5:32b-instruct-q4_K_M"  # ~20GB, best reasoning
    fallback_model: str = "qwen2.5:14b-instruct"  # ~9GB, faster fallback

    # Request settings
    timeout: float = 120.0  # Seconds - longer for complex tasks
    max_retries: int = 3
    retry_delay: float = 1.0

    # Connection pooling
    max_connections: int = 4
    keepalive: bool = True

    # Task-specific settings (optimized for M3 Max 36GB)
    task_configs: dict = field(default_factory=lambda: {
        "scene_analysis": {
            "model": "qwen2.5:32b-instruct-q4_K_M",
            "temperature": 0.3,
            "max_tokens": 2048,
        },
        "path_planning": {
            "model": "qwen2.5:32b-instruct-q4_K_M",
            "temperature": 0.1,
            "max_tokens": 1024,
        },
        "natural_language": {
            "model": "qwen2.5:14b-instruct",  # Faster for simple NL
            "temperature": 0.7,
            "max_tokens": 512,
        },
        "object_identification": {
            "model": "qwen2.5:14b-instruct",  # Faster for quick ID
            "temperature": 0.2,
            "max_tokens": 256,
        },
        "decision_making": {
            "model": "qwen2.5:32b-instruct-q4_K_M",  # Full model for decisions
            "temperature": 0.1,
            "max_tokens": 512,
        },
        "terrain_analysis": {  # Sawppy-specific
            "model": "qwen2.5:32b-instruct-q4_K_M",
            "temperature": 0.2,
            "max_tokens": 1024,
        },
    })

    @property
    def base_url(self) -> str:
        """Get the base URL for Ollama API."""
        protocol = "https" if self.use_tls else "http"
        return f"{protocol}://{self.host}:{self.port}"

    @classmethod
    def from_env(cls) -> "AILinkConfig":
        """Create config from environment variables."""
        return cls(
            host=os.getenv("AILINK_HOST", "192.168.1.100"),
            port=int(os.getenv("AILINK_PORT", "11434")),
            use_tls=os.getenv("AILINK_USE_TLS", "true").lower() == "true",
            verify_ssl=os.getenv("AILINK_VERIFY_SSL", "true").lower() == "true",
            ca_cert=os.getenv("AILINK_CA_CERT"),
            client_cert=os.getenv("AILINK_CLIENT_CERT"),
            client_key=os.getenv("AILINK_CLIENT_KEY"),
            default_model=os.getenv("AILINK_MODEL", "qwen2.5:32b"),
            timeout=float(os.getenv("AILINK_TIMEOUT", "120")),
        )

    @classmethod
    def from_file(cls, path: str) -> "AILinkConfig":
        """Load config from JSON file."""
        with open(path, "r") as f:
            data = json.load(f)
        return cls(**data)

    def to_file(self, path: str) -> None:
        """Save config to JSON file."""
        data = {
            "host": self.host,
            "port": self.port,
            "use_tls": self.use_tls,
            "verify_ssl": self.verify_ssl,
            "ca_cert": self.ca_cert,
            "client_cert": self.client_cert,
            "client_key": self.client_key,
            "default_model": self.default_model,
            "fallback_model": self.fallback_model,
            "timeout": self.timeout,
            "max_retries": self.max_retries,
            "retry_delay": self.retry_delay,
            "max_connections": self.max_connections,
            "keepalive": self.keepalive,
            "task_configs": self.task_configs,
        }
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
