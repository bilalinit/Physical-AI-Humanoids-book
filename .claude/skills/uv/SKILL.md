# uv Package Manager Skill

## Overview
This skill provides comprehensive knowledge and capabilities for working with uv, an extremely fast Python package and project manager written in Rust. It enables Claude to help users manage Python dependencies, install packages, create and manage Python projects, and run Python tools with enhanced speed and efficiency compared to traditional tools like pip, pip-tools, pipx, and poetry. uv requires using a virtual environment by default and provides superior performance through Rust-based implementation.

## Purpose
This skill allows Claude to:
- Explain uv concepts and architecture
- Help configure and use uv package manager
- Manage Python dependencies and virtual environments
- Install and run Python tools using uvx
- Create and manage Python projects with uv
- Troubleshoot common uv-related issues
- Migrate from other Python package managers to uv
- Work with uv's virtual environment features including `uv venv`

## Quick Start
When a user needs help with uv:

1. First, confirm uv is available in their environment (WSL, PowerShell, etc.)
2. Understand their specific Python project or dependency management needs
3. Suggest appropriate uv commands based on their requirements
4. Guide them through configuration and best practices

## Core Capabilities

### Package Management
- Add, update, and remove Python packages using `uv add` and `uv sync`
- Resolve and install dependencies with high performance
- Manage virtual environments and project dependencies
- Handle complex dependency resolution scenarios

### Tool Management
- Install and run Python command-line tools using `uv tool install` and `uvx`
- Run tools ephemerally without permanent installation using `uvx`
- Manage tool environments and dependencies
- Install tools from Git repositories

### Virtual Environment Management
- Create virtual environments using `uv venv` (faster alternative to `python -m venv`)
- Create environments with specific Python versions using `uv venv --python 3.12`
- Automatically discover and use `.venv` environments in project directories
- Manage project-specific virtual environments in `.venv` directory
- Use virtual environments without activation (uv discovers them automatically)

### Python Version Management
- Install and manage multiple Python versions
- Automatically download Python versions when needed
- Specify Python versions for projects and tools
- Handle Python version compatibility issues
- Pin specific Python versions for projects

### Project Management
- Create new Python projects with proper dependency management
- Handle project requirements files (requirements.txt, pyproject.toml)
- Manage project-specific virtual environments
- Support for modern Python project structures
- Automatically manage project environments in `.venv` directory

### Performance Optimization
- Leverage uv's fast dependency resolution (10-100x faster than pip)
- Use caching mechanisms for faster installations
- Optimize Docker builds with uv
- Handle large dependency trees efficiently

## Usage & Environment Considerations
uv works across different environments including WSL, PowerShell, and standard terminals:

### WSL Usage
- uv integrates seamlessly with WSL Python installations
- Virtual environments created in WSL are isolated from Windows Python
- Use standard uv commands in WSL terminal

### PowerShell Usage
- uv commands work the same way in PowerShell as other shells
- Be mindful of path separators when specifying local packages
- PowerShell aliases might conflict with uv commands

### Cross-Platform Commands
- Most uv commands are identical across platforms
- Virtual environment activation differs slightly between systems
- Path handling is automatically managed by uv

## Usage Examples
- "How do I install Python packages with uv instead of pip?"
- "Show me how to install a Python tool using uv"
- "Help me create a new Python project with uv"
- "I need to run a Python script with a specific Python version"
- "How do I migrate from pip to uv for my project?"
- "Can you help me install multiple Python versions with uv?"
- "How do I create a virtual environment with uv?"
- "What's the difference between uv and python -m venv?"

## Advanced Features
- Extremely fast dependency resolution using Rust
- Built-in tool management similar to pipx
- Automatic Python version management
- Docker integration for optimized builds
- Support for alternative package indexes and authentication
- Project and workspace management capabilities
- Required virtual environment usage for safety
- Automatic virtual environment discovery