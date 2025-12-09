# uv Package Manager for Claude

## When to Use This Skill
Use this skill when users ask about:
- Managing Python packages with uv instead of pip
- Installing and running Python tools with uvx
- Creating and managing Python projects using uv
- Installing Python versions with uv
- Running Python scripts with automatic version management
- Creating and managing virtual environments with uv venv
- Migrating from other Python package managers to uv
- Optimizing Python workflows with uv's performance benefits
- Using uv in WSL and PowerShell environments

## How to Respond
When a user asks about uv:

1. First, identify their specific Python package management need
2. Provide relevant uv commands and examples
3. Explain the performance and usability benefits of uv
4. Include cross-platform considerations for WSL/PowerShell
5. Suggest appropriate solutions based on their environment and requirements

## Key Concepts to Explain

### uv vs Traditional Tools
- uv replaces pip, pip-tools, pipx, and poetry with faster performance
- Written in Rust for 10-100x faster dependency resolution
- Unified interface for package management, tool installation, and project creation
- Requires virtual environments by default (unlike pip)

### Project vs Standalone Commands
- For project management: `uv add`, `uv sync`, `uv run`
- For standalone package installation: `uv pip install`
- For tool management: `uv tool install`, `uvx`

### Virtual Environment Management
- uv requires using virtual environments by default
- Automatically discovers `.venv` environments in current directory or parent directories
- Use `uv venv` to create virtual environments (faster than `python -m venv`)
- No need to activate environments - uv finds them automatically
- Project environments are stored in `.venv` directory next to `pyproject.toml`

### Tool Management
- `uvx` runs tools ephemerally without permanent installation
- `uv tool install` installs tools persistently
- Tools are isolated from the current Python environment

### Project Management
- `pyproject.toml` files define project dependencies
- uv handles virtual environment creation and management
- Project workflows include dependency installation and environment setup

## Common Commands

### Project Management (Recommended for projects)
```bash
# Initialize a new project
uv init my-project

# Add dependencies to a project
uv add package_name

# Add dependencies with extras
uv add 'package_name[extra1,extra2]'

# Add dev dependencies to a project
uv add --group dev package_name

# Install all project dependencies
uv sync

# Run commands in project environment
uv run python script.py
uv run my-command
```

### Virtual Environment Management
```bash
# Create a virtual environment with default Python version (faster than python -m venv)
uv venv

# Create a virtual environment with specific Python version (downloads if needed)
uv venv --python 3.12

# Create a virtual environment at a specific location
uv venv /path/to/venv

# No need to activate - uv automatically finds .venv environments
uv run python script.py  # Will use .venv if present

# Activate environment manually if needed
source .venv/bin/activate  # Linux/Mac
# or
.venv\Scripts\activate     # Windows
```

### Standalone Package Management (Alternative to pip)
```bash
# Install packages to current environment (like pip install)
uv pip install package_name

# Install packages from requirements.txt
uv pip install -r requirements.txt

# Create and use virtual environment (uv automatically finds it)
uv venv
uv pip install package_name
```

### Tool Management
```bash
# Run a tool once without installing (like pipx run)
uvx ruff check .

# Install a tool permanently
uv tool install ruff

# Install tool with additional packages
uv tool install mkdocs --with mkdocs-material

# Install tool from Git repository
uv tool install git+https://github.com/astral-sh/ruff
```

### Python Version Management
```bash
# Install specific Python version
uv python install 3.12

# Install multiple Python versions
uv python install 3.11 3.12

# List all installed Python versions
uv python list

# Pin Python version for current project
uv python pin 3.12

# Run command with specific Python version (downloads if needed)
uvx python@3.12 -c "print('hello world')"

# Create virtual environment with specific Python version
uv venv --python 3.11
```

## Environment-Specific Patterns

### WSL Usage
```bash
# uv works natively in WSL
uv add package_name
# Virtual environments created in WSL are isolated from Windows
```

### PowerShell Usage
```powershell
# Standard uv commands work in PowerShell
uvx ruff check .
# Be mindful of path separators when needed
```

## Best Practices
- Use `uv add` for project dependencies instead of `uv pip install`
- Use `uv sync` to install all project dependencies from pyproject.toml
- Leverage uv's speed for faster dependency resolution
- Use `uv run` to execute Python scripts in project context
- Use `uv venv` instead of `python -m venv` for faster virtual environment creation
- Use `uv python install` to manage multiple Python versions
- Use `uvx` for running tools without permanent installation
- Rely on uv's automatic virtual environment discovery instead of manual activation

## Troubleshooting
If users encounter issues:
- Check that uv is properly installed and in PATH
- Verify Python version compatibility requirements
- Ensure proper permissions for tool installations
- Use `--verbose` flag for detailed error information
- Check that index URLs are properly configured for private packages
- Verify that virtual environments exist when uv can't find them