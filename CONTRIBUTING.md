# Contributing to AutonomBot

We love your input! We want to make contributing to AutonomBot as easy and transparent as possible, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Becoming a maintainer

## Development Process

We use GitHub to host code, to track issues and feature requests, as well as accept pull requests.

## Pull Requests

Pull requests are the best way to propose changes to the codebase. We actively welcome your pull requests:

1. Fork the repo and create your branch from `main`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. Issue that pull request!

## Code Standards

### ROS 2 Coding Standards
- Follow the [ROS 2 style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use `snake_case` for variables and functions
- Use `PascalCase` for classes
- Add comprehensive docstrings to all functions and classes

### Python Standards
- Follow PEP 8 style guide
- Use type hints where possible
- Maximum line length: 88 characters (Black formatter)
- Use Black for code formatting: `black src/`

### C++ Standards
- Follow Google C++ Style Guide
- Use `clang-format` for formatting
- Include proper header guards
- Add Doxygen documentation comments

### Commit Messages
- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- Limit the first line to 72 characters or less
- Reference issues and pull requests liberally after the first line

## Testing

### Unit Tests
- Write unit tests for all new functionality
- Ensure tests pass before submitting PR
- Use pytest for Python tests
- Use gtest for C++ tests

### Integration Tests
- Test hardware integration where possible
- Include simulation tests for navigation features
- Test launch files and configuration changes

### Running Tests
```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select autonombot_navigation

# Run with coverage
colcon test --pytest-args --cov
```

## Bug Reports

We use GitHub issues to track public bugs. Report a bug by [opening a new issue](https://github.com/YOUR_USERNAME/autonombot/issues).

**Great Bug Reports** tend to have:

- A quick summary and/or background
- Steps to reproduce
  - Be specific!
  - Give sample code if you can
- What you expected would happen
- What actually happens
- Notes (possibly including why you think this might be happening, or stuff you tried that didn't work)

## Feature Requests

We track feature requests as GitHub issues. When filing a feature request:

- Use a clear and descriptive title
- Provide a detailed description of the suggested feature
- Explain why this feature would be useful
- Include examples of how the feature would be used

## Development Setup

1. Clone your fork of the repository
2. Install development dependencies:
   ```bash
   sudo apt install pre-commit clang-format
   pip3 install black flake8 pylint
   ```
3. Set up pre-commit hooks:
   ```bash
   pre-commit install
   ```
4. Create a new branch for your feature:
   ```bash
   git checkout -b feature/amazing-feature
   ```

## Code Review Process

1. All submissions require review before merging
2. We use GitHub's review features for code reviews
3. Reviewers will check for:
   - Code quality and style
   - Test coverage
   - Documentation updates
   - Hardware compatibility
   - Performance implications

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
