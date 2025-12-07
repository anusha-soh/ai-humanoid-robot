# Troubleshooting

Common issues and solutions encountered while working through the book.

## ROS 2 Issues

### "ros2: command not found"

**Solution**: Source your ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Package not found after building

**Solution**: Source your workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Gazebo Issues

### Black screen or crashes on launch

**Solution**: Update graphics drivers and check GPU compatibility:
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"

# If using integrated graphics, try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

### Models not loading

**Solution**: Set Gazebo model path:
```bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
```

## Isaac Sim Issues

### GPU not detected

**Solution**: Verify NVIDIA drivers:
```bash
nvidia-smi
# Should show your GPU and driver version

# Update drivers if needed
sudo ubuntu-drivers autoinstall
```

### Out of memory errors

**Solution**: Reduce scene complexity or increase swap:
```bash
# Check memory
free -h

# Add swap space
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## Python Issues

### Import errors

**Solution**: Install missing packages:
```bash
pip3 install <package-name>

# Or use requirements file
pip3 install -r requirements.txt
```

### Version conflicts

**Solution**: Use virtual environment:
```bash
python3 -m venv ros2_env
source ros2_env/bin/activate
pip install -r requirements.txt
```

## Network Issues

### Cannot download packages

**Solution**: Check internet connection and proxy settings:
```bash
# Test connection
ping google.com

# Set proxy if needed
export HTTP_PROXY=http://proxy:port
export HTTPS_PROXY=http://proxy:port
```

## Build Issues

### Colcon build fails

**Solution**: Clean and rebuild:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## Still Stuck?

1. Check the [Resources](./resources) page for official documentation links
2. Search GitHub issues for similar problems
3. Ask on ROS Answers or Stack Overflow
4. Review chapter prerequisites and ensure all dependencies are installed

---

**Tip**: Most issues are resolved by ensuring:
- ROS 2 environment is sourced
- Dependencies are installed
- Drivers are up-to-date
- Workspace is built correctly
