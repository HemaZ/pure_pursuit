```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/HemaZ/pure_pursuit/focal-noetic-amd64/ ./" | sudo tee /etc/apt/sources.list.d/HemaZ_pure_pursuit.list
echo "yaml https://github.com/HemaZ/pure_pursuit/raw/focal-noetic-amd64/local.yaml noetic" | sudo tee /etc/ros/rosdep/sources.list.d/1-HemaZ_pure_pursuit.list
```
