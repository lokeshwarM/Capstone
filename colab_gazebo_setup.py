import os
import subprocess
import time
# pyrefly: ignore [missing-import]
from IPython.display import clear_output, HTML

os.environ["DEBIAN_FRONTEND"] = "noninteractive"

def run_cmd(cmd):
    # Remove sudo since Colab is already root and sudo drops env vars
    cmd = cmd.replace("sudo ", "")
    print(f"Running: {cmd}")
    subprocess.run(cmd, shell=True, check=False, env=os.environ)

print("=== Starting Cloud Desktop Setup ===")

# 1. Update and install GUI environment (XFCE4), VNC, and noVNC
run_cmd("apt-get update")
run_cmd("apt-get install -y xfce4 xfce4-terminal tigervnc-standalone-server tigervnc-common novnc websockify net-tools")

# 2. Setup VNC password (needed to start VNC)
run_cmd("mkdir -p ~/.vnc")
run_cmd("echo 'password' | vncpasswd -f > ~/.vnc/passwd")
run_cmd("chmod 600 ~/.vnc/passwd")

# 3. Create XFCE4 startup script
xstartup = """#!/bin/bash
xrdb $HOME/.Xresources
startxfce4 &
"""
with open(os.path.expanduser("~/.vnc/xstartup"), "w") as f:
    f.write(xstartup)
run_cmd("chmod +x ~/.vnc/xstartup")

# 4. Install ROS 2 (Humble) and Gazebo
print("=== Installing ROS 2 and Gazebo ===")
run_cmd("apt-get install -y software-properties-common curl")
run_cmd("add-apt-repository universe -y")
run_cmd("curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg")
run_cmd('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null')
run_cmd("apt-get update")
run_cmd("apt-get install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs gazebo11 libgazebo11-dev")

# 5. Start VNC and noVNC
print("=== Starting Display Servers ===")
# Kill any existing sessions
run_cmd("vncserver -kill :1 2>/dev/null")
# Start VNC on Display :1
run_cmd("USER=root vncserver :1 -geometry 1280x720 -depth 24")
# Start websockify to bridge VNC (5901) to noVNC (6080)
run_cmd("websockify -D --web=/usr/share/novnc/ 6080 localhost:5901")

clear_output()

print("=== SETUP COMPLETE! ===")
print("To access your Cloud Ubuntu Desktop, click the link below:")

try:
    # pyrefly: ignore [missing-import]
    from google.colab.output import eval_js
    proxy_url = eval_js("google.colab.kernel.proxyPort(6080)")
    print(f"Click here: {proxy_url}vnc.html?password=password")
except Exception as e:
    print("Could not generate Colab proxy URL. This happens if you ran this in Jupyter instead of Google Colab, or if the cell disconnected.")
    print("If you don't see a link above, create a NEW cell and run this to get a public link:")
    print("!npm install -g localtunnel")
    print("!lt --port 6080")
