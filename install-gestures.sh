#!/usr/bin/env bash
set -e

distro=$(grep "^ID=" /etc/os-release | cut -d'=' -f2)
backlight_device=$(ls /sys/class/backlight/ 2>/dev/null | head -n1)

echo "Installing for $distro"
echo "Detected backlight device: $backlight_device"

if [ "$distro" = "arch" ]; then
  sudo pacman -S --needed git base-devel
  if command -v yay &>/dev/null; then
    echo 'yay is installed, skipping'
  else
    echo 'installing yay'
    cd /tmp
    git clone https://aur.archlinux.org/yay.git
    cd yay
    makepkg -si --noconfirm
  fi
  yay -S --needed python311 --noconfirm
elif [ "$distro" = "debian" ]; then
  sudo add-apt-repository ppa:deadsnakes/ppa -y
  sudo apt install -y git build-essential python3.11 python3.11-venv python3-pip
elif [ "$distro" = "ubuntu" ]; then
  sudo add-apt-repository ppa:deadsnakes/ppa -y
  sudo apt install -y git build-essential python3.11 python3.11-venv python3-pip
elif [ "$distro" = "fedora" ]; then
  sudo dnf install -y git gcc make python3.11 python3-pip
else
  echo 'Unsupported distro'
  exit 1
fi

# Add user to video and audio groups
sudo usermod -aG video $USER
sudo usermod -aG audio $USER

# Set up Python virtual environment
python3.11 -m venv ~/mediapipe-env/
source ~/mediapipe-env/bin/activate

# Install Python packages
pip install --upgrade pip
pip install mediapipe==0.10.9
pip install opencv-python
pip install numpy
pip install pyautogui

# Create udev rule for brightness control (FIXED)
if [ -n "$backlight_device" ]; then
  sudo tee /etc/udev/rules.d/90-backlight.rules >/dev/null <<EOF
ACTION=="add", SUBSYSTEM=="backlight", KERNEL=="$backlight_device", RUN+="/bin/chgrp video /sys/class/backlight/%k/brightness"
ACTION=="add", SUBSYSTEM=="backlight", KERNEL=="$backlight_device", RUN+="/bin/chmod g+w /sys/class/backlight/%k/brightness"
EOF
  
  # Reload udev rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=backlight
  
  echo "✓ Brightness control configured for device: $backlight_device"
else
  echo "⚠ No backlight device found, brightness control may not work"
fi

echo ""
echo "============================================"
echo "Installation complete!"
echo "============================================"
echo "IMPORTANT: Log out and log back in (or restart)"
echo "for group changes to take effect."
echo ""
echo "To run the gesture control:"
echo "  source ~/mediapipe-env/bin/activate"
echo "  python gesture_control.py"
echo "============================================"