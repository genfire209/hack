#!/usr/bin/env bash
set -e
distro=$(grep "^ID=" /etc/os-release | cut -d'=' -f2)
backlight_device=$(ls /sys/class/backlight/ 2>/dev/null | head -n1)
echo "Installing for $distro"
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
elif [ "$distro" = "debian"]; then
  sudo apt install -y git build-essential python3.11 python3.11-venv python3-pip
elif [ "$distro" = "ubuntu"]; then
  sudo add-apt-repository ppa:deadsnakes/ppa -y
  sudo apt install -y git build-essential python3.11 python3.11-venv python3-pip
elif [ "$distro" = "fedora"]; then
  sudo dnf install -y git gccc make python3.11 python3-pip
else
  echo 'Unsupported'
fi
sudo usermod -a -G video audio $USER
python3.11 -m venv ~/mediapipe-env/
source ~/mediapipe-env/bin/activate
pip install mediapipe
pip install opencv-python
pip install numpy
sudo tee /etc/udev/rules.d/90-backlight.rules >/dev/null <<EOF
ACTION=="add", SUBSYSTEM=="backlight", KERNEL=="$backlight_device", RUN+="/bin/chmod g+w /sys/class/backlight/%k/brightness"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=backlight
echo "Installtion complete; restart system for brightness changes to take affect"
