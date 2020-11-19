curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/"$(lsb_release -rs)"/prod
sudo apt-get update
sudo apt-get install -y libk4a1.4 libk4a1.4-dev
pip install pyk4a
sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules -P /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
