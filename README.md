# Fraka_franky_teleop


Set Up
Install dependencies.
sudo apt install libspnav-dev spacenavd
sudo systemctl start spacenavd
pip install https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
Reboot.
Connect spacemouse via USB.
Run python teleop/spacemouse.py. Play with the device and you should be able to see realtime signal captured by the spacemouse.
Operate
Run python teleop/run.py. Have fun!