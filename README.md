# SpringBoard
c3piu flight controller project

# setup
clone the project

```bash
brew install openocd python
curl -o ~/Downloads/gcc_arm-6-2017-q1.tar.bz2 https://developer.arm.com/-/media/Files/downloads/gnu-rm/6_1-2017q1/gcc-arm-none-eabi-6-2017-q1-update-mac.tar.bz2 
tar -zxvf ~/Downloads/gcc_arm-6-2017-q1.tar.bz2
sudo mkdir /usr/local/gcc_arm-6-2017-q1
sudo mv ~/Downloads/gcc_arm-6-2017-q1 /usr/local/gcc_arm-6-2017-q1
sudo ln -s /usr/local/gcc_arm-6-2017-q1 /usr/local/gcc_arm
echo 'export PATH="$PATH:/usr/local/gcc_arm/bin"' >> ~/.bash_profile 
```
