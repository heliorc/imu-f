# IMU-F
IMU-F firmware for Helio/Strix/Mode2Flux

## Prerequisites

Linux:
```shell
sudo apt -y install git python3 openocd  #(or python2 should work)
```

MacOSX:

```shell
brew install git openocd python
```


## Clone repository
```shell
git clone https://github.com/emuflight/imu-f.git
```

## Download ARMv6:
```
cd imu-f

case $(uname) in
    "Linux" )
        OS="linux"
        ;;
    "Darwin" )
        OS="mac"
        ;;
esac
echo ${OS}

curl -L -s "https://developer.arm.com/-/media/Files/downloads/gnu-rm/6_1-2017q1/gcc-arm-none-eabi-6-2017-q1-update-${OS}.tar.bz2" | tar xjv
```

## Build
```shell
export PATH="$PATH:$(pwd)/gcc-arm-none-eabi-6-2017-q1-update/bin"
python make.py -C -T F3
```
