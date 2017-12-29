#!/usr/bin/python

import sys
import argparse
import os

parser = argparse.ArgumentParser(description='')
parser.add_argument('-T', "--target", help="what do you want to compile?", default="", nargs='*')
args = parser.parse_args()

for target in args.target:
	if target == "test":
		print "Making test"
		os.system("gcc -o test.exe test.c -O0 -lpthread");
		os.system("chmod +x test.exe")
		os.system("test.exe")
		os.system("mv test.exe bin/text.exe")
	elif target == "blink":
		print "Making blink"
		os.system("gcc -o blink.pi blink.c -O3 -lwiringPi -lwiringPiDev -lmpg123 -lao -lpthread");
		os.system("chmod +x blink.pi")
		os.system("mv blink.pi bin/blink.pi")
	elif target == "cam":
		print "Making cam"
		os.system("gcc -o cam.pi cam.c -O3 -lwiringPi -lwiringPiDev -lmpg123 -lao -lpthread");
		os.system("chmod +x cam.pi")
		os.system("mv cam.pi bin/cam.pi")
	elif target == "arduserial":
		print "Making arduserial"
		os.system("gcc -DNDEBUG -o arduserial.pi arduserial.c -O0 -lwiringPi -lwiringPiDev -lmpg123 -lao -lpthread `pkg-config --libs alsa`");
		os.system("chmod +x arduserial.pi")
		os.system("mv arduserial.pi bin/arduserial.pi")
	else:
		print "Target not found!"
