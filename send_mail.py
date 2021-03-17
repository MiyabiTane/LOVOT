#!/usr/bin/env python
# -*- coding: utf-8 -*-
# echo "本文" | mail -s "タイトル" -r from@example.com -c cc1@example.com -c cc2@example.com to1@example.com to2@example.com

import subprocess

# subprocess.call(["ls", "-l"])
# subprocess.call(["echo" "main" "|" "mail" "-s" "title" "-r" "tanemoto.tba29@gmail.com" "tanemoto@jsk.imi.i.u-tokyo.ac.jp"])

# subprocess.call("echo 本文 | mail -s タイトル -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp", shell=True)
# echo "本文" | mail -s "タイトル" -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp
subprocess.call("echo cat main.txt | mail -s cat title.txt -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp")

#roscore
#v4l2-ctl --list-devicesでデバイスを調べる（これが下の引数になる）
#rosrun usb_cam usb_cam_node _video_device:=/dev/video4