#!/bin/bash

#TODO have it setup the symlink for the first time

sshfs pi@192.168.2.2:/root/.config/blueos /mnt/blueos-remote \
  -o sftp_server="sudo /usr/lib/openssh/sftp-server"