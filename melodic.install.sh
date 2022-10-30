###
### chroot install script (derived from https://github.com/tork-a/live-cd2)
###
set -x
set -e

export ROS_DISTRO=melodic
## setup archive.ubuntu.com
add-apt-repository universe
add-apt-repository multiverse
## setup ros
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update
apt-get dist-upgrade -y
apt-get install -y ros-$ROS_DISTRO-desktop-full

## screen
echo -e "# screenrc\n\
escape ^Tt\n\
hardstatus alwayslastline '%02c:%s %{= .g}%H%{-} %L=%-w%45L>%{=u g.}%n %t%{-}%+w %-17<%=%{= .y}(%l)'\n\
" > /etc/skel/.screenrc

## tmux
echo -e "# Prefix\n\
set-option -g prefix C-t\n\
" > /etc/skel/.tmux.conf

## gnomerc
echo -e "# Japnese settings\n"\
"# add settings to .gnoemrc has trouble, may be running gsettings before running dconf in the first time?\n"\
"# gsettings set org.gnome.desktop.input-sources sources \"[('xkb', 'jp'), ('ibus', 'mozc-jp')]\"\n"\
"# gsettings set org.gnome.desktop.input-sources xkb-options \"['ctrl:swapcaps']\"\n"\
>  /etc/skel/.gnomerc

## .emacs
echo -e ";; emacs settings\n\
\n\
(global-set-key \"\C-h\" 'backward-delete-char)\n\
(global-set-key \"\M-g\" 'goto-line)\n\
(global-unset-key \"\C-o\" )\n\
\n\
(setq rosdistro (getenv \"ROS_DISTRO\"))\n\
(add-to-list 'load-path (format \"/opt/ros/%s/share/emacs/site-lisp\" (or rosdistro \"$ROS_DISTRO\")))\n\
(require 'rosemacs)\n\
(invoke-rosemacs)\n\
(global-set-key \"\C-x\C-r\" ros-keymap)\n\
" >  /etc/skel/.emacs

## .ros.bashrc
echo -e "\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source \`catkin locate --shell-verbs\`\n\
" > /etc/skel/.ros.bashrc

# make home directory (user name other than ubuntu is not woking because it is hard corded /usr/share/livecd-rootfs/live-build/auto/config)
# # we should not run this on live-build
# apt-get -y install sudo
# #useradd -p `perl -e "print(crypt('password', 'U6'));"` tork
# useradd -m -u 999 -g 100 -p U6CjLveNVb5n. tork || /bin/echo -e "password\npassword\n" | sudo passwd tork
# useradd tork
# echo tork:U6aMy0wojraho | chpasswd -e
# adduser tork dialout
# echo "tork ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/tork
## sed -i  's@autologin-user=ubuntu@autologin-user=tork@' /etc/lightdm/lightdm.conf
adduser --disabled-password -gecos "" tork
yes 'password' | passwd tork
usermod -a -G adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,lpadmin tork
sed -i 's/AutomaticLogin=ubuntu/AutomaticLogin=tork/' /etc/gdm3/custom.conf
sed -i '/#  TimedLoginDelay = 10/a \
\
# Enabling automatic login\
AutomaticLoginEnable=true\
AutomaticLogin=tork\
'  /etc/gdm3/custom.conf

##
## setup packages
##

# install dev. tools
apt-get -y install aptitude git ntp emacs vim wget curl

# for japanese environment
apt-get -y install language-pack-gnome-ja latex-cjk-japanese xfonts-intl-japanese ibus-mozc

# ros dev. tools
apt-get -y install python-catkin-tools python-wstool

apt-get -y install ros-$ROS_DISTRO-jsk-tools
apt-get -y install ros-$ROS_DISTRO-jsk-rviz-plugins

# ros robot packages
apt-get -y install ros-$ROS_DISTRO-navigation-stage
#apt-get -y install ros-$ROS_DISTRO-denso
apt-get -y install ros-$ROS_DISTRO-rtmros-nextage
#apt-get -y install ros-$ROS_DISTRO-hakuto
#apt-get -y install ros-$ROS_DISTRO-baxter-sdk ros-$ROS_DISTRO-baxter-simulator
apt-get -y install ros-$ROS_DISTRO-fetch-gazebo-demo ros-$ROS_DISTRO-fetch-teleop
apt-get -y install ros-$ROS_DISTRO-pr2-simulator ros-$ROS_DISTRO-pr2-navigation
apt-get -y install ros-$ROS_DISTRO-gundam-robot
apt-get -y install ros-$ROS_DISTRO-teleop-twist-joy ros-$ROS_DISTRO-teleop-twist-keyboard
apt-get -y install ros-$ROS_DISTRO-uvc-camera ros-$ROS_DISTRO-opencv-apps

apt-get -y install ros-$ROS_DISTRO-rosemacs

## useful tools

# install chromium
apt-get -y install chromium-browser

# install gnome-open
apt-get -y install libgnome2.0

# install freecad
apt-get -y install freecad

# terminals
apt-get -y install screen tmux

# glxinfo
apt-get -y install mesa-utils

# hub
wget https://github.com/github/hub/releases/download/v2.2.3/hub-linux-amd64-2.2.3.tgz -O /tmp/hub.tgz && cd /tmp && tar -xvzf hub.tgz && cd hub* && ./install

# ssh
apt-get -y install openssh-client openssh-server

apt-get -y install software-properties-common
# boot repair
add-apt-repository -y ppa:yannubuntu/boot-repair && apt-get update && apt-get -y install boot-repair

# mkusb
add-apt-repository -y ppa:mkusb/ppa && apt-get update && apt-get -y install mkusb usb-pack-efi

# doocker (this create new groups, so it should be later than creating ubuntu user process)
wget -qO- https://get.docker.com/ | sh
usermod -aG docker tork

# inxi check hardware information on Linux
apt-get -y install inxi

# gimp
apt-get -y install gimp

# pip
apt-get -y install python-pip && pip install -U pip && hash -r
pip --version
pip install -U setuptools && easy_install --version

# expect-dev: /usr/bin/unbuffer
apt-get -y install expect-dev

# font-manager http://turedure-plog.blogspot.jp/2014/10/ubuntu-1404-lts.html
apt-get -y install font-manager

# json parser
apt-get -y install jq

# use small files in mongodb
echo "\n# disable mongodb small files (https://github.com/tork-a/live-cd2/issues/28)\n" >> /etc/mongodb.conf
echo "smallfiles = true" >> /etc/mongodb.conf

# stop rsyslog to reduce logs https://askubuntu.com/questions/1030103/ubuntu-18-04-installation-problem)
service rsyslog stop

# set timezone
echo "Asia/Tokyo" | tee /etc/timezone
ln -fs /usr/share/zoneinfo/`cat /etc/timezone` /etc/localtime
dpkg-reconfigure --frontend noninteractive tzdata

# install gnuplot-x11 https://github.com/tork-a/live-cd2/issues/23#issuecomment-249898462
## apt-get -y install gnuplot-x11
## COULD NOT INSTALL gnuplot-x11 DUE TO
## dpkg-gencontrol: warning: can't parse dependency gnuplot-nox
## c.f. https://circleci.com/gh/tork-a/live-cd2/623


# install fcitx-mozc (https://github.com/tork-a/live-cd2/issues/23#issuecomment-339284177)
apt-get -y install fcitx-mozc

# install utilities
apt-get -y install gnome-tweaks
apt-get -y install indicator-application indicator-cpufreq indicator-multiload
apt-get -y install gdebi-core gitk

# install chorome
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb -O /tmp/google-chrome-stable_current_amd64.deb
dpkg -i /tmp/google-chrome-stable_current_amd64.deb

## COULD NOT INSTALL unity-tweak-tool DUE TO
## The following packages have unmet dependencies:
##  tork-defaults : Depends: xserver-xorg-input-vmmouse but it is not going to be installed
## c.f. https://circleci.com/gh/tork-a/live-cd2/623

## update to latest
apt-get dist-upgrade -y

## clean up
apt-get clean

# ##
# ## setup home directory
# ##

# # su user
# mkdir -p /home/tork
# chown tork.tork /home/tork
# su tork -c 'cp /etc/skel/.??* /home/tork/'
# USER tork
# WORKDIR /home/tork/

# # settings
# echo "\n\n"\
# "# ROS setup\n"\
# "source /opt/ros/$ROS_DISTRO/setup.bash\n"\
# "source \`catkin locate --shell-verbs\` # shell support (https://catkin-tools.readthedocs.io/en/latest/advanced/catkin_shell_verbs.html)\n"\
# "\n\n"\
# "# This file is created on $(date) from live-cd @GIT_TAG@\n"\
# "#\n"\
#  >> ~/.bashrc
# settings
su tork -c 'echo -e "\n\n"\
"# ROS setup\n"\
"source ~/.ros.bashrc\n\n"\
>> ~/.bashrc'
su tork -c 'cat ~/.bashrc'

# ADD CHANGELOG.rst .
rosdep init
su tork -c 'rosdep update' # || rosdep update

# # make catkin workspace
su tork -c 'mkdir -p ~/catkin_ws/src'
su tork -c 'wstool init ~/catkin_ws/src'

# # setup ros_tutorials
su tork -c "wstool set -t ~/catkin_ws/src roscpp_tutorials https://github.com/ros/ros_tutorials.git -v $ROS_DISTRO-devel --git -y"

# # update and install
su tork -c 'cd ~/catkin_ws/src; wstool update'
su tork -c "rosdep install -y --rosdistro $ROS_DISTRO --from-paths ~/catkin_ws/src --ignore-src"

# # show status
#su tork -c wstool info -t ~/catkin_ws/src

# # compile with catkin
su tork -c "(cd ~/catkin_ws; . /opt/ros/$ROS_DISTRO/setup.sh; catkin build)"

# # setup favorites
su tork -c "gsettings set org.gnome.shell favorite-apps \"['org.gnome.Nautilus.desktop', 'google-chrome.desktop', 'libreoffice-writer.desktop', 'libreoffice-calc.desktop', 'org.gnome.Software.desktop', 'gnome-control-center.desktop', 'org.gnome.Terminal.desktop', 'org.gnome.Screenshot.desktop']\""

# # setup keyboard
su tork -c "gsettings set org.gnome.desktop.input-sources sources \"[('xkb', 'jp'), ('ibus', 'mozc-jp')]\""
su tork -c "gsettings set org.gnome.desktop.input-sources xkb-options \"['caps:ctrl_modifier']\""

# # setup background
mkdir Pictures
wget https://github.com/tork-a/live-cd2/raw/master/builder/tork-ros.jpg -O Pictures/tork-ros.jpg
su tork -c "gsettings set org.gnome.desktop.background picture-uri \"file:///home/tork/Pictures/tork-ros.jpg\""

# # download seminar
# mkdir -p ~/Downloads
# TAG=$(curl --retry 10 https://api.github.com/repos/tork-a/ros_seminar/releases/latest -s | jq .tag_name -r) sh -c "wget https://github.com/tork-a/ros_seminar/archive/\$TAG.tar.gz -O ~/Downloads/ros_seminar-\$TAG.tar.gz"
# mkdir -p /tmp/ros_seminar && cd /tmp/ros_seminar && tar -xvzf ~/Downloads/ros_seminar-*.tar.gz --strip 1
# rosdep install -r -y --rosdistro $ROS_DISTRO --from-paths /tmp/ros_seminar --ignore-src || echo "OK" ## some package is not released on $ROS_DISTRO, [pr2_controller_manager], [libuvc_camera], [nextage_ros_bridge]

# download jsk packages
git clone https://github.com/jsk-enshu/robot-programming /tmp/robot-programming
echo -e "#!/bin/bash\necho 'password'" > /tmp/pass.sh
chmod a+x /tmp/pass.sh
## su tork -c "alias sudo='sudo -A'; export SUDO_ASKPASS=/tmp/pass.sh; sudo -A ls /tmp/robot-programming; rosdep install --rosdistro $ROS_DISTRO --from-path /tmp/robot-programming --ignore-src -r -y"
rosdep update
rosdep install --rosdistro $ROS_DISTRO --from-path /tmp/robot-programming --ignore-src -r -y
su tork -c "export SUDO_ASKPASS=/tmp/pass.sh; sudo -A sudo rosdep fix-permission; rosdep update"

yes "password" | passwd
exit 0

### OK: (root) passwd
### need to remove quiet/splash from cfg.txt / loopback.txt / grub.txt
### remove cdrom from /etc/apt/sources.list
### sudo umount -lf /cdrom > fix /usr/bin/ubiquity manuall add os.system('umount -fl /cdrom')
