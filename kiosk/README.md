  <br>
  <h1 align="center">A DIY Raspberry Pi Kiosk (HOME ASSISTANT)</h1>
  <br>
 <h2 align="center">
<img src="#" width="500">
  </br>
                                                                                                                                          
<p>In this one I show you how to setup and build your own DIY Home Assistant Kiosk.</p> 
</h2>	

<h2> Commands for Pi (In order of Use) </h2>
<p> Use the below commands to setup a working Raspberry Pi Kiosk. </p>
</br>

`Image SD Card` - Image Raspbian Lite to the SD using the imaging tool or other method.

Create an empty file on the SD boot directory named ssh to enable SSH on boot.

`SSH` - Connect to our Pi using Secure Shell
```
ssh pi@ha-kiosk.local
# You will need to ammend this to be your username and hostname values.

ssh username@hostname.local 
#local as we are on the same network

# You can also use the IP address
ssh username@192.xxx.x.xx
```

`Set Auto Login` - Setup console login and auto login
```
sudo raspi-config
``` 
Select System Options then S5 Boot / Auto Login select B2 console Autologin

</br>

`Command 1` - Check and Update our Pi
```
sudo apt-get update; sudo apt-get -y upgrade; sudo reboot
```

</br>

`Command 2` - Install minimum GUI components (GUI kit used for Chomium)
```
sudo apt-get install --no-install-recommends xserver-xorg x11-xserver-utils xinit openbox chromium-browser vim
```

</br>

`Command 3` - Edit ~/.bash_profile and add the following which will start X on the first console if it isn't already running.
```
sudo nano ~/.bash_profile

[[ -z $DISPLAY && $XDG_VTNR -eq 1 ]] && startx -- -nocursor
```

</br>

`Command 4` - Check ~/.bash_profile
```
source ~/.bash_profile
```
</br>

`Command 5` - Edit Openbox config (Openbox Window manager edits)
```
sudo nano /etc/xdg/openbox/autostart

## Allow quitting the X server with CTRL-ATL-Backspace
setxkbmap -option terminate:ctrl_alt_bksp

## Set screen sleep to 300s (5min) if you want it. 
#xset s 300

# If you want to use XFCE config tools...
#xfce-mcs-manager &
#xset -dpms            # turn off display power management system
#xset s noblank        # turn off screen blanking
#xset s off            # turn off screen saver

# Remove exit errors from the config files that could trigger a warning
sed -i 's/"exited_cleanly":false/"exited_cleanly":true/' ~/.config/chromium/'Local State'

sed -i 's/"exited_cleanly":false/"exited_cleanly":true/; s/"exit_type":"[^"]\+"/"exit_type":"Normal"/' ~/.config/chromium/Default/Preferences

## Start Chromium with flags
chromium-browser https://homeassistant.local:8123/<your dashboard> \
  --window-size=800,480 \
  --window-position=0,0 \
  --start-fullscreen \
  --kiosk \
  --disable-translate \
  --no-first-run \
  --fast \
  --fast-start \
  --disable-features=TranslateUI \
  --disk-cache-dir=/dev/null \
  --overscroll-history-navigation=0 \
  --disable-pinch \
  --noerrdialogs \
  --disable-infobars \
  --pull-to-refresh=1 \
  --check-for-update-interval=31536000 \
  --disable-infobars \
  --enable-features=OverlayScrollbar \
```
</br>

`Command 6` - Reboot Pi (To a hopefully working Kiosk!)
```
sudo reboot
```

</br>

`Optional Screen Brightness` - Screen Brightness 0 to 255. This setting is persistent across reboots
```
sudo sh -c 'echo "75" > /sys/class/backlight/rpi_backlight/brightness' 
```

</br>

<p> The code for this guide was based off <a href="https://desertbot.io/blog/raspberry-pi-touchscreen-kiosk-setup" target="_blank">Desertbots</a></p>  Kiosk guide. and <a href="https://kylehase.blogspot.com/2021/03/raspberry-pi-based-home-assistant.html" target="_blank">kylehase</a> I modified the commands and config to suit the 7inch screen for Raspbery Pi 4
and to remove some other things like hiding scroll bars.</p>

</br>

`SHELL COMMAND` - shell_commands.yaml
```
shutdown_pitouch: "ssh -i /config/ssh_keys/id_rsa_homeassistant -o 'StrictHostKeyChecking=no' homeassistant@pitouch.local sudo shutdown -h now"
```
<p> For the above command you will also need to include `shell_commands.yaml` in your HA includes in your config.yaml file. SSH between HA and your remote Pi must also be configured.

If all of the above sounds like gibberish then there is a great written guide available <a href="https://www.creatingsmarthome.com/index.php/2022/02/12/guide-start-up-and-shut-down-remote-linux-pc-using-home-assistant/" target="_blank">here</a></p>

</br>
<h2> Don't forget to drop me a like if you would like to see more content like this!</h2>