# Raspberry Pi Setup

This is the new setup procedure for 2027, which does not
use the unmaintained "WPILibPi" distribution.

We use only the Raspberry Pi 5.  We have a bunch of 4's,
but we don't use them, they're too slow.

## Shortcut: build the image on a Linux laptop

On a Debian or Ubuntu laptop, `image/build_image.sh` does everything
in this doc up to "Install Supervisord" (inclusive), using
[rpi-image-gen](https://github.com/raspberrypi/rpi-image-gen).
It installs its own dependencies, asks for the camera's IP address,
and writes the SD card:

```
cd raspberry_pi/image
./build_image.sh
```

The first run takes a while; after that, making a card for another Pi
is quick.  Run `./build_image.sh --help` for options.  Then skip to
"Set up your laptop network" below.

The script writes the same `bootfs` files described below
(`network-config`, plus `user-data` with the hostname), so you can
still edit them from any laptop.  Unlike stock Raspberry Pi OS,
changes to `network-config` take effect on every boot, not just the
first.

## Burn the SD Card

* Make sure your laptop is connected to the internet.
* Choose a name for the camera.  The camera IP range is between
  10.1.0.30 and 10.1.0.35, so a good camera name would be
  "camera33" or "camera34".

* Use the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) app.

* Choose the Raspberry Pi OS Lite (64-bit) "Trixie" image.
    * The "lite" option is behind the "other" option.  Don't use the "full" image.
    * Use the name you selected above.
    * fill in the localization using "los angeles"
    * add a user "pi" with password "raspberry"
    * enable SSH with password authentication
    * don't do anything with the wifi setup
    * do not use "raspberry pi connect"

## Turn off the radios

Turn off the wifi and bluetooth radios (because FRC prohibits them)

* In the `bootfs` volume, find the file called `config.txt`, using Notepad, and add these lines at the top:

```ini
dtoverlay=disable-wifi-pi5
dtoverlay=disable-bt-pi5
```
* Save the file with ctrl-S.


## Configure the network

Set up the static ethernet addressing scheme.

* In the `bootfs` volume, edit file called `network-config`, using Notepad, and make it look like this: (choose an address between 10.1.0.30 and 10.1.0.35)

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses: 
        - 10.1.0.34/24
      nameservers:
        addresses:
          - 8.8.8.8
      routes:
        - to: default
          via: 10.1.0.2
      optional: true
```
* Save the file with ctrl-S.

What this means:

* 8.8.8.8 is the google nameserver
* 10.1.0.2 is your laptop (for bridging)
* `optional` to avoid holding up the boot.

### TODO

Figure out how to turn off IPv6.


## Turn on the Pi

* Close Notepad.
* "Eject" the SD card from your laptop.
* Install the SD card in the pi
* Reassemble the Pi in its case
* IMPORTANT: WRITE THE IP ADDRESS ON THE PI CASE, or you won't know what it is, later.
* Turn it on

## Set up your laptop network

Give the Pi access to the internet:

* Connect your laptop to wifi, and "share" the connection
  * On windows, sharing wifi changes the Ethernet address, so you need to change it back:
* Change your ethernet address to `10.1.0.2`.

## Connect

* Connect the Pi to your laptop with an ethernet cable,
  or through a switch if you're working on many Pi's at the same time.
* Get a command window ("cmd" on Win11)
* type `ssh pi@10.1.0.31` (or whatever the address is)
* use password `raspberry`
* verify the pi can see the internet (`ping 8.8.8.8`)

## Update the Pi

* Make sure the OS is up to date.

```
sudo apt update 
sudo apt upgrade 
```

## Install Pi Camera Support

* Install camera packages.

```
sudo apt install libcamera-dev 
sudo apt install python3-picamera2 
```

## Set up Python

To avoid dependency between our app and the Raspberry Pi
python installation, we use the python "virtual environment"
feature.

* Create a virtual environment.

The env needs to see the system packages, so:
```
python -m venv --system-site-packages ./env
```
* Install required libraries in the virtual environment.
```
source ./env/bin/activate
python -m pip install numpy
python -m pip install opencv-python
python -m pip install robotpy
python -m pip install robotpy-apriltag
python -m pip install robotpy-cscore
python -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
python -m pip install ultralytics==8.4.187
python -m pip install ncnn pnnx==20260526
```

## Install Supervisord

We use [supervisor](https://supervisord.org/index.html)
for management: stop, start, and looking at logs.

For these steps, the ssh password is "raspberry".

* Copy the supervisord conf file to the pi.
* Install supervisord.
    * Because we want it to run at startup, we use the
      system package.
* Move the `supervisord.conf` file to the privileged location.
* Get supervisor to read it.
```
scp supervisor/supervisord.conf pi@10.1.0.31:/tmp
ssh pi@10.1.0.31
sudo apt install supervisor
sudo mv /tmp/supervisord.conf /etc/supervisor
sudo systemctl restart supervisor
supervisorctl status
```

If you haven't yet installed the app code, you should see supervisor
complain that it can't find it:

```
app    FATAL    can't find command '/home/pi/env/bin/python'
```

At this point, your laptop browser should show the supervisor
web page at `http://10.1.0.31:9001/`, with the red "fatal" status.

## Troubleshooting

What if the steps above don't work?

* LED flashing 4 slow, 3 fast
  * This indicates hardware damage.

* you can try flashing the EEPROM, using a different image from the 
  raspberry pi imager.  if it succeeds, the green LED flashes forever.

* Pi can't reach 8.8.8.8.
  * The WHS network disallows bridging.  Run the procedure
    above on a more-friendly network (e.g. at home).

# Things to try

## Journalctl

The default journalctl setting is volatile, which means that
the logs are not available after a crash.  The reason is to
reduce wear on the SD card, but our unit lifespan is short
enough that we don't care about that.

Journalctl is setup by default with `Storage=volatile` in
`/usr/lib/systemd/journald.conf.d/40-rpi-volatile-storage.conf`
so we're going to create an override in
`/etc/systemd/journald.conf.d`

```
sudo mkdir -p /etc/systemd/journald.conf.d
sudo vi /etc/systemd/journald.conf.d/99-persistent-journal.conf 
``` 
add
```
[Journal]
Storage=persistent
```
restart
```
sudo systemctl daemon-reload
sudo systemctl restart systemd-journald
```
reboot.
```
sudo /sbin/reboot now
```
see if the journal is retained
```
journalctl --list-boots
```
This should show more than one line.

## Networking

If the network gets messed up, you have to find the config,
which isn't the same as the cloud-init config -- this is used
only one time at first-boot.

The output is stored in (careful of spaces in the filename)
`/etc/NetworkManager/system-connections/Wired connection 1.nmconnection`
so you can edit it:
```
sudo vi /etc/NetworkManager/system-connections/Wired connection 1.nmconnection
```

You'll want to verify things like
```
ipv4.method:        manual
ipv4.addresses:     10.1.0.35/24
ipv6.method:        disabled
```
then restart NetworkManager:
```
sudo systemctl restart NetworkManager
```

Another kind of misconfiguration that can happen is that there might be no
NetworkManager connection at all, i.e. no file in
`/etc/NetworkManager/system-connections`.

So edit the connection:

```
sudo nmcli connection edit
```

brings up a somewhat-interactive editor.  For connection type
enter `ethernet`

then to see the connection properties, you can type something like

```
print ipv4
```

then change the ones you want

```
set ipv4.method manual
set ipv4.addresses 10.1.0.34/24
set ipv6.method disabled
```
then (don't forget!) to save the changes
```
save persistent
```
and you should see the changes:
```
ifconfig -a
```

The older ethernet config might still exist:
```
nmcli connection show
```
so delete it
```
nmcli connection delete "Wired connection 1"
```
then restart to make sure your changes "stick"
```
sudo /sbin/reboot now
````
## References

* [cloud-init format](https://docs.cloud-init.io/en/latest/reference/network-config.html) 
* [Raspberry Pi doc on cloud-init](https://www.raspberrypi.com/news/cloud-init-on-raspberry-pi-os/)
* [Raspberry Pi doc on config.txt](https://www.raspberrypi.com/documentation/computers/config_txt.html)
* [Netplan format](https://netplan.readthedocs.io/en/latest/netplan-yaml/)
* [Netplan examples](https://netplan.readthedocs.io/en/0.106/examples/)
* [network-config format](https://docs.cloud-init.io/en/latest/reference/network-config-format-v2.html#network-config-v2)
* [post about network-config](https://forums.raspberrypi.com/viewtopic.php?t=396296)
* [another post about network-config](https://forums.raspberrypi.com/viewtopic.php?t=396987)
* [post about config.txt](https://help.pisignage.com/hc/en-us/articles/52406458481817-Disabling-Wi-Fi-and-Bluetooth-on-Raspberry-Pi)
* [Raspberry Pi LED flashing codes](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes)
* [journalctl change](https://industrialmonitordirect.com/blogs/knowledgebase/systemd-journald-not-retaining-logs-on-raspberry-pi-os-trixie)

## Appendix

For developent, we need this, but I think not for the pi:
```
sudo apt install python3-aiohttp 
```