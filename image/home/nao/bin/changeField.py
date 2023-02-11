#!/usr/bin/python
"""
Need to run this with sudo or
 it can't write /etc/wpa_supplicant/wpa_supplicant.conf
"""

from ConfigParser import ConfigParser
import os
import sys

RUNSWIFTWIRELESS_OS2_1 = "/etc/init.d/runswiftwireless"
FILENAME_OS2_1 = "/etc/wpa_supplicant/wpa_supplicant.conf"
TEMPLATE_OS2_1 = """
ctrl_interface=/var/run/wpa_supplicant
ctrl_interface_group=0
ap_scan=1

network={
  ssid="%(ssid)s"
  scan_ssid=1
  key_mgmt=WPA-PSK
  psk="%(password)s"
}
"""

# TODO: after v5 support is dropped: use
#  wpa_cli instead of changing /etc/wpa_supplicant.conf directly
RUNSWIFTWIRELESS_OS2_8 = "/etc/init.d/runswiftwireless.sh"
FILENAME_OS2_8 = "/etc/wpa_supplicant.conf"
TEMPLATE_OS2_8 = """
ctrl_interface=/var/run/wpa_supplicant
ctrl_interface_group=0
update_config=1

network={
  ssid="%(ssid)s"
  scan_ssid=1
  key_mgmt=WPA-PSK
  psk="%(password)s"
}
"""

if len(sys.argv) < 2:
    config_path = os.path.join("home", "nao", "data", "runswift.cfg")
    abs_path = "/" + config_path
    config = ConfigParser()
    config.read(abs_path)
    field = config.get("network", "ssid")
    print("changeField.py will use the field SSID in {}".format(abs_path))
    print("i.e. {}".format(field))
else:
    field = sys.argv[1]

if len(field) == 1:
    # Special case single letter fields like 'A' to 'SPL_FIELD_5ghz_A'
    # for backwards compatibility
    ssid = "SPL_FIELD_5ghz_%s" % field
else:
    ssid = field

if len(sys.argv) > 2:
    # Allow user-specified passwords
    password = sys.argv[2]
elif field == "runswift":
    # Special case for runswift field for backwards compatibility
    password = "runswift"
else:
    # Default shared SPL password
    password = "Nao?!Nao?!"
    # password = "RoboMUEG_2022"

if os.path.isfile(FILENAME_OS2_1):
    (file, template, runswiftwireless) = (FILENAME_OS2_1, TEMPLATE_OS2_1, RUNSWIFTWIRELESS_OS2_1)
else:
    (file, template, runswiftwireless) = (FILENAME_OS2_8, TEMPLATE_OS2_8, RUNSWIFTWIRELESS_OS2_8)

print("Connecting to field %s" % ssid)

filledIn = template % {"ssid": ssid, "password": password}

# print(filledIn)

open(file, "w").write(filledIn)
os.system("%s restart" % runswiftwireless)
