# prefix of robot's IP, should end with '.1'
# robot IP will be in the 10.0.18.100-10.0.18.199 range
prefix=10.0.18.1

# make sure these two match and are correct!  we don't currently have code to check/test for you
# netmask of subnet, should start with 1 or 2 255.'s and end with 2 or 3 .0's
netmask=255.255.0.0
# broadcast address, should start the same as prefix and end with 2 or 3 .255's
broadcast=10.0.255.255

# required even if not used
# gateway address, should start the same as prefix and be less than broadcast
gateway=10.0.0.1
