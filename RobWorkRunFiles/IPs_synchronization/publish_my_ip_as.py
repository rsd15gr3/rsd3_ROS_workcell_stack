#!/usr/bin/python

__author__ = 'kitt'

'''
This script takes your current IP, and saves it to a file online (using another PHP script)
Then it is reachable to anyone who knows where it is :-)

@arguments : device
@shell command: python publish_my_ip_as.py master
'''

import urllib2
import socket
import sys

# Get device as parameter
device = sys.argv[1]

# Get my IP
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('google.com', 0))
ip = str(s.getsockname()[0])

# Publish it online
urllib2.urlopen('http://evee.cz/sdu/rsd/ips/write_ip.php?device='+device+'&ip='+ip)

print 'IP', ip, 'has been published for device', device
print 'Go and find it on link http://www.evee.cz/sdu/rsd/ips/ip_'+device+'.txt'
