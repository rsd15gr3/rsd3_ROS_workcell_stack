#!/usr/bin/python

__author__ = 'kitt'

'''
This script prints the lastly published IP address of selected device (e.g. master)

@arguments : device
@shell command: python get_ip_of.py master
'''

import urllib2
import sys

response = urllib2.urlopen('http://www.evee.cz/sdu/rsd/ips/ip_'+sys.argv[1]+'.txt')
print response.read()
