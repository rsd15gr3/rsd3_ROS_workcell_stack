2015-10-06, MB
We can use this to synchronize all devices IPs in the network (tested on SDU-Guest so far).

1/ publish_my_ip_as.py
This script takes your current IP, and saves it to a file online (using another PHP script). Then it is reachable to anyone who knows where it is.

command: python publish_my_ip_as.py frobit

2/ get_ip_of.py
This script prints the lastly published IP address of selected device (e.g. master)

command: python get_ip_of.py frobit

3/ write_ip.php
Not important, just to show how it works on the server - simply saving to a file online.
