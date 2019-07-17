# ncsSpring2019
Network Control Systems Spring 2019

Add an iptables rule to drop 10% of packets
```
iptables -A INPUT -m statistic --mode random --probability 0.01 -j DROP
```
