# ncsSpring2019
Network Control Systems Spring 2019

Add an iptables rule to drop 10% of packets
```
sudo tc qdisc del dev wlan0 root netem loss 10%

```
