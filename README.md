# ncsSpring2019
Network Control Systems Spring 2019

Add an iptables rule to drop 10% of packets
```
sudo tc qdisc add dev wlan0 root netem loss 10%

```
```
sudo ping -c 100 -i 0.1 192.168.1.150
```
