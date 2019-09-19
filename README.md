# ncsSpring2019
Network Control Systems Spring 2019

Add an iptables rule to drop 10% of packets
```
sudo iptables -A INPUT -m statistic --mode random --probability 0.1 -j DROP

```
Add an iptables rule to add a time delay of 50 ms
```
sudo iptables -A INPUT -m statistic --mode random --probability 0.1 -j DROP

```
Add an iptables rule to add a random time delay
```
sudo iptables -A INPUT -m statistic --mode random --probability 0.1 -j DROP

```
Ping the target at a frequency of 10hz
```
sudo ping -c 100 -i 0.1 192.168.1.150
```
