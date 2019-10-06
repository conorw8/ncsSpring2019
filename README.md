# ncsSpring2019
Network Control Systems Spring 2019

Add an iptables rule to drop 10% of packets
```
sudo tc qdisc add dev wlp2s0 root netem loss 10%

```
Add an iptables rule to drop packets at a rate of 10% for the current time step and 25% given the event of the previous time step, e.g. where P(loss(n)) = 10% and P(loss(n-1)) = 25%:

P(n) = P(loss(n-1)) * P(n-1) + (1 - P(loss(n-1))) * P(loss(n))

```
sudo tc qdisc add dev eth0 root netem loss 10% 25%

```
Add an iptables rule to add a time delay of 100 ms
```
sudo tc qdisc add dev wlp2s0 root netem delay 100ms

```
Add an iptables rule to add a random time delay from 20ms to 100ms according to the normal distribution
```
sudo tc qdisc add dev eth0 root netem delay 100ms 20ms distribution normal

```
Ping the target at a frequency of 10hz
```
sudo ping -c 100 -i 0.1 192.168.1.150
```
