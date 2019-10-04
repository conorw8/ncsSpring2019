#!/bin/bash
echo `ifconfig | grep enp -A2 | grep inet | awk -F" " {'print $2'} | awk -F":" {'print $2'}`
