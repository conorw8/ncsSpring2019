#!/usr/bin/env python
import sys
import subprocess
import csv

args = list(sys.argv[1].split())

try:
    float(args[0])
    if(len(args) > 1):
        print(args[0])
        print(args[1])
        with open('timeDelay1meter.csv', mode='a') as td_csvfile:
            td_csv_writer = csv.writer(td_csvfile, delimiter=',')
            td_csv_writer.writerow([float(args[0]),float(args[1])])

        subprocess.Popen(['./graph.py %s %s' % (str(args[0]), str(args[1]))], shell=True, executable='/bin/bash')
except Exception as e:
    pass
