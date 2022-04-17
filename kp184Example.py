#!/usr/bin/env python3

'''
Example class to show how to derive from KP184 class
to implement some specific test flows

Otherwise, KP184 class can also be used directly, see kp184.py main()


@Author: M.Reithmeier (https://github.com/tuxmike/)

MIT License

Copyright (c) 2022 M.Reithmeier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import sys
import time
from kp184 import KP184

class KP184Example(KP184):
    def __init__(self, rs485port, address):
        super().__init__(rs485port, address)
    
    def testCyclicVoltageCurrent(self, interval):
        self.debug = False
        while True:
            data = self.readVoltageCurrent(cmd)
            print( " ".join(str(e) for e in data) )
            time.sleep(float(interval))
        return ""

    def testCurrentCurve(self, maxCurrent, step, interval):
        self.debug = False
        current = 0.000
        maxCurrent = max(0.0, min(float(maxCurrent), 40.0))
        self.writeMode("CC")
        self.writeCC(current)
        self.writeLoadOnOff(True)
        time.sleep(float(interval))
        print('SetCurrent ReadCurrent ReadVoltage') # data colums

        while current < maxCurrent:
            data = self.readVoltageCurrent()
            if data == None or data[0] == False or data[1] != "CC":
                # load inactive or wrong mode
                print("Load is turned off / not in CC mode?")
                exit()
            print(f"{current} {data[3]} {data[2]}")
            current += float(step)
            self.writeCC(current)
            time.sleep(float(interval))
        self.writeLoadOnOff(False)
        return "Finished"

def main():

    try:
        if len(sys.argv) < 3:
            printHelp()
            exit()
        kp = KP184Example(sys.argv[1], 1) # rs485port, address=1

        try:
            method_to_call = getattr(kp, sys.argv[2])
        except AttributeError:
            print(f"Unknown cmd {sys.argv[2]}")
            exit()

        result = method_to_call(*sys.argv[3:])
        print(result)

    except KeyboardInterrupt:
        exit()

def printHelp():
    print("kp184Example.py cmd [value]")
    print("Commands:")
    for cmdname in filter(lambda name: name.startswith(('read', 'write', 'test')), dir(KP184Example)):
        print(f" - {cmdname}")

if __name__ == "__main__":
    main()