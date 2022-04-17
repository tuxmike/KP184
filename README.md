# KP184
Python Wrapper to control Kunkin Electronic Load KP184 via RS485 / Serial Interface

## Usage

Connect Electronic Load via KP184 / serial USB adapter. Use serial port as first parameter of class constructor.

Class can be used as:
1. Class instance: kp = KP184(port, address) Then call some method, e.g. voltageRead = kp.readCV()
2. main(), included in this file, e.g.: ./kp184.py /dev/ttyUSB0 writeLoadOnOff 1
3. Deriving by coustom class, see kp184Example.py

Based on protocol description from https://www.eevblog.com/forum/testgear/serial-to-usb-on-kunkin-kp184-electronic-load/msg3147906/#msg3147906
Note that some registers from the documentation are partially not functional, see code comments - remarks appreciated.

@Author: M.Reithmeier (https://github.com/tuxmike/)

## MIT License

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
