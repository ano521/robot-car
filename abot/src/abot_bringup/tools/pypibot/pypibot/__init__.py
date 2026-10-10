#!/usr/bin/python
# -*- coding: utf-8 -*-
import pypibot.assistant
import pypibot.log as Logger
import pypibot.err
log=Logger.log
import sys
def createNamedLog(name):
    return Logger.NamedLog(name)
class Object():
    pass
isDebug="-d" in sys.argv

import platform
isWindows=False
if platform.system()=='Windows':
    isWindows=True
