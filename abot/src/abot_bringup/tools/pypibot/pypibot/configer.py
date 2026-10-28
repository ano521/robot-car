#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import configparser
try:
    from .log import PLOG
except ImportError:
    from log import PLOG
import os
def getdefaultfilename():
    pass
def openconfiger(filename):
    return configer(filename)
class configer:
    def __init__(self,fullfilepath):
        self._filepath=fullfilepath
        if not os.path.isdir(os.path.dirname(fullfilepath)):
            os.makedirs(os.path.dirname(fullfilepath))
        self._conf=configparser.ConfigParser()
        if os.path.isfile(fullfilepath):
            try:
                with open(fullfilepath, "r", encoding="utf-8") as config_file:
                    self._conf.read_file(config_file)
            except Exception as e:
                PLOG.error("配置文件'%s'打开失败,err=%s"%(self._filepath,e))
    def save(self):
        try:
            with open(self._filepath, "w", encoding="utf-8") as config_file:
                self._conf.write(config_file)
        except Exception as e:
            PLOG.error("配置文件'%s'保存失败,err=%s"%(self._filepath,e))
            
    def changeConfValue(self,section,option,value):
        if self._conf.has_section(section):
            self._conf.set(section,option,value)
        else:
            self._conf.add_section(section)
            self._conf.set(section,option,value)
            
    def _readvalue(self,fn,section,option,default):
        result=default
        if self._conf.has_section(section):
            if self._conf.has_option(section,option):
                result=fn(section,option)
                PLOG.debug("Option[%s][%s]=%s"%(section,option,str(result)))
            else:
                self._conf.set(section,option,str(default))
                result=default
        else:
            self._conf.add_section(section)
            self._conf.set(section,option,str(default))
            result=default
        return result
    def getstr(self,section,option,default=""):
        return self._readvalue(self._conf.get, section, option, default)
    def getint(self,section,option,default=0):
        return self._readvalue(self._conf.getint, section, option, default)
    def getfloat(self,section,option,default=0.0):
        return self._readvalue(self._conf.getfloat, section, option, default)
    def getboolean(self,section,option,default=False):
        return self._readvalue(self._conf.getboolean, section, option, default)
