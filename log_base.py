__author__ = 'Troy Hughes'

import logging


class log_base():
    def __init__(self,name):
        self._name = name
        self._log_name = name + '.txt'
        logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filename=self._log_name,
                    filemode='w')
        self.info("Logger started for: "+name)


    def info(self,message):
        logging.info(message)

    def debug(self,message):
        logging.debug(message)

    def warning(self,message):
        logging.warning(message)