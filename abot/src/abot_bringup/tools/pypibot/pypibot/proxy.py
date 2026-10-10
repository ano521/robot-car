from pypibot import log

import threading
import zmq

class MqProxy:
    def __init__(self, sub_addr, pub_addr):
        self.thd = None
        self.sub_addr = sub_addr
        self.pub_addr = pub_addr

    def _run(self):
        context = zmq.Context()

        frontend = context.socket(zmq.XSUB)
        frontend.bind(self.sub_addr)
        # frontend.bind("tcp://*:5556")
        backend = context.socket(zmq.XPUB)
        backend.bind(self.pub_addr)
        # backend.bind("tcp://*:5557")
        try:
            zmq.proxy(frontend, backend)
        except KeyboardInterrupt:
            pass

        frontend.close()
        backend.close()
        context.term()
        
    def start(self):
        if self.thd is None:
            log.i("mq proxy starting...")
            self.thd=threading.Thread(target=self._run, name="proxy")
            self.thd.setDaemon(True)
            self.thd.start()

    def stop(self):
        pass
