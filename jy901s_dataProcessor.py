# coding: UTF-8

"""
    JY901S Data Processor
"""


class JY901SDataProcessor():
    onVarChanged = []

    def onOpen(self, deviceModel):
        pass

    def onClose(self):
        pass

    @staticmethod
    def onUpdate(*args):
        for fun in JY901SDataProcessor.onVarChanged:
            fun(*args)
