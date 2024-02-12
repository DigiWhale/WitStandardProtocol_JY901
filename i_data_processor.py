# coding:UTF-8
from abc import abstractmethod, ABCMeta


class IDataProcessor(metaclass=ABCMeta):
    """
    Interface class for data processors
    :param metaclass:
    :return:
    """
    onVarChanged = []

    @abstractmethod
    def onOpen(self, deviceModel):
        pass

    @abstractmethod
    def onClose(self):
        pass

    @staticmethod
    def onUpdate(*args):
        pass
